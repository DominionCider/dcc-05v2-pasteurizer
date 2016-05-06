#include <ESP8266WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define these in the config.h file
//#define WIFI_SSID "yourwifi"
//#define WIFI_PASSWORD "yourpassword"
//#define INFLUX_HOSTNAME "data.example.com"
//#define INFLUX_PORT 8086
//#define INFLUX_PATH "/write?db=<database>&u=<user>&p=<pass>"
//#define WEBSERVER_USERNAME "something"
//#define WEBSERVER_PASSWORD "something"
#include "config.h"

#define DEVICE_NAME "dcc-05v2-pasteurizer"

#define RED_LED_PIN 16
#define GREEN_LED_PIN 14
#define RELAY_PIN 13
#define BUZZER_PIN 12
#define ONE_WIRE_PIN 2
#define SW2_LEFT_PIN 0
#define SW3_RIGHT_PIN 15

#define N_SENSORS 3
#define COMPRESSOR 0
#define EVAPORATOR 1
#define AMBIANT 2
byte sensorAddr[N_SENSORS][8] = {
  {0x28, 0xAA, 0x50, 0x80, 0x06, 0x00, 0x00, 0x10}, // (board)
  {0x28, 0xFF, 0xF0, 0xD2, 0xA1, 0x15, 0x04, 0xC7}, // (in-bottle-11)
  {0x28, 0xFF, 0x01, 0xD3, 0xA1, 0x15, 0x04, 0x23}, // (in-bottle-12)
};
char * sensorNames[N_SENSORS] = {
  "board",
  "in-bottle-1",
};


// The minimum time after shutting off the compressor
// before it is allowed to be turned on again (ms)
#define MIN_RESTART_TIME 300000

#define SETTINGS_VERSION "8G22"
struct Settings {
  float lowPoint;
  float highPoint;
  float lowComp;
  float highComp;
} settings = {
  10., 11., 55., 50.
};


#include "libdcc/webserver.h"
#include "libdcc/onewire.h"
#include "libdcc/settings.h"
#include "libdcc/influx.h"


// Flag to indicate that a settings report should be sent to InfluxDB
// during the next loop()
bool doPostSettings = false;

bool relayState = LOW;

String formatSettings() {
  return \
    String("lowPoint=") + String(settings.lowPoint, 3) + \
    String(",highPoint=") + String(settings.highPoint, 3) + \
    String(",lowComp=") + String(settings.lowComp, 3) + \
    String(",highComp=") + String(settings.highComp, 3);
}

void handleSettings() {
  REQUIRE_AUTH;

  for (int i=0; i<server.args(); i++) {
    if (server.argName(i).equals("lowPoint")) {
      settings.lowPoint = server.arg(i).toFloat();
    } else if (server.argName(i).equals("highPoint")) {
      settings.highPoint = server.arg(i).toFloat();
    } else if (server.argName(i).equals("lowComp")) {
      settings.lowComp = server.arg(i).toFloat();
    } else if (server.argName(i).equals("highComp")) {
      settings.highComp = server.arg(i).toFloat();
    } else {
      Serial.println("Unknown argument: " + server.argName(i) + ": " + server.arg(i));
    }
  }

  saveSettings();

  String msg = String("Settings saved: ") + formatSettings();
  Serial.println(msg);
  server.send(200, "text/plain", msg);

  doPostSettings = true;
}

// As per https://arduino-info.wikispaces.com/LCD-Blue-I2C#v1
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Debounce value
#define KEY_PRESS_DURATION 20

// Long push duration
#define KEY_PRESS_LONG 1000

volatile unsigned long leftKeyDownTime = 0;
volatile boolean leftKeyPressed = false;
void leftKeyChange() {
  noInterrupts();
  delayMicroseconds(15000);
  if (!digitalRead(SW2_LEFT_PIN)) {
    if (!leftKeyDownTime) {
      leftKeyDownTime = millis();
      Serial.println("LEFT DOWN");
    }
  } else {
    if (leftKeyDownTime == 0) return;
    Serial.println("LEFT UP");

    if (millis() - leftKeyDownTime > KEY_PRESS_DURATION) {
      // Register key press
      leftKeyPressed = true;
    }

    leftKeyDownTime = 0;
  }
  interrupts();
}

volatile unsigned long rightKeyDownTime = 0;
volatile boolean rightKeyPressed = false;
void rightKeyChange() {
  noInterrupts();
  delayMicroseconds(15000);
  if (digitalRead(SW3_RIGHT_PIN)) {
    if (!rightKeyDownTime) {
      rightKeyDownTime = millis();
      Serial.println("RIGHT DOWN");
    }
  } else {
    if (rightKeyDownTime == 0) return;
    Serial.println("RIGHT UP");

    if (millis() - rightKeyDownTime > KEY_PRESS_DURATION) {
      // Register key press
      rightKeyPressed = true;
    }

    rightKeyDownTime = 0;
  }
  interrupts();
}

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // SW2 is normally high, button pulls it to GND
  pinMode(SW2_LEFT_PIN, INPUT);
  digitalWrite(SW2_LEFT_PIN, HIGH);
  attachInterrupt(digitalPinToInterrupt(SW2_LEFT_PIN), leftKeyChange, CHANGE);

  // SW3 is normally low, button pulls it to 3.3v
  pinMode(SW3_RIGHT_PIN, INPUT);
  digitalWrite(SW3_RIGHT_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(SW3_RIGHT_PIN), rightKeyChange, CHANGE);

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  server.on("/settings", handleSettings);
  server.on("/restart", handleRestart);
  server.on("/status", handleStatus);
  server.on("/sensors", handleSensors);
  server.onNotFound(handleNotFound);
  server.begin();

  loadSettings();
  Serial.println(formatSettings());

  Wire.begin(4, 5); // SDA=4, SCL=5
  lcd.begin(16, 2);
  lcd.backlight();

  lcd.home();
  lcd.print("    Dominion");
  lcd.setCursor(0, 1);
  lcd.print ("    Cider Co.");
  delay(2000);

  lcd.setCursor(0, 1);
  lcd.print ("Pasteurizer v1.0");
  delay(2000);
}


unsigned long lastIteration;
void loop() {
  server.handleClient();

  /*
  if (!leftKeyPressed && leftKeyDownTime && (millis() - leftKeyDownTime > KEY_PRESS_LONG)) {
    leftKeyDownTime = 0;
    digitalWrite(GREEN_LED_PIN, !digitalRead(GREEN_LED_PIN));
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
  }
  if (!rightKeyPressed && rightKeyDownTime && (millis() - rightKeyDownTime > KEY_PRESS_LONG)) {
    rightKeyDownTime = 0;
    digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
  }
  */

  if (leftKeyPressed) {
    digitalWrite(GREEN_LED_PIN, !digitalRead(GREEN_LED_PIN));
    leftKeyPressed = false;
  }
  if (rightKeyPressed) {
    digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
    rightKeyPressed = false;
  }

  if (millis() < lastIteration + 10000) return;
  lastIteration = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to wifi...");
    return;
  }
  Serial.println("Wifi connected to " + WiFi.SSID() + " IP:" + WiFi.localIP().toString());

  lcd.clear();
  lcd.print(WiFi.SSID());
  lcd.setCursor(0, 1 );
  lcd.print(WiFi.localIP().toString());

}
