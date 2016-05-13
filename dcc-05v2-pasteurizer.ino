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

#define N_SENSORS 6
#define WATER_TEMP_SENSOR 5
#define BOTTLE_TEMP_SENSOR_MIN 2
#define BOTTLE_TEMP_SENSOR_MAX 4
byte sensorAddr[][8] = {
  {0x28, 0xAA, 0x50, 0x80, 0x06, 0x00, 0x00, 0x10}, // (board)
  {0x28, 0xB7, 0xD2, 0x80, 0x06, 0x00, 0x00, 0x49}, // (power box)
  {0x28, 0xFF, 0xF0, 0xD2, 0xA1, 0x15, 0x04, 0xC7}, // (in-bottle-11)
  {0x28, 0xFF, 0x01, 0xD3, 0xA1, 0x15, 0x04, 0x23}, // (in-bottle-12)
  {0x28, 0xFF, 0x85, 0x8F, 0xA1, 0x15, 0x04, 0x3D}, // (in-bottle-13)
  {0x28, 0xFF, 0x32, 0x8F, 0xA1, 0x15, 0x04, 0x23}, // (thermowell)
};
char * sensorNames[] = {
  "board",
  "powerbox",
  "in-bottle-1",
  "in-bottle-2",
  "in-bottle-3",
  "thermowell",
};

#define SENSOR_FREQ 1000
#define UPLOAD_FREQ 10000


#define SETTINGS_VERSION "8a2x"
struct Settings {
  float lowPoint;       // Waterbath thermostat ON point
  float highPoint;      // Waterbath thermostat OFF point
  float pastUnitLimit;  // PU at which buzzer sounds
} settings = {
  65.0, 65.2, 30.
};


#include "libdcc/webserver.h"
#include "libdcc/onewire.h"
#include "libdcc/settings.h"
#include "libdcc/influx.h"
#include "display.h"


// Flag to indicate that a settings report should be sent to InfluxDB
// during the next loop()
bool doPostSettings = false;

bool relayState = LOW;

String formatSettings() {
  return \
    String("lowPoint=") + String(settings.lowPoint, 3) + \
    String(",highPoint=") + String(settings.highPoint, 3) + \
    String(",pastUnitLimit=") + String(settings.pastUnitLimit, 3);
}

void handleSettings() {
  REQUIRE_AUTH;

  for (int i=0; i<server.args(); i++) {
    if (server.argName(i).equals("lowPoint")) {
      settings.lowPoint = server.arg(i).toFloat();
    } else if (server.argName(i).equals("highPoint")) {
      settings.highPoint = server.arg(i).toFloat();
    } else if (server.argName(i).equals("pastUnitLimit")) {
      settings.pastUnitLimit = server.arg(i).toFloat();
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
ICACHE_FLASH_ATTR void leftKeyChange() {
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
ICACHE_FLASH_ATTR void rightKeyChange() {
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

unsigned long lastSensorIteration;
unsigned long lastUploadIteration;

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
  loadCustomChars(lcd);

  lcd.home();
  lcd.print("    Dominion");
  lcd.setCursor(0, 1);
  lcd.print("    Cider Co.");
  delay(2000);

  lcd.setCursor(0, 1);
  lcd.print("Pasteurizer v1.0");
  delay(2000);
  lcd.clear();

  // Initialize zeroth iteration
  takeAllMeasurementsAsync();
  lastSensorIteration = millis();
  // Offset upload events from sensor events
  lastUploadIteration = millis() + SENSOR_FREQ / 2;
}


float pU;
bool running = false;

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
    if (running) {
      running = false;
    } else {
      running = true;
      pU = 0;
    }
    rightKeyPressed = false;
  }


  // If we are ready to do an upload iteration, do that now
  if (millis() > lastUploadIteration + UPLOAD_FREQ) {
    // FIXME: Upload to Influx
    lastUploadIteration = millis();
    return;
  }

  // If we are NOT ready to do a sensor iteraction, return early
  if (millis() < lastSensorIteration + SENSOR_FREQ) {
    return;
  }

  // Read sensors
  float temp[N_SENSORS];
  float accum = 0.0;
  float numAccum = 0;
  String sensorBody = String(DEVICE_NAME) + " uptime=" + String(millis()) + "i";
  for (int i=0; i<N_SENSORS; i++) {
    Serial.print("Temperature sensor ");
    Serial.print(i);
    Serial.print(": ");
    if (readTemperature(sensorAddr[i], &temp[i])) {
      Serial.print(temp[i]);
      Serial.println();
      if (i >= BOTTLE_TEMP_SENSOR_MIN && i <= BOTTLE_TEMP_SENSOR_MAX) {
        accum += temp[i];
        numAccum++;
      }
      sensorBody += String(",") + sensorNames[i] + "=" + String(temp[i], 3);
    } else {
      temp[i] = NAN;
    }
  }
  Serial.println(sensorBody);

  // Instruct sensors to take measurements for next iteration
  takeAllMeasurementsAsync();

  // Do calculations
  float bottleTemp = NAN;
  if (numAccum > 0) {
    bottleTemp = accum / numAccum;
  } else {
    running = false;
  }

  if (running) {
    pU += computePu(bottleTemp, millis() - lastSensorIteration);
  }
  lastSensorIteration = millis();

  // Regulate water temperature
  if (relayState && temp[WATER_TEMP_SENSOR] > settings.highPoint) {
    relayState = LOW;
  } else if (!relayState && temp[WATER_TEMP_SENSOR] < settings.lowPoint) {
    relayState = HIGH;
  }
  digitalWrite(RELAY_PIN, relayState);

  // Update LEDs
  digitalWrite(RED_LED_PIN, running);

  // Update Display
  lcd.setCursor(0, 0);
  lcd.print("Wtr: " + String(temp[WATER_TEMP_SENSOR], 1) + char(3) + "C");
  lcd.setCursor(0, 1);
  lcd.print("Btl: " + String(bottleTemp, 1) + char(3) + "C");
  lcd.setCursor(13, 1);
  lcd.print(String(int(pU)) + char(2));

  if (WiFi.status() == WL_CONNECTED) {
    lcd.setCursor(15, 0);
    lcd.print(char(0));
  } else {
    lcd.setCursor(15, 0);
    lcd.print(char(1));
  }

}

// As per https://sizes.com/units/pasteurization_unit.htm
double computePu(float temp, float ms) {
  if (temp < 60.0) {
    return 0;
  }

  return pow(1.393, temp - 60.0) * ms / 60000.0;
}
