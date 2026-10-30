#define BLYNK_TEMPLATE_ID   "TMPL3rJzyBv0H"
#define BLYNK_TEMPLATE_NAME "Smart Battery system"
#define BLYNK_AUTH_TOKEN    "q7PtNmzYdza_huaOtKHWtjOrysQrjfE-"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <math.h>

#define USE_DS18B20 1

#if USE_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

// -------- Wi-Fi --------
char ssid[] = "POWER OF OPTIONS";
char pass[] = "s9866919507";

// -------- ESP32 pins --------
const int VOLTAGE_PIN = 34;
const int ACS_PIN     = 35;
const int BUZZER_PIN  = 25;
const int SERVO_PIN   = 26;
const int DS18B20_PIN = 27;

// LCD SDA = 21, SCL = 22
LiquidCrystal_I2C lcd(0x27, 16, 2);
BlynkTimer timer;
Servo cutoffServo;

#if USE_DS18B20
OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
#endif

// -------- Calibration --------
const float VOLTAGE_RATIO = 4.55;

// ACS712 settings: unchanged
const float ACS_ZERO_MV = 2500.0;
const float ACS_MV_PER_AMP = 66.0;

// -------- Battery settings --------
const float EMPTY_VOLTAGE = 7.8;
const float FULL_VOLTAGE  = 12.55;
const float LOW_PERCENT   = 60.0;
const float CHARGE_CURRENT_THRESHOLD = -0.40;

const float TEMP_CUTOFF_C = 33.0;
const float TEMP_RESUME_C = 30.5;

const bool POSITIVE_CURRENT_IS_CHARGING = true;
const bool BUZZER_ACTIVE_HIGH = true;

// -------- Readings --------
float batteryVoltage = 0.0;
float batteryCurrent = 0.0;
float batteryPercent = 0.0;
float batteryTempC = -127.0;

bool charging = false;
bool previousCharging = false;
bool fullCutoffLatched = false;
bool lowAlertPlayed = false;
bool tempCutoff = false;
bool tempAlertPlayed = false;
bool tempSensorValid = false;
bool systemOnlineEventSent = false;

unsigned long lastLcdUpdate = 0;
unsigned long lastTempRead = 0;
unsigned long fullMessageUntil = 0;

// -------- Voltage-based charging detection --------
// A rise of 0.08 V starts charging detection.
// While charging, track the highest voltage.
// A fall of 0.08 V from that peak stops charging detection.
const float CHARGE_VOLTAGE_DIFFERENCE = 0.08;

float chargeReferenceVoltage = 0.0;
float chargingPeakVoltage = 0.0;
bool chargeReferenceReady = false;

// -------- Servo pulse state --------
bool servoPulseActive = false;
unsigned long servoReturnAt = 0;

// -------- LCD messages --------
enum ScreenState {
  NORMAL_SCREEN,
  CHARGING_SCREEN,
  LOW_SCREEN
};

ScreenState screenState = NORMAL_SCREEN;
unsigned long screenUntil = 0;

// -------- Buzzer --------
enum BuzzerState {
  BUZZER_IDLE,
  BUZZER_LOW,
  BUZZER_TEMP,
  BUZZER_FULL
};

BuzzerState buzzerState = BUZZER_IDLE;
int fullBeepPhase = 0;
unsigned long buzzerUntil = 0;

// -------- Hardware helpers --------
void setBuzzer(bool on) {
  digitalWrite(BUZZER_PIN,
      on == BUZZER_ACTIVE_HIGH ? HIGH : LOW);
}

void pulseServoToSwitchOff() {
  // The servo mechanically presses the switch.
  // It is not an electrical safety cutoff.
  if (servoPulseActive) return;

  cutoffServo.write(90);
  servoPulseActive = true;
  servoReturnAt = millis() + 700UL;
}

void updateServoPulse() {
  if (servoPulseActive &&
      (long)(millis() - servoReturnAt) >= 0) {
    cutoffServo.write(0);
    servoPulseActive = false;
  }
}

void printLine(int row, String text) {
  if (text.length() > 16) text = text.substring(0, 16);
  while (text.length() < 16) text += ' ';
  lcd.setCursor(0, row);
  lcd.print(text);
}

void showMessage(ScreenState state, unsigned long durationMs) {
  if (fullCutoffLatched || tempCutoff) return;

  screenState = state;
  screenUntil = millis() + durationMs;
}

// -------- Buzzer patterns --------
void startTimedBuzzer(BuzzerState state, unsigned long durationMs) {
  if (buzzerState == BUZZER_FULL) return;

  buzzerState = state;
  buzzerUntil = millis() + durationMs;
  setBuzzer(true);
}

void startFullBuzzer() {
  buzzerState = BUZZER_FULL;
  fullBeepPhase = 0;
  buzzerUntil = millis() + 1000;
  setBuzzer(true);
}

void updateBuzzer() {
  if (buzzerState == BUZZER_IDLE || millis() < buzzerUntil) {
    return;
  }

  if (buzzerState == BUZZER_LOW ||
      buzzerState == BUZZER_TEMP) {
    setBuzzer(false);
    buzzerState = BUZZER_IDLE;
    return;
  }

  if (buzzerState == BUZZER_FULL) {
    fullBeepPhase++;

    if (fullBeepPhase == 1) {
      setBuzzer(false);
      buzzerUntil = millis() + 1000;
    } else if (fullBeepPhase == 2) {
      setBuzzer(true);
      buzzerUntil = millis() + 1000;
    } else {
      setBuzzer(false);
      buzzerState = BUZZER_IDLE;
    }
  }
}

// -------- Sensor functions --------
float readVoltage() {
  float mv = analogReadMilliVolts(VOLTAGE_PIN);
  return (mv / 1000.0) * VOLTAGE_RATIO;
}

// ACS712 current-reading code unchanged
float readCurrent() {
  float mv = analogReadMilliVolts(ACS_PIN);
  return (mv - ACS_ZERO_MV) / ACS_MV_PER_AMP;
}

float estimatePercentage(float v) {
  if (v <= EMPTY_VOLTAGE) return 0.0;
  if (v >= 12.60) return 100.0;

  return constrain(
    (v - EMPTY_VOLTAGE) / (12.60 - EMPTY_VOLTAGE) * 100.0,
    0.0, 100.0
  );
}

// -------- Voltage charging detection --------
void detectChargingFromVoltage() {
  if (!chargeReferenceReady) {
    chargeReferenceVoltage = batteryVoltage;
    chargeReferenceReady = true;

    Serial.println("Charging detection initialized.");
    return;
  }

  if (!charging) {
    float rise = batteryVoltage - chargeReferenceVoltage;

    if (rise >= CHARGE_VOLTAGE_DIFFERENCE) {
      charging = true;
      chargingPeakVoltage = batteryVoltage;

      Serial.println("Charging detected: YES");
    } else if (batteryVoltage < chargeReferenceVoltage) {
      // Follow the lower resting voltage until a rise is detected.
      chargeReferenceVoltage = batteryVoltage;
    }
  } else {
    // Keep the highest voltage seen during this charging period.
    if (batteryVoltage > chargingPeakVoltage) {
      chargingPeakVoltage = batteryVoltage;
    }

    float fallFromPeak = chargingPeakVoltage - batteryVoltage;

    if (fallFromPeak >= CHARGE_VOLTAGE_DIFFERENCE) {
      charging = false;
      chargeReferenceVoltage = batteryVoltage;

      Serial.println("Charging detected: NO");
    }
  }
}

// -------- Temperature --------
void readTemperature() {
#if USE_DS18B20
  if (millis() - lastTempRead < 1000) return;
  lastTempRead = millis();

  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);

  tempSensorValid =
      (t != DEVICE_DISCONNECTED_C &&
       t > -55.0 && t < 125.0);

  if (tempSensorValid) {
    batteryTempC = t;
  }
#else
  tempSensorValid = false;
  batteryTempC = -127.0;
#endif
}

// -------- Temperature protection --------
void updateTemperatureProtection() {
#if USE_DS18B20
  if (!tempSensorValid) {
    tempCutoff = true;
  } else if (batteryTempC >= TEMP_CUTOFF_C) {
    tempCutoff = true;
  } else if (tempCutoff && batteryTempC <= TEMP_RESUME_C) {
    tempCutoff = false;
  }

  if (tempCutoff && !tempAlertPlayed) {
    tempAlertPlayed = true;
    pulseServoToSwitchOff();
    startTimedBuzzer(BUZZER_TEMP, 3000);

    if (Blynk.connected()) {
      Blynk.logEvent("battery_temp_high",
                     "Temperature high or sensor fault. Charging stopped.");
    }
  }

  if (!tempCutoff) {
    tempAlertPlayed = false;
  }
#else
  tempCutoff = false;
#endif
}

// -------- LCD --------
void updateLCD() {
  unsigned long now = millis();

  if (!tempCutoff &&
      screenState != NORMAL_SCREEN &&
      now >= screenUntil) {
    screenState = NORMAL_SCREEN;
  }

  if (now - lastLcdUpdate < 200) return;
  lastLcdUpdate = now;

  if (fullCutoffLatched && now < fullMessageUntil) {
    printLine(0, "Charging full");
    printLine(1, "Remove charger");
    return;
  }

  if (tempCutoff) {
    printLine(0, "TEMP HIGH/FAULT");
    printLine(1, "CHARGE STOPPED");
    return;
  }

  if (screenState == LOW_SCREEN) {
    printLine(0, "Battery below 60%");
    printLine(1, "Keep in charging");
    return;
  }

  if (screenState == CHARGING_SCREEN) {
    printLine(0, "Battery is");
    printLine(1, "charging");
    return;
  }

  char line1[17];
  char line2[17];

  snprintf(line1, sizeof(line1),
           "V:%5.2f I:%4.1fA",
           batteryVoltage, batteryCurrent);

#if USE_DS18B20
  if (tempSensorValid) {
    if (charging) {
      // Exactly 16 characters at 100%:
      // P100% T:28C  CHG
      snprintf(line2, sizeof(line2),
               "P%3.0f%% T:%2.0fC  CHG",
               batteryPercent, batteryTempC);
    } else {
      snprintf(line2, sizeof(line2),
               "P:%3.0f%% T:%2.0fC",
               batteryPercent, batteryTempC);
    }
  } else {
    if (charging) {
      snprintf(line2, sizeof(line2), "TEMP ERR  CHG");
    } else {
      snprintf(line2, sizeof(line2), "TEMP SENSOR ERR");
    }
  }
#else
  if (charging) {
    snprintf(line2, sizeof(line2), "P:%3.0f%% T:-- CHG",
             batteryPercent);
  } else {
    snprintf(line2, sizeof(line2), "P:%3.0f%% T:-- C",
             batteryPercent);
  }
#endif

  printLine(0, String(line1));
  printLine(1, String(line2));
}

// -------- Blynk connection and phone notification --------
void maintainBlynkConnection() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!Blynk.connected()) {
    Blynk.connect(1000);
  }

  if (Blynk.connected() && !systemOnlineEventSent) {
    Blynk.logEvent("system_online",
                   "Smart Battery System is online. Phone notifications are connected.");
    systemOnlineEventSent = true;
  }
}

// -------- Blynk data --------
void sendBlynkData() {
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(V1, (int)batteryPercent);
  Blynk.virtualWrite(V3,
      tempSensorValid ? batteryTempC : 0);
  Blynk.virtualWrite(V2, batteryVoltage);
  Blynk.virtualWrite(V5, charging ? 1 : 0);
  Blynk.virtualWrite(V4, batteryCurrent);
}

// -------- Main battery logic --------
void updateBatterySystem() {
  batteryVoltage = readVoltage();
  batteryCurrent = readCurrent();
  batteryPercent = estimatePercentage(batteryVoltage);

  readTemperature();
  updateTemperatureProtection();

  // Charging status is detected using battery-voltage change.
  // ACS712 current readings remain available for display.
  detectChargingFromVoltage();

  // Low battery alert: once per low-battery episode.
  if (batteryPercent < LOW_PERCENT && !lowAlertPlayed) {
    lowAlertPlayed = true;

    if (!fullCutoffLatched && !tempCutoff) {
      showMessage(LOW_SCREEN, 3000);
      startTimedBuzzer(BUZZER_LOW, 3000);
    }

    if (Blynk.connected()) {
      Blynk.logEvent("battery_low",
                     "Battery below 60%. Keep in charging.");
    }
  }

  if (batteryPercent >= LOW_PERCENT + 2.0) {
    lowAlertPlayed = false;
  }

  // Charging starts: existing notification event is unchanged.
  if (charging && !previousCharging &&
      !fullCutoffLatched && !tempCutoff) {
    showMessage(CHARGING_SCREEN, 3000);

    if (Blynk.connected()) {
      Blynk.logEvent("battery_charging",
                     "Battery charging detected.");
    }
  }

  // Full cutoff
  if (charging && batteryVoltage >= FULL_VOLTAGE &&
      !fullCutoffLatched) {
    fullCutoffLatched = true;
    fullMessageUntil = millis() + 3000UL;

    pulseServoToSwitchOff();
    startFullBuzzer();

    if (Blynk.connected()) {
      Blynk.logEvent("battery_full",
                     "Battery is full. Remove charger.");
    }
  }

  previousCharging = charging;

  Serial.print("V=");
  Serial.print(batteryVoltage, 2);
  Serial.print("V | I=");
  Serial.print(batteryCurrent, 2);
  Serial.print("A | Battery=");
  Serial.print(batteryPercent, 0);
  Serial.print("% | Charging=");
  Serial.print(charging ? "YES" : "NO");
  Serial.print(" | Temp=");

#if USE_DS18B20
  if (tempSensorValid) Serial.print(batteryTempC, 1);
  else Serial.print("INVALID");
#else
  Serial.print("NOT CONNECTED");
#endif

  Serial.print(" | Temp cutoff=");
  Serial.print(tempCutoff ? "YES" : "NO");
  Serial.print(" | Full cutoff=");
  Serial.println(fullCutoffLatched ? "YES" : "NO");

  updateLCD();
}

// -------- Manual full-cutoff reset --------
void checkSerialReset() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "RESET") {
    if (batteryVoltage < 12.20 &&
        fabs(batteryCurrent) < 0.20) {
      fullCutoffLatched = false;
      screenState = NORMAL_SCREEN;
      lowAlertPlayed = false;
      setBuzzer(false);
      buzzerState = BUZZER_IDLE;

      Serial.println("Full cutoff reset.");
      Serial.println("Verify charger and wiring before use.");
    } else {
      Serial.println("Reset refused: check charger, voltage and current.");
    }
  }
}

// -------- Setup --------
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  cutoffServo.setPeriodHertz(50);
  cutoffServo.attach(SERVO_PIN, 500, 2400);
  cutoffServo.write(0);

  setBuzzer(false);

  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);
  analogSetPinAttenuation(ACS_PIN, ADC_11db);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

#if USE_DS18B20
  tempSensor.begin();
#endif

  fullCutoffLatched = false;

  

  printLine(0, "Smart Battery");
  printLine(1, "System Ready");
  delay(2000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - startTime < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect(5000);
  }

  printLine(0, "System ready");
  printLine(1, "Monitoring...");
  delay(1000);

  updateBatterySystem();

  timer.setInterval(1000L, updateBatterySystem);
  timer.setInterval(10000L, maintainBlynkConnection);
  timer.setInterval(10000L, sendBlynkData);
}

// -------- Loop --------
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }

  timer.run();
  updateBuzzer();
  updateServoPulse();
  updateLCD();
  checkSerialReset();
}