#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ESP32Time.h>
#include <esp_timer.h>

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;
ESP32Time rtc;

// PIN CONFIGURATION (ESP32-S3 N16R8)
const int PIN_LS = 21;
const int PIN_RELAIS_HOCH = 16;
const int PIN_RELAIS_RUNTER = 17;
const int PIN_RUNTER = 3;
const int PIN_HOCH = 4;
const int PIN_BOARD = 14;

// TIMING & SAFETY CONSTANTS
const unsigned long TASTER_ENTPRELL_MS = 100;
const unsigned long LS_ENTPRELL_MS = 40;
const unsigned long BOARD_ENTPRELL_MS = 50;
const unsigned long RELAIS_OFF_DELAY_MS = 50;
const unsigned long MOTOR_NACHLAUF_MS = 500;
const unsigned long KALIBRIERUNG_TIMEOUT_MS = 120000;
const unsigned long DOUBLE_TAP_MS = 500;
const unsigned long MOTOR_MAX_RUNTIME_MS = 120000;
const unsigned long WIFI_TIMEOUT_MS = 10000;
const unsigned long HEAP_CHECK_INTERVAL_MS = 30000;
const unsigned long WATCHDOG_TIMEOUT_S = 30;
const unsigned long CALIB_AGE_WARNING_MS = 365 * 24 * 3600 * 1000UL;

// LIMITS & SAFETY
const long MAX_IMPULSE = 50000;
const long MIN_RANGE = 5;
const long DEFAULT_HOCH = 500;
const long DEFAULT_RUNTER = 0;
const long EEPROM_NOT_FOUND = -1;
const long ENDPUNKT_BUFFER = 3;
const long MAX_DRIFT_PERCENT = 5;
const long REALISTIC_MAX_IMPULSE = 3000;
const uint32_t MIN_FREE_HEAP = 25000;

// ENUMS
enum SystemState { STATE_BOOT, STATE_NORMAL, STATE_KALIBRIERUNG };
enum MotorState { MOTOR_IDLE, MOTOR_UP, MOTOR_DOWN, MOTOR_STOPPING };
enum KalibrierPhase { KALIBRIERUNG_IDLE, KALIBRIERUNG_REFERENZ, KALIBRIERUNG_OBEN };

// STATE VARIABLES (VOLATILE FOR ISR)
volatile SystemState systemState = STATE_BOOT;
volatile MotorState motorState = MOTOR_IDLE;
volatile KalibrierPhase kalibrierPhase = KALIBRIERUNG_IDLE;

// ISR VARIABLES
volatile long impulsZaehler = 0;
volatile unsigned long lsChangeZeit = 0;
volatile long autoTargetPos = 0;
volatile bool autoFahrtAktiv = false;
volatile bool motorFailed = false;
volatile unsigned long motorStartTime = 0;

// CALIBRATION
long endpunktHoch = DEFAULT_HOCH;
long endpunktRunter = DEFAULT_RUNTER;
unsigned long motorStoppZeit = 0;
unsigned long calibrationAgeMs = 0;

// BUTTONS (HIGH = pressed with +3.3V pulldown)
bool tasterHochAktuell = false;
bool tasterRunterAktuell = false;
bool tasterBoardAktuell = false;
bool tasterHochVorher = false;
bool tasterRunterVorher = false;
bool tasterBoardVorher = false;
bool tasterHochGedrueckt = false;
bool tasterRunterGedrueckt = false;
bool webHochGedrueckt = false;
bool webRunterGedrueckt = false;

// SCHEDULER
int aufStunde = 7, aufMinute = 0;
int zuStunde = 20, zuMinute = 0;
bool aufGesperrt = false;
bool zuGesperrt = false;
int letzteMinute = -1;

// TIMING
unsigned long tasterHochChangeZeit = 0;
unsigned long tasterRunterChangeZeit = 0;
unsigned long tasterBoardChangeZeit = 0;
unsigned long kalibrierStartZeit = 0;
unsigned long lastHeapCheckTime = 0;

// FLAGS
bool eepromDirty = false;
bool halbOffenEnabled = false;
bool bootReferenzErforderlich = true;
unsigned long lastHochTapTime = 0;
int hochTapCount = 0;

// WIFI
String storedSSID = "";
String storedPASS = "";
volatile bool wifiConnecting = false;
unsigned long wifiConnectStart = 0;

// CRC8 TABLE
const uint8_t crc8_table[256] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

// FORWARD DECLARATIONS
uint8_t calculateCRC8(long hoch, long runter);
bool loadCalibration();
bool isDriftDetected(long oldEndpointHoch, long measured);
bool hasElapsed(unsigned long& lastTime, unsigned long interval);
long getImpulsCounterSafe();
void getMotorStatesSafe(long& pos, long& targetUp, long& targetDown, bool& active);
void manageMotoRunoff(unsigned long now);
void manageScheduler();
void readSensors(unsigned long now);
void handleBootState();
void handleNormalState(unsigned long now);
void handleCalibrationState(unsigned long now);
void saveEepromIfNeeded();
void startCalibration();
void driveMotor(MotorState direction);
void stopMotor();
void attemptWifiConnect(boolean verbose);
void manageWifiConnection(unsigned long now);
void checkHeapHealth();
void checkMotorOverrun(unsigned long now);
void handleRoot();

// CRC8 CHECKSUM
uint8_t calculateCRC8(long hoch, long runter) {
  uint8_t crc = 0;
  const uint8_t* data = (const uint8_t*)&hoch;
  for (int i = 0; i < 4; i++) {
    crc = crc8_table[crc ^ data[i]];
  }
  data = (const uint8_t*)&runter;
  for (int i = 0; i < 4; i++) {
    crc = crc8_table[crc ^ data[i]];
  }
  return crc;
}

// SAFE ELAPSED TIME (OVERFLOW-PROOF)
bool hasElapsed(unsigned long& lastTime, unsigned long interval) {
  unsigned long now = millis();
  if ((now - lastTime) >= interval) {
    lastTime = now;
    return true;
  }
  return false;
}

// ISR: OPTICAL SENSOR
void IRAM_ATTR lsInterruptISR() {
  uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
  if ((uint32_t)(now - lsChangeZeit) > LS_ENTPRELL_MS) {
    lsChangeZeit = now;
    
    if (motorState == MOTOR_UP) {
      if (systemState == STATE_KALIBRIERUNG || impulsZaehler < MAX_IMPULSE) {
        impulsZaehler++;
      }
    } else if (motorState == MOTOR_DOWN) {
      if (systemState == STATE_KALIBRIERUNG || systemState == STATE_BOOT) {
        if (impulsZaehler > -MAX_IMPULSE) impulsZaehler--;
      } else if (impulsZaehler > endpunktRunter) {
        impulsZaehler--;
      }
    }
  }
}

// HELPERS
long getImpulsCounterSafe() {
  long value;
  noInterrupts();
  value = impulsZaehler;
  interrupts();
  return value;
}

void getMotorStatesSafe(long& pos, long& targetUp, long& targetDown, bool& active) {
  noInterrupts();
  pos = impulsZaehler;
  targetUp = autoTargetPos;
  targetDown = autoTargetPos;
  active = autoFahrtAktiv;
  interrupts();
}

bool isDriftDetected(long oldEndpointHoch, long measured) {
  if (oldEndpointHoch <= 0) return false;
  long drift = abs(measured - oldEndpointHoch);
  long maxAllowed = (oldEndpointHoch * MAX_DRIFT_PERCENT) / 100;
  return drift > maxAllowed;
}

// WEB: ROOT PAGE
void handleRoot() {
  char aufBuf[10], zuBuf[10], zeitStr[10];
  snprintf(aufBuf, sizeof(aufBuf), "%02d:%02d", aufStunde, aufMinute);
  snprintf(zuBuf, sizeof(zuBuf), "%02d:%02d", zuStunde, zuMinute);
  snprintf(zeitStr, sizeof(zeitStr), "--:--:--");
  
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(zeitStr, sizeof(zeitStr), "%H:%M:%S", &timeinfo);
  }

  long measuredPos = getImpulsCounterSafe();
  int prozent = 0;
  if (endpunktHoch > 0) {
    prozent = (measuredPos * 100) / endpunktHoch;
    if (prozent > 100) prozent = 100;
    if (prozent < 0) prozent = 0;
  }

  bool motorRunning = (motorState == MOTOR_UP || motorState == MOTOR_DOWN);

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  server.sendContent("<!DOCTYPE html><html><head>");
  server.sendContent("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  server.sendContent("<meta charset='utf-8'>");
  if (motorRunning) server.sendContent("<meta http-equiv='refresh' content='2'>");
  server.sendContent("<style>");
  server.sendContent("body{font-family:Arial;text-align:center;background:#f4f4f4;margin:0;padding:20px}");
  server.sendContent("h2{color:#333;margin-bottom:5px}.btn{display:block;width:80%;max-width:320px;margin:10px auto;padding:14px;font-size:18px;color:white;border:none;border-radius:10px;cursor:pointer}");
  server.sendContent(".btn-up{background:#4CAF50}.btn-down{background:#2196F3}.btn-stop{background:#f44336}");
  server.sendContent(".box{background:#fff;padding:12px;border-radius:8px;box-shadow:0 2px 6px rgba(0,0,0,0.08);max-width:360px;margin:10px auto}");
  server.sendContent(".box input{width:90%;padding:8px;margin:5px 0;border:1px solid #ddd;border-radius:4px;box-sizing:border-box}");
  server.sendContent(".small{font-size:13px;color:#666;margin-bottom:8px}.warn{color:red;font-weight:bold}");
  server.sendContent("</style></head><body>");

  server.sendContent("<h2>Rollladen-Steuerung</h2>");

  if (motorFailed) {
    server.sendContent("<div class='warn'>🚨 FEHLER: Motor gesperrt (Sensorausfall?)</div>");
  } else if (bootReferenzErforderlich && systemState == STATE_BOOT) {
    server.sendContent("<div class='warn'>⚠️ Nach Stromausfall: RUNTER fahren & stoppen</div>");
  } else if (calibrationAgeMs > 0 && (millis() - calibrationAgeMs) > CALIB_AGE_WARNING_MS) {
    server.sendContent("<div style='color:orange;font-weight:bold;'>⚠️ Kalibrierung >1 Jahr alt (optional neu kalibrieren)</div>");
  } else {
    server.sendContent("<div class='small'>ESP: ");
    server.sendContent(zeitStr);
    server.sendContent("</div>");
  }

  server.sendContent("<div class='box'><div style='font-weight:bold;color:#333;margin-bottom:8px;'>Position: ");
  server.sendContent(String(prozent));
  server.sendContent("%</div>");

  if (motorState == MOTOR_UP) {
    server.sendContent("<button class='btn btn-stop' onclick=\"location.href='/hoch'\">🛑 STOPP</button>");
    server.sendContent("<button class='btn btn-down' onclick=\"location.href='/runter'\">▼ RUNTER</button>");
  } else if (motorState == MOTOR_DOWN) {
    server.sendContent("<button class='btn btn-up' onclick=\"location.href='/hoch'\">▲ HOCH</button>");
    server.sendContent("<button class='btn btn-stop' onclick=\"location.href='/runter'\">🛑 STOPP</button>");
  } else {
    server.sendContent("<button class='btn btn-up' onclick=\"location.href='/hoch'\">▲ HOCH</button>");
    server.sendContent("<button class='btn btn-down' onclick=\"location.href='/runter'\">▼ RUNTER</button>");
  }
  server.sendContent("</div>");

  server.sendContent("<div class='box'><h3>Zeitschaltuhr</h3><form action='/save_timer' method='GET'>");
  server.sendContent("Öffnen: <input type='time' name='auf' value='");
  server.sendContent(aufBuf);
  server.sendContent("'><br>");
  server.sendContent("Schließen: <input type='time' name='zu' value='");
  server.sendContent(zuBuf);
  server.sendContent("'><br><button type='submit' class='btn' style='background:#ff9800'>Speichern</button></form></div>");

  server.sendContent("<div class='box'><form action='/save_wifi' method='GET'>");
  server.sendContent("SSID: <input name='ssid' type='text'><br>");
  server.sendContent("Pass: <input name='pass' type='password'><br>");
  server.sendContent("<button class='btn' style='background:#607d8b' type='submit'>WLAN speichern</button></form></div>");

  server.sendContent("</body></html>");
}

// SETUP
void setup() {
  delay(500);
  Serial.begin(115200);
  delay(200);

  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.println("[SYSTEM] Watchdog enabled");

  pinMode(PIN_RELAIS_HOCH, OUTPUT);
  pinMode(PIN_RELAIS_RUNTER, OUTPUT);
  pinMode(PIN_RUNTER, INPUT_PULLDOWN);
  pinMode(PIN_HOCH, INPUT_PULLDOWN);
  pinMode(PIN_BOARD, INPUT_PULLDOWN);

  digitalWrite(PIN_RELAIS_HOCH, LOW);
  digitalWrite(PIN_RELAIS_RUNTER, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_LS), lsInterruptISR, FALLING);

  rtc.setTime(0, 0, 12, 1, 1, 2026);

  preferences.begin("rolladen", true);
  aufStunde = preferences.getInt("aufH", 7);
  aufMinute = preferences.getInt("aufM", 0);
  zuStunde = preferences.getInt("zuH", 20);
  zuMinute = preferences.getInt("zuM", 0);
  halbOffenEnabled = preferences.getBool("halbOn", false);
  preferences.end();

  preferences.begin("wifi", true);
  storedSSID = preferences.getString("ssid", "");
  storedPASS = preferences.getString("pass", "");
  preferences.end();

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP_STA);
  delay(100);
  WiFi.softAP("Rollladen-S3", "Cvbn3456+*");
  WiFi.setTxPower(WIFI_POWER_7dBm);
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  dnsServer.start(53, "*", local_ip);

  server.on("/", handleRoot);
  server.on("/hoch", []() { webHochGedrueckt = true; server.sendHeader("Location", "/"); server.send(303); });
  server.on("/runter", []() { webRunterGedrueckt = true; server.sendHeader("Location", "/"); server.send(303); });
  server.on("/save_timer", []() {
    if (server.hasArg("auf") && server.hasArg("zu")) {
      String auf = server.arg("auf"), zu = server.arg("zu");
      aufStunde = auf.substring(0, 2).toInt();
      aufMinute = auf.substring(3, 5).toInt();
      zuStunde = zu.substring(0, 2).toInt();
      zuMinute = zu.substring(3, 5).toInt();
      preferences.begin("rolladen", false);
      preferences.putInt("aufH", aufStunde);
      preferences.putInt("aufM", aufMinute);
      preferences.putInt("zuH", zuStunde);
      preferences.putInt("zuM", zuMinute);
      preferences.end();
      Serial.println("[SETTINGS] Timer saved");
    }
    server.sendHeader("Location", "/");
    server.send(303);
  });
  server.on("/save_wifi", []() {
    if (server.hasArg("ssid")) {
      String ssid = server.arg("ssid"), pass = server.hasArg("pass") ? server.arg("pass") : "";
      preferences.begin("wifi", false);
      preferences.putString("ssid", ssid);
      preferences.putString("pass", pass);
      preferences.end();
      storedSSID = ssid;
      storedPASS = pass;
      attemptWifiConnect(true);
    }
    server.sendHeader("Location", "/");
    server.send(303);
  });
  server.onNotFound(handleRoot);
  server.begin();

  unsigned long now = millis();
  lsChangeZeit = (unsigned long)(esp_timer_get_time() / 1000ULL);
  tasterHochChangeZeit = now;
  tasterRunterChangeZeit = now;
  tasterBoardChangeZeit = now;
  lastHeapCheckTime = now;

  if (!loadCalibration()) {
    preferences.begin("kalib", false);
    preferences.putLong("hoch", DEFAULT_HOCH);
    preferences.putLong("runter", DEFAULT_RUNTER);
    preferences.putUChar("crc", calculateCRC8(DEFAULT_HOCH, DEFAULT_RUNTER));
    preferences.putULong("age", now);
    preferences.end();
    endpunktHoch = DEFAULT_HOCH;
    endpunktRunter = DEFAULT_RUNTER;
    calibrationAgeMs = now;
    bootReferenzErforderlich = false;
  } else {
    systemState = STATE_BOOT;
    bootReferenzErforderlich = true;
    noInterrupts();
    impulsZaehler = 9999;
    interrupts();
    Serial.println("[CALIB] Loaded - waiting for reference drive");
  }

  if (storedSSID.length() > 0) {
    attemptWifiConnect(true);
  }

  Serial.println("[SYSTEM] Setup complete");
}

// MAIN LOOP
void loop() {
  esp_task_wdt_reset();

  unsigned long now = millis();

  dnsServer.processNextRequest();
  server.handleClient();

  manageMotoRunoff(now);
  manageWifiConnection(now);
  checkHeapHealth();
  checkMotorOverrun(now);

  if (systemState == STATE_NORMAL) {
    manageScheduler();
  }

  readSensors(now);

  switch (systemState) {
    case STATE_BOOT:
      handleBootState();
      break;
    case STATE_NORMAL:
      handleNormalState(now);
      break;
    case STATE_KALIBRIERUNG:
      handleCalibrationState(now);
      break;
  }

  saveEepromIfNeeded();
}

// MOTOR OVERRUN PROTECTION
void checkMotorOverrun(unsigned long now) {
  if ((motorState == MOTOR_UP || motorState == MOTOR_DOWN) &&
      (now - motorStartTime) > MOTOR_MAX_RUNTIME_MS) {
    motorFailed = true;
    stopMotor();
    Serial.println("[FATAL] Motor timeout - sensor failure suspected");
  }
}

// SCHEDULER
void manageScheduler() {
  if (systemState != STATE_NORMAL) return;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;

  int h = timeinfo.tm_hour;
  int m = timeinfo.tm_min;

  if (m == letzteMinute) return;

  if (!aufGesperrt && h == aufStunde && m == aufMinute) {
    long target = halbOffenEnabled
      ? endpunktRunter + (endpunktHoch - endpunktRunter) / 2
      : endpunktHoch;

    noInterrupts();
    autoTargetPos = target;
    autoFahrtAktiv = true;
    interrupts();

    driveMotor(MOTOR_UP);
    letzteMinute = m;
    Serial.println("[SCHED] Open triggered");
  }

  if (!zuGesperrt && h == zuStunde && m == zuMinute) {
    noInterrupts();
    autoTargetPos = endpunktRunter;
    autoFahrtAktiv = true;
    interrupts();

    driveMotor(MOTOR_DOWN);
    letzteMinute = m;
    Serial.println("[SCHED] Close triggered");
  }

  if (m != aufMinute && m != zuMinute) {
    letzteMinute = -1;
  }
}

// NORMAL STATE
void handleNormalState(unsigned long now) {
  if (motorFailed) return;

  static bool boardLastState = LOW;

  if (tasterBoardAktuell == HIGH && boardLastState == LOW) {
    boardLastState = HIGH;
    Serial.println("[NORMAL] Board button - start calib");
    startCalibration();
    return;
  } else if (tasterBoardAktuell == LOW) {
    boardLastState = LOW;
  }

  long pos, targetUp, targetDown;
  bool autoActive;
  getMotorStatesSafe(pos, targetUp, targetDown, autoActive);

  if (motorState == MOTOR_UP && pos >= (targetUp - ENDPUNKT_BUFFER)) {
    stopMotor();
  }
  if (motorState == MOTOR_DOWN && pos <= (targetDown + ENDPUNKT_BUFFER)) {
    stopMotor();
  }

  if (tasterHochGedrueckt || webHochGedrueckt) {
    bool was = tasterHochGedrueckt;
    tasterHochGedrueckt = false;
    webHochGedrueckt = false;

    if (was) {
      unsigned long tapNow = millis();
      if ((tapNow - lastHochTapTime) <= DOUBLE_TAP_MS) {
        hochTapCount++;
      } else {
        hochTapCount = 1;
      }
      lastHochTapTime = tapNow;

      if (hochTapCount == 2 && motorState == MOTOR_IDLE) {
        long half = endpunktRunter + (endpunktHoch - endpunktRunter) / 2;
        noInterrupts();
        autoTargetPos = half;
        autoFahrtAktiv = true;
        interrupts();
        Serial.println("[DOUBLE-TAP] Half-open");
        driveMotor(MOTOR_UP);
        hochTapCount = 0;
        return;
      }

      if (motorState != MOTOR_IDLE) {
        stopMotor();
        noInterrupts();
        autoFahrtAktiv = false;
        interrupts();
      } else {
        noInterrupts();
        autoFahrtAktiv = false;
        interrupts();
        if (pos < (endpunktHoch - ENDPUNKT_BUFFER)) {
          driveMotor(MOTOR_UP);
        }
      }
    }
  } else if (tasterRunterGedrueckt || webRunterGedrueckt) {
    tasterRunterGedrueckt = false;
    webRunterGedrueckt = false;

    if (motorState != MOTOR_IDLE) {
      stopMotor();
      noInterrupts();
      autoFahrtAktiv = false;
      interrupts();
    } else {
      noInterrupts();
      autoFahrtAktiv = false;
      interrupts();
      if (pos > (endpunktRunter + ENDPUNKT_BUFFER)) {
        driveMotor(MOTOR_DOWN);
      }
    }
  }
}

// CALIBRATION
void startCalibration() {
  systemState = STATE_KALIBRIERUNG;
  kalibrierPhase = KALIBRIERUNG_REFERENZ;
  kalibrierStartZeit = millis();
  motorFailed = false;
  noInterrupts();
  impulsZaehler = -999;
  interrupts();
  stopMotor();
  Serial.println("[CALIB] Phase 1: Drive DOWN, press board");
}

void handleCalibrationState(unsigned long now) {
  if (now - kalibrierStartZeit > KALIBRIERUNG_TIMEOUT_MS) {
    Serial.println("[CALIB] Timeout!");
    stopMotor();
    systemState = STATE_NORMAL;
    kalibrierPhase = KALIBRIERUNG_IDLE;
    return;
  }

  webHochGedrueckt = false;
  webRunterGedrueckt = false;

  static bool boardMustRelease = true;
  static bool boardLastState = LOW;
  bool boardEdge = false;

  if (tasterBoardAktuell == HIGH) {
    if (boardLastState == LOW) {
      boardLastState = HIGH;
      if (!boardMustRelease) {
        boardEdge = true;
      }
    }
  } else {
    boardLastState = LOW;
    boardMustRelease = false;
  }

  if (kalibrierPhase == KALIBRIERUNG_REFERENZ) {
    if (tasterRunterGedrueckt) {
      tasterRunterGedrueckt = false;
      if (motorState == MOTOR_DOWN) stopMotor();
      else driveMotor(MOTOR_DOWN);
    }
    if (tasterHochGedrueckt) {
      tasterHochGedrueckt = false;
      if (motorState == MOTOR_UP) stopMotor();
      else driveMotor(MOTOR_UP);
    }

    if (boardEdge && motorState == MOTOR_IDLE) {
      noInterrupts();
      impulsZaehler = 0;
      interrupts();
      endpunktRunter = 0;
      kalibrierPhase = KALIBRIERUNG_OBEN;
      boardMustRelease = true;
      Serial.println("[CALIB] Phase 2: Drive UP, press board");
      return;
    }
  } else if (kalibrierPhase == KALIBRIERUNG_OBEN) {
    if (tasterHochGedrueckt) {
      tasterHochGedrueckt = false;
      if (motorState == MOTOR_UP) stopMotor();
      else driveMotor(MOTOR_UP);
    }
    if (tasterRunterGedrueckt) {
      tasterRunterGedrueckt = false;
      if (motorState == MOTOR_DOWN) stopMotor();
      else driveMotor(MOTOR_DOWN);
    }

    if (boardEdge && motorState == MOTOR_IDLE) {
      long measured;
      noInterrupts();
      measured = impulsZaehler;
      interrupts();

      bool valid = (measured >= MIN_RANGE && 
                   measured <= min((long)MAX_IMPULSE, REALISTIC_MAX_IMPULSE));

      if (valid) {
        // Check drift
        if (isDriftDetected(endpunktHoch, measured)) {
          Serial.println("[CALIB] Drift detected but accepting");
        }

        endpunktHoch = measured;
        calibrationAgeMs = millis();
        eepromDirty = true;
        systemState = STATE_NORMAL;
        kalibrierPhase = KALIBRIERUNG_IDLE;

        Serial.print("[CALIB] Success: ");
        Serial.println(measured);

        delay(250);
        digitalWrite(PIN_RELAIS_HOCH, HIGH);
        delay(80);
        digitalWrite(PIN_RELAIS_HOCH, LOW);
        delay(120);
        digitalWrite(PIN_RELAIS_HOCH, HIGH);
        delay(80);
        digitalWrite(PIN_RELAIS_HOCH, LOW);
      } else {
        Serial.print("[CALIB] Invalid: ");
        Serial.println(measured);
        boardMustRelease = true;
      }
    }
  }
}

// SENSOR READING
void readSensors(unsigned long now) {
  bool hochNew = (digitalRead(PIN_HOCH) == HIGH);
  if (hochNew != tasterHochAktuell) {
    if (hasElapsed(tasterHochChangeZeit, TASTER_ENTPRELL_MS)) {
      tasterHochAktuell = hochNew;
      if (tasterHochAktuell && !tasterHochVorher) {
        tasterHochGedrueckt = true;
      }
    }
  } else {
    tasterHochChangeZeit = now;
  }

  bool runterNew = (digitalRead(PIN_RUNTER) == HIGH);
  if (runterNew != tasterRunterAktuell) {
    if (hasElapsed(tasterRunterChangeZeit, TASTER_ENTPRELL_MS)) {
      tasterRunterAktuell = runterNew;
      if (tasterRunterAktuell && !tasterRunterVorher) {
        tasterRunterGedrueckt = true;
      }
    }
  } else {
    tasterRunterChangeZeit = now;
  }

  bool boardNew = (digitalRead(PIN_BOARD) == HIGH);
  if (boardNew != tasterBoardAktuell) {
    if (hasElapsed(tasterBoardChangeZeit, BOARD_ENTPRELL_MS)) {
      tasterBoardAktuell = boardNew;
    }
  }

  tasterHochVorher = tasterHochAktuell;
  tasterRunterVorher = tasterRunterAktuell;
  tasterBoardVorher = tasterBoardAktuell;
}

// BOOT STATE
void handleBootState() {
  static bool driveActive = false;
  static bool boardLastState = LOW;
  static bool firstRun = true;

  if (firstRun) {
    firstRun = false;
    tasterHochGedrueckt = false;
    tasterRunterGedrueckt = false;
    webHochGedrueckt = false;
    webRunterGedrueckt = false;
    stopMotor();
    Serial.println("[BOOT] Waiting for reference");
  }

  if (tasterBoardAktuell == HIGH && boardLastState == LOW) {
    boardLastState = HIGH;
    bootReferenzErforderlich = false;
    startCalibration();
    return;
  } else if (tasterBoardAktuell == LOW) {
    boardLastState = LOW;
  }

  webHochGedrueckt = false;
  webRunterGedrueckt = false;

  if (bootReferenzErforderlich) {
    if (tasterHochGedrueckt) {
      tasterHochGedrueckt = false;
      Serial.println("[BOOT] UP blocked - go DOWN first");
    }
    if (tasterRunterGedrueckt) {
      tasterRunterGedrueckt = false;
      if (motorState == MOTOR_DOWN) {
        stopMotor();
      } else {
        driveMotor(MOTOR_DOWN);
        driveActive = true;
      }
    }
    if (driveActive && motorState == MOTOR_IDLE) {
      noInterrupts();
      impulsZaehler = 0;
      interrupts();
      bootReferenzErforderlich = false;
      systemState = STATE_NORMAL;
      driveActive = false;
      Serial.println("[BOOT] Reference complete");
    }
  } else {
    systemState = STATE_NORMAL;
  }
}

// MOTOR CONTROL
void driveMotor(MotorState direction) {
  if (motorState == MOTOR_STOPPING || motorFailed) return;

  digitalWrite(PIN_RELAIS_HOCH, LOW);
  digitalWrite(PIN_RELAIS_RUNTER, LOW);
  delay(RELAIS_OFF_DELAY_MS);

  if (direction == MOTOR_UP) {
    digitalWrite(PIN_RELAIS_HOCH, HIGH);
    motorState = MOTOR_UP;
    motorStartTime = millis();
  } else if (direction == MOTOR_DOWN) {
    digitalWrite(PIN_RELAIS_RUNTER, HIGH);
    motorState = MOTOR_DOWN;
    motorStartTime = millis();
  }
}

void stopMotor() {
  if (motorState == MOTOR_IDLE || motorState == MOTOR_STOPPING) return;

  digitalWrite(PIN_RELAIS_HOCH, LOW);
  digitalWrite(PIN_RELAIS_RUNTER, LOW);
  motorState = MOTOR_STOPPING;
  motorStoppZeit = millis();
}

void manageMotoRunoff(unsigned long now) {
  if (motorState == MOTOR_STOPPING && (now - motorStoppZeit >= MOTOR_NACHLAUF_MS)) {
    motorState = MOTOR_IDLE;
  }
}

// CALIBRATION EEPROM
bool loadCalibration() {
  preferences.begin("kalib", true);
  long hoch = preferences.getLong("hoch", EEPROM_NOT_FOUND);
  long runter = preferences.getLong("runter", EEPROM_NOT_FOUND);
  uint8_t storedCrc = preferences.getUChar("crc", 0);
  calibrationAgeMs = preferences.getULong("age", 0);
  preferences.end();

  if (hoch == EEPROM_NOT_FOUND || runter == EEPROM_NOT_FOUND || (hoch - runter) < MIN_RANGE) {
    return false;
  }

  uint8_t calcCrc = calculateCRC8(hoch, runter);
  if (calcCrc != storedCrc) {
    Serial.println("[CALIB] CRC FAILED - data corrupted!");
    return false;
  }

  endpunktHoch = hoch;
  endpunktRunter = runter;
  return true;
}

void saveEepromIfNeeded() {
  if (!eepromDirty) return;

  if (endpunktHoch < MIN_RANGE || endpunktHoch > REALISTIC_MAX_IMPULSE) {
    Serial.println("[EEPROM] Invalid endpoint - not saved");
    eepromDirty = false;
    return;
  }

  uint8_t crc = calculateCRC8(endpunktHoch, endpunktRunter);

  preferences.begin("kalib", false);
  preferences.putLong("hoch", endpunktHoch);
  preferences.putLong("runter", endpunktRunter);
  preferences.putUChar("crc", crc);
  preferences.putULong("age", millis());
  preferences.end();

  eepromDirty = false;
  Serial.println("[EEPROM] Calibration saved with CRC");
}

// WIFI
void attemptWifiConnect(boolean verbose) {
  if (storedSSID.length() == 0) return;

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(storedSSID.c_str(), storedPASS.c_str());
  wifiConnecting = true;
  wifiConnectStart = millis();

  if (verbose) {
    Serial.print("[WIFI] Connecting to ");
    Serial.println(storedSSID);
  }
}

void manageWifiConnection(unsigned long now) {
  if (!wifiConnecting) return;

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] Connected: ");
    Serial.println(WiFi.localIP());
    wifiConnecting = false;
    return;
  }

  if (now - wifiConnectStart > WIFI_TIMEOUT_MS) {
    Serial.println("[WIFI] Timeout - AP active");
    wifiConnecting = false;
  }
}

// HEAP MONITORING
void checkHeapHealth() {
  static unsigned long lastCheck = 0;

  if (!hasElapsed(lastCheck, HEAP_CHECK_INTERVAL_MS)) {
    return;
  }

  uint32_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

  if (freeHeap < MIN_FREE_HEAP) {
    Serial.print("[WARN] Low heap: ");
    Serial.print(freeHeap);
    Serial.println(" bytes");
  }
}
