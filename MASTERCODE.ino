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

// ============================================================================
// PIN-KONFIGURATION & KONSTANTEN
// ============================================================================
const int PIN_LS = 21;            // Lichtschranke (fallende Flanke = Impuls)
const int PIN_RELAIS_HOCH = 16;   // Relais Motor nach OBEN
const int PIN_RELAIS_RUNTER = 17; // Relais Motor nach UNTEN
const int PIN_RUNTER = 22;        // Wandtaster UNTEN (gegen GND)
const int PIN_HOCH = 23;          // Wandtaster OBEN (gegen GND)
const int PIN_BOARD = 0;          // Board-Taste (Kalibrierung, gegen GND)

const unsigned long TASTER_ENTPRELL_MS = 100;
const unsigned long LS_ENTPRELL_MS = 40;      // FIX: Erhöht von 15ms auf 40ms für zuverlässige Entprellung
const unsigned long BOARD_ENTPRELL_MS = 50;
const unsigned long RELAIS_OFF_DELAY_MS = 50;
const unsigned long MOTOR_NACHLAUF_MS = 500;
const unsigned long KALIBRIERUNG_TIMEOUT_MS = 120000;
const unsigned long DOUBLE_TAP_MS = 500; // Doppeltipp-Fenster für HOCH

const long MAX_IMPULSE = 50000;
const long MIN_RANGE = 5;           // Mindestbereich auf 5 Impulse gesetzt
const long DEFAULT_HOCH = 500;
const long DEFAULT_RUNTER = 0;
const long EEPROM_NOT_FOUND = -1;

enum SystemState { STATE_BOOT, STATE_NORMAL, STATE_KALIBRIERUNG };
enum MotorState { MOTOR_IDLE, MOTOR_UP, MOTOR_DOWN, MOTOR_STOPPING };
enum KalibrierPhase { KALIBRIERUNG_IDLE, KALIBRIERUNG_REFERENZ, KALIBRIERUNG_OBEN };

SystemState systemState = STATE_BOOT;
MotorState motorState = MOTOR_IDLE;
KalibrierPhase kalibrierPhase = KALIBRIERUNG_IDLE;

volatile long impulsZaehler = 0;            // Wird in ISR verändert -> atomar behandeln
volatile unsigned long lsChangeZeit = 0;    // ms (ISR-sicher mit esp_timer_get_time())
volatile long autoTargetPos = 0;            // FIX: volatile für ISR-Sicherheit
volatile bool autoFahrtAktiv = false;       // FIX: volatile für ISR-Sicherheit

long endpunktHoch = DEFAULT_HOCH;
long endpunktRunter = DEFAULT_RUNTER;
unsigned long motorStoppZeit = 0;

// Taster-Variablen (LOW = gedrückt bei dir)
bool tasterHochAktuell = false;
bool tasterRunterAktuell = false;
bool tasterBoardAktuell = true;

bool tasterHochVorher = false;
bool tasterRunterVorher = false;
bool tasterBoardVorher = true;

bool tasterHochGedrueckt = false;
bool tasterRunterGedrueckt = false;
bool tasterBoardGedrueckt = false;

bool webHochGedrueckt = false;
bool webRunterGedrueckt = false;

int aufStunde = 7, aufMinute = 0;
int zuStunde = 20, zuMinute = 0;
bool aufGesperrt = false;
bool zuGesperrt = false;
int letzteMinute = -1;

unsigned long tasterChangeZeit = 0;
unsigned long tasterBoardChangeZeit = 0;
unsigned long kalibrierStartZeit = 0;
bool eepromDirty = false;

// Halboffen-Feature (persistiert)
bool halbOffenEnabled = false; // persistent setting

// Double-tap detection for HOCH
unsigned long lastHochTapTime = 0;
int hochTapCount = 0;

bool bootReferenzErforderlich = true; // Erzwingt einmalige Abwärtsfahrt nach dem Einschalten

// WiFi credentials stored in preferences
String storedSSID = "";
String storedPASS = "";

// WiFi connection state for non-blocking connection
volatile bool wifiConnecting = false;
unsigned long wifiConnectStart = 0;

// Forward declarations
bool lade_kalibrierung_aus_eeprom();
void verwalte_motor_nachlauf(unsigned long jetzt);
void verwalte_zeitschaltuhr();
void lese_sensoren(unsigned long jetzt);
void handle_boot_state();
void handle_normal_state(unsigned long jetzt);
void handle_kalibrierung_state(unsigned long jetzt);
void speichere_eeprom_wenn_noetig();
void debug_ausgabe();
void starte_kalibrierung();
void fahre_motor(MotorState richtung);
void stoppe_motor();
long getImpulsZaehlerSafe();
void attemptConnectToWifiAsync(boolean showSerial);
void verwalte_wifi_verbindung(unsigned long jetzt);

// ============================================================================
// Helper: atomisch sicheren Wert vom Impulszähler holen
// ============================================================================
long getImpulsZaehlerSafe() {
  long v;
  noInterrupts();
  v = impulsZaehler;
  interrupts();
  return v;
}

// ============================================================================
// FIX: Helper: atomisch sicheren Motorstatus und Zielposition holen
// ============================================================================
void getMotorStatesSafe(long& pos, long& targetUp, long& targetDown, bool& active) {
  noInterrupts();
  pos = impulsZaehler;
  targetUp = autoTargetPos;
  targetDown = autoTargetPos;
  active = autoFahrtAktiv;
  interrupts();
}

// ============================================================================
// Web: Root page
// ============================================================================
void handleRoot() {
  char auf_buf[10]; sprintf(auf_buf, "%02d:%02d", aufStunde, aufMinute);
  char zu_buf[10]; sprintf(zu_buf, "%02d:%02d", zuStunde, zuMinute);

  struct tm timeinfo;
  char zeit_str[10] = "--:--:--";
  if (getLocalTime(&timeinfo)) {
    strftime(zeit_str, sizeof(zeit_str), "%H:%M:%S", &timeinfo);
  }

  long measuredPos = getImpulsZaehlerSafe();
  int prozent = 0;
  if (endpunktHoch > 0) {
    prozent = (measuredPos * 100) / endpunktHoch;
    if (prozent > 100) prozent = 100;
    if (prozent < 0) prozent = 0;
  }

  String animiert = (motorState == MOTOR_UP || motorState == MOTOR_DOWN) ? "true" : "false";

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'><meta charset='utf-8'>";
  if (animiert == "true") html += "<meta http-equiv='refresh' content='2'>";

  html += "<style>body{font-family:Arial;text-align:center;background:#f4f4f4;margin:0;padding:20px;}";
  html += "h2{color:#333;margin-bottom:5px;} .btn{display:block;width:80%;max-width:320px;margin:10px auto;padding:14px;font-size:18px;color:white;border:none;border-radius:10px;cursor:pointer;}";
  html += ".btn-up{background:#4CAF50}.btn-down{background:#2196F3}.btn-stop{background:#f44336}.box{background:#fff;padding:12px;border-radius:8px;box-shadow:0 2px 6px rgba(0,0,0,0.08);max-width:360px;margin:10px auto;}";
  html += ".small{font-size:13px;color:#666;margin-bottom:8px}.toggle-on{background:#4CAF50;padding:6px 10px;border-radius:6px;color:#fff}.toggle-off{background:#aaa;padding:6px 10px;border-radius:6px;color:#fff}input{width:90%;padding:8px;margin:5px 0;}";
  html += "</style>";
  html += "</head><body>";

  html += "<h2>Rollladen-Steuerung</h2>";

  if (bootReferenzErforderlich && systemState == STATE_BOOT) {
    html += "<div style='color:red;font-weight:bold;margin-bottom:12px;'>⚠️ Nach Stromausfall: Bitte zuerst ganz nach UNTEN fahren und stoppen.</div>";
  } else {
    html += "<div class='small'>ESP-Uhrzeit: " + String(zeit_str) + "</div>";
  }

  html += "<div class='box'>";
  html += "<div style='font-weight:bold;color:#333;margin-bottom:8px;'>Position: " + String(prozent) + "% geöffnet</div>";

  // Buttons
  if (motorState == MOTOR_UP) {
    html += "<button class='btn btn-stop' onclick=\"location.href='/hoch'\">🛑 STOPP (HOCH)</button>";
    html += "<button class='btn btn-down' onclick=\"location.href='/runter'\">▼ RUNTER STARTEN</button>";
  } else if (motorState == MOTOR_DOWN) {
    html += "<button class='btn btn-up' onclick=\"location.href='/hoch'\">▲ HOCH STARTEN</button>";
    html += "<button class='btn btn-stop' onclick=\"location.href='/runter'\">🛑 STOPP (RUNTER)</button>";
  } else {
    html += "<button class='btn btn-up' onclick=\"location.href='/hoch'\">▲ HOCH STARTEN</button>";
    html += "<button class='btn btn-down' onclick=\"location.href='/runter'\">▼ RUNTER STARTEN</button>";
  }

  html += "</div>"; // box

  // Timer + Halboffen
  html += "<div class='box'><h3>Zeitschaltuhr</h3>";
  html += "<form action='/save_timer' method='GET'>";
  html += "<div class='small'>Öffnen um:</div> <input type='time' name='auf' value='" + String(auf_buf) + "'><br><br>";
  html += "<div class='small'>Schließen um:</div> <input type='time' name='zu' value='" + String(zu_buf) + "'><br><br>";
  html += "<button type='submit' class='btn' style='background:#ff9800'>Zeiten speichern</button></form>";

  // Halboffen persistent Toggle
  String halbClass = halbOffenEnabled ? "toggle-on" : "toggle-off";
  String halbText = halbOffenEnabled ? "Halboffen: Aktiv" : "Halboffen: Aus";
  html += "<div style='margin-top:10px;text-align:left;'>";
  html += "<form action='/toggle_halb' method='GET'>";
  html += "<button type='submit' class='" + halbClass + "' style='border:none;'>" + halbText + "</button>";
  html += "</form></div>";

  html += "</div>"; // box

  // WiFi Einstellungen
  html += "<div class='box'><h3>WLAN (Heimnetz)</h3>";
  html += "<div class='small'>Optional: Verbinde den ESP mit deinem Heimnetzwerk, damit du nicht das Handy-WLAN wechseln musst.</div>";
  preferences.begin("wifi", true);
  String ssid = preferences.getString("ssid", "");
  preferences.end();
  html += "<form action='/save_wifi' method='GET'>";
  html += "SSID: <input name='ssid' type='text' value='" + ssid + "'><br><br>";
  html += "Passwort: <input name='pass' type='password' value=''><br><br>";
  html += "<button class='btn' style='background:#607d8b' type='submit'>Speichern & Verbinden</button>";
  html += "</form>";

  html += "</div>"; // box

  // Kurzanleitung kompakt und verständlich
  html += "<div class='box'><h3>Kurz-Anleitung</h3>";
  html += "<div class='small'>1) Board-Taste kurz drücken → Start Kalibrierung.<br>";
  html += "2) RUNTER mit Wandtaster fahren, stoppen (setzt Nullpunkt), Board-Taste drücken.<br>";
  html += "3) HOCH ganz fahren lassen, stoppen (misst Weg), Board-Taste drücken → Fertig.<br><br>";
  html += "Wichtig: Motor immer zuerst mit Wandtaster stoppen, bevor Board-Taste bestätigt wird.<br>";
  html += "Doppeltipp auf HOCH-Taster (Wand) = halboffen (nur während Normalbetrieb).<br>";
  html += "Der Web-Button 'Halboffen' bestimmt, ob Automatik-Öffnen nur halb öffnet.</div></div>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ============================================================================
// ISR: Impulszählung (IRAM, sehr kurz halten)
// ============================================================================
void IRAM_ATTR lsInterruptISR() {
  uint32_t jetzt = (uint32_t)(esp_timer_get_time() / 1000ULL); // ms since boot
  if ((uint32_t)(jetzt - lsChangeZeit) > LS_ENTPRELL_MS) {
    lsChangeZeit = jetzt;
    if (motorState == MOTOR_UP) {
      // FIX: Overflow-Schutz bei hochfahren
      if (systemState == STATE_KALIBRIERUNG || impulsZaehler < MAX_IMPULSE) impulsZaehler++;
    } else if (motorState == MOTOR_DOWN) {
      if (systemState == STATE_KALIBRIERUNG || systemState == STATE_BOOT) impulsZaehler--;
      else if (impulsZaehler > endpunktRunter) impulsZaehler--;
    }
  }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  delay(500);
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_RELAIS_HOCH, OUTPUT);
  pinMode(PIN_RELAIS_RUNTER, OUTPUT);
  pinMode(PIN_RUNTER, INPUT_PULLUP);
  pinMode(PIN_HOCH, INPUT_PULLUP);
  pinMode(PIN_BOARD, INPUT_PULLUP);

  digitalWrite(PIN_RELAIS_HOCH, LOW);
  digitalWrite(PIN_RELAIS_RUNTER, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_LS), lsInterruptISR, FALLING);

  rtc.setTime(0, 0, 12, 1, 1, 2026);

  // Load settings
  preferences.begin("rolladen", true);
  aufStunde = preferences.getInt("aufH", 7);
  aufMinute = preferences.getInt("aufM", 0);
  zuStunde = preferences.getInt("zuH", 20);
  zuMinute = preferences.getInt("zuM", 0);
  aufGesperrt = preferences.getBool("aufGesp", false);
  zuGesperrt = preferences.getBool("zuGesp", false);
  halbOffenEnabled = preferences.getBool("halbOn", false);
  preferences.end();

  preferences.begin("wifi", true);
  storedSSID = preferences.getString("ssid", "");
  storedPASS = preferences.getString("pass", "");
  preferences.end();

  // Start AP (always) and try STA if credentials present
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP_STA);
  delay(100);
  WiFi.softAP("Rollladen-Steuerung", "Cvbn3456+*");
  WiFi.setTxPower(WIFI_POWER_7dBm);
  IPAddress local_ip(192,168,4,1);
  IPAddress gateway(192,168,4,1);
  IPAddress subnet(255,255,255,0);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  dnsServer.start(53, "*", local_ip);

  server.on("/", handleRoot);
  server.on("/hoch", [](){ webHochGedrueckt = true; server.sendHeader("Location", "/"); server.send(303); });
  server.on("/runter", [](){ webRunterGedrueckt = true; server.sendHeader("Location", "/"); server.send(303); });
  server.on("/save_timer", [](){
    if (server.hasArg("auf") && server.hasArg("zu")) {
      String auf = server.arg("auf"); String zu = server.arg("zu");
      aufStunde = auf.substring(0,2).toInt(); aufMinute = auf.substring(3,5).toInt();
      zuStunde = zu.substring(0,2).toInt(); zuMinute = zu.substring(3,5).toInt();
      preferences.begin("rolladen", false);
      preferences.putInt("aufH", aufStunde); preferences.putInt("aufM", aufMinute);
      preferences.putInt("zuH", zuStunde); preferences.putInt("zuM", zuMinute);
      preferences.end();
      Serial.println("[EEPROM] Zeitschaltuhr-Daten gesichert.");
    }
    server.sendHeader("Location", "/"); server.send(303);
  });

  server.on("/toggle_halb", [](){
    halbOffenEnabled = !halbOffenEnabled;
    preferences.begin("rolladen", false);
    preferences.putBool("halbOn", halbOffenEnabled);
    preferences.end();
    Serial.print("[HALB] halbOffenEnabled="); Serial.println(halbOffenEnabled);
    server.sendHeader("Location", "/"); server.send(303);
  });

  server.on("/save_wifi", [](){
    if (server.hasArg("ssid")) {
      String ssid = server.arg("ssid");
      String pass = server.hasArg("pass") ? server.arg("pass") : String("");
      preferences.begin("wifi", false);
      preferences.putString("ssid", ssid);
      preferences.putString("pass", pass);
      preferences.end();
      Serial.println("[WIFI] Zugangsdaten gespeichert. Versuche Verbindung...");
      storedSSID = ssid; storedPASS = pass;
      attemptConnectToWifiAsync(true);
    }
    server.sendHeader("Location", "/"); server.send(303);
  });

  server.on("/sync", [](){
    if (server.hasArg("time")) {
      rtc.setTime(server.arg("time").toInt());
      setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
      tzset();
      Serial.println("[Uhrzeit] Synchronisiert mit automatischer Sommer-/Winterzeit.");
    }
    server.send(200, "text/plain", "OK");
  });

  server.onNotFound(handleRoot);
  server.begin();

  unsigned long jetzt = millis();
  lsChangeZeit = (unsigned long)(esp_timer_get_time() / 1000ULL);
  tasterChangeZeit = jetzt; tasterBoardChangeZeit = jetzt;

  // Deine Taster gegen GND -> LOW = gedrückt
  tasterHochAktuell = (digitalRead(PIN_HOCH) == LOW);
  tasterRunterAktuell = (digitalRead(PIN_RUNTER) == LOW);
  tasterBoardAktuell = (digitalRead(PIN_BOARD) == LOW);
  tasterHochVorher = tasterHochAktuell; tasterRunterVorher = tasterRunterAktuell; tasterBoardVorher = tasterBoardAktuell;

  // load calibration or set boot
  if (!lade_kalibrierung_aus_eeprom()) {
    preferences.begin("kalib", false);
    preferences.putLong("hoch", DEFAULT_HOCH);
    preferences.putLong("runter", DEFAULT_RUNTER);
    preferences.end();
    endpunktHoch = DEFAULT_HOCH; endpunktRunter = DEFAULT_RUNTER;
    systemState = STATE_BOOT;
    bootReferenzErforderlich = false; // normales Anlernen
  } else {
    systemState = STATE_BOOT;
    bootReferenzErforderlich = true;
    noInterrupts(); impulsZaehler = 9999; interrupts();
    Serial.println("[System] Kalibrierung geladen. Warte auf Referenzfahrt nach UNTEN...");
  }

  // Try to connect to wifi stored credentials asynchronously
  if (storedSSID.length() > 0) attemptConnectToWifiAsync(true);
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
  unsigned long jetzt = millis();
  dnsServer.processNextRequest();
  server.handleClient();

  verwalte_motor_nachlauf(jetzt);
  verwalte_wifi_verbindung(jetzt);   // FIX: Non-blocking WiFi-Management
  if (systemState == STATE_NORMAL) verwalte_zeitschaltuhr();
  lese_sensoren(jetzt);

  switch (systemState) {
    case STATE_BOOT: handle_boot_state(); break;
    case STATE_NORMAL: handle_normal_state(jetzt); break;
    case STATE_KALIBRIERUNG: handle_kalibrierung_state(jetzt); break;
  }

  speichere_eeprom_wenn_noetig();
  if (jetzt % 2000 < 10) debug_ausgabe();
}

// ============================================================================
// Zeitschaltuhr
// ============================================================================
void verwalte_zeitschaltuhr() {
  if (systemState != STATE_NORMAL) return;
  static bool webToggleHandled = false;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  int h = timeinfo.tm_hour; int m = timeinfo.tm_min;
  if (m == letzteMinute) return;

  // Automatik: bei Halbmodus Ziel anpassen
  if (!aufGesperrt && h == aufStunde && m == aufMinute) {
    long pos = getImpulsZaehlerSafe();
    long target;
    if (halbOffenEnabled) {
      target = endpunktRunter + (endpunktHoch - endpunktRunter) / 2;
    } else target = endpunktHoch;
    // FIX: Atomare Zuweisung mit noInterrupts()
    noInterrupts();
    autoTargetPos = target; 
    autoFahrtAktiv = true;
    interrupts();
    fahre_motor(MOTOR_UP);
    letzteMinute = m;
  }
  if (!zuGesperrt && h == zuStunde && m == zuMinute) {
    noInterrupts();
    autoTargetPos = endpunktRunter; 
    autoFahrtAktiv = true;
    interrupts();
    fahre_motor(MOTOR_DOWN);
    letzteMinute = m;
  }

  if (m != aufMinute && m != zuMinute) letzteMinute = -1;
}

// ============================================================================
// Normalbetrieb
// ============================================================================
void handle_normal_state(unsigned long jetzt) {
  static bool boardTasteLetzterZustandNormal = HIGH;

  // Board-Taste: starte Kalibrierung
  if (tasterBoardAktuell == LOW && boardTasteLetzterZustandNormal == HIGH) {
    boardTasteLetzterZustandNormal = LOW;
    Serial.println("[SYSTEM] Board-Taste erkannt: Starte Kalibrierung...");
    starte_kalibrierung();
    return;
  } else if (tasterBoardAktuell == HIGH) boardTasteLetzterZustandNormal = HIGH;

  // FIX: Sicherheits-Stopp bei Erreichen der Endpunkte oder autoTargetPos (mit Atomic Read)
  long pos, targetUp, targetDown;
  bool autoActive;
  getMotorStatesSafe(pos, targetUp, targetDown, autoActive);
  
  if (motorState == MOTOR_UP && pos >= targetUp) stoppe_motor();
  if (motorState == MOTOR_DOWN && pos <= targetDown) stoppe_motor();

  // Taster/Web HOCH
  if (tasterHochGedrueckt || webHochGedrueckt) {
    // FIX: Variablenwert vor dem Reset speichern für Double-Tap-Logik
    bool wasPressedHoch = tasterHochGedrueckt;
    bool wasPressedWeb = webHochGedrueckt;
    tasterHochGedrueckt = false; 
    webHochGedrueckt = false;
    
    if (wasPressedHoch || wasPressedWeb) {  // Double-tap detection (nur im Normalbetrieb)
      unsigned long now = millis();
      if (now - lastHochTapTime <= DOUBLE_TAP_MS) {
        hochTapCount++;
      } else {
        hochTapCount = 1;
      }
      lastHochTapTime = now;

      if (hochTapCount == 2 && motorState == MOTOR_IDLE) {
        // Doppeltipp -> halboffen (nur bei Normalbetrieb)
        long half = endpunktRunter + (endpunktHoch - endpunktRunter) / 2;
        noInterrupts();
        autoTargetPos = half; 
        autoFahrtAktiv = true;
        interrupts();
        Serial.println("[DOPPELT] HOCH-Doppeltipp: Fahre auf Halboffen");
        fahre_motor(MOTOR_UP);
        hochTapCount = 0;
        return;
      }

      // Normales Verhalten: wenn Motor läuft -> stoppe, sonst fahre hoch (voll)
      if (motorState != MOTOR_IDLE) {
        stoppe_motor();
        noInterrupts();
        autoFahrtAktiv = false;
        interrupts();
      } else {
        noInterrupts();
        autoFahrtAktiv = false;
        interrupts();
        if (pos < endpunktHoch) fahre_motor(MOTOR_UP);
      }
    }
  }

  // Taster/Web RUNTER
  else if (tasterRunterGedrueckt || webRunterGedrueckt) {
    tasterRunterGedrueckt = false; 
    webRunterGedrueckt = false;
    if (motorState != MOTOR_IDLE) {
      stoppe_motor(); 
      noInterrupts();
      autoFahrtAktiv = false;
      interrupts();
    } else {
      noInterrupts();
      autoFahrtAktiv = false;
      interrupts();
      if (pos > endpunktRunter) fahre_motor(MOTOR_DOWN);
    }
  }
}

// ============================================================================
// Kalibrierung
// ============================================================================
void starte_kalibrierung() {
  systemState = STATE_KALIBRIERUNG; kalibrierPhase = KALIBRIERUNG_REFERENZ; kalibrierStartZeit = millis();
  noInterrupts(); impulsZaehler = -999; interrupts();
  stoppe_motor();
  Serial.println("[KALIBRIERUNG] Starte Referenzfahrt: Fahre nach UNTEN. Wenn gestoppt: Board-Taste drücken.");
}

void handle_kalibrierung_state(unsigned long jetzt) {
  if (jetzt - kalibrierStartZeit > KALIBRIERUNG_TIMEOUT_MS) {
    Serial.println("[KALIBRIERUNG] Timeout! Abbruch.");
    stoppe_motor(); systemState = STATE_NORMAL; kalibrierPhase = KALIBRIERUNG_IDLE; return;
  }
  webHochGedrueckt = false; webRunterGedrueckt = false;

  static bool boardTasteMussErstLosgelassenWerden = true;
  static bool boardTasteLetzterZustandKali = HIGH;
  bool boardTasteFlankeKali = false;

  if (tasterBoardAktuell == LOW) {
    if (boardTasteLetzterZustandKali == HIGH) {
      boardTasteLetzterZustandKali = LOW;
      if (!boardTasteMussErstLosgelassenWerden) boardTasteFlankeKali = true;
    }
  } else {
    boardTasteLetzterZustandKali = HIGH; boardTasteMussErstLosgelassenWerden = false;
  }

  if (kalibrierPhase == KALIBRIERUNG_REFERENZ) {
    if (tasterRunterGedrueckt) { tasterRunterGedrueckt = false; if (motorState == MOTOR_DOWN) stoppe_motor(); else fahre_motor(MOTOR_DOWN); }
    if (tasterHochGedrueckt) { tasterHochGedrueckt = false; if (motorState == MOTOR_UP) stoppe_motor(); else fahre_motor(MOTOR_UP); }

    if (boardTasteFlankeKali && motorState == MOTOR_IDLE) {
      noInterrupts(); impulsZaehler = 0; interrupts();
      endpunktRunter = 0; kalibrierPhase = KALIBRIERUNG_OBEN; boardTasteMussErstLosgelassenWerden = true;
      Serial.println("[KALIBRIERUNG] Nullpunkt unten gesetzt. Jetzt nach OBEN fahren und stoppen.");
      return;
    }
  } else if (kalibrierPhase == KALIBRIERUNG_OBEN) {
    if (tasterHochGedrueckt) { tasterHochGedrueckt = false; if (motorState == MOTOR_UP) stoppe_motor(); else fahre_motor(MOTOR_UP); }
    if (tasterRunterGedrueckt) { tasterRunterGedrueckt = false; if (motorState == MOTOR_DOWN) stoppe_motor(); else fahre_motor(MOTOR_DOWN); }

    if (boardTasteFlankeKali && motorState == MOTOR_IDLE) {
      long measured;
      noInterrupts(); measured = impulsZaehler; interrupts();
      if (measured >= MIN_RANGE && measured <= MAX_IMPULSE) {
        endpunktHoch = measured; eepromDirty = true; systemState = STATE_NORMAL; kalibrierPhase = KALIBRIERUNG_IDLE;
        Serial.print("[KALIBRIERUNG] Erfolg. EndpunktHoch="); Serial.println(measured);
        // Relais-Doppelklick zur Bestätigung
        delay(250); digitalWrite(PIN_RELAIS_HOCH, HIGH); delay(80); digitalWrite(PIN_RELAIS_HOCH, LOW); delay(120); digitalWrite(PIN_RELAIS_HOCH, HIGH); delay(80); digitalWrite(PIN_RELAIS_HOCH, LOW);
      } else {
        Serial.print("[KALIBRIERUNG] Fehler: Ungültige Impulszahl: "); Serial.println(measured);
        boardTasteMussErstLosgelassenWerden = true;
      }
    }
  }
}

// ============================================================================
// Sensorik & Taster-Entprellung
// ============================================================================
void lese_sensoren(unsigned long jetzt) {
  static unsigned long tasterHochChangeZeit = 0;
  static unsigned long tasterRunterChangeZeit = 0;

  bool hoch_neu = (digitalRead(PIN_HOCH) == LOW);
  if (hoch_neu != tasterHochAktuell) {
    if (jetzt - tasterHochChangeZeit > TASTER_ENTPRELL_MS) {
      tasterHochAktuell = hoch_neu; tasterHochChangeZeit = jetzt;
      if (tasterHochAktuell && !tasterHochVorher) tasterHochGedrueckt = true;
    }
  } else tasterHochChangeZeit = jetzt;

  bool runter_neu = (digitalRead(PIN_RUNTER) == LOW);
  if (runter_neu != tasterRunterAktuell) {
    if (jetzt - tasterRunterChangeZeit > TASTER_ENTPRELL_MS) {
      tasterRunterAktuell = runter_neu; tasterRunterChangeZeit = jetzt;
      if (tasterRunterAktuell && !tasterRunterVorher) tasterRunterGedrueckt = true;
    }
  } else tasterRunterChangeZeit = jetzt;

  bool board_neu = (digitalRead(PIN_BOARD) == LOW);
  if (board_neu != tasterBoardAktuell && (jetzt - tasterBoardChangeZeit) > BOARD_ENTPRELL_MS) {
    tasterBoardAktuell = board_neu; tasterBoardChangeZeit = jetzt;
  }

  // update previous states for edge detection
  tasterHochVorher = tasterHochAktuell; tasterRunterVorher = tasterRunterAktuell; tasterBoardVorher = tasterBoardAktuell;
}

// ============================================================================
// Boot-State (Referenz nach Stromausfall)
// ============================================================================
void handle_boot_state() {
  static bool fahrtAktiviert = false;
  static bool boardTasteLetzterZustandBoot = HIGH;
  static bool erststartSperre = true;

  if (erststartSperre) { erststartSperre = false; tasterHochGedrueckt = false; tasterRunterGedrueckt = false; webHochGedrueckt = false; webRunterGedrueckt = false; stoppe_motor(); Serial.println("[System] Boot-State aktiviert."); }

  if (tasterBoardAktuell == LOW && boardTasteLetzterZustandBoot == HIGH) { boardTasteLetzterZustandBoot = LOW; Serial.println("[ERST] Board-Taste: Starte Kalibrierung"); bootReferenzErforderlich = false; starte_kalibrierung(); }
  else if (tasterBoardAktuell == HIGH) boardTasteLetzterZustandBoot = HIGH;

  webHochGedrueckt = false; webRunterGedrueckt = false;

  if (bootReferenzErforderlich) {
    if (tasterHochGedrueckt) { tasterHochGedrueckt = false; Serial.println("[Sicherheit] Fahrt gesperrt: Nach Stromausfall zuerst nach UNTEN fahren."); }
    if (tasterRunterGedrueckt) {
      tasterRunterGedrueckt = false;
      if (motorState == MOTOR_DOWN) stoppe_motor(); else { fahre_motor(MOTOR_DOWN); fahrtAktiviert = true; }
    }
    if (fahrtAktiviert && motorState == MOTOR_IDLE) {
      noInterrupts(); impulsZaehler = 0; interrupts(); bootReferenzErforderlich = false; systemState = STATE_NORMAL; fahrtAktiviert = false; Serial.println("[System] Referenzfahrt beendet. Normalbetrieb gestartet.");
    }
  } else systemState = STATE_NORMAL;
}

// ============================================================================
// Motorsteuerung
// ============================================================================
void fahre_motor(MotorState richtung) {
  if (motorState == MOTOR_STOPPING) return;
  digitalWrite(PIN_RELAIS_HOCH, LOW); digitalWrite(PIN_RELAIS_RUNTER, LOW);
  delay(RELAIS_OFF_DELAY_MS);
  if (richtung == MOTOR_UP) { digitalWrite(PIN_RELAIS_HOCH, HIGH); motorState = MOTOR_UP; }
  else if (richtung == MOTOR_DOWN) { digitalWrite(PIN_RELAIS_RUNTER, HIGH); motorState = MOTOR_DOWN; }
}

void stoppe_motor() {
  if (motorState == MOTOR_IDLE || motorState == MOTOR_STOPPING) return;
  digitalWrite(PIN_RELAIS_HOCH, LOW); digitalWrite(PIN_RELAIS_RUNTER, LOW);
  motorState = MOTOR_STOPPING; motorStoppZeit = millis();
}

void verwalte_motor_nachlauf(unsigned long jetzt) {
  if (motorState == MOTOR_STOPPING && (jetzt - motorStoppZeit >= MOTOR_NACHLAUF_MS)) motorState = MOTOR_IDLE;
}

// ============================================================================
// EEPROM: Laden/Speichern der Kalibrierung
// ============================================================================
bool lade_kalibrierung_aus_eeprom() {
  preferences.begin("kalib", true);
  long hoch = preferences.getLong("hoch", EEPROM_NOT_FOUND);
  long runter = preferences.getLong("runter", EEPROM_NOT_FOUND);
  preferences.end();
  if (hoch == EEPROM_NOT_FOUND || runter == EEPROM_NOT_FOUND || (hoch - runter) < MIN_RANGE) return false;
  endpunktHoch = hoch; endpunktRunter = runter; return true;
}

void speichere_eeprom_wenn_noetig() {
  if (!eepromDirty) return;
  if (endpunktHoch < MIN_RANGE || endpunktHoch > MAX_IMPULSE) { Serial.println("[EEPROM] Ungültiger EndpunktHoch, nicht gespeichert."); eepromDirty = false; return; }
  preferences.begin("kalib", false); preferences.putLong("hoch", endpunktHoch); preferences.putLong("runter", endpunktRunter); preferences.end();
  eepromDirty = false; Serial.println("[EEPROM] Kalibrierdaten gesichert.");
}

// ============================================================================
// Debug
// ============================================================================
void debug_ausgabe() {
  long pos = getImpulsZaehlerSafe();
  Serial.print("[DEBUG] Zeit: "); Serial.print(rtc.getTime("%H:%M:%S"));
  Serial.print(" | Pos: "); Serial.print(pos);
  Serial.print(" / "); Serial.println(endpunktHoch);
}

// ============================================================================
// WiFi: Asynchrone (Non-Blocking) Verbindung
// ============================================================================
void attemptConnectToWifiAsync(boolean showSerial) {
  if (storedSSID.length() == 0) return;
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(storedSSID.c_str(), storedPASS.c_str());
  wifiConnecting = true;
  wifiConnectStart = millis();
  if (showSerial) Serial.print("[WIFI] Verbindung zu "); Serial.print(storedSSID);
}

// FIX: Non-blocking WiFi management in main loop
void verwalte_wifi_verbindung(unsigned long jetzt) {
  if (!wifiConnecting) return;
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" verbunden, IP= "); Serial.println(WiFi.localIP());
    wifiConnecting = false;
  } else if (jetzt - wifiConnectStart > 8000) {
    Serial.println(" fehlgeschlagen (AP weiter verfügbar)");
    wifiConnecting = false;
  }
}
