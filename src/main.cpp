// main.cpp - Serielles Test-Interface:
//   Arduino Opta RS485  --(Modbus RTU / RS485)-->  MKS SERVO42D
//
// Bedienung ueber einen seriellen Monitor (USB-C), dann "help" eingeben.
//
// Aufbau dieser Datei:
//   1. Konfiguration           4. Befehlstabelle      7. Zyklische Aufgaben
//   2. Prototypen              5. Kommandozeile       8. Ausgabe-Helfer
//   3. setup() / loop()        6. Befehle             9. Argument-Parser

#include <Arduino.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <stdarg.h>

#include "MksServo.h"

// =============================================================================
// 1. Konfiguration
// =============================================================================

// Muss mit dem Servo-Menue uebereinstimmen (UartBaud / UartAddr / Mb_RTU).
constexpr uint32_t MODBUS_BAUD = 38400;         // MKS-Default: 38400
constexpr uint16_t MODBUS_FORMAT = SERIAL_8N1;  // 8 Datenbits, keine Paritaet, 1 Stoppbit
constexpr uint8_t SERVO_SLAVE_ID = 1;           // MKS-Default: 1
constexpr unsigned long MODBUS_TIMEOUT_MS = 200;

// RS485-Umschaltzeiten (Sender ein/aus) in Mikrosekunden.
//  - PRE : 3,5 Zeichen Ruhe vor einem Telegramm (wie im offiziellen Opta-Beispiel).
//  - POST: 1,5 Zeichen. Der Opta haelt den Sender nach dem letzten Byte noch so
//          lange aktiv und schaltet erst danach auf Empfang. Ist der Wert zu gross,
//          wird der Anfang einer schnellen Servo-Antwort verpasst (-> Timeout).
constexpr float BIT_US = 1e6f / MODBUS_BAUD;
constexpr int RS485_PRE_DELAY_US = static_cast<int>(BIT_US * 9.6f * 3.5f);
constexpr int RS485_POST_DELAY_US = static_cast<int>(BIT_US * 10.0f * 1.5f);

// Referenzfahrt-Ueberwachung
constexpr unsigned long HOMING_TIMEOUT_MS = 120000;
constexpr unsigned long HOMING_POLL_MS = 100;
constexpr uint8_t HOMING_OK_POLLS = 3;     // 0,3 s Stillstand + Status "ok"  -> erfolgreich
constexpr uint8_t HOMING_FAIL_POLLS = 20;  // 2 s Stillstand ohne Status "ok" -> fehlgeschlagen

// Startwerte fuer Bewegungen (zur Laufzeit mit "set" aenderbar)
uint16_t gRpm = 300;  // 0..3000 U/min (SR_CLOSE max. 1500, SR_OPEN max. 400)
uint8_t gAcc = 200;   // 0..255, groesser = steilere Rampe, 0 = ohne Rampe

// ---- Globale Objekte und Zustand --------------------------------------------

mks::MksServo servo(ModbusRTUClient, RS485, SERVO_SLAVE_ID);

unsigned long gWatchIntervalMs = 0;  // 0 = zyklische Ausgabe aus
unsigned long gLastWatchMs = 0;

// Zustand einer laufenden Referenzfahrt (nicht blockierend, siehe pollHoming)
struct HomingMonitor {
  bool active = false;
  bool seenMoving = false;
  uint8_t errors = 0;        // aufeinanderfolgende Lesefehler
  uint8_t stoppedPolls = 0;  // aufeinanderfolgende Abfragen mit Motor "steht"
  uint8_t lastState = 0xFF;  // fuer das Aenderungsprotokoll
  uint16_t lastRaw = 0xFFFF;
  unsigned long startMs = 0;
  unsigned long lastPollMs = 0;
} gHoming;

// =============================================================================
// 2. Prototypen
// =============================================================================

// Kommandozeile und zyklische Aufgaben
void pollSerial();
void executeLine(char* line);
void pollWatch();
void pollHoming();

// Befehle (Signatur wie main: Anzahl und Liste der Woerter einer Eingabezeile)
void cmdHelp(int argc, char** argv);
void cmdStatus(int argc, char** argv);
void cmdWatch(int argc, char** argv);
void cmdEnable(int argc, char** argv);
void cmdDisable(int argc, char** argv);
void cmdSpeed(int argc, char** argv);
void cmdRel(int argc, char** argv);
void cmdAbs(int argc, char** argv);
void cmdRot(int argc, char** argv);
void cmdGoto(int argc, char** argv);
void cmdStop(int argc, char** argv);
void cmdEstop(int argc, char** argv);
void cmdZero(int argc, char** argv);
void cmdHomeConfig(int argc, char** argv);
void cmdHome(int argc, char** argv);
void cmdUnlock(int argc, char** argv);
void cmdMode(int argc, char** argv);
void cmdCurrent(int argc, char** argv);
void cmdSet(int argc, char** argv);
void cmdRawRead(int argc, char** argv);
void cmdRawWrite(int argc, char** argv);

// Ausgabe
void out(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void report(bool ok);
void printError();
void printWatchLine();
void printInt64(int64_t value);
const char* homeStatusText(uint8_t status);
double axisToDegrees(int64_t axis);
double angleErrorToDegrees(int32_t error);

// Argument-Parser
bool toLong(const char* s, long& value);
bool toDouble(const char* s, double& value);
bool argLong(int argc, char** argv, int index, long minVal, long maxVal, long& value, const char* name);
bool optRpmAcc(int argc, char** argv, int first, uint16_t& rpm, uint8_t& acc);
bool degreesToAxis(const char* s, int32_t& axis);

// =============================================================================
// 3. setup() / loop()
// =============================================================================

void setup() {
  Serial.begin(115200);
  const unsigned long start = millis();
  while (!Serial && millis() - start < 3000) {
    // bis zu 3 s auf den seriellen Monitor warten
  }

  RS485.setDelays(RS485_PRE_DELAY_US, RS485_POST_DELAY_US);  // muss vor begin() stehen
  if (!ModbusRTUClient.begin(MODBUS_BAUD, MODBUS_FORMAT)) {
    Serial.println("FEHLER: Modbus RTU Client konnte nicht gestartet werden");
    while (true) {
      delay(1000);
    }
  }
  ModbusRTUClient.setTimeout(MODBUS_TIMEOUT_MS);

  Serial.println();
  Serial.println("=== Opta RS485 -> MKS SERVO42D (Modbus RTU) ===");
  out("Bus: %lu Baud, 8N1, Servo-ID %u\r\n", static_cast<unsigned long>(MODBUS_BAUD), servo.slaveId());

  uint8_t state;
  if (servo.readMotorStatus(state)) {
    out("Servo antwortet: %s\r\n", mks::motorStatusText(state));
  } else {
    printError();
  }
  Serial.println("'help' fuer alle Befehle. Erster Test: en -> spd 60 -> stop");
  Serial.print("> ");
}

void loop() {
  pollSerial();
  pollWatch();
  pollHoming();
}

// =============================================================================
// 4. Befehlstabelle
// =============================================================================

struct Command {
  const char* name;
  const char* args;
  const char* help;
  void (*handler)(int argc, char** argv);
};

const Command COMMANDS[] = {
    {"help", "", "Diese Hilfe", cmdHelp},
    {"status", "", "Status lesen (Zustand, Drehzahl, Position ...)", cmdStatus},
    {"watch", "[ms]", "Drehzahl/Position/Winkelfehler zyklisch, 'watch 0' = aus", cmdWatch},
    {"en", "", "Motor aktivieren (Haltemoment)", cmdEnable},
    {"dis", "", "Motor deaktivieren - nur im Stillstand", cmdDisable},
    {"spd", "<+-rpm> [acc]", "Drehzahlmodus, Vorzeichen = Richtung, 0 = Stopp", cmdSpeed},
    {"rel", "<+-pulse> [rpm] [acc]", "Relativ um Pulse (16 Mikroschritte: 3200 = 1 U)", cmdRel},
    {"abs", "<pulse> [rpm] [acc]", "Absolut auf Pulsposition", cmdAbs},
    {"rot", "<+-grad> [rpm] [acc]", "Relativ um Winkel drehen (Encoder)", cmdRot},
    {"goto", "<grad> [rpm] [acc]", "Absolut auf Winkel fahren (Encoder)", cmdGoto},
    {"stop", "[acc]", "Letzte Bewegung stoppen (acc 0 = sofort)", cmdStop},
    {"estop", "", "Not-Halt (sofort)", cmdEstop},
    {"zero", "", "Aktuelle Position = 0 setzen", cmdZero},
    {"hcfg", "<dir> <rpm> <mA> [grad]", "Referenzfahrt gegen Anschlag einstellen", cmdHomeConfig},
    {"home", "", "Referenzfahrt starten (vorher hcfg)", cmdHome},
    {"unlock", "", "Blockierschutz zuruecksetzen", cmdUnlock},
    {"mode", "<0..5>", "Arbeitsmodus setzen (5 = SR_vFOC)", cmdMode},
    {"cur", "<mA>", "Arbeitsstrom setzen (max. 3000) - nur im Stillstand", cmdCurrent},
    {"set", "[rpm|acc|id <wert>]", "Standardwerte anzeigen/aendern", cmdSet},
    {"rd", "<reg> [anzahl]", "Input-Register roh lesen (FC 0x04)", cmdRawRead},
    {"wr", "<reg> <wert>", "Holding-Register roh schreiben (FC 0x06)", cmdRawWrite},
};

// =============================================================================
// 5. Kommandozeile
// =============================================================================

// Sammelt Zeichen vom USB-Seriell-Port und fuehrt jede vollstaendige Zeile aus.
void pollSerial() {
  static char line[96];
  static size_t len = 0;

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (len > 0) {
        line[len] = '\0';
        executeLine(line);
        len = 0;
        Serial.print("> ");
      }
    } else if ((c == '\b' || c == 0x7F) && len > 0) {
      --len;
    } else if (len < sizeof(line) - 1 && isprint(static_cast<unsigned char>(c))) {
      line[len++] = c;
    }
  }
}

// Zerlegt die Zeile in Woerter und ruft den passenden Befehl auf.
void executeLine(char* line) {
  constexpr int MAX_ARGS = 6;
  char* argv[MAX_ARGS];
  int argc = 0;
  for (char* tok = strtok(line, " \t"); tok != nullptr && argc < MAX_ARGS; tok = strtok(nullptr, " \t")) {
    argv[argc++] = tok;
  }
  if (argc == 0) return;

  for (char* p = argv[0]; *p; ++p) *p = static_cast<char>(tolower(*p));
  if (strcmp(argv[0], "?") == 0) argv[0] = const_cast<char*>("help");

  for (const Command& c : COMMANDS) {
    if (strcmp(argv[0], c.name) == 0) {
      c.handler(argc, argv);
      return;
    }
  }
  out("Unbekannter Befehl '%s' - 'help' fuer Hilfe\r\n", argv[0]);
}

// =============================================================================
// 6. Befehle
// =============================================================================

void cmdHelp(int, char**) {
  Serial.println("Befehle (Zahlen dezimal oder 0x.. hex):");
  for (const Command& c : COMMANDS) {
    char left[32];
    snprintf(left, sizeof(left), "%s %s", c.name, c.args);
    out("  %-28s %s\r\n", left, c.help);
  }
  out("Aktuell: rpm=%u acc=%u id=%u\r\n", gRpm, gAcc, servo.slaveId());
}

void cmdStatus(int, char**) {
  uint8_t state;
  if (!servo.readMotorStatus(state)) {
    printError();
    return;
  }
  out("Servo (ID %u)\r\n", servo.slaveId());

  uint32_t version;
  if (servo.readVersion(version)) {
    out("  Firmware   : V%u.%u.%u (0x%08lX)\r\n", static_cast<unsigned>((version >> 16) & 0xFF),
        static_cast<unsigned>((version >> 8) & 0xFF), static_cast<unsigned>(version & 0xFF),
        static_cast<unsigned long>(version));
  }
  out("  Zustand    : %s (%u)\r\n", mks::motorStatusText(state), state);

  bool flag;
  if (servo.readEnabled(flag)) out("  Aktiviert  : %s\r\n", flag ? "ja" : "nein");
  if (servo.readProtected(flag)) out("  Blockiert  : %s\r\n", flag ? "JA (-> unlock)" : "nein");

  uint16_t homeRaw;
  if (servo.readInputRegisters(mks::IR_HOME_STATUS, &homeRaw, 1)) {
    out("  Referenz   : %s (Reg 0x3B = 0x%04X)\r\n", homeStatusText(mks::homingStatusFromRaw(homeRaw)),
        homeRaw);
  }

  int16_t rpm;
  if (servo.readSpeed(rpm)) out("  Drehzahl   : %d U/min\r\n", rpm);

  int64_t axis;
  if (servo.readEncoderAddition(axis)) {
    Serial.print("  Position   : ");
    printInt64(axis);
    Serial.print(" (");
    Serial.print(axisToDegrees(axis), 2);
    Serial.println(" Grad)");
  }

  int32_t value;
  if (servo.readPulses(value)) out("  Pulse      : %ld\r\n", static_cast<long>(value));
  if (servo.readAngleError(value)) {
    Serial.print("  Winkelfehl.: ");
    Serial.print(static_cast<long>(value));
    Serial.print(" (");
    Serial.print(angleErrorToDegrees(value), 2);
    Serial.println(" Grad)");
  }
}

void cmdWatch(int argc, char** argv) {
  long ms = 500;
  if (argc > 1 && !argLong(argc, argv, 1, 0, 60000, ms, "ms")) return;
  if (ms != 0 && ms < 50) ms = 50;
  gWatchIntervalMs = static_cast<unsigned long>(ms);
  if (gWatchIntervalMs == 0) {
    Serial.println("Watch aus");
  } else {
    out("Watch alle %lu ms (aus mit: watch 0)\r\n", gWatchIntervalMs);
  }
}

void cmdEnable(int, char**) { report(servo.enable(true)); }
void cmdDisable(int, char**) { report(servo.enable(false)); }

void cmdSpeed(int argc, char** argv) {
  long rpm;
  if (!argLong(argc, argv, 1, -static_cast<long>(mks::MAX_RPM), mks::MAX_RPM, rpm, "+-rpm")) return;
  uint8_t acc = gAcc;
  if (argc > 2) {
    long v;
    if (!argLong(argc, argv, 2, 0, 255, v, "acc")) return;
    acc = static_cast<uint8_t>(v);
  }
  report(servo.runSpeed(static_cast<int16_t>(rpm), acc));
}

void cmdRel(int argc, char** argv) {
  long pulses;
  uint16_t rpm;
  uint8_t acc;
  if (!argLong(argc, argv, 1, -2147483647L, 2147483647L, pulses, "+-pulse")) return;
  if (!optRpmAcc(argc, argv, 2, rpm, acc)) return;
  report(servo.moveRelativePulses(pulses, rpm, acc));
}

void cmdAbs(int argc, char** argv) {
  long pulses;
  uint16_t rpm;
  uint8_t acc;
  if (!argLong(argc, argv, 1, -2147483647L, 2147483647L, pulses, "pulse")) return;
  if (!optRpmAcc(argc, argv, 2, rpm, acc)) return;
  report(servo.moveAbsolutePulses(pulses, rpm, acc));
}

void cmdRot(int argc, char** argv) {
  int32_t axis;
  uint16_t rpm;
  uint8_t acc;
  if (argc < 2) {
    Serial.println("Parameter <+-grad> fehlt");
    return;
  }
  if (!degreesToAxis(argv[1], axis) || !optRpmAcc(argc, argv, 2, rpm, acc)) return;
  report(servo.moveRelativeAxis(axis, rpm, acc));
}

void cmdGoto(int argc, char** argv) {
  int32_t axis;
  uint16_t rpm;
  uint8_t acc;
  if (argc < 2) {
    Serial.println("Parameter <grad> fehlt");
    return;
  }
  if (!degreesToAxis(argv[1], axis) || !optRpmAcc(argc, argv, 2, rpm, acc)) return;
  report(servo.moveAbsoluteAxis(axis, rpm, acc));
}

void cmdStop(int argc, char** argv) {
  long acc = gAcc;
  if (argc > 1 && !argLong(argc, argv, 1, 0, 255, acc, "acc")) return;
  report(servo.stop(static_cast<uint8_t>(acc)));
}

void cmdEstop(int, char**) { report(servo.emergencyStop()); }
void cmdZero(int, char**) { report(servo.setZero()); }

// hcfg <dir> <rpm> <mA> [rueckzug_grad]: Referenzfahrt gegen mechanischen Anschlag
void cmdHomeConfig(int argc, char** argv) {
  long dir, rpm, mA;
  if (argc < 4) {
    Serial.println("Verwendung: hcfg <dir 0=CW|1=CCW> <rpm> <mA> [rueckzug_grad]");
    Serial.println("  Beispiel : hcfg 0 60 500 10   (langsam, wenig Kraft, 10 Grad zurueck)");
    return;
  }
  if (!argLong(argc, argv, 1, 0, 1, dir, "dir")) return;
  if (!argLong(argc, argv, 2, 1, mks::MAX_RPM, rpm, "rpm")) return;
  if (!argLong(argc, argv, 3, 0, 3000, mA, "mA")) return;
  double offsetDeg = 180.0;  // Firmware-Default
  if (argc > 4 && (!toDouble(argv[4], offsetDeg) || offsetDeg < 0 || offsetDeg > 36000)) {
    Serial.println("Ungueltiger Rueckzug (0..36000 Grad)");
    return;
  }
  const uint32_t offset = static_cast<uint32_t>(lround(offsetDeg * mks::AXIS_PER_REV / 360.0));

  if (!servo.setHomeParams(static_cast<uint8_t>(dir), static_cast<uint16_t>(rpm)) ||
      !servo.setHomeMode(mks::HomeMode::Mechanical, offset, static_cast<uint16_t>(mA))) {
    printError();
    return;
  }
  out("OK: Anschlag-Referenzfahrt %s, %ld U/min, %ld mA, Rueckzug ", dir ? "CCW" : "CW", rpm, mA);
  Serial.print(offsetDeg, 1);
  Serial.println(" Grad -> starten mit 'home'");
}

void cmdHome(int, char**) {
  if (!servo.goHome()) {
    printError();
    return;
  }
  Serial.println("Referenzfahrt gestartet (Abbruch: estop)");
  gHoming = HomingMonitor{};
  gHoming.active = true;
  gHoming.startMs = millis();
}

void cmdUnlock(int, char**) { report(servo.releaseProtection()); }

void cmdMode(int argc, char** argv) {
  long mode;
  if (!argLong(argc, argv, 1, 0, 5, mode, "0..5")) {
    Serial.println("  0 CR_OPEN  1 CR_CLOSE  2 CR_vFOC  (Puls/Dir)");
    Serial.println("  3 SR_OPEN  4 SR_CLOSE  5 SR_vFOC  (seriell/Modbus)");
    return;
  }
  out("Setze Modus %s ... ", mks::workModeText(static_cast<uint8_t>(mode)));
  report(servo.setWorkMode(static_cast<mks::WorkMode>(mode)));
}

void cmdCurrent(int argc, char** argv) {
  long mA;
  if (!argLong(argc, argv, 1, 0, 3000, mA, "mA")) return;
  report(servo.setWorkCurrent(static_cast<uint16_t>(mA)));
}

void cmdSet(int argc, char** argv) {
  if (argc >= 3) {
    long v;
    if (strcmp(argv[1], "rpm") == 0) {
      if (!argLong(argc, argv, 2, 0, mks::MAX_RPM, v, "rpm")) return;
      gRpm = static_cast<uint16_t>(v);
    } else if (strcmp(argv[1], "acc") == 0) {
      if (!argLong(argc, argv, 2, 0, 255, v, "acc")) return;
      gAcc = static_cast<uint8_t>(v);
    } else if (strcmp(argv[1], "id") == 0) {
      if (!argLong(argc, argv, 2, 1, 247, v, "id")) return;
      servo.setSlaveId(static_cast<uint8_t>(v));
    } else {
      Serial.println("Unbekannt. Verwendung: set rpm|acc|id <wert>");
      return;
    }
  }
  out("rpm=%u  acc=%u  id=%u  (Bus: %lu Baud 8N1, pre/post %d/%d us)\r\n", gRpm, gAcc, servo.slaveId(),
      static_cast<unsigned long>(MODBUS_BAUD), RS485_PRE_DELAY_US, RS485_POST_DELAY_US);
}

void cmdRawRead(int argc, char** argv) {
  long reg, count = 1;
  if (!argLong(argc, argv, 1, 0, 0xFFFF, reg, "reg")) return;
  if (argc > 2 && !argLong(argc, argv, 2, 1, 8, count, "anzahl")) return;
  uint16_t values[8];
  if (!servo.readInputRegisters(static_cast<uint16_t>(reg), values, static_cast<uint8_t>(count))) {
    printError();
    return;
  }
  for (long i = 0; i < count; ++i) {
    out("  0x%04lX = 0x%04X (%u)\r\n", static_cast<unsigned long>(reg + i), values[i], values[i]);
  }
}

void cmdRawWrite(int argc, char** argv) {
  long reg, value;
  if (!argLong(argc, argv, 1, 0, 0xFFFF, reg, "reg")) return;
  if (!argLong(argc, argv, 2, 0, 0xFFFF, value, "wert")) return;
  report(servo.writeRegister(static_cast<uint16_t>(reg), static_cast<uint16_t>(value)));
}

// =============================================================================
// 7. Zyklische Aufgaben
// =============================================================================

void pollWatch() {
  if (gWatchIntervalMs == 0 || millis() - gLastWatchMs < gWatchIntervalMs) return;
  gLastWatchMs = millis();
  printWatchLine();
}

// Meldet das Ende einer Referenzfahrt. Fertig ist sie erst, wenn der Motor
// wirklich steht (Status 1) und das Statusregister "referenziert" meldet.
void pollHoming() {
  if (!gHoming.active || millis() - gHoming.lastPollMs < HOMING_POLL_MS) return;
  gHoming.lastPollMs = millis();

  uint8_t state;
  uint16_t raw;
  if (!servo.readMotorStatus(state) || !servo.readInputRegisters(mks::IR_HOME_STATUS, &raw, 1)) {
    if (++gHoming.errors >= 10) {
      gHoming.active = false;
      out("Referenzfahrt: keine Antwort mehr (%s) - mit 'status' pruefen\r\n> ", servo.lastError());
    }
    return;
  }
  gHoming.errors = 0;

  const unsigned long elapsed = millis() - gHoming.startMs;
  const uint8_t home = mks::homingStatusFromRaw(raw);

  // Protokoll: jede Statusaenderung mit Zeitstempel ausgeben
  if (state != gHoming.lastState || raw != gHoming.lastRaw) {
    out("  [%6lu ms] Motor: %-14s Reg 0x3B = 0x%04X\r\n", elapsed, mks::motorStatusText(state), raw);
    gHoming.lastState = state;
    gHoming.lastRaw = raw;
  }

  if (elapsed > HOMING_TIMEOUT_MS) {
    gHoming.active = false;
    Serial.println("Referenzfahrt: Timeout - Not-Halt wird gesendet");
    report(servo.emergencyStop());
    Serial.print("> ");
    return;
  }

  if (state != static_cast<uint8_t>(mks::MotorStatus::Stopped)) {
    gHoming.seenMoving = true;
    gHoming.stoppedPolls = 0;
    return;
  }
  if (!gHoming.seenMoving && elapsed < 1500) return;  // Start der Fahrt abwarten
  if (gHoming.stoppedPolls < 255) ++gHoming.stoppedPolls;

  if (home == 1 && gHoming.stoppedPolls >= HOMING_OK_POLLS) {
    gHoming.active = false;
    int64_t axis;
    Serial.print("Referenzfahrt OK");
    if (servo.readEncoderAddition(axis)) {
      Serial.print(", Position ");
      Serial.print(axisToDegrees(axis), 2);
      Serial.print(" Grad");
    }
    Serial.println();
    Serial.print("> ");
  } else if (gHoming.stoppedPolls >= HOMING_FAIL_POLLS) {
    gHoming.active = false;
    bool blocked = false;
    servo.readProtected(blocked);
    out("Referenzfahrt FEHLGESCHLAGEN (%s, Reg 0x3B = 0x%04X)%s\r\n> ", homeStatusText(home), raw,
        blocked ? " - Blockierschutz ausgeloest, 'unlock' senden" : "");
  }
}

// =============================================================================
// 8. Ausgabe-Helfer
// =============================================================================

// printf-artige Ausgabe auf den USB-Seriell-Port (ohne %f, siehe Serial.print)
void out(const char* fmt, ...) {
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
}

// "OK" oder Fehlermeldung zum letzten Servo-Befehl
void report(bool ok) {
  if (ok) {
    Serial.println("OK");
  } else {
    printError();
  }
}

void printError() {
  out("FEHLER: %s\r\n", servo.lastError());
  if (servo.lastWasRejected()) {
    Serial.println("  -> Laeuft der Motor? Manche Befehle (z. B. dis, cur) gehen nur im Stillstand,");
    Serial.println("     vorher 'stop' senden.");
  } else {
    out("  -> Pruefen: Servo-Versorgung, A/B-Verdrahtung, Mb_RTU=Enable,\r\n"
        "     UartBaud=%lu, UartAddr=%u, Mode=SR_xxx\r\n",
        static_cast<unsigned long>(MODBUS_BAUD), servo.slaveId());
  }
}

void printWatchLine() {
  int16_t rpm;
  int64_t axis;
  int32_t error;
  uint8_t state;
  if (!servo.readSpeed(rpm) || !servo.readEncoderAddition(axis) || !servo.readAngleError(error) ||
      !servo.readMotorStatus(state)) {
    out("[%lu] FEHLER: %s\r\n", millis(), servo.lastError());
    return;
  }
  out("[%lu] %5d U/min  pos ", millis(), rpm);
  Serial.print(axisToDegrees(axis), 2);
  Serial.print(" Grad  err ");
  Serial.print(angleErrorToDegrees(error), 2);
  out(" Grad  %s\r\n", mks::motorStatusText(state));
}

// Serial.print kennt kein int64_t
void printInt64(int64_t value) {
  char buf[24];
  char* p = buf + sizeof(buf);
  *--p = '\0';
  const bool negative = value < 0;
  uint64_t u = negative ? static_cast<uint64_t>(-(value + 1)) + 1 : static_cast<uint64_t>(value);
  do {
    *--p = static_cast<char>('0' + (u % 10));
    u /= 10;
  } while (u != 0);
  if (negative) *--p = '-';
  Serial.print(p);
}

const char* homeStatusText(uint8_t status) {
  switch (status) {
    case 0: return "laeuft";
    case 1: return "referenziert";
    case 2: return "nicht referenziert";
    default: return "unbekannt";
  }
}

double axisToDegrees(int64_t axis) { return static_cast<double>(axis) * 360.0 / mks::AXIS_PER_REV; }
double angleErrorToDegrees(int32_t error) { return error * 360.0 / 51200.0; }

// =============================================================================
// 9. Argument-Parser
// =============================================================================

// Ganzzahl dezimal oder mit 0x-Praefix hexadezimal
bool toLong(const char* s, long& value) {
  const bool hex = s[0] == '0' && (s[1] == 'x' || s[1] == 'X');
  char* end = nullptr;
  value = strtol(s, &end, hex ? 16 : 10);
  return end != s && *end == '\0';
}

bool toDouble(const char* s, double& value) {
  char* end = nullptr;
  value = strtod(s, &end);
  return end != s && *end == '\0';
}

// Liest argv[index] als Ganzzahl im Bereich minVal..maxVal, meldet Fehler selbst.
bool argLong(int argc, char** argv, int index, long minVal, long maxVal, long& value, const char* name) {
  if (index >= argc) {
    out("Parameter <%s> fehlt\r\n", name);
    return false;
  }
  if (!toLong(argv[index], value) || value < minVal || value > maxVal) {
    out("Ungueltiger Wert fuer %s (erlaubt %ld..%ld)\r\n", name, minVal, maxVal);
    return false;
  }
  return true;
}

// Liest die optionalen Parameter [rpm] [acc] ab Position 'first'.
bool optRpmAcc(int argc, char** argv, int first, uint16_t& rpm, uint8_t& acc) {
  long v;
  rpm = gRpm;
  acc = gAcc;
  if (argc > first) {
    if (!argLong(argc, argv, first, 0, mks::MAX_RPM, v, "rpm")) return false;
    rpm = static_cast<uint16_t>(v);
  }
  if (argc > first + 1) {
    if (!argLong(argc, argv, first + 1, 0, 255, v, "acc")) return false;
    acc = static_cast<uint8_t>(v);
  }
  return true;
}

// Winkel in Grad -> Encoder-Achseinheiten (0x4000 pro Umdrehung)
bool degreesToAxis(const char* s, int32_t& axis) {
  double deg;
  if (!toDouble(s, deg) || deg > 1e7 || deg < -1e7) {
    Serial.println("Ungueltiger Winkel (Grad, z. B. 90 oder -45.5)");
    return false;
  }
  axis = static_cast<int32_t>(lround(deg * mks::AXIS_PER_REV / 360.0));
  return true;
}
