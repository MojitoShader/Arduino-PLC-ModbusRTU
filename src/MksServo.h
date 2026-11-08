// MksServo.h - Schlanker Modbus-RTU-Treiber fuer MKS SERVO42D/57D (RS485-Version)
//
// Registerbelegung nach "MKS SERVO42D/57D_RS485 User Manual V1.0.4", Part 8,
// abgeglichen mit "MKS SERVO42&57D Modbus RTU User Manual V1.0.9" (Firmware V1.0.9).
// Alle Mehrbyte-Werte sind Big-Endian (High-Word zuerst).
//
// Voraussetzungen am Servo (Menue am Display):
//   Mode    = SR_vFOC / SR_CLOSE / SR_OPEN  (serielle Steuerung)
//   Mb_RTU  = Enable
//   UartBaud/UartAddr passend zur Konfiguration in main.cpp
#pragma once

#include <Arduino.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

namespace mks {

// ---- Input-Register (Funktionscode 0x04, nur lesen) ------------------------
enum InputReg : uint16_t {
  IR_ENCODER_CARRY    = 0x0030,  // 3 Reg: int32 carry + uint16 value (0..0x3FFF)
  IR_ENCODER_ADDITION = 0x0031,  // 3 Reg: int48, 0x4000 = 1 Umdrehung
  IR_SPEED_RPM        = 0x0032,  // 1 Reg: int16, CCW > 0, CW < 0
  IR_PULSES           = 0x0033,  // 2 Reg: int32, empfangene Pulse
  IR_IO_STATUS        = 0x0034,  // 1 Reg: Bit0 IN_1, Bit1 IN_2, Bit2 OUT_1, Bit3 OUT_2
  IR_ANGLE_ERROR      = 0x0039,  // 2 Reg: int32, 51200 = 360 Grad
  IR_ENABLE_STATUS    = 0x003A,  // 1 Reg: 1 = enabled
  IR_HOME_STATUS      = 0x003B,  // 1 Reg: Hi = Referenzfahrt, Lo = Single-Turn-Null
                                 //        je 0 laeuft, 1 ok, 2 nicht referenziert/Fehler
                                 //        (am Geraet mit FW V1.0.9 gemessen - das Handbuch
                                 //         V1.0.9 beschreibt die Bytes vertauscht)
  IR_PROTECT_STATUS   = 0x003E,  // 1 Reg: 1 = blockiert (Stall-Flag)
  IR_VERSION          = 0x0040,  // 2 Reg: uint32, untere 3 Bytes = Version (0x01010009 -> V1.0.9)
  IR_MOTOR_STATUS     = 0x00F1,  // 1 Reg: siehe MotorStatus
};

// ---- Holding-Register (0x06 einzeln / 0x10 mehrfach, schreiben) -----------
enum HoldingReg : uint16_t {
  HR_RELEASE_PROTECT  = 0x003D,  // 0x06, Wert 1
  HR_RESTORE_DEFAULTS = 0x003F,  // 0x06, Wert 1 (danach neu kalibrieren!)
  HR_CALIBRATE        = 0x0080,  // 0x06, Wert 1 (Motor ohne Last!)
  HR_WORK_MODE        = 0x0082,  // 0x06, WorkMode
  HR_WORK_CURRENT     = 0x0083,  // 0x06, mA (42D max. 3000)
  HR_HOLD_CURRENT     = 0x009B,  // 0x06, 0..8 = 10..90 %
  HR_MICROSTEP        = 0x0084,  // 0x06, Mikroschritte
  HR_EN_PIN_ACTIVE    = 0x0085,  // 0x06, 0 L, 1 H, 2 Hold
  HR_PROTECT_ENABLE   = 0x0088,  // 0x06, 0/1
  HR_BAUDRATE         = 0x008A,  // 0x06, 1..7 (4 = 38400)
  HR_SLAVE_ADDR       = 0x008B,  // 0x06, Adresse
  HR_MODBUS_ENABLE    = 0x008E,  // 0x06, 0/1
  HR_KEY_LOCK         = 0x008F,  // 0x06, 0/1
  HR_HOME_PARAMS      = 0x0090,  // 0x10, 3 Reg, ABER nur 5 Datenbytes (MKS-Sonderformat):
                                 //   trig:8 dir:8 speed:16 endLimit:8 -> siehe setHomeParams()
  HR_GO_HOME          = 0x0091,  // 0x06, 1 = Referenzfahrt, 2 = auf Koordinaten-Null fahren
  HR_SET_ZERO         = 0x0092,  // 0x06, Wert 1 (aktuelle Position = 0)
  HR_HOME_MODE        = 0x0094,  // 0x10, 4 Reg: uint32 Offset, mode, Strom mA
  HR_HEARTBEAT        = 0x0098,  // 0x10, 2 Reg: uint32 ms ohne Telegramm -> Not-Halt (0 = aus)
  HR_POS_ERR_PROTECT  = 0x009D,  // 0x10, 3 Reg: [0|enable] [Zeit x15ms] [Fehler, 28000 = 360 Grad]
  HR_ENABLE           = 0x00F3,  // 0x06, 0/1
  HR_POS_REL_AXIS     = 0x00F4,  // 0x10, 4 Reg: acc, speed, int32 relAxis
  HR_POS_ABS_AXIS     = 0x00F5,  // 0x10, 4 Reg: acc, speed, int32 absAxis
  HR_SPEED_MODE       = 0x00F6,  // 0x10, 2 Reg: (dir<<8)|acc, speed
  HR_EMERGENCY_STOP   = 0x00F7,  // 0x06, Wert 1
  HR_POS_REL_PULSES   = 0x00FD,  // 0x10, 4 Reg: (dir<<8)|acc, speed, uint32 pulses
  HR_POS_ABS_PULSES   = 0x00FE,  // 0x10, 4 Reg: acc, speed, int32 absPulses
  HR_SPEED_SAVE       = 0x00FF,  // 0x06, 0xC8 speichern / 0xCA loeschen
};

enum class WorkMode : uint8_t {
  CR_OPEN = 0, CR_CLOSE = 1, CR_vFOC = 2,  // Puls-Schnittstelle (STEP/DIR)
  SR_OPEN = 3, SR_CLOSE = 4, SR_vFOC = 5,  // serielle Schnittstelle (RS485)
};

enum class MotorStatus : uint8_t {
  ReadFail = 0, Stopped = 1, SpeedUp = 2, SpeedDown = 3,
  FullSpeed = 4, Homing = 5, Calibrating = 6,
};

// Referenzfahrt-Art (Register 0x0094)
enum class HomeMode : uint8_t {
  Endstop = 0,     // mit Endschalter an IN_1
  Mechanical = 1,  // gegen mechanischen Anschlag mit begrenztem Strom (sensorlos)
  SingleTurn = 2,  // Null innerhalb einer Umdrehung
};

// Welcher Bewegungsbefehl zuletzt gesendet wurde (fuer ein passendes "stop").
enum class Motion : uint8_t { None, Speed, RelPulses, AbsPulses, RelAxis, AbsAxis };

// Status der Referenzfahrt aus dem Rohwert von Register 0x3B (High-Byte, siehe oben)
constexpr uint8_t homingStatusFromRaw(uint16_t raw) { return static_cast<uint8_t>(raw >> 8); }

constexpr uint16_t MAX_RPM = 3000;
constexpr int32_t AXIS_PER_REV = 0x4000;  // Encoder-Einheiten pro Umdrehung

const char* motorStatusText(uint8_t status);
const char* workModeText(uint8_t mode);

class MksServo {
 public:
  // rs485 wird nur fuer Telegramme im MKS-Sonderformat (0x0090) direkt benutzt.
  MksServo(ModbusClient& bus, RS485Class& rs485, uint8_t slaveId)
      : _bus(bus), _rs485(rs485), _id(slaveId) {}

  void setSlaveId(uint8_t id) { _id = id; }
  uint8_t slaveId() const { return _id; }
  Motion lastMotion() const { return _lastMotion; }

  // ---- Grundfunktionen ----
  bool enable(bool on);
  bool emergencyStop();
  bool releaseProtection();
  bool setZero();
  bool setWorkMode(WorkMode mode);

  // ---- Referenzfahrt ----
  // dir: 0 = CW, 1 = CCW. rpm: Suchdrehzahl. endLimit nur fuer Endschalter-Betrieb.
  bool setHomeParams(uint8_t dir, uint16_t rpm, bool endLimit = false, uint8_t trigLevel = 0);
  // offsetAxis: Rueckzug vom Anschlag in Encoder-Einheiten (0x4000 = 360 Grad),
  // currentMa: Strom (= Kraft) beim Anfahren des Anschlags.
  bool setHomeMode(HomeMode mode, uint32_t offsetAxis, uint16_t currentMa);
  bool goHome();    // Referenzfahrt starten
  bool goToZero();  // schnell auf den Nullpunkt fahren (nach erfolgter Referenzfahrt)
  bool setWorkCurrent(uint16_t milliAmps);

  // ---- Bewegung ----
  // Vorzeichen = Richtung: >= 0 -> dir 0 (laut Handbuch CCW), < 0 -> dir 1 (CW)
  // acc: 0..255 (0 = ohne Rampe), rpm: 0..3000
  bool runSpeed(int16_t signedRpm, uint8_t acc);
  bool moveRelativePulses(int32_t signedPulses, uint16_t rpm, uint8_t acc);
  bool moveAbsolutePulses(int32_t absPulses, uint16_t rpm, uint8_t acc);
  bool moveRelativeAxis(int32_t relAxis, uint16_t rpm, uint8_t acc);
  bool moveAbsoluteAxis(int32_t absAxis, uint16_t rpm, uint8_t acc);
  // Stoppt die zuletzt gestartete Bewegungsart. acc > 0 = mit Rampe, 0 = sofort.
  bool stop(uint8_t acc);

  // ---- Status lesen ----
  bool readSpeed(int16_t& rpm);
  bool readEncoderAddition(int64_t& value);
  bool readPulses(int32_t& pulses);
  bool readAngleError(int32_t& error);
  bool readEnabled(bool& enabled);
  bool readProtected(bool& isProtected);
  bool readMotorStatus(uint8_t& status);
  bool readHomeStatus(uint8_t& status);  // 0 laeuft, 1 ok, 2 nicht referenziert/Fehler
  bool readVersion(uint32_t& version);

  // ---- Rohzugriff ----
  bool readInputRegisters(uint16_t address, uint16_t* out, uint8_t count);
  bool writeRegister(uint16_t address, uint16_t value);
  bool writeRegisters(uint16_t address, const uint16_t* values, uint8_t count);

  const char* lastError() const { return _lastError; }

 private:
  bool readU16(uint16_t address, uint16_t& value);
  bool moveAccSpeedPos(uint16_t reg, Motion motion, int32_t pos, uint16_t rpm, uint8_t acc);
  // Sendet pdu (ohne CRC) direkt ueber RS485 und erwartet eine Antwort von
  // respLen Bytes (inkl. CRC), deren erste 6 Bytes der Anfrage entsprechen.
  bool rawRequest(const uint8_t* pdu, size_t pduLen, size_t respLen);

  ModbusClient& _bus;
  RS485Class& _rs485;
  uint8_t _id;
  Motion _lastMotion = Motion::None;
  const char* _lastError = "";
};

}  // namespace mks
