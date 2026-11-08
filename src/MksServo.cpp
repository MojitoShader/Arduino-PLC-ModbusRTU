// MksServo.cpp - Implementierung, siehe MksServo.h
#include "MksServo.h"

namespace mks {

namespace {

uint16_t hiWord(uint32_t v) { return static_cast<uint16_t>(v >> 16); }
uint16_t loWord(uint32_t v) { return static_cast<uint16_t>(v & 0xFFFF); }
uint16_t clampRpm(uint32_t rpm) { return rpm > MAX_RPM ? MAX_RPM : static_cast<uint16_t>(rpm); }

// Modbus-CRC16 (Polynom 0xA001, Startwert 0xFFFF), Low-Byte wird zuerst gesendet
uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

constexpr unsigned long RAW_TIMEOUT_MS = 200;

}  // namespace

const char* motorStatusText(uint8_t status) {
  switch (status) {
    case 0: return "Lesefehler";
    case 1: return "steht";
    case 2: return "beschleunigt";
    case 3: return "bremst";
    case 4: return "volle Drehzahl";
    case 5: return "Referenzfahrt";
    case 6: return "kalibriert";
    default: return "unbekannt";
  }
}

const char* workModeText(uint8_t mode) {
  static const char* const names[] = {"CR_OPEN", "CR_CLOSE", "CR_vFOC",
                                      "SR_OPEN", "SR_CLOSE", "SR_vFOC"};
  return mode < 6 ? names[mode] : "?";
}

// ---------------------------------------------------------------------------
// Rohzugriff
// ---------------------------------------------------------------------------

bool MksServo::writeRegister(uint16_t address, uint16_t value) {
  // Funktionscode 0x06 (Write Single Register)
  if (_bus.holdingRegisterWrite(_id, address, value)) {
    _lastError = "";
    return true;
  }
  const char* err = _bus.lastError();
  _lastError = err ? err : "Schreiben fehlgeschlagen";
  return false;
}

bool MksServo::writeRegisters(uint16_t address, const uint16_t* values, uint8_t count) {
  // Funktionscode 0x10 (Write Multiple Registers)
  if (_bus.beginTransmission(_id, HOLDING_REGISTERS, address, count)) {
    for (uint8_t i = 0; i < count; ++i) {
      _bus.write(values[i]);
    }
    if (_bus.endTransmission()) {
      _lastError = "";
      return true;
    }
  }
  const char* err = _bus.lastError();
  _lastError = err ? err : "Schreiben fehlgeschlagen";
  return false;
}

bool MksServo::readInputRegisters(uint16_t address, uint16_t* out, uint8_t count) {
  // Funktionscode 0x04 (Read Input Registers)
  if (_bus.requestFrom(_id, INPUT_REGISTERS, address, count) != count) {
    const char* err = _bus.lastError();
    _lastError = err ? err : "Lesen fehlgeschlagen";
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) {
    out[i] = static_cast<uint16_t>(_bus.read());
  }
  _lastError = "";
  return true;
}

bool MksServo::readU16(uint16_t address, uint16_t& value) {
  return readInputRegisters(address, &value, 1);
}

bool MksServo::rawRequest(const uint8_t* pdu, size_t pduLen, size_t respLen) {
  // Fuer Telegramme, die nicht dem Modbus-Standard entsprechen und deshalb
  // nicht ueber ArduinoModbus gesendet werden koennen.
  uint8_t frame[32];
  uint8_t resp[16];
  if (pduLen + 2 > sizeof(frame) || respLen > sizeof(resp) || respLen < 8) {
    _lastError = "Rohtelegramm zu lang";
    return false;
  }
  memcpy(frame, pdu, pduLen);
  const uint16_t crc = crc16(frame, pduLen);
  frame[pduLen] = static_cast<uint8_t>(crc & 0xFF);
  frame[pduLen + 1] = static_cast<uint8_t>(crc >> 8);

  while (_rs485.available() > 0) _rs485.read();  // alte Bytes verwerfen
  _rs485.noReceive();
  _rs485.beginTransmission();
  _rs485.write(frame, pduLen + 2);
  _rs485.endTransmission();
  _rs485.receive();

  size_t n = 0;
  const unsigned long start = millis();
  while (n < respLen && millis() - start < RAW_TIMEOUT_MS) {
    if (_rs485.available() > 0) {
      resp[n++] = static_cast<uint8_t>(_rs485.read());
      if (n == 5 && (resp[1] & 0x80)) break;  // Exception-Antwort hat nur 5 Byte
    }
  }

  if (n < 5) {
    _lastError = "Connection timed out";
    return false;
  }
  if (crc16(resp, n - 2) != static_cast<uint16_t>(resp[n - 2] | (resp[n - 1] << 8))) {
    _lastError = "Invalid CRC";
    return false;
  }
  if (resp[1] & 0x80) {
    _lastError = resp[2] == 0x02 ? "Illegal data address" : "Modbus-Exception";
    return false;
  }
  if (n != respLen || memcmp(resp, frame, 6) != 0) {
    _lastError = "Unerwartete Antwort";
    return false;
  }
  _lastError = "";
  return true;
}

// ---------------------------------------------------------------------------
// Grundfunktionen
// ---------------------------------------------------------------------------

bool MksServo::enable(bool on) { return writeRegister(HR_ENABLE, on ? 1 : 0); }
bool MksServo::emergencyStop() { return writeRegister(HR_EMERGENCY_STOP, 1); }
bool MksServo::releaseProtection() { return writeRegister(HR_RELEASE_PROTECT, 1); }
bool MksServo::setZero() { return writeRegister(HR_SET_ZERO, 1); }
bool MksServo::goHome() { return writeRegister(HR_GO_HOME, 1); }
bool MksServo::goToZero() { return writeRegister(HR_GO_HOME, 2); }

bool MksServo::setHomeParams(uint8_t dir, uint16_t rpm, bool endLimit, uint8_t trigLevel) {
  // MKS-Sonderformat laut Handbuch V1.0.9, 5.3.2:
  //   01 10 00 90 00 03 05 <trig> <dir> <speed hi> <speed lo> <endLimit> CRC
  // (3 Register angekuendigt, aber nur 5 Datenbytes -> nicht ueber ArduinoModbus moeglich)
  rpm = clampRpm(rpm);
  const uint8_t pdu[] = {_id,
                         0x10,
                         0x00,
                         static_cast<uint8_t>(HR_HOME_PARAMS),
                         0x00,
                         0x03,
                         0x05,
                         trigLevel,
                         dir,
                         static_cast<uint8_t>(rpm >> 8),
                         static_cast<uint8_t>(rpm & 0xFF),
                         static_cast<uint8_t>(endLimit ? 1 : 0)};
  return rawRequest(pdu, sizeof(pdu), 8);  // Antwort: id 10 00 90 00 03 CRC
}

bool MksServo::setHomeMode(HomeMode mode, uint32_t offsetAxis, uint16_t currentMa) {
  // Register 0x0094, 4 Register: [offset hi] [offset lo] [mode] [Strom mA]
  const uint16_t regs[4] = {hiWord(offsetAxis), loWord(offsetAxis), static_cast<uint16_t>(mode),
                            currentMa};
  return writeRegisters(HR_HOME_MODE, regs, 4);
}

bool MksServo::setWorkMode(WorkMode mode) {
  return writeRegister(HR_WORK_MODE, static_cast<uint16_t>(mode));
}

bool MksServo::setWorkCurrent(uint16_t milliAmps) {
  return writeRegister(HR_WORK_CURRENT, milliAmps);
}

// ---------------------------------------------------------------------------
// Bewegung
// ---------------------------------------------------------------------------

bool MksServo::runSpeed(int16_t signedRpm, uint8_t acc) {
  // Register 0x00F6, 2 Register: [dir:8 | acc:8] [speed:16]
  const uint8_t dir = signedRpm < 0 ? 1 : 0;
  const uint32_t rpm = signedRpm < 0 ? -static_cast<int32_t>(signedRpm) : signedRpm;
  const uint16_t regs[2] = {static_cast<uint16_t>((dir << 8) | acc), clampRpm(rpm)};
  if (!writeRegisters(HR_SPEED_MODE, regs, 2)) return false;
  _lastMotion = Motion::Speed;
  return true;
}

bool MksServo::moveRelativePulses(int32_t signedPulses, uint16_t rpm, uint8_t acc) {
  // Register 0x00FD, 4 Register: [dir:8 | acc:8] [speed:16] [pulses:32]
  const uint8_t dir = signedPulses < 0 ? 1 : 0;
  const uint32_t pulses = signedPulses < 0 ? static_cast<uint32_t>(-static_cast<int64_t>(signedPulses))
                                           : static_cast<uint32_t>(signedPulses);
  const uint16_t regs[4] = {static_cast<uint16_t>((dir << 8) | acc), clampRpm(rpm),
                            hiWord(pulses), loWord(pulses)};
  if (!writeRegisters(HR_POS_REL_PULSES, regs, 4)) return false;
  _lastMotion = Motion::RelPulses;
  return true;
}

bool MksServo::moveAccSpeedPos(uint16_t reg, Motion motion, int32_t pos, uint16_t rpm, uint8_t acc) {
  // Register 0x00FE / 0x00F4 / 0x00F5, 4 Register: [acc:16] [speed:16] [pos:int32]
  const uint32_t raw = static_cast<uint32_t>(pos);
  const uint16_t regs[4] = {acc, clampRpm(rpm), hiWord(raw), loWord(raw)};
  if (!writeRegisters(reg, regs, 4)) return false;
  _lastMotion = motion;
  return true;
}

bool MksServo::moveAbsolutePulses(int32_t absPulses, uint16_t rpm, uint8_t acc) {
  return moveAccSpeedPos(HR_POS_ABS_PULSES, Motion::AbsPulses, absPulses, rpm, acc);
}

bool MksServo::moveRelativeAxis(int32_t relAxis, uint16_t rpm, uint8_t acc) {
  return moveAccSpeedPos(HR_POS_REL_AXIS, Motion::RelAxis, relAxis, rpm, acc);
}

bool MksServo::moveAbsoluteAxis(int32_t absAxis, uint16_t rpm, uint8_t acc) {
  return moveAccSpeedPos(HR_POS_ABS_AXIS, Motion::AbsAxis, absAxis, rpm, acc);
}

bool MksServo::stop(uint8_t acc) {
  // Stopp = gleicher Bewegungsbefehl mit speed = 0. Das erste Register ist bei
  // allen Modi "acc" (bei F6/FD steht im High-Byte dir = 0).
  const uint16_t regs[4] = {acc, 0, 0, 0};
  switch (_lastMotion) {
    case Motion::RelPulses: return writeRegisters(HR_POS_REL_PULSES, regs, 4);
    case Motion::AbsPulses: return writeRegisters(HR_POS_ABS_PULSES, regs, 4);
    case Motion::RelAxis:   return writeRegisters(HR_POS_REL_AXIS, regs, 4);
    case Motion::AbsAxis:   return writeRegisters(HR_POS_ABS_AXIS, regs, 4);
    case Motion::Speed:
    case Motion::None:
    default:                return writeRegisters(HR_SPEED_MODE, regs, 2);
  }
}

// ---------------------------------------------------------------------------
// Status lesen
// ---------------------------------------------------------------------------

bool MksServo::readSpeed(int16_t& rpm) {
  uint16_t v;
  if (!readU16(IR_SPEED_RPM, v)) return false;
  rpm = static_cast<int16_t>(v);
  return true;
}

bool MksServo::readEncoderAddition(int64_t& value) {
  uint16_t r[3];
  if (!readInputRegisters(IR_ENCODER_ADDITION, r, 3)) return false;
  uint64_t raw = (static_cast<uint64_t>(r[0]) << 32) | (static_cast<uint64_t>(r[1]) << 16) | r[2];
  if (raw & 0x800000000000ULL) raw |= 0xFFFF000000000000ULL;  // int48 -> int64 Vorzeichen
  value = static_cast<int64_t>(raw);
  return true;
}

bool MksServo::readPulses(int32_t& pulses) {
  uint16_t r[2];
  if (!readInputRegisters(IR_PULSES, r, 2)) return false;
  pulses = static_cast<int32_t>((static_cast<uint32_t>(r[0]) << 16) | r[1]);
  return true;
}

bool MksServo::readAngleError(int32_t& error) {
  uint16_t r[2];
  if (!readInputRegisters(IR_ANGLE_ERROR, r, 2)) return false;
  error = static_cast<int32_t>((static_cast<uint32_t>(r[0]) << 16) | r[1]);
  return true;
}

bool MksServo::readEnabled(bool& enabled) {
  uint16_t v;
  if (!readU16(IR_ENABLE_STATUS, v)) return false;
  enabled = (v & 0xFF) == 1;
  return true;
}

bool MksServo::readProtected(bool& isProtected) {
  uint16_t v;
  if (!readU16(IR_PROTECT_STATUS, v)) return false;
  isProtected = (v & 0xFF) == 1;
  return true;
}

bool MksServo::readMotorStatus(uint8_t& status) {
  uint16_t v;
  if (!readU16(IR_MOTOR_STATUS, v)) return false;
  status = static_cast<uint8_t>(v & 0xFF);
  return true;
}

bool MksServo::readHomeStatus(uint8_t& status) {
  uint16_t v;
  if (!readU16(IR_HOME_STATUS, v)) return false;
  status = homingStatusFromRaw(v);
  return true;
}

bool MksServo::readVersion(uint32_t& version) {
  uint16_t r[2];
  if (!readInputRegisters(IR_VERSION, r, 2)) return false;
  version = (static_cast<uint32_t>(r[0]) << 16) | r[1];
  return true;
}

}  // namespace mks
