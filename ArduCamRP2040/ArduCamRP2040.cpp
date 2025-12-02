#include "ArduCamRP2040.h"

// ===== Constructor & basic init =====

ArduCamRP2040::ArduCamRP2040(SensorModel model, uint8_t csPin)
: _model(model), _csPin(csPin)
{
}

void ArduCamRP2040::begin() {
  // Chip-select pin for ArduChip
  pinMode(_csPin, OUTPUT);
  csHigh();  // deselect by default

  // Start I2C and SPI with RP2040-friendly settings
  Wire.begin();

  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}

// ===== SPI low-level (ArduChip side) =====

void ArduCamRP2040::csLow() {
  digitalWrite(_csPin, LOW);
}

void ArduCamRP2040::csHigh() {
  digitalWrite(_csPin, HIGH);
}

void ArduCamRP2040::busWrite(uint8_t addr, uint8_t val) {
  csLow();
  // ArduChip convention: write = addr | 0x80
  SPI.transfer(addr | 0x80);
  SPI.transfer(val);
  csHigh();
}

uint8_t ArduCamRP2040::busRead(uint8_t addr) {
  csLow();
  // ArduChip convention: read = addr & 0x7F
  SPI.transfer(addr & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  csHigh();
  return v;
}

void ArduCamRP2040::writeChipReg(uint8_t addr, uint8_t val) {
  busWrite(addr, val);
}

uint8_t ArduCamRP2040::readChipReg(uint8_t addr) {
  return busRead(addr);
}

// ===== I2C low-level (sensor side) =====

bool ArduCamRP2040::wrSensorReg8_8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(OV2640_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool ArduCamRP2040::rdSensorReg8_8(uint8_t reg, uint8_t &val_out) {
  Wire.beginTransmission(OV2640_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(OV2640_I2C_ADDR, (uint8_t)1) != 1) {
    return false;
  }

  val_out = Wire.read();
  return true;
}

// ===== FIFO & capture control (ArduChip side) =====

void ArduCamRP2040::flushFifo() {
  // Clears FIFO write-done flag / pointers (per ArduCAM docs)
  writeChipReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCamRP2040::clearFifoFlag() {
  // Same as library: clear capture done flag
  writeChipReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCamRP2040::startCapture() {
  // Tell ArduChip to start capturing a frame into FIFO
  writeChipReg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

bool ArduCamRP2040::captureDone() {
  // Check CAP_DONE bit in ARDUCHIP_TRIG
  uint8_t v = readChipReg(ARDUCHIP_TRIG);
  return (v & CAP_DONE_MASK) != 0;
}

uint32_t ArduCamRP2040::fifoLength() {
  // Combine FIFO_SIZE1/2/3 into 19-bit length (max ~512k)
  uint32_t len1 = readChipReg(FIFO_SIZE1);
  uint32_t len2 = readChipReg(FIFO_SIZE2);
  uint32_t len3 = readChipReg(FIFO_SIZE3) & 0x07; // upper bits

  uint32_t length = ((len3 << 16) | (len2 << 8) | len1) & 0x07FFFF;
  return length;
}

void ArduCamRP2040::fifoBurstRead(Stream &out) {
  // Burst-read entire FIFO contents and dump to a Stream
  uint32_t len = fifoLength();
  csLow();
  SPI.transfer(BURST_FIFO_READ);
  for (uint32_t i = 0; i < len; i++) {
    uint8_t b = SPI.transfer(0x00);
    out.write(b);
  }
  csHigh();
}

// ===== Sensor register table writer =====

bool ArduCamRP2040::wrSensorRegs(const struct sensor_reg *regs) {
  const struct sensor_reg *p = regs;

  // We stop when we hit the sentinel {0xFFFF, 0xFFFF}
  while (!(p->reg == 0xFFFF && p->val == 0xFFFF)) {
    if (!wrSensorReg8_8((uint8_t)p->reg, (uint8_t)p->val)) {
      return false;  // bail out on first failure
    }
    p++;
  }
  return true;
}

// ===== High-level sensor init =====

bool ArduCamRP2040::initOV2640() {
  if (_model != OV2640_MODEL) return false;

  // 1) Soft reset
  if (!wrSensorReg8_8(OV2640_BANK_SEL_REG, OV2640_BANK_SENSOR)) return false;
  if (!wrSensorReg8_8(0x12, 0x80)) return false; // COM7 reset
  delay(100);

  // 2) Base QVGA config
  if (!wrSensorRegs(ov2640_qvga_base)) return false;

  // 3) YUV422 output config
  if (!wrSensorRegs(ov2640_yuv422)) return false;

  // 4) JPEG init + JPEG mode
  if (!wrSensorRegs(ov2640_jpeg_init)) return false;
  if (!wrSensorRegs(ov2640_jpeg_mode)) return false;

  return true;
}


#define OV2640_320x240   0

bool ArduCamRP2040::setJpegSize(uint8_t sizeConst) {
  if (_model != OV2640_MODEL) return false;

  switch (sizeConst) {
    case OV2640_320x240:
      return wrSensorRegs(ov2640_320x240_jpeg);
    default:
      return false;
  }
}

void ArduCamRP2040::readFifoToBuffer(uint8_t *buf, uint32_t len) {
  csLow();
  SPI.transfer(BURST_FIFO_READ);
  for (uint32_t i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  csHigh();
}

bool ArduCamRP2040::captureToBuffer(uint8_t *buf,
                                    uint32_t &outLen,
                                    uint32_t maxLen,
                                    uint32_t timeoutMs) {
  if (_debug) _debug->println("[ArduCamRP2040] captureToBuffer: start");

  flushFifo();
  clearFifoFlag();
  startCapture();

  unsigned long start = millis();
  bool done = false;
  while (millis() - start < timeoutMs) {
    if (captureDone()) {
      done = true;
      break;
    }
  }
  if (!done) {
    if (_debug) _debug->println("[ArduCamRP2040] captureToBuffer: timeout waiting for CAP_DONE");
    return false;
  }

  uint32_t len = fifoLength();
  if (_debug) {
    _debug->print("[ArduCamRP2040] FIFO length = ");
    _debug->println(len);
  }

  if (len == 0) {
    if (_debug) _debug->println("[ArduCamRP2040] captureToBuffer: FIFO length is zero");
    return false;
  }
  if (len > maxLen) {
    if (_debug) {
      _debug->print("[ArduCamRP2040] captureToBuffer: buffer too small, max=");
      _debug->print(maxLen);
      _debug->print(" needed=");
      _debug->println(len);
    }
    return false;
  }

  readFifoToBuffer(buf, len);
  outLen = len;

  if (_debug) _debug->println("[ArduCamRP2040] captureToBuffer: success");
  return true;
}


#include <string.h>  // for memmove

bool ArduCamRP2040::trimJpegInPlace(uint8_t *buf, uint32_t &len) {
  if (!buf || len < 4) {
    return false;
  }

  // Find SOI (0xFFD8)
  int32_t soi = -1;
  for (uint32_t i = 0; i + 1 < len; i++) {
    if (buf[i] == 0xFF && buf[i + 1] == 0xD8) {
      soi = (int32_t)i;
      break;
    }
  }
  if (soi < 0) {
    // No SOI marker found
    return false;
  }

  // Find last EOI (0xFFD9)
  int32_t eoi = -1;
  for (int32_t i = (int32_t)len - 2; i >= 1; i--) {
    if (buf[i] == 0xFF && buf[i + 1] == 0xD9) {
      eoi = i;
      break;
    }
  }
  if (eoi < 0 || eoi <= soi) {
    // No valid EOI after SOI
    return false;
  }

  uint32_t jpegLen = (uint32_t)(eoi + 2 - soi);  // include 0xFF 0xD9

  // If JPEG doesn't start at index 0, move it down
  if (soi != 0) {
    memmove(buf, buf + soi, jpegLen);
  }

  len = jpegLen;
  return true;
}

bool ArduCamRP2040::captureJpegCore(uint8_t *buf,
                                    uint32_t &outLen,
                                    uint32_t maxLen,
                                    uint32_t timeoutMs) {
  // First, do a normal capture into the buffer
  if (!captureToBuffer(buf, outLen, maxLen, timeoutMs)) {
    return false;
  }

  // Now trim to SOI/EOI
  if (!trimJpegInPlace(buf, outLen)) {
    // Optional: you might want to keep the untrimmed data for debugging
    // but for the "core" API we report failure.
    return false;
  }

  return true;
}

bool ArduCamRP2040::setBrightness(Brightness level) {
  if (_model != OV2640_MODEL) return false;

  if (_debug) {
    _debug->print("[ArduCamRP2040] setBrightness level=");
    _debug->println((int)level);
  }

  // TODO: implement using OV2640 brightness registers or tables.
  // For now, just return true so it doesn't fail hard in user code.
  // (or return false if you prefer to signal "not implemented yet")
  return false;
}

bool ArduCamRP2040::setContrast(Contrast level) {
  if (_model != OV2640_MODEL) return false;
  if (_debug) {
    _debug->print("[ArduCamRP2040] setContrast level=");
    _debug->println((int)level);
  }
  // TODO: implement
  return false;
}

bool ArduCamRP2040::setSaturation(Saturation level) {
  if (_model != OV2640_MODEL) return false;
  if (_debug) {
    _debug->print("[ArduCamRP2040] setSaturation level=");
    _debug->println((int)level);
  }
  // TODO: implement
  return false;
}

bool ArduCamRP2040::setFlip(bool enable) {
  if (_model != OV2640_MODEL) return false;
  if (_debug) {
    _debug->print("[ArduCamRP2040] setFlip ");
    _debug->println(enable ? "ON" : "OFF");
  }
  // TODO: implement using OV2640 flip bit (vertical) in the correct register.
  return false;
}

bool ArduCamRP2040::setMirror(bool enable) {
  if (_model != OV2640_MODEL) return false;
  if (_debug) {
    _debug->print("[ArduCamRP2040] setMirror ");
    _debug->println(enable ? "ON" : "OFF");
  }
  // TODO: implement using OV2640 mirror bit (horizontal) in the correct register.
  return false;
}
