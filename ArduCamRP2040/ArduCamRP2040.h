#pragma once

#include "ov2640_regs.h"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Simple JPEG size IDs – align these with your tables
#define OV2640_320x240   0   // QVGA
// #define OV2640_640x480   1   // VGA, if/when you add that table

// TODO: adjust if your working OV2640 I2C test used a different 7-bit address
#define OV2640_I2C_ADDR  0x30  

// From OV2640 docs / ArduCAM examples
#define OV2640_BANK_SEL_REG 0xFF
#define OV2640_BANK_DSP     0x00
#define OV2640_BANK_SENSOR  0x01

#define OV2640_CHIPID_HIGH  0x0A
#define OV2640_CHIPID_LOW   0x0B

// ArduChip SPI register addresses (from ArduCAM docs)
#define ARDUCHIP_TEST1      0x00  // test register

#define ARDUCHIP_FIFO       0x04  // FIFO and I2C control
#define FIFO_CLEAR_MASK     0x01
#define FIFO_START_MASK     0x02

#define BURST_FIFO_READ     0x3C  // burst FIFO read command

#define ARDUCHIP_TRIG       0x41  // trigger source
#define CAP_DONE_MASK       0x08  // capture done flag

#define FIFO_SIZE1          0x42  // FIFO size [7:0]
#define FIFO_SIZE2          0x43  // FIFO size [15:8]
#define FIFO_SIZE3          0x44  // FIFO size [18:16]



// Forward declaration for future use (we'll add the real struct later)
struct sensor_reg;

class ArduCamRP2040 {
public:

  // Discrete levels for image adjustments
  enum Brightness {
    BRIGHTNESS_NEG2 = -2,
    BRIGHTNESS_NEG1 = -1,
    BRIGHTNESS_0    =  0,
    BRIGHTNESS_POS1 =  1,
    BRIGHTNESS_POS2 =  2
  };

  enum Contrast {
    CONTRAST_NEG2 = -2,
    CONTRAST_NEG1 = -1,
    CONTRAST_0    =  0,
    CONTRAST_POS1 =  1,
    CONTRAST_POS2 =  2
  };

  enum Saturation {
    SATURATION_NEG2 = -2,
    SATURATION_NEG1 = -1,
    SATURATION_0    =  0,
    SATURATION_POS1 =  1,
    SATURATION_POS2 =  2
  };

  // OV2640 image controls (JPEG mode)
  bool setBrightness(Brightness level);
  bool setContrast(Contrast level);
  bool setSaturation(Saturation level);

  // Flip and mirror (boolean toggles)
  bool setFlip(bool enable);    // vertical flip
  bool setMirror(bool enable);  // horizontal mirror


  enum SensorModel {
    OV2640_MODEL = 1,
  };

  // Constructor
  ArduCamRP2040(SensorModel model, uint8_t csPin);

  // Basic init: sets up SPI, I2C, and CS pin
  void begin();

  // --- Debug helpers (we'll use these first to replicate your tests) ---
  void    writeChipReg(uint8_t addr, uint8_t val); // ArduChip SPI reg write
  uint8_t readChipReg(uint8_t addr);               // ArduChip SPI reg read

  bool    wrSensorReg8_8(uint8_t reg, uint8_t val);      // OV2640 I2C write
  bool    rdSensorReg8_8(uint8_t reg, uint8_t &val_out); // OV2640 I2C read

  // --- FIFO & capture control (ArduChip side) ---
  void     flushFifo();
  void     clearFifoFlag();
  void     startCapture();
  bool     captureDone();
  uint32_t fifoLength();
  void     fifoBurstRead(Stream &out);  // stream out JPEG data


  // Sensor config using register tables
  bool    initOV2640();                       // run base init sequence
  bool    setJpegSize(uint8_t sizeConst);     // choose resolution

  // Generic table writer (used internally)
  bool    wrSensorRegs(const struct sensor_reg *regs);

  // Read exactly 'len' bytes from FIFO into a RAM buffer
  void readFifoToBuffer(uint8_t *buf, uint32_t len);

  // High-level capture helper:
  // - flushes FIFO
  // - starts capture
  // - waits for CAP_DONE (with timeout)
  // - reads data into 'buf'
  // Returns true on success, false on timeout or buffer too small.
  bool captureToBuffer(uint8_t *buf,
                       uint32_t &outLen,
                       uint32_t maxLen,
                       uint32_t timeoutMs = 3000);

    // Trim a JPEG buffer in-place:
  //   - Searches for SOI (0xFFD8) and the last EOI (0xFFD9).
  //   - Moves the valid JPEG to the start of the buffer if needed.
  //   - Updates 'len' to the trimmed length.
  // Returns true on success, false if a valid JPEG is not found.
  bool trimJpegInPlace(uint8_t *buf, uint32_t &len);

  // Convenience: capture + trim in one call.
  // Uses captureToBuffer(...) internally and then trimJpegInPlace(...).
  bool captureJpegCore(uint8_t *buf,
                       uint32_t &outLen,
                       uint32_t maxLen,
                       uint32_t timeoutMs = 3000);

  // Optional debug output. Pass &Serial (or any Stream) to enable,
  // or nullptr to disable.
  void setDebug(Stream *dbg) { _debug = dbg; }


private:
  SensorModel _model;
  uint8_t     _csPin;

  Stream *_debug = nullptr;

  // Low-level SPI helpers
  void    csLow();
  void    csHigh();
  void    busWrite(uint8_t addr, uint8_t val);
  uint8_t busRead(uint8_t addr);
};


