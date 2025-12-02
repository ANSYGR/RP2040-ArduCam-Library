//Custom library file
#pragma once
#include <Arduino.h>

struct sensor_reg {
  uint16_t reg;
  uint16_t val;
};

// Base & mode tables
extern const struct sensor_reg ov2640_qvga_base[];   // from OV2640_QVGA
extern const struct sensor_reg ov2640_yuv422[];      // from OV2640_YUV422
extern const struct sensor_reg ov2640_jpeg_init[];   // from OV2640_JPEG_INIT
extern const struct sensor_reg ov2640_jpeg_mode[];   // from OV2640_JPEG

// Resolution-specific JPEG table
extern const struct sensor_reg ov2640_320x240_jpeg[];
