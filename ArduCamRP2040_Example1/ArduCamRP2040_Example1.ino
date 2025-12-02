#include <ArduCamRP2040.h>
#include <SD.h>

// ===========================

// CS Pins
#define CAM_CS_PIN   10    // Camera CS
#define SD_CS_PIN     5    // SD card CS

// Max image size to buffer in RAM (bytes)
#define MAX_IMAGE_SIZE  60000

// Global image buffer (static to avoid malloc)
static uint8_t imageBuffer[MAX_IMAGE_SIZE];

// Camera instance
ArduCamRP2040 cam(ArduCamRP2040::OV2640_MODEL, CAM_CS_PIN);

// Photo index used for sequential filenames
static uint16_t photoIndex = 0;



// ===========================
// BLOCK: File index helpers
// ===========================
// These helpers keep track of the next image number in INDEX.TXT
// on the SD card. You can copy this block into any sketch that
// needs sequential image names like IMG_0000.JPG.

  void loadPhotoIndex() {
    if (!SD.exists("INDEX.TXT")) {
      photoIndex = 0;
      return;
    }

    File f = SD.open("INDEX.TXT", FILE_READ);
    if (!f) {
      photoIndex = 0;
      return;
    }

    String s = f.readStringUntil('\n');
    f.close();
    int val = s.toInt();
    if (val < 0) val = 0;
    photoIndex = (uint16_t)val;
  }

  void savePhotoIndex() {
    SD.remove("INDEX.TXT");
    File f = SD.open("INDEX.TXT", FILE_WRITE);
    if (!f) {
      Serial.println("Warning: failed to update INDEX.TXT");
      return;
    }
    f.println(photoIndex);
    f.close();
  }

  void makeNextPhotoName(char *jpgName, size_t len) {
    // Generates "IMG_0000.JPG", "IMG_0001.JPG", ...
    snprintf(jpgName, len, "IMG_%04u.JPG", photoIndex);
    photoIndex++;
    savePhotoIndex();
  }



// ===========================
// BLOCK: Camera image settings
// ===========================

void applyImageSettings() {
  // Example default settings:
  // - Neutral brightness/contrast/saturation
  // - No flip, no mirror

  bool ok;

//    Brightness levels: BRIGHTNESS_NEG2, BRIGHTNESS_NEG1, BRIGHTNESS_0, BRIGHTNESS_POS1, BRIGHTNESS_POS2
  ok = cam.setBrightness(ArduCamRP2040::BRIGHTNESS_0);
  Serial.print("setBrightness: "); Serial.println(ok ? "OK" : "FAIL");

//    Contrast levels:   CONTRAST_NEG2, CONTRAST_NEG1, CONTRAST_0, CONTRAST_POS1, CONTRAST_POS2
  ok = cam.setContrast(ArduCamRP2040::CONTRAST_0);
  Serial.print("setContrast: "); Serial.println(ok ? "OK" : "FAIL");

//    Saturation levels: SATURATION_NEG2, SATURATION_NEG1, SATURATION_0, SATURATION_POS1, SATURATION_POS2
  ok = cam.setSaturation(ArduCamRP2040::SATURATION_0);
  Serial.print("setSaturation: "); Serial.println(ok ? "OK" : "FAIL");

//     cam.setFlip(true/false);
  ok = cam.setFlip(false);
  Serial.print("setFlip(false): "); Serial.println(ok ? "OK" : "FAIL");

//     cam.setMirror(true/false);
  ok = cam.setMirror(false);
  Serial.print("setMirror(false): "); Serial.println(ok ? "OK" : "FAIL");
}



// ===========================
// BLOCK: Snapshot to SD (with JPEG trimming)
// ===========================
//
// This uses:
//   cam.captureJpegCore(buffer, len, maxLen), captures a frame into buffer, trims to valid JPEG from SOI (0xFFD8) to last EOI (0xFFD9)
//
// If you want raw FIFO (no trim), use captureToBuffer instead.

bool captureSnapshotToSD() {
  Serial.println("Capturing snapshot...");

  digitalWrite(SD_CS_PIN, HIGH);

  uint32_t imgLen = 0;

  bool ok = cam.captureJpegCore(imageBuffer, imgLen, MAX_IMAGE_SIZE, 3000);
  if (!ok) {
    Serial.println("captureJpegCore failed (timeout, buffer too small, or no JPEG markers).");
    return false;
  }

  Serial.print("Captured trimmed JPEG of ");
  Serial.print(imgLen);
  Serial.println(" bytes.");

  digitalWrite(CAM_CS_PIN, HIGH);

  char jpgName[13];
  makeNextPhotoName(jpgName, sizeof(jpgName));

  Serial.print("Saving image as ");
  Serial.println(jpgName);

  File imgFile = SD.open(jpgName, FILE_WRITE);
  if (!imgFile) {
    Serial.println("Failed to open JPEG file on SD.");
    return false;
  }

  size_t written = imgFile.write(imageBuffer, imgLen);
  imgFile.close();

  Serial.print("Wrote ");
  Serial.print(written);
  Serial.println(" bytes to JPEG file.");

  if (written != imgLen) {
    Serial.println("Warning: not all bytes were written to SD!");
    return false;
  }

  Serial.println("Snapshot saved successfully.");
  return true;
}

// ===========================
// SETUP & LOOP
// ===========================



void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Camera Settings + Snapshot Example ===");

//=============COPY THIS=================

  // CS pins
  pinMode(CAM_CS_PIN, OUTPUT);
  digitalWrite(CAM_CS_PIN, HIGH);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  // Optional: enable debug prints from library
  cam.setDebug(&Serial);

  // Initialize camera
  Serial.println("Initializing camera...");
  cam.begin();

  cam.writeChipReg(ARDUCHIP_TEST1, 0x55);
  uint8_t t = cam.readChipReg(ARDUCHIP_TEST1);
  Serial.print("ArduChip TEST1 = 0x"); Serial.println(t, HEX);

  uint8_t vid = 0, pid = 0;
  cam.wrSensorReg8_8(OV2640_BANK_SEL_REG, OV2640_BANK_SENSOR);
  cam.rdSensorReg8_8(OV2640_CHIPID_HIGH, vid);
  cam.rdSensorReg8_8(OV2640_CHIPID_LOW,  pid);
  Serial.print("OV2640 VID = 0x"); Serial.print(vid, HEX);
  Serial.print("  PID = 0x"); Serial.println(pid, HEX);

  bool initOk = cam.initOV2640();
  Serial.print("initOV2640() = "); Serial.println(initOk ? "OK" : "FAIL");
  if (!initOk) {
    Serial.println("Camera init failed. Halting.");
    while (1) { delay(1000); }
  }

  bool sizeOk = cam.setJpegSize(OV2640_320x240);
  Serial.print("setJpegSize(QVGA) = "); Serial.println(sizeOk ? "OK" : "FAIL");
  if (!sizeOk) {
    Serial.println("JPEG size set failed. Halting.");
    while (1) { delay(1000); }
  }

  // Apply image settings.
  applyImageSettings();

  // Initialize SD
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD.begin() failed! Check wiring/card.");
    while (1) { delay(1000); }
  }
  Serial.println("SD card initialized.");

  // Load current index for filenames
  loadPhotoIndex();
  Serial.print("Starting photo index = ");
  Serial.println(photoIndex);


// Take one snapshot on startup
// Copy this command to take a snapshot and save it to the SD card
  captureSnapshotToSD();
}
//====================================================================

void loop() {



}
