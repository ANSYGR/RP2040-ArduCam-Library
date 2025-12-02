ArduCamRP2040 is a from-scratch rewrite of the ArduCAM driver, built specifically for the RP2040 platform (Feather RP2040, Pico, etc.). It was created to support Cansat payloads, where reliability, low complexity, and predictable behavior are far more important than compatibility with dozens of unrelated microcontrollers.


This library:

Supports ArduCAM Mini with OV2640 (2MP)
Uses pure RP2040 SPI and I²C (no AVR/ESP legacy code)
Provides clean APIs for:
Capturing JPEG images
Accessing FIFO safely
Trimming JPEG padding
Adjusting image properties (brightness, contrast, saturation, flip, mirror)
Allows simultaneous use of SD cards and radio modules by avoiding SPI conflicts


Includes examples for:

Saving snapshots to SD card
Naming photos sequentially
Adjusting image settings
Preparing data for wireless transmission (future example)
This library is intentionally small, predictable, and mission-oriented.


🧩 Supported Hardware:

Adafruit Feather RP2040
ArduCAM Mini 2MP OV2640 (SPI + I²C)
MicroSD via SPI
(library includes functions to safely arbitrate SPI bus)

Radio modules (optional)
RFM69, LoRa, etc.
(not part of the library, but compatible since we avoid SPI conflicts)


📦 Installation:

Copy the library folder ArduCamRP2040/ into:
Documents/Arduino/libraries/

✨ Key Features

✔ Image Controls
These will be familiar to users from common camera systems:
cam.setBrightness(BRIGHTNESS_POS1);
cam.setContrast(CONTRAST_NEG1);
cam.setSaturation(SATURATION_0);
cam.setFlip(true);      // vertical flip
cam.setMirror(true);    // horizontal mirror

✔ Sequential File Naming

Helper functions create:
IMG_0000.JPG
IMG_0001.JPG
IMG_0002.JPG
Stored in INDEX.TXT for persistence.

✔ Debug Mode

Enable detailed logs:
cam.setDebug(&Serial);

Disable logs:
cam.setDebug(nullptr);


🧪 Included Examples
ArduCamRP2040_Example1 shows how to:

Adjust image properties
Toggle flip/mirror
Capture an image
Save to SD
Use sequential naming


🧠 How the Camera Pipeline Works

A typical capture flow:

flushFifo()
clearFifoFlag()
startCapture()
Wait for captureDone()
Read FIFO into RAM
Trim JPEG padding
Save/transmit
