# UVC Scope
Stream LCD screen of your Rigol DS1074 using UVC to your PC.

## UVC video capture
This repository contains source code for Cypress FX3 UVC video capture.
USB descriptors and FX3 GPIF is set-up to capture 800x480 RGB24 video.
Capture using USB 2.0 is disabled.

## Compile
To compile this you will need Cypress FX3 SDK 1.3.1 placed at `../`.
Precompiled version is available in releases.

## Flash
You can flash compiled firmware into EEPROM on `CYUSB3KIT-003 EZ-USB® FX3™ SuperSpeed Explorer Kit`.

## HW connections
Signal polarity is set up specificaly for Rigol DS1074.

- GPIO16 `PCLK` = pixel clock
- GPIO28 `CTL11` = data enable
- GPIO29 `CTL12` = vertical sync
- GPIO0 to GPIO7 `DQ0 to DQ7` = blue
- GPIO8 to GPIO15 `DQ8 to DQ15` = green
- GPIO16 to GPIO23 `DQ16 to DQ23` = red

