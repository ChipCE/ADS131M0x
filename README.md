


# ADS131M02 and ADS131M04 Arduino Library

Arduino lib for the  ADS131M02 and ADS131M04 24-bit Analog to Digital Converter.

Mostly based on the work of https://github.com/LucasEtchezuri/Arduino-ADS131M04 and https://github.com/raibisch/ADS131M0x

## My changes to Raibisch's lib
* Fix problem of channel offset when using ADS131M04
* Add example of using ESP32 cled as clock source for ADS131M0x
* Split ADS131M0x to ADS131M02 and ADS131M04
* Fix examples
* Add support for rp2040

## Raibisch's change to original LucasEtchezuri's lib
* adaption for ADS131M02 and conditional code parts for ADS131M02 and ADS131M04
* modified interface for flexible SPI-Port
* method to change SPI clock 
* minor changes for speed optimation
* additonal interface description

Please notice: all modifications are only testet with ESP32, rp2040

## Helpfull Links
Datasheet = https://www.ti.com/product/ADS131M02

Datasheet = https://www.ti.com/product/ADS131M04

## Download and Installation

### Arduino IDE
To download click the DOWNLOAD ZIP button, rename the uncompressed folder ADS131M0x. Check that the ADS131M0x folder contains ADS131M0x.cpp and ADS131M0x.h

Place the ADS131M0x library folder your arduinosketchfolder/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE.

### Platform IO
Add in 'platform.ini' (no need to download before)
```
lib_deps =
  https://github.com/ChipCE/ADS131M0x
``` 


