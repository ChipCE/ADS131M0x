# ADS131M02 / ADS131M04 API Reference

## Overview

This library exposes two closely related classes for Texas Instruments’ ADS131M0x simultaneous-sampling delta-sigma ADCs:

- `ADS131M04` for the 4-channel device
- `ADS131M02` for the 2-channel device

Both classes share the same API where possible. Examples and method descriptions below apply to both unless explicitly noted. Tested platforms include ESP32, ESP32-S2, ESP32-S3, and ESP32-C3.

## Data Structures

### `struct adcOutput`

```cpp
struct adcOutput {
  uint16_t status;  // Status word returned with each conversion
  int32_t  ch0;     // Channel 0 result (24-bit signed)
  int32_t  ch1;     // Channel 1 result (24-bit signed)
  int32_t  ch2;     // Channel 2 result (ADS131M04 only)
  int32_t  ch3;     // Channel 3 result (ADS131M04 only)
};
```

## Classes

```cpp
#include "ADS131M04.h"
#include "ADS131M02.h"
```

Instantiate the class that matches your hardware:

```cpp
ADS131M04 adc4; // 4 channels
ADS131M02 adc2; // 2 channels
```

## Initialization

### `setClockSpeed`

```cpp
void setClockSpeed(uint32_t hz);
```

Configure the SPI clock frequency (default 1 MHz). Call before any overload of `begin`.

### `begin`

```cpp
void begin(SPIClass *spi,
           uint8_t clk_pin,
           uint8_t miso_pin,
           uint8_t mosi_pin,
           uint8_t cs_pin,
           uint8_t drdy_pin);
```

Initialise the ADC with the given SPI bus and GPIO assignments.

```cpp
void begin(SPIClass *spi,
           uint8_t clk_pin,
           uint8_t miso_pin,
           uint8_t mosi_pin,
           uint8_t cs_pin,
           uint8_t drdy_pin,
           uint8_t clkin_pin,
           unsigned int clkin_freq,
           uint8_t clkin_channel);
```

ESP32-specific overload that configures an LEDC channel to output the CLKIN signal before invoking the standard initialisation. On non-ESP32 boards the extra parameters are ignored.

### `reset`

```cpp
void reset(uint8_t reset_pin);
```

Pulse the hardware reset line (active low).

### `resetDevice`

```cpp
bool resetDevice();
```

Issue the software RESET command; returns `true` when the ADC acknowledges with `RSP_RESET_OK`.

## Configuration Methods

### Power and Sample Rate

```cpp
bool setPowerMode(uint8_t mode);   // POWER_MODE_*
bool setOsr(uint16_t osr);         // OSR_*
```

### Channel Control

```cpp
bool setChannelEnable(uint8_t channel, uint16_t enable);
bool setChannelPGA(uint8_t channel, uint16_t gain);
bool setInputChannelSelection(uint8_t channel, uint8_t mux);
bool setChannelOffsetCalibration(uint8_t channel, int32_t offset);
bool setChannelGainCalibration(uint8_t channel, uint32_t gain);
```

For `ADS131M02`, channel indices above 1 return `false`.

### Global Options

```cpp
void setGlobalChop(uint16_t enable);
void setGlobalChopDelay(uint16_t delay);
bool setDrdyFormat(uint8_t format);
bool setDrdyStateWhenUnavailable(uint8_t state);
```

## Status and Data

```cpp
bool     isDataReady() const;
int8_t   isDataReadySoft(uint8_t channel);
bool     isResetStatus() const;
bool     isLockSPI() const;
uint16_t isResetOK() const;
adcOutput readADC();
int32_t  readfastCh0();
```

### ADS131M04 Additional Helpers

```cpp
void     wakeup();
uint16_t readStatusRegister();
uint16_t readRegisterRaw(uint8_t address);
```

## Constants

- Power modes: `POWER_MODE_VERY_LOW_POWER`, `POWER_MODE_LOW_POWER`, `POWER_MODE_HIGH_RESOLUTION`
- Oversampling: `OSR_128` through `OSR_16384`
- Channel PGA gains: `CHANNEL_PGA_1` … `CHANNEL_PGA_128`
- Input multiplexer: `INPUT_CHANNEL_MUX_DIFF_PAIR`, `INPUT_CHANNEL_MUX_INPUT_SHORTED`, `INPUT_CHANNEL_MUX_POSITIVE_DC_TEST_SIGNAL`, `INPUT_CHANNEL_MUX_NEGATIVE_DC_TEST_SIGNAL`
- DRDY state: `DRDY_STATE_LOGIC_HIGH`, `DRDY_STATE_HI_Z`
- SPI dummy words: `SPI_MASTER_DUMMY`, `SPI_MASTER_DUMMY16`, `SPI_MASTER_DUMMY32`

## Usage Examples

### ADS131M04 (four channels, ESP32 driving CLKIN)

```cpp
#include <Arduino.h>
#include "ADS131M04.h"

constexpr uint8_t PIN_SCK   = 14;
constexpr uint8_t PIN_MISO  = 12;
constexpr uint8_t PIN_MOSI  = 13;
constexpr uint8_t PIN_CS    = 5;
constexpr uint8_t PIN_DRDY  = 19;
constexpr uint8_t PIN_RESET = 4;
constexpr uint8_t PIN_CLKIN = 27;
constexpr uint8_t CLKIN_CH  = 0;
constexpr uint32_t CLKIN_HZ = 1'000'000;

ADS131M04 adc;

void setup() {
  Serial.begin(115200);
  adc.reset(PIN_RESET);
  adc.setClockSpeed(2'000'000);
  adc.begin(&SPI, PIN_SCK, PIN_MISO, PIN_MOSI,
            PIN_CS, PIN_DRDY, PIN_CLKIN, CLKIN_HZ, CLKIN_CH);

  adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
  adc.setOsr(OSR_1024);

  for (uint8_t ch = 0; ch < 4; ++ch) {
    adc.setChannelEnable(ch, 1);
    adc.setChannelPGA(ch, CHANNEL_PGA_1);
    adc.setInputChannelSelection(ch, INPUT_CHANNEL_MUX_DIFF_PAIR);
  }
}

void loop() {
  if (!adc.isDataReady()) {
    return;
  }
  adcOutput res = adc.readADC();
  Serial.printf("CH0=%ld\tCH1=%ld\tCH2=%ld\tCH3=%ld\n",
                res.ch0, res.ch1, res.ch2, res.ch3);
}
```

### ADS131M02 (external CLKIN)

```cpp
#include <Arduino.h>
#include "ADS131M02.h"

ADS131M02 adc;

void setup() {
  Serial.begin(115200);
  adc.setClockSpeed(1'000'000);
  adc.begin(&SPI, 14, 12, 13, 5, 19);
  adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
  adc.setOsr(OSR_2048);
  adc.setChannelEnable(0, 1);
  adc.setChannelEnable(1, 1);
  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DIFF_PAIR);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DIFF_PAIR);
}

void loop() {
  if (adc.isDataReady()) {
    adcOutput res = adc.readADC();
    Serial.printf("CH0=%ld\tCH1=%ld\n", res.ch0, res.ch1);
  }
}
```

## Notes

- Conversion results are 24-bit two’s complement values.
- In high-resolution mode, `fDATA = fCLKIN / (2 × OSR)`.
- When using the LEDC-enabled `begin` overload, ensure the chosen channel is free.
- Consult the TI datasheets for timing, calibration, and noise performance details.

## References

- [ADS131M02 Datasheet](https://www.ti.com/product/ADS131M02)
- [ADS131M04 Datasheet](https://www.ti.com/product/ADS131M04)
- [Project Repository](https://github.com/ChipCE/ADS131M0x)
