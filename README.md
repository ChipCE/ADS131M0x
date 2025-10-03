# ADS131M0x Arduino Library

Arduino support for the Texas Instruments ADS131M02 (dual-channel) and ADS131M04 (quad-channel) 24-bit delta-sigma ADCs with SPI interface.

## Features

- Dedicated classes for ADS131M02 and ADS131M04 devices
- Flexible SPI pin assignment with configurable SPI clock speed
- Optional hardware reset handling
- Individual channel enable, PGA, and input selection per channel
- Programmable oversampling ratio (OSR) and power modes
- Fast channel-0 read helper
- Offset and gain calibration per channel
- DRDY pin handling plus software data-ready checks
- Optional ESP32 LEDC-based CLKIN generation via an extended `begin()` overload

## Supported Devices and Platforms

- **Devices:** ADS131M02 (2 channels) and ADS131M04 (4 channels)
- **Tested MCUs:** ESP32, ESP32-S2, ESP32-S3, ESP32-C3 (other Arduino-compatible boards may work but are untested)

## Installation

### Arduino IDE

1. Download the library ZIP from [https://github.com/ChipCE/ADS131M0x](https://github.com/ChipCE/ADS131M0x).
2. In Arduino IDE, choose **Sketch → Include Library → Add .ZIP Library...**
3. Select the downloaded ZIP and restart the IDE.

Manual install: extract the ZIP, rename the folder to `ADS131M0x`, place it in your Arduino `libraries/` directory, then restart the IDE.

### PlatformIO

Add the repository directly to `lib_deps` in `platformio.ini`:

```ini
lib_deps =
    https://github.com/ChipCE/ADS131M0x
```

## Quick Start

### ADS131M04 Example (ESP32 with CLKIN generation)

```cpp
#include <Arduino.h>
#include "ADS131M04.h"

// SPI and control pins
constexpr uint8_t PIN_SCK   = 14;
constexpr uint8_t PIN_MISO  = 12;
constexpr uint8_t PIN_MOSI  = 13;
constexpr uint8_t PIN_CS    = 5;
constexpr uint8_t PIN_DRDY  = 19;
constexpr uint8_t PIN_RESET = 4;

// Optional: drive CLKIN from ESP32 LEDC
constexpr uint8_t PIN_CLKIN     = 27;
constexpr uint8_t CLKIN_CHANNEL = 0;
constexpr uint32_t CLKIN_FREQ   = 1'000'000; // 1 MHz

ADS131M04 adc;

void setup() {
  Serial.begin(115200);

  // Perform hardware reset if wired
  adc.reset(PIN_RESET);

  adc.setClockSpeed(2'000'000); // 2 MHz SPI

  // begin() with CLKIN parameters (ESP32 LEDC handled internally)
  adc.begin(&SPI, PIN_SCK, PIN_MISO, PIN_MOSI,
            PIN_CS, PIN_DRDY, PIN_CLKIN, CLKIN_FREQ, CLKIN_CHANNEL);

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
  Serial.printf("CH0: %ld\tCH1: %ld\tCH2: %ld\tCH3: %ld\n",
                res.ch0, res.ch1, res.ch2, res.ch3);
}
```

### ADS131M02 Example (external CLKIN)

```cpp
#include <Arduino.h>
#include "ADS131M02.h"

ADS131M02 adc;

void setup() {
  Serial.begin(115200);

  adc.setClockSpeed(1'000'000);
  adc.begin(&SPI, 14, 12, 13, 5, 19); // No CLKIN parameters needed

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
    Serial.printf("CH0: %ld\tCH1: %ld\n", res.ch0, res.ch1);
  }
}
```

## Key Configuration Options

### Power Modes

```cpp
adc.setPowerMode(POWER_MODE_VERY_LOW_POWER);
adc.setPowerMode(POWER_MODE_LOW_POWER);
adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION); // default
```

### Oversampling Ratio (OSR)

```cpp
adc.setOsr(OSR_128);
// ... up to
adc.setOsr(OSR_16384);
```

### Programmable Gain Amplifier (per channel)

```cpp
adc.setChannelPGA(channel, CHANNEL_PGA_1);   // Gain = 1 (±1.2 V)
adc.setChannelPGA(channel, CHANNEL_PGA_8);   // Gain = 8 (±0.15 V)
// ... up to CHANNEL_PGA_128
```

### Input Multiplexer

```cpp
adc.setInputChannelSelection(channel, INPUT_CHANNEL_MUX_DIFF_PAIR);          // External differential pair
adc.setInputChannelSelection(channel, INPUT_CHANNEL_MUX_INPUT_SHORTED);      // Short inputs
adc.setInputChannelSelection(channel, INPUT_CHANNEL_MUX_POSITIVE_DC_TEST_SIGNAL);
adc.setInputChannelSelection(channel, INPUT_CHANNEL_MUX_NEGATIVE_DC_TEST_SIGNAL);
```

## Converting Raw Data to Voltage

The converters return signed 24-bit values.

```cpp
float voltage = (raw / 8'388'608.0f) * (1.2f / pga_gain);
```

`1.2 V` is the internal reference; adjust if you use an external reference.

## Examples

- `continuousConversion` – Continuous reading of all channels
- `continuousPromReads` – Continuous reading with promiscuous mode
- `testSampleRateSpeed` – Measures effective sample period
- `XiaoESP32S3_Basic` – Voltage conversion example for Seeed XIAO ESP32S3
- `XiaoESP32S3_Tare` – Adds tare/zeroing button logic

## API Reference

See [API.md](API.md) for the full class reference.

## Helpful Links

- [ADS131M02 Datasheet](https://www.ti.com/product/ADS131M02)
- [ADS131M04 Datasheet](https://www.ti.com/product/ADS131M04)
- [Project Repository](https://github.com/ChipCE/ADS131M0x)

## Troubleshooting Tips

- **No data / DRDY stays high:** verify wiring (especially CS, DRDY, CLKIN), ensure the clock is present, and confirm power supplies.
- **Unexpected readings:** double-check PGA gain, input range, and reference voltage; consider performing calibration.
- **Sample rate too low:** reduce OSR, increase CLKIN, or raise the SPI clock. The effective rate is `fDATA = fCLKIN / (2 × OSR)` in high-resolution mode.
- **ADS131M02 channel errors:** confirm you instantiate `ADS131M02` (and include `"ADS131M02.h"`) when targeting the dual-channel part.

## Contributing

Contributions are welcome! Please open issues or pull requests at [https://github.com/ChipCE/ADS131M0x](https://github.com/ChipCE/ADS131M0x).

## Credits

- Original work by [Lucas Etchezuri](https://github.com/LucasEtchezuri/Arduino-ADS131M04)
- Subsequent enhancements by [raibisch](https://github.com/raibisch)
- Maintained and extended by [ChipCE](https://github.com/ChipCE)
