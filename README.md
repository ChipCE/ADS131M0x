
# ADS131M02 and ADS131M04 Arduino Library

Arduino library for the Texas Instruments ADS131M02 (2-channel) and ADS131M04 (4-channel) 24-bit Analog-to-Digital Converters with SPI interface.

## Features

- Support for both ADS131M02 (2-channel) and ADS131M04 (4-channel)
- Flexible SPI port configuration
- Configurable SPI clock speed
- Hardware and software reset support
- Individual channel configuration (PGA, input selection)
- Programmable oversampling ratio (OSR)
- Multiple power modes
- Fast single-channel reading optimized for channel 0
- Offset and gain calibration per channel
- Data ready detection (hardware pin and software register)

## Supported Platforms

This library has been tested with:
- ESP32
- ESP32-S2
- ESP32-S3
- ESP32-C3

Other Arduino-compatible platforms may work but have not been tested.

## Hardware Overview

### ADS131M02 vs ADS131M04

| Feature | ADS131M02 | ADS131M04 |
|---------|-----------|-----------|
| Channels | 2 | 4 |
| Resolution | 24-bit | 24-bit |
| Sample Rate | Up to 64 kSPS | Up to 64 kSPS |
| Interface | SPI | SPI |
| Input Range | ±1.2V (with PGA=1) | ±1.2V (with PGA=1) |
| PGA Gain | 1 to 128 | 1 to 128 |

### Pin Connections

Required connections:
- **SCLK** - SPI clock
- **MISO** - Master In Slave Out (data from ADC)
- **MOSI** - Master Out Slave In (data to ADC)
- **CS** - Chip select (active low)
- **DRDY** - Data ready output (goes low when new data available)
- **RESET** - Hardware reset (optional, active low)
- **VDD/VSS** - Power supply
- **AINxP/AINxN** - Analog input pairs

## Installation

### Arduino IDE

1. Click the **Code** button and select **Download ZIP**
2. In Arduino IDE, go to **Sketch → Include Library → Add .ZIP Library**
3. Select the downloaded ZIP file
4. Restart Arduino IDE

Alternatively, manually install:
1. Download and extract the ZIP file
2. Rename the folder to `ADS131M0x`
3. Move the folder to your Arduino libraries folder (usually `Documents/Arduino/libraries/`)
4. Restart Arduino IDE

### PlatformIO

Add to your `platformio.ini`:

```ini
lib_deps =
    https://github.com/raibisch/ADS131M0x
```

## Quick Start

### Basic Example

```cpp
#include <Arduino.h>
#include <ADS131M0x.h>

ADS131M0x adc;

void setup() {
    Serial.begin(115200);
    
    // Initialize with SPI pins: CLK, MISO, MOSI, CS, DRDY
    adc.begin(&SPI, 14, 12, 13, 5, 19);
    
    // Configure channels
    adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
    adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
    
    delay(1000);
}

void loop() {
    if (adc.isDataReady()) {
        adcOutput res = adc.readADC();
        
        Serial.print("CH0: ");
        Serial.print(res.ch0);
        Serial.print(" | CH1: ");
        Serial.println(res.ch1);
        
        delay(100);
    }
}
```

### Advanced Configuration

```cpp
#include <Arduino.h>
#include <ADS131M0x.h>

ADS131M0x adc;

void setup() {
    Serial.begin(115200);
    
    // Optional: Set custom SPI clock speed (call before begin)
    adc.setClockSpeed(2000000); // 2 MHz
    
    // Initialize
    adc.begin(&SPI, 14, 12, 13, 5, 19);
    
    // Optional: Hardware reset
    // adc.reset(4); // Reset pin 4
    
    // Set power mode
    adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
    
    // Set oversampling ratio
    adc.setOsr(OSR_1024); // Default
    
    // Configure channel 0
    adc.setChannelEnable(0, 1);  // Enable
    adc.setChannelPGA(0, CHANNEL_PGA_8);  // Gain = 8
    adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
    
    // Configure channel 1
    adc.setChannelEnable(1, 1);
    adc.setChannelPGA(1, CHANNEL_PGA_1);  // Gain = 1
    adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
    
    delay(1000);
}

void loop() {
    if (adc.isDataReady()) {
        adcOutput res = adc.readADC();
        
        // Convert to voltage (example for PGA=1, Vref=1.2V)
        float voltage_ch0 = (res.ch0 / 8388608.0) * 1.2;
        float voltage_ch1 = (res.ch1 / 8388608.0) * 1.2;
        
        Serial.print("CH0: ");
        Serial.print(voltage_ch0, 6);
        Serial.print("V | CH1: ");
        Serial.print(voltage_ch1, 6);
        Serial.println("V");
    }
}
```

## Key Configuration Options

### Power Modes

```cpp
adc.setPowerMode(POWER_MODE_VERY_LOW_POWER);  // Lowest power
adc.setPowerMode(POWER_MODE_LOW_POWER);
adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION); // Default, best performance
```

### Oversampling Ratio (OSR)

Controls the trade-off between speed and noise:

```cpp
adc.setOsr(OSR_128);    // Fastest, 32 kSPS with 8MHz clock
adc.setOsr(OSR_256);
adc.setOsr(OSR_512);
adc.setOsr(OSR_1024);   // Default
adc.setOsr(OSR_2048);
adc.setOsr(OSR_4096);
adc.setOsr(OSR_8192);
adc.setOsr(OSR_16384);  // Slowest, lowest noise
```

### Programmable Gain Amplifier (PGA)

```cpp
adc.setChannelPGA(0, CHANNEL_PGA_1);    // Gain = 1 (±1.2V)
adc.setChannelPGA(0, CHANNEL_PGA_2);    // Gain = 2 (±0.6V)
adc.setChannelPGA(0, CHANNEL_PGA_4);    // Gain = 4 (±0.3V)
adc.setChannelPGA(0, CHANNEL_PGA_8);    // Gain = 8 (±0.15V)
// ... up to CHANNEL_PGA_128
```

## Converting ADC Values to Voltage

The ADC returns 24-bit signed integers. To convert to voltage:

```cpp
// Formula: Voltage = (ADC_Value / 2^23) * Vref / PGA_Gain
// Where: 2^23 = 8388608 (max positive value)
//        Vref = 1.2V (internal reference)

float voltage = (adc_value / 8388608.0) * (1.2 / pga_gain);
```

Example:
```cpp
int32_t raw_value = res.ch0;
float pga_gain = 8.0;  // If PGA set to CHANNEL_PGA_8
float voltage = (raw_value / 8388608.0) * (1.2 / pga_gain);
```

## Examples

The library includes several examples in the `examples/` folder:

### General Examples
- **continuousConversion.ino** - Continuous reading of all channels
- **continuousPromReads.ino** - Continuous reading with promiscuous mode
- **testSampleRateSpeed.ino** - Measure actual sample rate
- **simplelog.cpp** - Simple data logging example

### Xiao ESP32S3 Specific Examples
- **XiaoESP32S3_Basic.ino** - Simple continuous reading with voltage conversion
- **XiaoESP32S3_Tare.ino** - Advanced example with button-triggered zero/tare function

For detailed Xiao ESP32S3 setup instructions, see [XIAO_ESP32S3_GUIDE.md](XIAO_ESP32S3_GUIDE.md).

## API Documentation

For complete API documentation, see [API.md](API.md).

## Changes from Original Library

This library is based on [LucasEtchezuri/Arduino-ADS131M04](https://github.com/LucasEtchezuri/Arduino-ADS131M04) with the following improvements:

- Support for both ADS131M02 and ADS131M04 with conditional compilation
- Flexible SPI port selection (not limited to default SPI)
- Configurable SPI clock speed
- Speed optimizations
- Enhanced documentation and examples

## Helpful Links

- [ADS131M02 Datasheet](https://www.ti.com/product/ADS131M02)
- [ADS131M04 Datasheet](https://www.ti.com/product/ADS131M04)
- [TI Product Page](https://www.ti.com/)

## Selecting M02 vs M04

The library uses a compile-time define to select between ADS131M02 and ADS131M04. 

In `ADS131M0x.h`:
```cpp
// Define for 2-channel version ADS131M02
#define IS_M02

// Comment out the above line for ADS131M04 (4-channel version)
```

- **For ADS131M02**: Keep `#define IS_M02` uncommented
- **For ADS131M04**: Comment out or remove `#define IS_M02`

## Troubleshooting

### No data / DRDY never goes low
- Check wiring, especially CS and DRDY pins
- Verify power supply is stable (3.3V or 5V depending on device)
- Ensure clock is configured correctly
- Try hardware reset

### Wrong readings
- Verify PGA gain setting matches your input range
- Check that input is within valid range for selected PGA
- Verify reference voltage
- Consider calibration

### Slow sample rate
- Reduce OSR setting: `adc.setOsr(OSR_128)`
- Increase SPI clock speed: `adc.setClockSpeed(8000000)`
- Use `readfastCh0()` if you only need channel 0

### Compilation errors about channels 2/3
- Make sure `IS_M02` define matches your hardware
- For ADS131M02, keep `#define IS_M02` in ADS131M0x.h
- For ADS131M04, comment out `#define IS_M02`

## License

This library is provided as-is. Please refer to the original repository for license information.

## Contributing

Contributions are welcome! Please submit pull requests or issues on GitHub.

## Credits

- Original work by [LucasEtchezuri](https://github.com/LucasEtchezuri/Arduino-ADS131M04)
- Modifications and enhancements by [raibisch](https://github.com/raibisch)

