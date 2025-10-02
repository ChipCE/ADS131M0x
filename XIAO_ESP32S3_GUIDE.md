# Xiao ESP32S3 Setup Guide for ADS131M0x

## Overview

This guide explains how to use the ADS131M02/M04 ADC with the Seeed Studio Xiao ESP32S3 module.

## Pin Mapping

### Xiao ESP32S3 to ADS131M0x Connection

| Xiao Pin | Function | ADS131M0x Pin | Description |
|----------|----------|---------------|-------------|
| D10 | MOSI | DIN | SPI Master Out Slave In |
| D9 | MISO | DOUT | SPI Master In Slave Out |
| D8 | SCK | SCLK | SPI Clock |
| D1 | CS | CS | Chip Select (active low) |
| D2 | Input | DRDY | Data Ready (goes low when ready) |
| D0 | Output | RESET | Hardware Reset (active low) |
| 3V3 | Power | AVDD, DVDD | 3.3V Power Supply |
| GND | Ground | AVSS, DVSS | Ground |

### Optional: Button for Tare Function

| Xiao Pin | Component | Description |
|----------|-----------|-------------|
| D6 | Button to GND | Tare/Zero button (internal pullup used) |

## Wiring Diagram

```
Xiao ESP32S3          ADS131M0x
============          =========
    D10    ------>    DIN (MOSI)
    D9     <------    DOUT (MISO)
    D8     ------>    SCLK
    D1     ------>    CS
    D2     <------    DRDY
    D0     ------>    RESET
    3V3    ------>    AVDD, DVDD
    GND    ------>    AVSS, DVSS

Optional Button:
    D6     ------>    Button -----> GND
```

## Hardware Notes for Xiao ESP32S3

### Power Supply

- Xiao ESP32S3 outputs 3.3V which is perfect for ADS131M0x
- Ensure stable power supply with decoupling capacitors:
  - 100nF ceramic near each power pin
  - 10µF bulk capacitor

### Pin Selection

The chosen pins are optimal because:
- **D8, D9, D10**: Native SPI pins (HSPI) - best performance
- **D0**: Suitable for reset control
- **D1**: General GPIO for CS
- **D2**: Suitable for interrupt-capable input (DRDY)
- **D6**: General GPIO with built-in pullup capability

### Important: Pin Label Differences

Xiao ESP32S3 labels pins as D0-D10, but these map to specific GPIO numbers:

| Xiao Label | Actual GPIO |
|------------|-------------|
| D0 | GPIO1 |
| D1 | GPIO2 |
| D2 | GPIO3 |
| D6 | GPIO43 |
| D8 | GPIO7 (SCK) |
| D9 | GPIO8 (MISO) |
| D10 | GPIO9 (MOSI) |

The examples use the `D` labels which are automatically handled by the Xiao board definitions.

## Examples

### 1. Basic Example (XiaoESP32S3_Basic.ino)

Simple continuous reading that outputs both raw values and voltages.

**Features:**
- Continuous ADC reading
- Outputs raw ADC values and converted voltages
- Easy to understand and modify

**Usage:**
1. Upload the sketch
2. Open Serial Monitor (115200 baud)
3. View real-time ADC readings

**Output Format:**
```
CH0: 123456 (0.017632V) | CH1: -45678 (-0.006535V)
```

### 2. Tare Function Example (XiaoESP32S3_Tare.ino)

Advanced example with zero/tare functionality for applications like scales or sensors.

**Features:**
- Continuous ADC reading
- Button-triggered tare (zero offset)
- Independent zero point for each channel
- Optimized for Serial Plotter
- Debounced button input
- Averages multiple readings for accurate tare

**Usage:**
1. Upload the sketch
2. Open Serial Plotter (Tools → Serial Plotter) or Serial Monitor
3. Press button on D6 to set current reading as zero
4. All subsequent readings are offset from this zero point

**Use Cases:**
- Weight scales (tare out container weight)
- Pressure sensors (zero at atmospheric pressure)
- Load cells (zero at rest position)
- Strain gauges (zero at neutral position)

## Configuration Options

### Sample Rate Selection

Modify the OSR (OverSampling Ratio) to change sample rate:

```cpp
adc.setOsr(OSR_128);   // ~2000 Hz - Fastest
adc.setOsr(OSR_256);   // ~1000 Hz
adc.setOsr(OSR_512);   // ~500 Hz
adc.setOsr(OSR_1024);  // ~250 Hz - Default, good balance
adc.setOsr(OSR_2048);  // ~125 Hz
adc.setOsr(OSR_4096);  // ~62.5 Hz - Slower, lower noise
adc.setOsr(OSR_8192);  // ~31 Hz
adc.setOsr(OSR_16384); // ~15.6 Hz - Slowest, lowest noise
```

**Trade-off:** Lower OSR = faster sampling but more noise

### PGA Gain Selection

Adjust gain based on your input signal amplitude:

```cpp
adc.setChannelPGA(0, CHANNEL_PGA_1);    // ±1.2V range
adc.setChannelPGA(0, CHANNEL_PGA_2);    // ±0.6V range
adc.setChannelPGA(0, CHANNEL_PGA_4);    // ±0.3V range
adc.setChannelPGA(0, CHANNEL_PGA_8);    // ±0.15V range
adc.setChannelPGA(0, CHANNEL_PGA_16);   // ±75mV range
adc.setChannelPGA(0, CHANNEL_PGA_32);   // ±37.5mV range
adc.setChannelPGA(0, CHANNEL_PGA_64);   // ±18.75mV range
adc.setChannelPGA(0, CHANNEL_PGA_128);  // ±9.375mV range
```

**Rule:** Use highest gain that doesn't clip your signal for best resolution.

### SPI Clock Speed

Adjust SPI clock speed if needed:

```cpp
adc.setClockSpeed(1000000);  // 1 MHz - Safe, works everywhere
adc.setClockSpeed(2000000);  // 2 MHz - Default for examples
adc.setClockSpeed(4000000);  // 4 MHz - Faster
adc.setClockSpeed(8000000);  // 8 MHz - Maximum
```

**Note:** Higher SPI clock may require shorter wires and better signal integrity.

## Voltage Conversion

The ADC returns 24-bit signed integers. Convert to voltage:

```cpp
// For PGA = 1, Vref = 1.2V (internal reference)
float voltage = (adc_value / 8388608.0) * 1.2;

// For other PGA gains, divide by gain:
float voltage = (adc_value / 8388608.0) * (1.2 / pga_gain);
```

Where:
- `8388608` = 2^23 (maximum positive value for 24-bit signed)
- `1.2` = internal reference voltage
- `pga_gain` = 1, 2, 4, 8, 16, 32, 64, or 128

## Troubleshooting

### No data / DRDY stays high

1. Check wiring, especially:
   - CS pin connection
   - DRDY pin connection
   - Power and ground
2. Verify 3.3V power supply is stable
3. Add Serial.println() in code to check if setup completes
4. Try hardware reset: Press reset button on Xiao

### Noisy readings

1. Add decoupling capacitors (100nF + 10µF) near ADC power pins
2. Use shorter wires
3. Increase OSR: `adc.setOsr(OSR_4096);`
4. Add input filtering
5. Keep analog wires away from digital signals

### Button doesn't trigger tare

1. Verify button wiring (D6 to GND)
2. Check Serial Monitor for "TARE TRIGGERED" message
3. Ensure debounceDelay is appropriate (default 300ms)
4. Test button with simple digitalWrite test

### Wrong voltage readings

1. Check PGA gain setting matches expected input range
2. Verify input signal is within ADC range
3. Check reference voltage (1.2V internal)
4. Ensure proper differential input connections (xP and xN)

### Compilation errors

1. Ensure ADS131M0x library is installed
2. Verify board is set to "XIAO_ESP32S3" in Arduino IDE
3. Check `IS_M02` define in ADS131M0x.h matches your hardware:
   - For ADS131M02: Keep `#define IS_M02`
   - For ADS131M04: Comment out `#define IS_M02`

## Serial Plotter Usage

For XiaoESP32S3_Tare.ino:

1. Upload sketch
2. Go to **Tools → Serial Plotter**
3. Set baud rate to **115200**
4. You should see live graphs of all channels
5. Press tare button - all traces should jump to zero

**Tips:**
- Serial Plotter shows space-separated values as different traces
- Each channel appears as a different colored line
- Use tare to zero out DC offsets for better visualization

## Advanced: Using with Multiple ADCs

If you need to use multiple ADS131M0x devices:

1. Use separate CS pins for each device
2. Share MOSI, MISO, SCK, DRDY across all devices
3. Use different CS pins to select which device to talk to
4. Consider using external clock to synchronize sampling

## Performance Notes

With Xiao ESP32S3:
- **Maximum sample rate:** ~2 kSPS per channel (OSR_128)
- **Typical use:** 250-1000 Hz per channel
- **SPI overhead:** Minimal with chosen pins (native HSPI)
- **Button debounce:** 300ms default (adjustable)

## Example Applications

1. **Multi-channel data logger** - Record from multiple sensors
2. **Digital scale** - Use tare function for zeroing
3. **Vibration monitor** - High-speed sampling with low OSR
4. **Temperature measurement** - Slow sampling with high OSR for precision
5. **Pressure sensor array** - Multiple independent channels

## References

- [Xiao ESP32S3 Wiki](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [ADS131M02 Datasheet](https://www.ti.com/product/ADS131M02)
- [ADS131M04 Datasheet](https://www.ti.com/product/ADS131M04)
- [ADS131M0x API Documentation](API.md)
