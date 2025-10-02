# ADS131M0x API Documentation

## Table of Contents
- [Overview](#overview)
- [Class: ADS131M0x](#class-ads131m0x)
- [Data Structures](#data-structures)
- [Constants and Definitions](#constants-and-definitions)
- [Public Methods](#public-methods)
- [Usage Examples](#usage-examples)

## Overview

The ADS131M0x library provides an Arduino interface for the Texas Instruments ADS131M02 (2-channel) and ADS131M04 (4-channel) 24-bit analog-to-digital converters with SPI communication.

**Supported Devices:**
- ADS131M02 (2 channels)
- ADS131M04 (4 channels)

**Tested Platforms:**
- ESP32
- ESP32-S2
- ESP32-S3
- ESP32-C3

## Data Structures

### adcOutput

Structure containing ADC conversion results for all channels.

```cpp
struct adcOutput {
    uint16_t status;  // Status register value
    int32_t ch0;      // Channel 0 reading (24-bit signed)
    int32_t ch1;      // Channel 1 reading (24-bit signed)
    int32_t ch2;      // Channel 2 reading (24-bit signed, M04 only)
    int32_t ch3;      // Channel 3 reading (24-bit signed, M04 only)
};
```

## Class: ADS131M0x

### Constructor

```cpp
ADS131M0x()
```

Creates an instance of the ADS131M0x class.

## Public Methods

### Initialization Methods

#### begin()

```cpp
void begin(SPIClass *port, uint8_t clk_pin, uint8_t miso_pin, 
           uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin)
```

Initialize the ADC with specified SPI pins.

**Parameters:**
- `port` - Pointer to SPIClass object (e.g., &SPI)
- `clk_pin` - SPI clock pin number
- `miso_pin` - SPI MISO (Master In Slave Out) pin number
- `mosi_pin` - SPI MOSI (Master Out Slave In) pin number
- `cs_pin` - Chip select pin number
- `drdy_pin` - Data ready pin number

**Example:**
```cpp
ADS131M0x adc;
adc.begin(&SPI, 14, 12, 13, 5, 19);
```

**Note:** Call `setClockSpeed()` before `begin()` if you need a custom SPI clock speed.

#### setClockSpeed()

```cpp
void setClockSpeed(uint32_t cspeed)
```

Set the SPI clock speed. Must be called before `begin()`.

**Parameters:**
- `cspeed` - SPI clock speed in Hz (default: 1 MHz)

**Example:**
```cpp
adc.setClockSpeed(2000000); // 2 MHz
adc.begin(&SPI, 14, 12, 13, 5, 19);
```

#### reset()

```cpp
void reset(uint8_t reset_pin)
```

Perform hardware reset using the reset pin (active low).

**Parameters:**
- `reset_pin` - Pin number connected to ADC reset

**Example:**
```cpp
adc.reset(4);
```

#### resetDevice()

```cpp
bool resetDevice(void)
```

Reset the device via software command.

**Returns:**
- `true` if device responded with reset confirmation
- `false` if reset failed

**Example:**
```cpp
if (adc.resetDevice()) {
    Serial.println("Reset successful");
}
```

### Configuration Methods

#### setPowerMode()

```cpp
bool setPowerMode(uint8_t powerMode)
```

Set the power mode of the ADC.

**Parameters:**
- `powerMode` - Power mode selection:
  - `POWER_MODE_VERY_LOW_POWER` (0)
  - `POWER_MODE_LOW_POWER` (1)
  - `POWER_MODE_HIGH_RESOLUTION` (2) - Default

**Returns:**
- `true` on success
- `false` if invalid parameter

**Example:**
```cpp
adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
```

#### setOsr()

```cpp
bool setOsr(uint16_t osr)
```

Set the oversampling ratio for the digital filter.

**Parameters:**
- `osr` - Oversampling ratio:
  - `OSR_128` (0)
  - `OSR_256` (1)
  - `OSR_512` (2)
  - `OSR_1024` (3) - Default
  - `OSR_2048` (4)
  - `OSR_4096` (5)
  - `OSR_8192` (6)
  - `OSR_16384` (7)

**Returns:**
- `true` on success
- `false` if invalid parameter

**Example:**
```cpp
adc.setOsr(OSR_128); // 32KSPS with 8MHz clock
```

#### setChannelEnable()

```cpp
bool setChannelEnable(uint8_t channel, uint16_t enable)
```

Enable or disable a specific input channel.

**Parameters:**
- `channel` - Channel number (0-3 for M04, 0-1 for M02)
- `enable` - 1 to enable, 0 to disable

**Returns:**
- `true` on success
- `false` if invalid channel

**Example:**
```cpp
adc.setChannelEnable(0, 1); // Enable channel 0
adc.setChannelEnable(2, 0); // Disable channel 2
```

#### setChannelPGA()

```cpp
bool setChannelPGA(uint8_t channel, uint16_t pga)
```

Set the programmable gain amplifier (PGA) for a channel.

**Parameters:**
- `channel` - Channel number (0-3 for M04, 0-1 for M02)
- `pga` - PGA gain setting:
  - `CHANNEL_PGA_1` (0) - Gain = 1
  - `CHANNEL_PGA_2` (1) - Gain = 2
  - `CHANNEL_PGA_4` (2) - Gain = 4
  - `CHANNEL_PGA_8` (3) - Gain = 8
  - `CHANNEL_PGA_16` (4) - Gain = 16
  - `CHANNEL_PGA_32` (5) - Gain = 32
  - `CHANNEL_PGA_64` (6) - Gain = 64
  - `CHANNEL_PGA_128` (7) - Gain = 128

**Returns:**
- `true` on success
- `false` if invalid channel

**Example:**
```cpp
adc.setChannelPGA(0, CHANNEL_PGA_8); // Set channel 0 to gain of 8
```

#### setInputChannelSelection()

```cpp
bool setInputChannelSelection(uint8_t channel, uint8_t input)
```

Configure the input multiplexer for a channel.

**Parameters:**
- `channel` - Channel number (0-3 for M04, 0-1 for M02)
- `input` - Input selection:
  - `INPUT_CHANNEL_MUX_AIN0P_AIN0N` (0) - Normal differential input (default)
  - `INPUT_CHANNEL_MUX_INPUT_SHORTED` (1) - Inputs shorted
  - `INPUT_CHANNEL_MUX_POSITIVE_DC_TEST_SIGNAL` (2) - Positive DC test signal
  - `INPUT_CHANNEL_MUX_NEGATIVE_DC_TEST_SIGNAL` (3) - Negative DC test signal

**Returns:**
- `true` on success
- `false` if invalid channel

**Example:**
```cpp
adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
```

#### setChannelOffsetCalibration()

```cpp
bool setChannelOffsetCalibration(uint8_t channel, int32_t offset)
```

Set offset calibration value for a channel.

**Parameters:**
- `channel` - Channel number (0-3 for M04, 0-1 for M02)
- `offset` - 24-bit signed offset value

**Returns:**
- `true` on success
- `false` if invalid channel

**Example:**
```cpp
adc.setChannelOffsetCalibration(0, 100);
```

#### setChannelGainCalibration()

```cpp
bool setChannelGainCalibration(uint8_t channel, uint32_t gain)
```

Set gain calibration value for a channel.

**Parameters:**
- `channel` - Channel number (0-3 for M04, 0-1 for M02)
- `gain` - 24-bit unsigned gain value

**Returns:**
- `true` on success
- `false` if invalid channel

**Example:**
```cpp
adc.setChannelGainCalibration(0, 0x800000);
```

#### setGlobalChop()

```cpp
void setGlobalChop(uint16_t global_chop)
```

Enable or disable global chop mode.

**Parameters:**
- `global_chop` - 1 to enable, 0 to disable

**Example:**
```cpp
adc.setGlobalChop(1); // Enable global chop
```

#### setGlobalChopDelay()

```cpp
void setGlobalChopDelay(uint16_t delay)
```

Set global chop delay value.

**Parameters:**
- `delay` - Delay value (refer to datasheet for units)

**Example:**
```cpp
adc.setGlobalChopDelay(5);
```

#### setDrdyFormat()

```cpp
bool setDrdyFormat(uint8_t drdyFormat)
```

Set the data ready (DRDY) pin format.

**Parameters:**
- `drdyFormat` - 0 or 1 (refer to datasheet)

**Returns:**
- `true` on success
- `false` if invalid parameter

#### setDrdyStateWhenUnavailable()

```cpp
bool setDrdyStateWhenUnavailable(uint8_t drdyState)
```

Set DRDY pin state when data is unavailable.

**Parameters:**
- `drdyState`:
  - `DRDY_STATE_LOGIC_HIGH` (0) - Default
  - `DRDY_STATE_HI_Z` (1) - High impedance

**Returns:**
- `true` on success
- `false` if invalid parameter

### Data Reading Methods

#### readADC()

```cpp
adcOutput readADC(void)
```

Read all ADC channels and status.

**Returns:**
- `adcOutput` structure containing status and channel readings

**Example:**
```cpp
adcOutput res = adc.readADC();
Serial.print("CH0 = ");
Serial.println(res.ch0);
Serial.print("CH1 = ");
Serial.println(res.ch1);
```

#### readfastCh0()

```cpp
int32_t readfastCh0(void)
```

Fast read of channel 0 only.

**Returns:**
- 24-bit signed value from channel 0

**Example:**
```cpp
int32_t value = adc.readfastCh0();
Serial.println(value);
```

### Status Methods

#### isDataReady()

```cpp
bool isDataReady(void)
```

Check if new ADC data is ready (hardware pin method).

**Returns:**
- `true` if data is ready
- `false` if data is not ready

**Example:**
```cpp
if (adc.isDataReady()) {
    adcOutput res = adc.readADC();
}
```

#### isDataReadySoft()

```cpp
int8_t isDataReadySoft(byte channel)
```

Check if data is ready for a specific channel (software method via register read).

**Parameters:**
- `channel` - Channel number (0-3 for M04, 0-1 for M02)

**Returns:**
- Non-zero if data ready
- 0 if data not ready
- -1 if invalid channel

**Example:**
```cpp
if (adc.isDataReadySoft(0)) {
    // Channel 0 data is ready
}
```

#### isResetStatus()

```cpp
bool isResetStatus(void)
```

Check if the device is in reset state.

**Returns:**
- `true` if in reset state
- `false` otherwise

#### isLockSPI()

```cpp
bool isLockSPI(void)
```

Check if SPI interface is locked.

**Returns:**
- `true` if locked
- `false` if unlocked

#### isResetOK()

```cpp
uint16_t isResetOK(void)
```

Read the reset command response register.

**Returns:**
- `RSP_RESET_OK` (0xFF22 for M02, 0xFF24 for M04) if reset successful
- `RSP_RESET_NOK` (0x0011) if reset failed

## Constants and Definitions

### Power Modes
- `POWER_MODE_VERY_LOW_POWER` - Very low power mode
- `POWER_MODE_LOW_POWER` - Low power mode
- `POWER_MODE_HIGH_RESOLUTION` - High resolution mode (default)

### Oversampling Ratios
- `OSR_128` to `OSR_16384` - Various oversampling ratios

### Channel PGA Gains
- `CHANNEL_PGA_1` to `CHANNEL_PGA_128` - Programmable gain amplifier settings

### Input Channel Multiplexer
- `INPUT_CHANNEL_MUX_AIN0P_AIN0N` - Normal differential input
- `INPUT_CHANNEL_MUX_INPUT_SHORTED` - Inputs shorted together
- `INPUT_CHANNEL_MUX_POSITIVE_DC_TEST_SIGNAL` - Positive DC test
- `INPUT_CHANNEL_MUX_NEGATIVE_DC_TEST_SIGNAL` - Negative DC test

### DRDY States
- `DRDY_STATE_LOGIC_HIGH` - Logic high when unavailable
- `DRDY_STATE_HI_Z` - High impedance when unavailable

## Usage Examples

### Basic Initialization and Reading

```cpp
#include <Arduino.h>
#include <ADS131M0x.h>

ADS131M0x adc;

void setup() {
    Serial.begin(115200);
    
    // Initialize ADC with SPI pins
    adc.begin(&SPI, 14, 12, 13, 5, 19);
    
    // Configure channels
    adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
    adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
    
    // Set channel gains
    adc.setChannelPGA(0, CHANNEL_PGA_1);
    adc.setChannelPGA(1, CHANNEL_PGA_1);
    
    delay(1000);
}

void loop() {
    if (adc.isDataReady()) {
        adcOutput res = adc.readADC();
        
        Serial.print("Status: ");
        Serial.println(res.status, BIN);
        Serial.print("CH0: ");
        Serial.println(res.ch0);
        Serial.print("CH1: ");
        Serial.println(res.ch1);
        
        delay(100);
    }
}
```

### High-Speed Sampling

```cpp
#include <Arduino.h>
#include <ADS131M0x.h>

ADS131M0x adc;

void setup() {
    Serial.begin(115200);
    
    // Set faster SPI clock
    adc.setClockSpeed(8000000); // 8 MHz
    adc.begin(&SPI, 14, 12, 13, 5, 19);
    
    // Configure for high-speed sampling
    adc.setOsr(OSR_128); // 32 KSPS with 8MHz clock
    adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
    
    adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
}

void loop() {
    static unsigned long sampleCount = 0;
    static unsigned long lastTime = 0;
    
    if (adc.isDataReady()) {
        adcOutput res = adc.readADC();
        sampleCount++;
    }
    
    // Print samples per second
    if (millis() - lastTime >= 1000) {
        Serial.print("SPS: ");
        Serial.println(sampleCount);
        sampleCount = 0;
        lastTime = millis();
    }
}
```

### Using Hardware Reset

```cpp
#include <Arduino.h>
#include <ADS131M0x.h>

#define RESET_PIN 4

ADS131M0x adc;

void setup() {
    Serial.begin(115200);
    
    // Hardware reset before initialization
    adc.reset(RESET_PIN);
    
    adc.begin(&SPI, 14, 12, 13, 5, 19);
    
    // Verify reset was successful
    if (adc.isResetStatus()) {
        Serial.println("Device in reset state");
    }
}

void loop() {
    // Your code here
}
```

## Notes

- Always call `setClockSpeed()` before `begin()` if you need a custom SPI clock speed (default is 1 MHz)
- The library uses conditional compilation for M02 vs M04 via the `IS_M02` define in ADS131M0x.h
- All channel values are 24-bit signed integers in two's complement format
- For best performance, use `readfastCh0()` if you only need channel 0
- Refer to the [TI ADS131M02 datasheet](https://www.ti.com/product/ADS131M02) and [ADS131M04 datasheet](https://www.ti.com/product/ADS131M04) for detailed register descriptions and electrical specifications
