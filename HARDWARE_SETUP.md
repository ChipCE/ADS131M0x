# Hardware Setup Guide

## Table of Contents
- [Overview](#overview)
- [Pin Descriptions](#pin-descriptions)
- [Wiring Diagram](#wiring-diagram)
- [Power Supply](#power-supply)
- [Analog Inputs](#analog-inputs)
- [SPI Interface](#spi-interface)
- [Design Considerations](#design-considerations)

## Overview

This guide provides hardware setup instructions for connecting the ADS131M02/M04 ADC to your microcontroller (ESP32, Arduino, etc.).

## Pin Descriptions

### Power Pins

| Pin | Description | Notes |
|-----|-------------|-------|
| AVDD | Analog power supply | Typically 3.3V or 5V |
| DVDD | Digital power supply | Same as AVDD or separate |
| AVSS | Analog ground | Connect to analog ground plane |
| DVSS | Digital ground | Connect to ground |

### SPI Interface Pins

| Pin | Direction | Description | Notes |
|-----|-----------|-------------|-------|
| SCLK | Input | SPI clock | Max 8.192 MHz |
| DIN (MOSI) | Input | Master Out Slave In | Data from MCU to ADC |
| DOUT (MISO) | Output | Master In Slave Out | Data from ADC to MCU |
| CS | Input | Chip Select | Active low |
| DRDY | Output | Data Ready | Goes low when new data available |

### Other Pins

| Pin | Direction | Description | Notes |
|-----|-----------|-------------|-------|
| RESET | Input | Hardware reset | Active low, optional |
| CLKIN | Input | External clock input | Optional, internal clock available |

### Analog Input Pins

**ADS131M02:**
- AIN0P, AIN0N (Channel 0)
- AIN1P, AIN1N (Channel 1)

**ADS131M04:**
- AIN0P, AIN0N (Channel 0)
- AIN1P, AIN1N (Channel 1)
- AIN2P, AIN2N (Channel 2)
- AIN3P, AIN3N (Channel 3)

## Wiring Diagram

### ESP32 to ADS131M02/M04

```
ESP32          ADS131M0x
-----          ---------
GPIO14    -->  SCLK
GPIO12    <--  DOUT (MISO)
GPIO13    -->  DIN (MOSI)
GPIO5     -->  CS
GPIO19    <--  DRDY
GPIO4     -->  RESET (optional)

3.3V      -->  AVDD
3.3V      -->  DVDD
GND       -->  AVSS
GND       -->  DVSS
```

### Typical Arduino to ADS131M02/M04

```
Arduino        ADS131M0x
-------        ---------
D13 (SCK)  -->  SCLK
D12 (MISO) <--  DOUT
D11 (MOSI) -->  DIN
D10 (SS)   -->  CS
D2         <--  DRDY
D3         -->  RESET (optional)

5V or 3.3V -->  AVDD
5V or 3.3V -->  DVDD
GND        -->  AVSS
GND        -->  DVSS
```

**Note:** Pin assignments are configurable in software using the `begin()` method.

## Power Supply

### Supply Voltage

- **Operating Range:** 3.0V to 5.25V
- **Typical:** 3.3V or 5V
- **Analog and Digital:** Can use same supply or separate for noise reduction

### Decoupling Capacitors

Place close to power pins:
- **100nF ceramic** capacitor on each AVDD and DVDD pin
- **10µF tantalum or ceramic** bulk capacitor per supply rail
- Keep traces short and low impedance

### Ground Plane

- Use a solid ground plane for best noise performance
- Separate analog and digital ground planes if possible
- Connect AVSS and DVSS close to the device

## Analog Inputs

### Input Configuration

Each channel has differential inputs (AINxP, AINxN):

```
        +--------+
Signal-->| AINxP  |
        |        |---> To ADC
        | AINxN  |
GND  -->|        |
        +--------+
```

### Input Range

With internal reference (1.2V):

| PGA Gain | Input Range (differential) |
|----------|---------------------------|
| 1        | ±1.2V                     |
| 2        | ±0.6V                     |
| 4        | ±0.3V                     |
| 8        | ±0.15V                    |
| 16       | ±75mV                     |
| 32       | ±37.5mV                   |
| 64       | ±18.75mV                  |
| 128      | ±9.375mV                  |

### Input Protection

- Add series resistors (e.g., 100Ω to 1kΩ) for current limiting
- Add TVS diodes or clamp diodes for overvoltage protection
- Keep input impedance low for best noise performance

### Anti-Aliasing Filter

For signals above Nyquist frequency:

```
Signal --[R]--+---> AINxP
              |
             [C]
              |
GND ----------+
```

Recommended:
- RC filter with cutoff below fs/2
- Example: R=1kΩ, C=100nF → fc ≈ 1.6kHz

## SPI Interface

### SPI Configuration

- **Mode:** SPI Mode 1 (CPOL=0, CPHA=1)
- **Clock Speed:** Up to 8.192 MHz
- **Bit Order:** MSB first
- **Data Width:** 24-bit (3 bytes) per channel

### Clock Speed Selection

Choose based on sample rate requirements:

| Application | SPI Clock | Notes |
|-------------|-----------|-------|
| Low speed | 1 MHz | Default, safe for all setups |
| Medium speed | 2-4 MHz | Good balance |
| High speed | 8 MHz | Maximum throughput |

Set in code:
```cpp
adc.setClockSpeed(2000000); // 2 MHz
```

### CS (Chip Select) Timing

- Pull CS low at least 10ns before first SCLK edge
- Keep CS low during entire transaction
- Pull CS high after last SCLK edge

### DRDY (Data Ready)

- Monitor DRDY to know when new data is available
- DRDY goes from high to low when data ready
- Can use interrupt or polling

Polling example:
```cpp
if (adc.isDataReady()) {
    // Read data
}
```

Interrupt example (ESP32):
```cpp
attachInterrupt(digitalPinToInterrupt(DRDY_PIN), drdyISR, FALLING);
```

## Design Considerations

### PCB Layout Guidelines

1. **Ground Plane**
   - Use solid ground plane under ADC
   - Minimize ground loops
   - Star ground configuration recommended

2. **Power Supply**
   - Place decoupling caps as close as possible to pins
   - Use low-ESR ceramic capacitors
   - Separate analog and digital supplies if possible

3. **Signal Routing**
   - Keep analog traces short and direct
   - Route away from digital signals
   - Use ground guards around sensitive traces
   - Differential pairs should be matched length

4. **SPI Signals**
   - Keep SPI traces short
   - Use series termination resistors if traces are long
   - 22-33Ω series resistors on SCLK, MOSI

### Noise Reduction

1. **Shielding**
   - Shield analog inputs if in noisy environment
   - Use twisted pair or shielded cable for remote sensors

2. **Filtering**
   - Add RC filters on analog inputs
   - Use ferrite beads on power supplies
   - Consider analog and digital supply separation

3. **Software**
   - Use global chop mode for better DC performance
   - Increase OSR for lower noise (at expense of speed)
   - Average multiple readings

### Sample Rate vs SPI Speed

Ensure SPI is fast enough for your sample rate:

```
Minimum SPI Speed ≥ (Sample Rate) × (Channels) × (3 bytes) × (8 bits/byte) × (Safety Factor)
```

Example for 4 channels at 8kSPS:
```
Min SPI = 8000 × 4 × 3 × 8 × 1.5 = 1.152 MHz
Recommended: 2-4 MHz
```

### External Clock

If using external clock on CLKIN:
- Must be stable and low jitter
- Frequency range: 2.048 MHz to 8.192 MHz
- Use crystal oscillator for best performance

Internal clock is adequate for most applications.

## Troubleshooting Hardware Issues

### No Communication

1. Check power supply voltages
2. Verify ground connections
3. Confirm SPI wiring (especially MISO/MOSI swap)
4. Check CS is connected and going low
5. Verify SPI mode (Mode 1)

### DRDY Never Goes Low

1. Check DRDY pin connection
2. Verify power to ADC
3. Send WAKEUP command if in standby
4. Try hardware reset

### Noisy Readings

1. Check ground connections
2. Add/verify decoupling capacitors
3. Reduce SPI clock speed
4. Add input filtering
5. Check for nearby noise sources
6. Verify input is within range for PGA setting

### Unstable Operation

1. Check power supply stability
2. Add more decoupling
3. Verify temperature range
4. Check for signal reflections on long traces
5. Reduce SPI clock speed

## Example Schematics

### Minimal Setup

```
                    +3.3V
                      |
                      |
        +-------------+-------------+
        |             |             |
       [C1]          [C2]          [C3]
       100nF         100nF         10µF
        |             |             |
        |             |             |
        |    ADS131M0x              |
        +----+--------+-------------+
        |AVDD|   DVDD |             |
        |AVSS|   DVSS |             |
        +----+--------+       +-----+-----+
        |  CS  |                    |
        | SCLK |                   GND
        | DIN  |
        | DOUT |
        | DRDY |
        +------+
```

### With Input Protection

```
Signal ---[R1]---+---[R2]---> AINxP
               100Ω    |
                      [C]
                     100nF
                       |
                      GND

R2 + C form anti-aliasing filter
R1 provides input protection
```

## References

- [ADS131M02 Datasheet](https://www.ti.com/product/ADS131M02)
- [ADS131M04 Datasheet](https://www.ti.com/product/ADS131M04)
- TI Application Note: "Layout Guidelines for ADCs"
- TI Application Note: "Noise Analysis for High-Resolution ADCs"
