/*
 * ADS131M0x Basic Example for Xiao ESP32S3
 * 
 * Simple continuous reading example
 * Outputs raw ADC values to Serial Monitor and Serial Plotter
 * 
 * Pin Configuration:
 * - MOSI: D10
 * - MISO: D9
 * - SCK: D8
 * - SYNC/RESET: D0
 * - CS: D1
 * - DRDY: D2
 */

#include <Arduino.h>
#include <ADS131M0x.h>

// Pin definitions for Xiao ESP32S3
#define PIN_MOSI    D10
#define PIN_MISO    D9
#define PIN_SCK     D8
#define PIN_RESET   D0
#define PIN_CS      D1
#define PIN_DRDY    D2

// ADC instance
ADS131M0x adc;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("ADS131M0x - Xiao ESP32S3 Basic Example");
    Serial.println("======================================");
    
    // Hardware reset
    Serial.println("Resetting ADC...");
    adc.reset(PIN_RESET);
    delay(100);
    
    // Initialize ADC
    Serial.println("Initializing ADC...");
    adc.setClockSpeed(2000000); // 2 MHz SPI
    adc.begin(&SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_DRDY);
    
    delay(100);
    
    // Configure ADC
    adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
    adc.setOsr(OSR_1024); // 250 Hz sample rate
    
    // Configure Channel 0
    adc.setChannelEnable(0, 1);
    adc.setChannelPGA(0, CHANNEL_PGA_1);
    adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DIFF_PAIR);
    adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DIFF_PAIR);
    adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DIFF_PAIR);
    adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DIFF_PAIR);
    adc.setChannelPGA(1, CHANNEL_PGA_1);
    adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DIFF_PAIR);
    
#ifndef IS_M02
    // Configure channels 2 and 3 for ADS131M04
    adc.setChannelEnable(2, 1);
    adc.setChannelPGA(2, CHANNEL_PGA_1);
    adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DIFF_PAIR);
    
    adc.setChannelEnable(3, 1);
    adc.setChannelPGA(3, CHANNEL_PGA_1);
    adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DIFF_PAIR);
#endif
    
    delay(500);
    Serial.println("Setup complete!");
    Serial.println("======================================");
    Serial.println();
}

void loop() {
    if (adc.isDataReady()) {
        adcOutput res = adc.readADC();
        
        // Convert to voltage (Vref = 1.2V, PGA = 1)
        float voltage_ch0 = (res.ch0 / 8388608.0) * 1.2;
        float voltage_ch1 = (res.ch1 / 8388608.0) * 1.2;
        
        // Print to Serial Monitor
        Serial.print("CH0: ");
        Serial.print(res.ch0);
        Serial.print(" (");
        Serial.print(voltage_ch0, 6);
        Serial.print("V) | CH1: ");
        Serial.print(res.ch1);
        Serial.print(" (");
        Serial.print(voltage_ch1, 6);
        Serial.print("V)");
        
#ifndef IS_M02
        float voltage_ch2 = (res.ch2 / 8388608.0) * 1.2;
        float voltage_ch3 = (res.ch3 / 8388608.0) * 1.2;
        
        Serial.print(" | CH2: ");
        Serial.print(res.ch2);
        Serial.print(" (");
        Serial.print(voltage_ch2, 6);
        Serial.print("V) | CH3: ");
        Serial.print(res.ch3);
        Serial.print(" (");
        Serial.print(voltage_ch3, 6);
        Serial.print("V)");
#endif
        
        Serial.println();
        
        // Optional: Add delay to slow down output
        // delay(100);
    }
}
