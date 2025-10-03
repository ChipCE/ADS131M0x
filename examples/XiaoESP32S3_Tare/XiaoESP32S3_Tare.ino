/*
 * ADS131M0x Example for Xiao ESP32S3
 * 
 * Features:
 * - Reads ADC channels continuously
 * - Outputs to Serial Plotter
 * - Button press sets current reading as "zero" (tare function)
 * - Each channel has independent zero offset
 * 
 * Pin Configuration:
 * - MOSI: D10
 * - MISO: D9
 * - SCK: D8
 * - SYNC/RESET: D0
 * - CS: D1
 * - DRDY: D2
 * - Button: D6 (active low, internal pullup)
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
#define PIN_BUTTON  D6

// ADC instance
ADS131M0x adc;

// Zero offsets for each channel (tare values)
int32_t zeroOffset_ch0 = 0;
int32_t zeroOffset_ch1 = 0;
#ifndef IS_M02
int32_t zeroOffset_ch2 = 0;
int32_t zeroOffset_ch3 = 0;
#endif

// Button debounce variables
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 300; // 300ms debounce

// Button state
bool lastButtonState = HIGH;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=================================");
    Serial.println("ADS131M0x - Xiao ESP32S3");
    Serial.println("Tare Function Example");
    Serial.println("=================================");
    
    // Setup button with internal pullup
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    
    // Hardware reset of ADC
    Serial.println("Performing hardware reset...");
    adc.reset(PIN_RESET);
    delay(100);
    
    // Initialize ADC with custom SPI pins
    Serial.println("Initializing ADC...");
    adc.setClockSpeed(2000000); // 2 MHz SPI clock
    adc.begin(&SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_DRDY);
    
    // Wait for ADC to be ready
    delay(100);
    
    // Configure ADC settings
    Serial.println("Configuring ADC...");
    
    // Set power mode to high resolution
    adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
    
    // Set oversampling ratio (lower OSR = faster sampling)
    adc.setOsr(OSR_1024); // 250 Hz with internal 8.192 MHz clock
    
    // Enable and configure channels
    adc.setChannelEnable(0, 1);
    adc.setChannelEnable(1, 1);
    adc.setChannelPGA(0, CHANNEL_PGA_1);  // Gain = 1
    adc.setChannelPGA(1, CHANNEL_PGA_1);  // Gain = 1
    adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DIFF_PAIR);
    adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DIFF_PAIR);
    adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DIFF_PAIR);
    adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DIFF_PAIR);
    // Configure channels 2 and 3 for ADS131M04
    adc.setChannelEnable(2, 1);
    adc.setChannelEnable(3, 1);
    adc.setChannelPGA(2, CHANNEL_PGA_1);
    adc.setChannelPGA(3, CHANNEL_PGA_1);
    adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DIFF_PAIR);
    adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DIFF_PAIR);
#endif
    
    delay(500);
    
    Serial.println("Setup complete!");
    Serial.println("Press button (D6) to TARE (set current reading as zero)");
    Serial.println("=================================");
    Serial.println();
    
    // Print header for Serial Plotter
#ifdef IS_M02
    Serial.println("CH0 CH1");
#else
    Serial.println("CH0 CH1 CH2 CH3");
#endif
    
    delay(1000);
}

void loop() {
    // Check button for tare function
    checkButton();
    
    // Read ADC when data is ready
    if (adc.isDataReady()) {
        adcOutput res = adc.readADC();
        
        // Apply zero offsets (tare)
        int32_t ch0_zeroed = res.ch0 - zeroOffset_ch0;
        int32_t ch1_zeroed = res.ch1 - zeroOffset_ch1;
        
        // Print to Serial Plotter (space-separated for multiple traces)
        Serial.print(ch0_zeroed);
        Serial.print(" ");
        Serial.print(ch1_zeroed);
        
#ifndef IS_M02
        int32_t ch2_zeroed = res.ch2 - zeroOffset_ch2;
        int32_t ch3_zeroed = res.ch3 - zeroOffset_ch3;
        Serial.print(" ");
        Serial.print(ch2_zeroed);
        Serial.print(" ");
        Serial.print(ch3_zeroed);
#endif
        
        Serial.println();
    }
}

void checkButton() {
    bool buttonState = digitalRead(PIN_BUTTON);
    
    // Detect button press (HIGH to LOW transition with debounce)
    if (buttonState == LOW && lastButtonState == HIGH) {
        unsigned long currentTime = millis();
        
        if (currentTime - lastButtonPress > debounceDelay) {
            lastButtonPress = currentTime;
            performTare();
        }
    }
    
    lastButtonState = buttonState;
}

void performTare() {
    Serial.println();
    Serial.println(">>> TARE TRIGGERED <<<");
    Serial.println("Reading current values...");
    
    // Take multiple readings and average for better accuracy
    const int numReadings = 10;
    int64_t sum_ch0 = 0;
    int64_t sum_ch1 = 0;
#ifndef IS_M02
    int64_t sum_ch2 = 0;
    int64_t sum_ch3 = 0;
#endif
    
    int validReadings = 0;
    
    for (int i = 0; i < numReadings; i++) {
        // Wait for data ready
        unsigned long timeout = millis();
        while (!adc.isDataReady()) {
            if (millis() - timeout > 100) {
                Serial.println("Timeout waiting for data!");
                break;
            }
            delay(1);
        }
        
        if (adc.isDataReady()) {
            adcOutput res = adc.readADC();
            sum_ch0 += res.ch0;
            sum_ch1 += res.ch1;
#ifndef IS_M02
            sum_ch2 += res.ch2;
            sum_ch3 += res.ch3;
#endif
            validReadings++;
            delay(10);
        }
    }
    
    if (validReadings > 0) {
        // Calculate averages as new zero points
        zeroOffset_ch0 = sum_ch0 / validReadings;
        zeroOffset_ch1 = sum_ch1 / validReadings;
#ifndef IS_M02
        zeroOffset_ch2 = sum_ch2 / validReadings;
        zeroOffset_ch3 = sum_ch3 / validReadings;
#endif
        
        Serial.println("New zero offsets set:");
        Serial.print("  CH0: ");
        Serial.println(zeroOffset_ch0);
        Serial.print("  CH1: ");
        Serial.println(zeroOffset_ch1);
#ifndef IS_M02
        Serial.print("  CH2: ");
        Serial.println(zeroOffset_ch2);
        Serial.print("  CH3: ");
        Serial.println(zeroOffset_ch3);
#endif
        Serial.println("All channels zeroed!");
    } else {
        Serial.println("ERROR: Could not read ADC for tare!");
    }
    
    Serial.println("=================================");
    Serial.println();
}
