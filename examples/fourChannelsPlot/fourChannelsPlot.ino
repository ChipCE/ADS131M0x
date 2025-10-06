#include <Arduino.h>
#include "ADS131M04.h"

ADS131M04 adc;

#define PIN_MOSI    D10
#define PIN_MISO    D9
#define PIN_SCK     D8
#define PIN_RESET   D0
#define PIN_CS      D1
#define PIN_DRDY    D2

// use esp32 clock to drive clkin of ADS131M04
#define CLKIN_PIN D7
#define CLKIN_FREQ 1024000 // 1024KHz
#define CLKIN_CHANNEL 0

#define PIN_BUTTON D6

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  
  // Hardware reset
  Serial.println("Resetting ADC...");
  adc.reset(PIN_RESET);
  delay(100);
  adc.setClockSpeed(2000000); // 2 MHz SPI
  adc.begin(&SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_DRDY, CLKIN_PIN, CLKIN_FREQ, CLKIN_CHANNEL);
  delay(100);
  Serial.println(adc.isResetStatus());

  uint16_t id = adc.readRegisterRaw(REG_ID);
  uint16_t status = adc.readStatusRegister();
  Serial.print("ID=0x");
  Serial.println(id, HEX);
  Serial.print("STATUS=0b");
  Serial.println(status, BIN);

  // Configure ADC
  adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
  adc.setOsr(OSR_512); // 1024K / 2 / 512 = 1000 Hz sample rate

  adc.setChannelEnable(0, 1);
  adc.setChannelPGA(0, CHANNEL_PGA_8);
  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DIFF_PAIR);

  adc.setChannelEnable(1, 1);
  adc.setChannelPGA(1, CHANNEL_PGA_8);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DIFF_PAIR);

  adc.setChannelEnable(2, 1);
  adc.setChannelPGA(2, CHANNEL_PGA_8);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DIFF_PAIR);

  adc.setChannelEnable(3, 1);
  adc.setChannelPGA(3, CHANNEL_PGA_8);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DIFF_PAIR);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
}


unsigned long lastRead = micros();

void loop()
{
  if (!digitalRead(PIN_BUTTON)){
    while(!adc.isDataReady()); // wait for adc ready
    adcOutput res = adc.readADC();
    adc.setChannelOffsetCalibration(0, res.ch0);
    adc.setChannelOffsetCalibration(1, res.ch1);
    adc.setChannelOffsetCalibration(2, res.ch2);
    adc.setChannelOffsetCalibration(3, res.ch3);
    Serial.println("Calibrated");
    delay(1000);
  }

  if (adc.isDataReady())
  {
    adcOutput res = adc.readADC();
    //Serial.println(res.ch0);
    unsigned long lap = micros() - lastRead;
    lastRead = micros();
    // calc sps
    float sps = 1000000.0 / lap;
    //Serial.print("SPS: ");
    // Serial.println(sps);
    // Serial.print("ch0:");
    // Serial.print(res.ch0);
    // Serial.print(",ch1:");
    // Serial.print(res.ch1);
    // Serial.print(",ch2:");
    // Serial.print(res.ch2);
    // Serial.print(",ch3:");
    // Serial.print(res.ch3);
    // Serial.println("");

    //26382.1 per 1kpa
    Serial.print("ch0:");
    Serial.print(res.ch0 / 26382.1);
    Serial.print(",ch1:");
    Serial.print(res.ch1 / 26382.1);
    Serial.print(",ch2:");
    Serial.print(res.ch2 / 26382.1);
    Serial.print(",ch3:");
    Serial.print(res.ch3 / 26382.1);
    Serial.println("");
  }
}
