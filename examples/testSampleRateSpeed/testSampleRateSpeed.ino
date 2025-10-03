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
#define CLKIN_FREQ 1000000 // 1 MHz
#define CLKIN_CHANNEL 0



void setup()
{
  Serial.begin(115200);
  while (!Serial) {
        // wait for Serial to become active
    }
  delay(1000);
  Serial.println("");

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
  adc.setOsr(OSR_1024); // 250 Hz sample rate

  adc.setChannelEnable(0, 1);
  adc.setChannelPGA(0, CHANNEL_PGA_1);
  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DIFF_PAIR);

  adc.setChannelEnable(1, 1);
  adc.setChannelPGA(1, CHANNEL_PGA_1);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DIFF_PAIR);

  adc.setChannelEnable(2, 1);
  adc.setChannelPGA(2, CHANNEL_PGA_1);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DIFF_PAIR);

  adc.setChannelEnable(3, 1);
  adc.setChannelPGA(3, CHANNEL_PGA_1);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DIFF_PAIR);
}


unsigned long lastRead = micros();

void loop()
{
  if (adc.isDataReady())
  {
      adcOutput res = adc.readADC();
      //Serial.println(res.ch0);
      unsigned long lap = micros() - lastRead;
      lastRead = micros();
      // calc sps
      float sps = 1000000.0 / lap;
      //Serial.print("SPS: ");
      Serial.println(sps);
  }
}
