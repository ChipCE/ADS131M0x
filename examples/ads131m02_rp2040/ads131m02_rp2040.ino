#include <Arduino.h>
#include "ADS131M02.h"

// https://github.com/khoih-prog/RP2040_PWM
#include "RP2040_PWM.h"
RP2040_PWM* PWM_Instance;

ADS131M02 adc;

#define CLKIN_PIN D4
#define CLKIN_FREQ 8192000

#define PIN_MOSI    D10
#define PIN_MISO    D9
#define PIN_SCK     D8
#define PIN_RESET   D6
#define PIN_CS      D7
#define PIN_DRDY    D5

unsigned long lastRead = micros();

void setup()
{
  Serial.begin(115200);

  // use RP2040_PWM to generate the clock for ads131m02
  PWM_Instance = new RP2040_PWM(CLKIN_PIN, CLKIN_FREQ, 50);
  PWM_Instance->setPWM();

  adc.begin(&SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_DRDY);

  adc.reset(PIN_RESET);
  delay(100);
  adc.setClockSpeed(2000000); // 2 MHz SPI
  adc.begin(&SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_DRDY);
  delay(100);

  // Configure ADC
  adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
  adc.setOsr(OSR_2048); // 8192K(CLKIN_FREQ) / 2 / 2048 = 2KHz sample rate

  adc.setChannelEnable(0, 1);
  adc.setChannelPGA(0, CHANNEL_PGA_1);
  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DIFFERENTIAL_PAIR);

  adc.setChannelEnable(1, 1);
  adc.setChannelPGA(1, CHANNEL_PGA_1);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DIFFERENTIAL_PAIR);

}

void loop()
{
  if (adc.isDataReady())
  {
    adcOutput res = adc.readADC();
    unsigned long lap = micros() - lastRead;
    lastRead = micros();
    // Serial.print("sps:");
    // Serial.println(1000000 / lap);
    Serial.println(res.ch0);
  }
}
