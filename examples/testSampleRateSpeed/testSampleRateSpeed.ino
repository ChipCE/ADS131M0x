#include <Arduino.h>
#include "ADS131M0x.h"

ADS131M0x adc;

#define PIN_MOSI    D10
#define PIN_MISO    D9
#define PIN_SCK     D8
#define PIN_RESET   D0
#define PIN_CS      D1
#define PIN_DRDY    D2


void setup()
{
  Serial.begin(115200);
  //void begin(SPIClass *port, uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin);
  
   

  delay(1000);
  Serial.println("");
  adc.setClockSpeed(1000000); // 2 MHz SPI
  adc.reset(PIN_RESET);
  adc.begin(&SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_DRDY);
  //Serial.println(adc.isResetOK);

  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setOsr(OSR_128);      // 32KSPS  only with 8MHZ clock
 
}


unsigned long lastRead = millis();

void loop()
{
  if ((millis() - lastRead) > 100)
    {
      adcOutput res = adc.readADC();
      lastRead = millis();
      Serial.println(lastRead);
      Serial.println(res.ch0);
    }
}
