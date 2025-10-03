#include <Arduino.h>
#include "ADS131M04.h"

ADS131M04 adc;

void setup()
{
  Serial.begin(115200);
  adc.begin(14, 12, 13, 5, 19);

  delay(1000);
  Serial.println("");

  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DIFF_PAIR);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DIFF_PAIR);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DIFF_PAIR);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DIFF_PAIR);
}

void loop()
{
  adcOutput res;
  delay(100);

  while (1)
  {
    if (adc.isDataReady())
    {
      res = adc.readADC();

      Serial.print("Status = ");
      Serial.println(res.status, BIN);

      Serial.print("CH0 = ");
      Serial.println(res.ch0);

      Serial.print("CH1 = ");
      Serial.println(res.ch1);
      Serial.print("CH2 = ");
      Serial.println(res.ch2);
      Serial.print("CH3 = ");
      Serial.println(res.ch3);
      Serial.println("");
      delay(500);
    }
  }
}
