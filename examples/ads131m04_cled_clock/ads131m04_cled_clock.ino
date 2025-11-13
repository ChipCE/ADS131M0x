#include <Arduino.h>
#include "ADS131M04.h"

// #include <driver/ledc.h> // for 3.x api

// This sample use Xiao esp32_s3 board
// Use D7 pin and CLED function of esp32 to provide the clock for CLKIN of ADS131M04

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

volatile bool dataReady = false;

void onDRDY()
{
  dataReady = true;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  // setup cled as clock output 2.x api
  ledcSetup(CLKIN_CHANNEL, CLKIN_FREQ, 1);
  ledcAttachPin(CLKIN_PIN, CLKIN_CHANNEL);
  ledcWrite(CLKIN_CHANNEL, 1);

  // setup cled as clock output 3.x api
  // ledc_channel_config_t ledc_channel = {
  //   .gpio_num   = CLKIN_PIN,
  //   .speed_mode = LEDC_LOW_SPEED_MODE,
  //   .channel    = (ledc_channel_t)CLKIN_CHANNEL,
  //   .intr_type  = LEDC_INTR_DISABLE,
  //   .timer_sel  = LEDC_TIMER_0,
  //   .duty       = 1,      // 50% for 1-bit resolution
  //   .hpoint     = 0
  // };
  // ledc_timer_config_t ledc_timer = {
  //   .speed_mode      = LEDC_LOW_SPEED_MODE,
  //   .duty_resolution = LEDC_TIMER_1_BIT,
  //   .timer_num       = LEDC_TIMER_0,
  //   .freq_hz         = CLKIN_FREQ,
  //   .clk_cfg         = LEDC_AUTO_CLK
  // };
  // ledc_timer_config(&ledc_timer);
  // ledc_channel_config(&ledc_channel);


  // Hardware reset
  Serial.println("Resetting ADC...");
  adc.reset(PIN_RESET);
  delay(100);
  adc.setClockSpeed(2000000); // 2 MHz SPI
  adc.begin(&SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_DRDY);
  delay(100);
  Serial.println(adc.isResetStatus());

  // can use adc.isDataReady() or setup interrupt
  // pinMode(PIN_DRDY, INPUT_PULLUP);
  // attachInterrupt(PIN_DRDY, onDRDY, FALLING);

  // Configure ADC
  adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
  adc.setOsr(OSR_512); // 1024K(CLKIN_FREQ) / 2 / 512 = 1000 Hz sample rate

  adc.setChannelEnable(0, 1);
  adc.setChannelPGA(0, CHANNEL_PGA_8);
  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_DIFFERENTIAL_PAIR);

  adc.setChannelEnable(1, 1);
  adc.setChannelPGA(1, CHANNEL_PGA_8);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_DIFFERENTIAL_PAIR);

  adc.setChannelEnable(2, 1);
  adc.setChannelPGA(2, CHANNEL_PGA_8);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_DIFFERENTIAL_PAIR);

  adc.setChannelEnable(3, 1);
  adc.setChannelPGA(3, CHANNEL_PGA_8);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_DIFFERENTIAL_PAIR);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
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
    // float sps = 1000000.0 / lap;
    // Serial.print("SPS:");
    // Serial.println(sps);

    Serial.print("ch0:");
    Serial.print(res.ch0);
    Serial.print(",ch1:");
    Serial.print(res.ch1);
    Serial.print(",ch2:");
    Serial.print(res.ch2);
    Serial.print(",ch3:");
    Serial.print(res.ch3);
    Serial.println("");
  }

  // if (dataReady)
  // {
  //   dataReady = false;
  //   adcOutput res = adc.readADC();
  //   //Serial.println(res.ch0);
  //   unsigned long lap = micros() - lastRead;
  //   lastRead = micros();
  //   // calc sps
  //   // float sps = 1000000.0 / lap;
  //   // Serial.print("SPS:");
  //   // Serial.println(sps);

  //   Serial.print("ch0:");
  //   Serial.print(res.ch0);
  //   Serial.print(",ch1:");
  //   Serial.print(res.ch1);
  //   Serial.print(",ch2:");
  //   Serial.print(res.ch2);
  //   Serial.print(",ch3:");
  //   Serial.print(res.ch3);
  //   Serial.println("");
  // }
}
