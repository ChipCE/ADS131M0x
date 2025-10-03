#include "Arduino.h"
#include "ADS131M02.h"
#include "SPI.h"

#ifdef IS_M02
#define DO_PRAGMA(x) _Pragma (#x)
#define INFO(x) DO_PRAGMA(message ("\nREMARK: " #x))
//INFO Version for ADS131M02
#endif

int32_t ADS131M02::val32Ch0 = 0x7FFFFF;

/**
 * @brief Construct a new ADS131M02::ADS131M02 object
 * 
 */
ADS131M02::ADS131M02()
{
}

/**
 * @brief Set SPI speed (call bevor "begin" to change default 2MHz)
 *
 * @param cspeed value in Hz
 */
void ADS131M02::setClockSpeed(uint32_t cspeed)
{
  spiClockSpeed = cspeed;
}

/**
 * @brief Write to ADC131M01 or ADC131M04 register
 * 
 * @param address 
 * @param value 
 * @return uint8_t 
 */
uint8_t ADS131M02::writeRegister(uint8_t address, uint16_t value)
{
  uint16_t res;
  uint8_t addressRcv;
  uint8_t bytesRcv;
  uint16_t cmd = 0;

  digitalWrite(csPin, LOW);
  delayMicroseconds(1);

  cmd = (CMD_WRITE_REG) | (address << 7) | 0;

  //res = spiPort->transfer16(cmd);
  spiPort->transfer16(cmd);
  spiPort->transfer(0x00);

  spiPort->transfer16(value);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);


  res = spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);

  addressRcv = (res & REGMASK_CMD_READ_REG_ADDRESS) >> 7;
  bytesRcv = (res & REGMASK_CMD_READ_REG_BYTES);

  if (addressRcv == address)
  {
    return bytesRcv + 1;
  }
  return 0;
}

/**
 * @brief 
 *
 * @param address
 * @return uint16_t
 */
uint16_t ADS131M02::readRegister(uint8_t address)
{
  uint16_t cmd;
  uint16_t data;

  cmd = CMD_READ_REG | (address << 7 | 0);

  digitalWrite(csPin, LOW);
  delayMicroseconds(1);

  spiPort->transfer16(cmd);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  data = spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  spiPort->transfer16(0x0000);
  spiPort->transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);
  return data;
}

/**
 * @brief Write a value to the register, applying the mask to touch only the necessary bits.
 * It does not carry out the shift of bits (shift), it is necessary to pass the shifted value to the correct position
 *
 * @param address
 * @param value
 * @param mask
 */
void ADS131M02::writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask)
{
  //Read the current content of the register
  uint16_t register_contents = readRegister(address);
  // Change the mask bit by bit (it remains 1 in the bits that must not be touched and 0 in the bits to be modified)
  // An AND is performed with the current content of the record. "0" remain in the part to be modified
  register_contents = register_contents & ~mask;
 // OR is made with the value to load in the registry. value must be in the correct position (shitf)
  register_contents = register_contents | value;
  writeRegister(address, register_contents);
}

/// @brief Hardware reset (reset low activ) 
/// @param reset_pin 
void ADS131M02::reset(uint8_t reset_pin)
{
  pinMode(reset_pin, OUTPUT);
  digitalWrite(reset_pin, HIGH);
  delay(100);
  digitalWrite(reset_pin, LOW);
  delay(100);
  digitalWrite(reset_pin, HIGH);
  delay(1);
}

/**
 * @brief read status cmd-register
 * 
 * @return uint16_t 
 */
uint16_t ADS131M02::isResetOK(void)
{
  return (readRegister(CMD_RESET) );
}

/**
 * @brief basic initialisation,  
 * call '.SetClockSpeed' before to set custom SPI-Clock (default=1MHz), 
 * call '.reset' to make extra hardware-reset (optional)
 *
 * @param port      Pointer to SPIClass object
 * @param clk_pin
 * @param miso_pin
 * @param mosi_pin
 * @param cs_pin
 * @param drdy_pin
 */
void ADS131M02::begin(SPIClass *port, uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin)
{
  // Set pins up
  csPin = cs_pin;
  drdyPin = drdy_pin;
  spiPort = port;
  
  spiPort->begin(clk_pin, miso_pin, mosi_pin, cs_pin); // SCLK, MISO, MOSI, SS
  SPISettings settings(spiClockSpeed, SPI_MSBFIRST, SPI_MODE1);
  spiPort->beginTransaction(settings);
  delay(1);
  
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin,HIGH); // CS HIGH --> not selected
  pinMode(drdyPin, INPUT);  // DRDY Input
}

void ADS131M02::begin(SPIClass *port, uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin, uint8_t clkin_pin, unsigned int clkin_freq, uint8_t clkin_channel)
{
#if defined(ARDUINO_ARCH_ESP32)
  ledcSetup(clkin_channel, clkin_freq, 1);
  ledcAttachPin(clkin_pin, clkin_channel);
  ledcWrite(clkin_channel, 1);
#else
  (void)clkin_pin;
  (void)clkin_freq;
  (void)clkin_channel;
#endif

  begin(port, clk_pin, miso_pin, mosi_pin, cs_pin, drdy_pin);
}

/**
 * @brief software test of ADC data is ready
 * 
 * @param channel 
 * @return int8_t 
 */
int8_t ADS131M02::isDataReadySoft(byte channel)
{
  if (channel == 0)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY0);
  }
  else if (channel == 1)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY1);
  }

 return -1;
}


/**
 * @brief read reset status (see datasheet)
 * 
 * @return true 
 * @return false 
 */
bool ADS131M02::isResetStatus(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_RESET);
}

/**
 * @brief read locked status (see datasheet)
 *
 * @return true
 * @return false
 */
bool ADS131M02::isLockSPI(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_LOCK);
}

/**
 * @brief set DRDY format (see datasheet)
 * 
 * @param drdyFormat 
 * @return true 
 * @return false 
 */
bool ADS131M02::setDrdyFormat(uint8_t drdyFormat)
{
  if (drdyFormat > 1)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_MODE, drdyFormat, REGMASK_MODE_DRDY_FMT);
    return true;
  }
}

/**
 * @brief set DRDY state (see datasheet)
 * 
 * @param drdyState 
 * @return true 
 * @return false 
 */
bool ADS131M02::setDrdyStateWhenUnavailable(uint8_t drdyState)
{
  if (drdyState > 1)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_MODE, drdyState < 1, REGMASK_MODE_DRDY_HiZ);
    return true;
  }
}

/**
 * @brief set power mode (see datasheet)
 * 
 * @param powerMode 
 * @return true 
 * @return false 
 */
bool ADS131M02::setPowerMode(uint8_t powerMode)
{
  if (powerMode > 3)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
    return true;
  }
}

/**
 * @brief set OSR digital filter (see datasheet)
 * 
 * @param osr 
 * @return true 
 * @return false 
 */
bool ADS131M02::setOsr(uint16_t osr)
{
  if (osr > 7)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_CLOCK, osr << 2 , REGMASK_CLOCK_OSR);
    return true;
  }
}

/**
 * @brief input channel enable 
 * 
 * @param channel 
 * @param enable 
 * @return true 
 * @return false 
 */
bool ADS131M02::setChannelEnable(uint8_t channel, uint16_t enable)
{
  if (channel > 3)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN);
    return true;
  }

  return false;
}

/**
 * @brief set gain per channel (see datasheet)
 * 
 * @param channel 
 * @param pga 
 * @return true 
 * @return false 
 */
bool ADS131M02::setChannelPGA(uint8_t channel, uint16_t pga)
{
  if (channel == 0)
  {
    writeRegisterMasked(REG_GAIN, pga, REGMASK_GAIN_PGAGAIN0);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_GAIN, pga << 4, REGMASK_GAIN_PGAGAIN1);
    return true;
  }

  return false;
}

/// @brief Set global Chop (see datasheet)
/// @param global_chop 
void ADS131M02::setGlobalChop(uint16_t global_chop)
{
  writeRegisterMasked(REG_CFG, global_chop << 8, REGMASK_CFG_GC_EN);
}


/// @brief Set global Chop Delay
/// @param delay todo:  ms or us ??
void ADS131M02::setGlobalChopDelay(uint16_t delay)
{
  writeRegisterMasked(REG_CFG, delay << 9, REGMASK_CFG_GC_DLY);
}

bool ADS131M02::setInputChannelSelection(uint8_t channel, uint8_t input)
{
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }

  return false;
}

/// @brief set offset calibration per channel
/// @param channel 
/// @param offset 
/// @return 
bool ADS131M02::setChannelOffsetCalibration(uint8_t channel, int32_t offset)
{

  uint16_t MSB = offset >> 8;
  uint8_t LSB = offset;

 
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }

  return false;
}

/// @brief set gain calibration per channel 
/// @param channel 
/// @param gain 
/// @return 
bool ADS131M02::setChannelGainCalibration(uint8_t channel, uint32_t gain)
{

  uint16_t MSB = gain >> 8;
  uint8_t LSB = gain;

  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }

  return false;
}

/// @brief hardware-pin test if data is ready 
/// @return 
bool ADS131M02::isDataReady()
{
  if (digitalRead(drdyPin) == HIGH)
  {
    return false;
  }
  return true;
}

/// @brief Read only CH0 fast
/// @param  
/// @return ch0 (int32)
int32_t ADS131M02::readfastCh0(void)
{
  uint8_t x = 0;
  uint8_t x2 = 0;
  uint8_t x3 = 0;
  int32_t aux;
  adcOutput res;

  digitalWrite(csPin, LOW);
  //NOP();
  x = spiPort->transfer(0x00);
  x2 = spiPort->transfer(0x00);
  spiPort->transfer(0x00);

  res.status = ((x << 8) | x2);

  // read CH0 --------
  x = spiPort->transfer(0x00);
  x2 = spiPort->transfer(0x00);
  x3 = spiPort->transfer(0x00);
  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    val32Ch0 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    val32Ch0 = aux;
  }
  
  spiPort->write16(0x00);
  spiPort->write(0x00);

  spiPort->write16(0x00);
  spiPort->write(0x00);

  spiPort->write16(0x00);
  spiPort->write(0x00);
  
  /* slower
  spiPort->transfer(0x00);
  spiPort->transfer(0x00);
  spiPort->transfer(0x00);

  spiPort->transfer(0x00);
  spiPort->transfer(0x00);
  spiPort->transfer(0x00);

  spiPort->transfer(0x00);
  spiPort->transfer(0x00);
  spiPort->transfer(0x00);
  */

  //delay(1);
  //NOP();
  digitalWrite(csPin, HIGH);

  return val32Ch0;
}

/// @brief reset device from register, read CAP 8.5.1.10 Commands from official documentation  
/// @param  
/// @return True if the device responded with the RSP_RESET_OK message
bool ADS131M02::resetDevice(void){
  uint8_t x = 0;
  uint8_t x2 = 0;
  uint16_t ris = 0;

  digitalWrite(csPin, LOW);
  #ifndef NO_CS_DELAY
    delayMicroseconds(1);
  #endif

  x = spiPort->transfer(0x00);
  x2 = spiPort->transfer(0x11);
  spiPort->transfer(0x00);

  ris =  ((x << 8) | x2);

  digitalWrite(csPin, HIGH);
  #ifndef NO_CS_DELAY
    delayMicroseconds(1);
  #endif

  if(RSP_RESET_OK == ris){
    return true;
  }
  return false;
}


/// @brief Read ADC port (all Ports)
/// @param  
/// @return 
adcOutput ADS131M02::readADC(void)
{
  uint8_t x = 0;
  uint8_t x2 = 0;
  uint8_t x3 = 0;
  int32_t aux;
  adcOutput res;

  digitalWrite(csPin, LOW);
#ifndef NO_CS_DELAY
  delayMicroseconds(1);
#endif
  x = spiPort->transfer(0x00);
  x2 = spiPort->transfer(0x00);
  spiPort->transfer(0x00);

  res.status = ((x << 8) | x2);

  // read CH0 --------
  x = spiPort->transfer(0x00);
  x2 = spiPort->transfer(0x00);
  x3 = spiPort->transfer(0x00);
  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    res.ch0 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    res.ch0 = aux;
  }

  // faster!!!
  spiPort->transfer(0x00);
  spiPort->transfer(0x00);
  spiPort->transfer(0x00);
  // read CH1 --------
  x = spiPort->transfer(0x00);
  x2 = spiPort->transfer(0x00);
  x3 = spiPort->transfer(0x00);
  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux > 0x7FFFFF)
  {
    res.ch1 = ((~(aux)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    res.ch1 = aux;
  }
  
  spiPort->transfer(0x00);
  spiPort->transfer(0x00);
  spiPort->transfer(0x00);
#ifndef NO_CS_DELAY
  delayMicroseconds(1);
#endif
  digitalWrite(csPin, HIGH);
  return res;
}
