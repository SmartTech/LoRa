#include "Module.h"

Module::Module() :
	_SPI(&SPI)
{
}

Module::Module(SPIClass* _spi) :
	_SPI(_spi)
{
}

void Module::initModule(int nss, int dio0, int dio1) {
  _nss = nss;
  _dio0 = dio0;
  _dio1 = dio1;
  
  pinMode(_nss, OUTPUT);
  pinMode(_dio0, INPUT);
  pinMode(_dio1, INPUT);
  
  digitalWrite(_nss, HIGH);
  
  _SPI->begin();
}

uint8_t Module::getRegValue(uint8_t reg, uint8_t msb, uint8_t lsb) {
  if((msb > 7) || (lsb > 7) || (lsb > msb)) {
    return(ERR_INVALID_BIT_RANGE);
  }
  
  uint8_t rawValue = readRegister(reg);
  uint8_t maskedValue = rawValue & ((0b11111111 << lsb) & (0b11111111 >> (7 - msb)));
  return(maskedValue);
}

uint8_t Module::readRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t* inBytes) {
  digitalWrite(_nss, LOW);
  _SPI->transfer(reg | SPI_READ);
  for(uint8_t i = 0; i < numBytes; i++) {
    inBytes[i] = _SPI->transfer(reg);
  }
  digitalWrite(_nss, HIGH);
  return(ERR_NONE);
}

uint8_t Module::readRegisterBurstStr(uint8_t reg, uint8_t numBytes, char* inBytes) {
  digitalWrite(_nss, LOW);
  _SPI->transfer(reg | SPI_READ);
  for(uint8_t i = 0; i < numBytes; i++) {
    inBytes[i] = _SPI->transfer(reg);
  }
  digitalWrite(_nss, HIGH);
  return(ERR_NONE);
}

uint8_t Module::readRegister(uint8_t reg) {
  uint8_t inByte;
  digitalWrite(_nss, LOW);
  #ifdef USE_SPI_TRANSACTION
  _SPI->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  #endif
  _SPI->transfer(reg | SPI_READ);
  #ifdef USE_SPI_TRANSACTION
  _SPI->endTransaction();
  #endif
  inByte = _SPI->transfer(0x00);
  digitalWrite(_nss, HIGH);
  return(inByte);
}

uint8_t Module::setRegValue(uint8_t reg, uint8_t value, uint8_t msb, uint8_t lsb) {
  if((msb > 7) || (lsb > 7) || (lsb > msb)) {
    return(ERR_INVALID_BIT_RANGE);
  }
  
  uint8_t currentValue = readRegister(reg);
  uint8_t mask = ~((0b11111111 << (msb + 1)) | (0b11111111 >> (8 - lsb)));
  uint8_t newValue = (currentValue & ~mask) | (value & mask);
  writeRegister(reg, newValue);
  return(ERR_NONE);
}

void Module::writeRegisterBurst(uint8_t reg, uint8_t* data, uint8_t numBytes) {
  digitalWrite(_nss, LOW);
  _SPI->transfer(reg | SPI_WRITE);
  for(uint8_t i = 0; i < numBytes; i++) {
    _SPI->transfer(data[i]);
  }
  digitalWrite(_nss, HIGH);
}

void Module::writeRegisterBurstStr(uint8_t reg, const char* data, uint8_t numBytes) {
  digitalWrite(_nss, LOW);
  _SPI->transfer(reg | SPI_WRITE);
  for(uint8_t i = 0; i < numBytes; i++) {
    _SPI->transfer(data[i]);
  }
  digitalWrite(_nss, HIGH);
}

void Module::writeRegister(uint8_t reg, uint8_t data) {
  digitalWrite(_nss, LOW);
  #ifdef USE_SPI_TRANSACTION
  _SPI->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  #endif
  _SPI->transfer(reg | SPI_WRITE);
  _SPI->transfer(data);
  #ifdef USE_SPI_TRANSACTION
  _SPI->endTransaction();
  #endif
  digitalWrite(_nss, HIGH);
}
