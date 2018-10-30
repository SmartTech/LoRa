#ifndef _SMART_LORA_H
#define _SMART_LORA_H

#include <EEPROM.h>
#include <SPI.h>

#include "TypeDef.h"
#include "Packet.h"

#include "Module.h"

#include "SX1272.h"
#include "SX1273.h"

#include "SX1278.h"
#include "SX1276.h"
#include "SX1277.h"
#include "SX1279.h"


class SmartLoRa {
	
  public:
  
    SmartLoRa(Chip ch = CH_SX1278, int nss = 7, float freq = 434.0, Bandwidth bw = BW_125_00_KHZ, SpreadingFactor sf = SF_9, CodingRate cr = CR_4_7, int dio0 = 2, int dio1 = 3);
    SmartLoRa(SPIClass* _spi, Chip ch = CH_SX1278, int nss = 7, float freq = 434.0, Bandwidth bw = BW_125_00_KHZ, SpreadingFactor sf = SF_9, CodingRate cr = CR_4_7, int dio0 = 2, int dio1 = 3);
    
    float dataRate;
    int8_t lastPacketRSSI;
    float lastPacketSNR;
    
    uint8_t  begin(uint16_t addrEeprom = 0);
	uint32_t handle();
    
    uint8_t transmit(Packet& pack);
    uint8_t receive(Packet& pack);
	
	uint8_t transmit(char* data, uint8_t  size);
	uint8_t receive (char* data, uint8_t* size);
	
    //TODO: CAD mode
    
    uint8_t sleep();
    uint8_t standby();
    
    uint8_t setBandwidth(Bandwidth bw);
    uint8_t setSpreadingFactor(SpreadingFactor sf);
    uint8_t setCodingRate(CodingRate cr);
    uint8_t setFrequency(float freq);
	
	void    setAddress(uint8_t addr[8]);
	void    setAddress(const char addr[24]);
	void    setAddress(uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t d4,uint8_t d5,uint8_t d6,uint8_t d7);
  
  
  private:
  
    Module* _mod;
	SPIClass* _SPI;
    
    uint8_t _address[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint16_t _addrEeprom;
    
	uint8_t parseByte(char c);
    void generateLoRaAdress();
};

#endif // _SMART_LORA_H
