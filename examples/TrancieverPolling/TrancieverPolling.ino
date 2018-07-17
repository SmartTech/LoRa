#include <SmartTech_sx1272.h>
#include <EEPROM.h>

#define EEPROM_ADDR     10

#define LORA_SPI         1
#define LORA_NSS       PA0
#define LORA_RST       PA1
#define LORA_VDD       PA2
#define LORA_CTRL      PA3
#define LORA_DIO0      PA4
#define LORA_DIO1      PA5
#define LORA_DIO2      PA6
#define LORA_DIO3      PA7

tRadioDriver* lora = NULL;
SPIClass LoraSPI(LORA_SPI);

const char* localAddr;
const char* remoteAddr;

struct __packed config_t {
  tLoRaSettings settings = DEFAULT_LORA_SETTINGS;
} config;

void setup() {

  Serial.begin(115200);
  
  setupEEPROM();

  lora = RadioDriverInit(&LoraSPI, LORA_NSS, LORA_RST);
  lora->setPinVDD(LORA_VDD);
  lora->setPinCTRL(LORA_CTRL);
  lora->setPinDIO(0, LORA_DIO0);
  lora->setPinDIO(1, LORA_DIO1);
  lora->setPinDIO(2, LORA_DIO2);
  lora->setPinDIO(3, LORA_DIO3);
  lora->setVDD(HIGH);
  lora->setConfig(&config.settings);
  lora->init();
  
  //lora->setLocalAddr(getDeviceID());
  //lora->setRemoteAddr();
  
  lora->startRx();

}

void loop() {
  
  if(lora==NULL) return; // чип не проинициализирован
  
  switch( lora->handle() ) {

      case RF_RX_TIMEOUT : lora->setTxPacket( 0, 10 ); break;
      case RF_TX_TIMEOUT : lora->startRx(); break;
      case RF_RX_DONE    : lora->setTxPacket( 0, 10 ); break;
      case RF_TX_DONE    : lora->startRx(); break;
        
  }

}

const char* getDeviceID()
{
  uint16_t idBase0 = *(uint16 *)(0x1FFFF7E8);
  uint16_t idBase1 = *(uint16 *)(0x1FFFF7E8+0x02);
  uint32_t idBase2 = *(uint32 *)(0x1FFFF7E8+0x04);
  uint32_t idBase3 = *(uint32 *)(0x1FFFF7E8+0x08);
  static char bufDeviceID[30];
  memset(bufDeviceID, 0, sizeof(bufDeviceID));
  sprintf(bufDeviceID, "%04X%04X%08X%08X", idBase0, idBase1, idBase2, idBase3);
  return bufDeviceID;
}

// Инициализация EEPROM (Выделенного участка во FLASH-памяти)
void setupEEPROM() {
  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x400;
  EEPROM.init();
  if(!readEEPROM()) {
    writeEEPROM();
  }
}

// чтение структуры данных настраиваемых параметров
uint8_t readEEPROM() {
  uint8_t ret = true;
  uint16_t data[sizeof(config)/2];
  for(uint8_t i=0; i<sizeof(config)/2; i++) {
    uint16_t status = EEPROM.read(EEPROM_ADDR+i, &data[i]);
    if(data[i] == 0xFFFF) ret = false;
  }
  if(ret) memcpy(&config, &data, sizeof(config));
  return ret;
}

// Запсиь структуры данных настраиваемых параметров
uint8_t writeEEPROM() {
  uint8_t  ret = true;
  uint16_t data[sizeof(config)/2];
  memcpy(&data, &config, sizeof(config));
  EEPROM.format();
  for(uint8_t i=0; i<sizeof(config)/2; i++) {
    uint16_t status = EEPROM.write(EEPROM_ADDR+i, data[i]);
  }
  return ret;
}

// Сброс настраиваемых параметров структуры данных на дефолтные значения
void resetEEPROM() {
  config_t defConfig;
  memcpy(&config, &defConfig, sizeof(config));
  writeEEPROM();
}