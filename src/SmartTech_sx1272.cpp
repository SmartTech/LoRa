#include "SmartTech_sx1272.h"

#define UNUSED(x) ((void)(x))

#define PIN_UNCONNECTED     255
#define SX1272_DIO_COUNT      6

uint8_t sx1272_pin_nss  = PIN_UNCONNECTED;
uint8_t sx1272_pin_rst  = PIN_UNCONNECTED;
uint8_t sx1272_pin_vdd  = PIN_UNCONNECTED;
uint8_t sx1272_pin_ctrl = PIN_UNCONNECTED;
uint8_t sx1272_pin_dio[SX1272_DIO_COUNT] = {PIN_UNCONNECTED};

// Default settings for high data rate
tLoRaSettings LoRaSettings = DEFAULT_LORA_SETTINGS;

tRadioDriver RadioDriver;

tRadioDriver* RadioDriverInit( SPIClass* spi, uint8_t nss, uint8_t rst )
{
	SX1272_SPI = spi;
	SX1272_SPI->begin();
	sx1272_pin_nss          = nss;
	sx1272_pin_rst          = rst;
    RadioDriver.init        = SX1272Init;
    RadioDriver.reset       = SX1272Reset;
    RadioDriver.setConfig   = SX1272setConfig;
    RadioDriver.setPinVDD   = SX1272setPinVDD;
    RadioDriver.setPinCTRL  = SX1272setPinCTRL;
    RadioDriver.setPinDIO   = SX1272setPinDIO;
    RadioDriver.setVDD      = SX1272setVDD;
    RadioDriver.startRx     = SX1272LoRaStartRx;
    RadioDriver.getRxPacket = SX1272LoRaGetRxPacket;
    RadioDriver.setTxPacket = SX1272LoRaSetTxPacket;
    RadioDriver.handle      = SX1272LoRaProcess;
    return &RadioDriver;
}

void SX1272setConfig( tLoRaSettings* settings ) {
	memcpy(&LoRaSettings, settings, sizeof(tLoRaSettings));
}

void SX1272setVDD(uint8_t state) {
	if(sx1272_pin_vdd==PIN_UNCONNECTED) return;
	pinMode(sx1272_pin_vdd, OUTPUT);
	digitalWrite(sx1272_pin_vdd, state);
}

void SX1272setPinVDD(uint8_t pin) {
	sx1272_pin_vdd = pin;
}

void SX1272setPinCTRL(uint8_t pin) {
	sx1272_pin_ctrl = pin;
}

void SX1272setPinDIO(uint8_t num, uint8_t pin) {
	if(num >= SX1272_DIO_COUNT) return;
	sx1272_pin_dio[num] = pin;
}

/*!
 * SX1272 registers variable
 */
uint8_t SX1272Regs[0x70];

static bool LoRaOn = false;
static bool LoRaOnState = false;

/*!
 * Frequency hopping frequencies table
 */
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

/*!
 * SX1272 LoRa registers variable
 */
tSX1272LR* SX1272LR;
SPIClass*  SX1272_SPI;

/*!
 * RF state machine variable
 */
static uint8_t RFLRState = RFLR_STATE_IDLE;

/*!
 * Rx management support variables
 */
static int8_t RxPacketSnrEstimate;

/*!
 * Local RF buffer for communication support
 */
static uint8_t RFBuffer[RF_BUFFER_SIZE];

/*!
 * Rx management support variables
 */

/*!
 * PacketTimeout holds the RF packet timeout
 * SyncSize = [0..8]
 * VariableSize = [0;1]
 * AddressSize = [0;1]
 * PayloadSize = [0..RF_BUFFER_SIZE]
 * CrcSize = [0;2]
 * PacketTimeout = ( ( 8 * ( VariableSize + AddressSize + PayloadSize + CrcSize ) / BR ) * 1000.0 ) + 1
 * Computed timeout is in miliseconds
 */
static uint32_t PacketTimeout;

static uint16_t RxPacketSize = 0;
static double RxPacketRssiValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;

/*!
 * Tx management support variables
 */
static uint16_t TxPacketSize = 0;
//static uint32_t TxTimeoutTimer = 0;

void SX1272Init( void )
{
    // Initialize LoRa registers structure
    SX1272LR = (tSX1272LR*)SX1272Regs;
    SX1272InitIo();
    SX1272Reset();
    LoRaOn = true;
    SX1272SetLoRaOn( LoRaOn );
    // Initialize LoRa modem
    SX1272LoRaInit( );
}

void SX1272Reset( void )
{
    SX1272SetReset( RADIO_RESET_ON );  delay(1);
    SX1272SetReset( RADIO_RESET_OFF ); delay(6);
}

void SX1272SetLoRaOn( bool enable )
{
    if( LoRaOnState == enable ) return;
	
    LoRaOnState = enable;
    LoRaOn = enable;

    if( LoRaOn == true )
    {
        SX1272LoRaSetOpMode( RFLR_OPMODE_SLEEP );
        
        SX1272LR->RegOpMode = ( SX1272LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;
        SX1272Write( REG_LR_OPMODE, SX1272LR->RegOpMode );
        
        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );
                                        // RxDone               RxTimeout                   FhssChangeChannel           CadDone
        SX1272LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                        // CadDetected          ModeReady
        SX1272LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1272WriteBuffer( REG_LR_DIOMAPPING1, &SX1272LR->RegDioMapping1, 2 );
        
        SX1272ReadBuffer( REG_LR_OPMODE, SX1272Regs + 1, 0x70 - 1 );
    }
    else
    {
        SX1272LoRaSetOpMode( RFLR_OPMODE_SLEEP );
        
        SX1272LR->RegOpMode = ( SX1272LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
        SX1272Write( REG_LR_OPMODE, SX1272LR->RegOpMode );
        
        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );
        
        SX1272ReadBuffer( REG_OPMODE, SX1272Regs + 1, 0x70 - 1 );
    }
}

bool SX1272GetLoRaOn( void )
{
    return LoRaOn;
}

void SX1272SetOpMode( uint8_t opMode )
{
    SX1272LoRaSetOpMode( opMode );
}

uint8_t SX1272GetOpMode( void )
{
    return SX1272LoRaGetOpMode( );
}

double SX1272ReadRssi( void )
{
    return SX1272LoRaReadRssi( );
}

uint8_t SX1272ReadRxGain( void )
{
    return SX1272LoRaReadRxGain( );
}

uint8_t SX1272GetPacketRxGain( void )
{
    return RxGain;
}

int8_t SX1272GetPacketSnr( void )
{
    return RxPacketSnrEstimate;
}

double SX1272GetPacketRssi( void )
{
    return RxPacketRssiValue;
}

void SX1272InitIo( void )
{
    digitalWrite(sx1272_pin_nss, HIGH);
    pinMode(sx1272_pin_nss, OUTPUT);

    digitalWrite(sx1272_pin_nss, LOW);
    // Set pinMode only, if no dummy set
	if(sx1272_pin_ctrl != PIN_UNCONNECTED) {
		pinMode(sx1272_pin_ctrl, OUTPUT);
	}
	
	for(uint8_t i=0; i<SX1272_DIO_COUNT; i++) {
		if(sx1272_pin_dio[i] != PIN_UNCONNECTED) {
			pinMode(sx1272_pin_dio[i], INPUT);
		}
	}

}

void SX1272SetReset( uint8_t state )
{
	if(sx1272_pin_rst==PIN_UNCONNECTED) return;
    if( state == RADIO_RESET_ON ) {
        pinMode(sx1272_pin_rst, OUTPUT);
        digitalWrite(sx1272_pin_rst, HIGH);
    } else {
        pinMode(sx1272_pin_rst, INPUT);
    }
}

void SX1272HoldReset( void )
{
	if(sx1272_pin_rst==PIN_UNCONNECTED) return;
    pinMode(sx1272_pin_rst, OUTPUT);
    digitalWrite(sx1272_pin_rst, HIGH);
}


void SX1272Write( uint8_t addr, uint8_t data )
{
    SX1272WriteBuffer( addr, &data, 1 );
}

void SX1272Read( uint8_t addr, uint8_t *data )
{
    SX1272ReadBuffer( addr, data, 1 );
}

void SX1272WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    digitalWrite(sx1272_pin_nss, LOW);
    
    SX1272_SPI->transfer( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SX1272_SPI->transfer( buffer[i] );
    }

    //NSS = 1;
    digitalWrite(sx1272_pin_nss, HIGH);
}

void SX1272ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    digitalWrite(sx1272_pin_nss, LOW);
    SX1272_SPI->transfer( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        buffer[i] = SX1272_SPI->transfer( 0 );
    }

    //NSS = 1;
    digitalWrite(sx1272_pin_nss, HIGH);
}

void SX1272WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1272WriteBuffer( 0, buffer, size );
}

void SX1272ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1272ReadBuffer( 0, buffer, size );
}

inline uint8_t SX1272ReadDio0( void )
{
	if(sx1272_pin_dio[0]==PIN_UNCONNECTED) return 0;
    return digitalRead(sx1272_pin_dio[0]);
}

inline uint8_t SX1272ReadDio1( void )
{
	if(sx1272_pin_dio[1]==PIN_UNCONNECTED) return 0;
    return digitalRead(sx1272_pin_dio[1]);
}

inline uint8_t SX1272ReadDio2( void )
{
	if(sx1272_pin_dio[2]==PIN_UNCONNECTED) return 0;
    return digitalRead(sx1272_pin_dio[2]);
}

inline uint8_t SX1272ReadDio3( void )
{
	if(sx1272_pin_dio[3]==PIN_UNCONNECTED) return 0;
    return digitalRead(sx1272_pin_dio[3]);
}

inline uint8_t SX1272ReadDio4( void )
{
	if(sx1272_pin_dio[4]==PIN_UNCONNECTED) return 0;
    return digitalRead(sx1272_pin_dio[4]);
}

inline uint8_t SX1272ReadDio5( void )
{
	if(sx1272_pin_dio[5]==PIN_UNCONNECTED) return 0;
    return digitalRead(sx1272_pin_dio[5]);
}

inline void SX1272WriteRxTx( uint8_t txEnable )
{
	if(sx1272_pin_ctrl==PIN_UNCONNECTED) return;
    digitalWrite(sx1272_pin_ctrl, txEnable);
}

void SX1272LoRaInit( void )
{
    RFLRState = RFLR_STATE_IDLE;

    SX1272LoRaSetDefaults();

    SX1272ReadBuffer( REG_LR_OPMODE, SX1272Regs + 1, 0x70 - 1 );

    SX1272LR->RegPaConfig = ( SX1272LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) |
                            RFLR_PACONFIG_PASELECT_PABOOST;

    SX1272LR->RegLna = RFLR_LNA_GAIN_G1 | RFLR_LNA_BOOST_ON;

    SX1272WriteBuffer( REG_LR_OPMODE, SX1272Regs + 1, 0x70 - 1 );

    // set the RF settings
    SX1272LoRaSetRFFrequency( LoRaSettings.RFFrequency );
    SX1272LoRaSetPa20dBm( true );
    SX1272LoRaSetRFPower( LoRaSettings.Power );
    SX1272LoRaSetSpreadingFactor( LoRaSettings.SpreadingFactor ); // SF6 only operates in implicit header mode.
    SX1272LoRaSetErrorCoding( LoRaSettings.ErrorCoding );
    SX1272LoRaSetPacketCrcOn( LoRaSettings.CrcOn );
    SX1272LoRaSetSignalBandwidth( LoRaSettings.SignalBw );

    SX1272LoRaSetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );
    SX1272LoRaSetSymbTimeout( 0x3FF );
    SX1272LoRaSetPayloadLength( LoRaSettings.PayloadLength );
    SX1272LoRaSetLowDatarateOptimize( true );

    SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );
}

void SX1272LoRaSetDefaults( void )
{
    // REMARK: See SX1272 datasheet for modified default values.

    // Sets IF frequency selection manual
    SX1272LR->RegDetectOptimize = 0x43; // default value 0xC3
    SX1272Write( 0x31, SX1272LR->RegDetectOptimize );

    SX1272Read( REG_LR_VERSION, &SX1272LR->RegVersion );
}

void SX1272LoRaSetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;

    opModePrev = SX1272LR->RegOpMode & ~RFLR_OPMODE_MASK;

    if( opMode != opModePrev )
    {
        if( opMode == RFLR_OPMODE_TRANSMITTER )
        {
            antennaSwitchTxOn = true;
        }
        else
        {
            antennaSwitchTxOn = false;
        }
        if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
            RXTX( antennaSwitchTxOn ); // Antenna switch control
        }
        SX1272LR->RegOpMode = ( SX1272LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;

        SX1272Write( REG_LR_OPMODE, SX1272LR->RegOpMode );
    }
}

uint8_t SX1272LoRaGetOpMode( void )
{
    SX1272Read( REG_LR_OPMODE, &SX1272LR->RegOpMode );

    return SX1272LR->RegOpMode & ~RFLR_OPMODE_MASK;
}

uint8_t SX1272LoRaReadRxGain( void )
{
    SX1272Read( REG_LR_LNA, &SX1272LR->RegLna );
    return( SX1272LR->RegLna >> 5 ) & 0x07;
}

double SX1272LoRaReadRssi( void )
{
    // Reads the RSSI value
    SX1272Read( REG_LR_RSSIVALUE, &SX1272LR->RegRssiValue );

    return RSSI_OFFSET + ( double )SX1272LR->RegRssiValue;
}

void SX1272LoRaStartRx( void )
{
    SX1272LoRaSetRFState( RFLR_STATE_RX_INIT );
}

void SX1272LoRaGetRxPacket( void *buffer, uint16_t *size )
{
    *size = RxPacketSize;
    RxPacketSize = 0;
    memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
}

void SX1272LoRaSetTxPacket( const void *buffer, uint16_t size )
{
    TxPacketSize = size;
    memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize );

    RFLRState = RFLR_STATE_TX_INIT;
}

uint8_t SX1272LoRaGetRFState( void )
{
    return RFLRState;
}

void SX1272LoRaSetRFState( uint8_t state )
{
    RFLRState = state;
}

/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1272 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY,
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint32_t SX1272LoRaProcess( void )
{
    uint32_t result = RF_BUSY;

    switch( RFLRState )
    {
    case RFLR_STATE_IDLE:
        break;
    case RFLR_STATE_RX_INIT:

        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        SX1272LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    //RFLR_IRQFLAGS_RXDONE |
                                    //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1272Write( REG_LR_IRQFLAGSMASK, SX1272LR->RegIrqFlagsMask );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1272LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
            SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1272LR->RegHopPeriod = 255;
        }

        SX1272Write( REG_LR_HOPPERIOD, SX1272LR->RegHopPeriod );

                                    // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
        SX1272LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CadDetected               ModeReady
        SX1272LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1272WriteBuffer( REG_LR_DIOMAPPING1, &SX1272LR->RegDioMapping1, 2 );

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {

            SX1272LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
        else // Rx continuous mode
        {
            SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoRxBaseAddr;
            SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );

            SX1272LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
        }

        memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );

        PacketTimeout = LoRaSettings.RxPacketTimeout;
        RxTimeoutTimer = millis( );
        RFLRState = RFLR_STATE_RX_RUNNING;
        break;
    case RFLR_STATE_RX_RUNNING:

        if( SX1272ReadDio0( ) == 1 ) // RxDone
        {
            RxTimeoutTimer = millis( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
                SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
            RFLRState = RFLR_STATE_RX_DONE;
        }
        if( SX1272ReadDio2( ) == 1 ) // FHSS Changed Channel
        {
            RxTimeoutTimer = millis( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
                SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
            // Debug
            RxGain = SX1272LoRaReadRxGain( );
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            if( ( millis( ) - RxTimeoutTimer ) > PacketTimeout )
            {
                RFLRState = RFLR_STATE_RX_TIMEOUT;
            }
        }
        break;
    case RFLR_STATE_RX_DONE:
        SX1272Read( REG_LR_IRQFLAGS, &SX1272LR->RegIrqFlags );
        if( ( SX1272LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
        {
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );

            if( LoRaSettings.RxSingleOn == true ) // Rx single mode
            {
                RFLRState = RFLR_STATE_RX_INIT;
            }
            else
            {
                RFLRState = RFLR_STATE_RX_RUNNING;
            }
            break;
        }

        {
            uint8_t rxSnrEstimate;
            SX1272Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
            if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
                RxPacketSnrEstimate = -RxPacketSnrEstimate;
            }
            else
            {
                // Divide by 4
                RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
            }
        }

        SX1272Read( REG_LR_PKTRSSIVALUE, &SX1272LR->RegPktRssiValue );

        if( RxPacketSnrEstimate < 0 )
        {
            RxPacketRssiValue = RSSI_OFFSET + ( ( double )SX1272LR->RegPktRssiValue ) + RxPacketSnrEstimate;
        }
        else
        {
            RxPacketRssiValue = RSSI_OFFSET + ( 1.0666 * ( ( double )SX1272LR->RegPktRssiValue ) );
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoRxBaseAddr;
            SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1272LR->RegPayloadLength;
                SX1272ReadFifo( RFBuffer, SX1272LR->RegPayloadLength );
            }
            else
            {
                SX1272Read( REG_LR_NBRXBYTES, &SX1272LR->RegNbRxBytes );
                RxPacketSize = SX1272LR->RegNbRxBytes;
                SX1272ReadFifo( RFBuffer, SX1272LR->RegNbRxBytes );
            }
        }
        else // Rx continuous mode
        {
            SX1272Read( REG_LR_FIFORXCURRENTADDR, &SX1272LR->RegFifoRxCurrentAddr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1272LR->RegPayloadLength;
                SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoRxCurrentAddr;
                SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );
                SX1272ReadFifo( RFBuffer, SX1272LR->RegPayloadLength );
            }
            else
            {
                SX1272Read( REG_LR_NBRXBYTES, &SX1272LR->RegNbRxBytes );
                RxPacketSize = SX1272LR->RegNbRxBytes;
                SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoRxCurrentAddr;
                SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );
                SX1272ReadFifo( RFBuffer, SX1272LR->RegNbRxBytes );
            }
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            RFLRState = RFLR_STATE_RX_INIT;
        }
        else // Rx continuous mode
        {
            RFLRState = RFLR_STATE_RX_RUNNING;
        }
        result = RF_RX_DONE;
        break;
    case RFLR_STATE_RX_TIMEOUT:
        RFLRState = RFLR_STATE_RX_INIT;
        result = RF_RX_TIMEOUT;
        break;
    case RFLR_STATE_TX_INIT:

        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1272LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1272LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
            SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1272LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1272LR->RegHopPeriod = 0;
        }
        SX1272Write( REG_LR_HOPPERIOD, SX1272LR->RegHopPeriod );
        SX1272Write( REG_LR_IRQFLAGSMASK, SX1272LR->RegIrqFlagsMask );

        // Initializes the payload size
        SX1272LR->RegPayloadLength = TxPacketSize;
        SX1272Write( REG_LR_PAYLOADLENGTH, SX1272LR->RegPayloadLength );

        SX1272LR->RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
        SX1272Write( REG_LR_FIFOTXBASEADDR, SX1272LR->RegFifoTxBaseAddr );

        SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoTxBaseAddr;
        SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );

        // Write payload buffer to LORA modem
        SX1272WriteFifo( RFBuffer, SX1272LR->RegPayloadLength );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader
        SX1272LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
        SX1272LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
        SX1272WriteBuffer( REG_LR_DIOMAPPING1, &SX1272LR->RegDioMapping1, 2 );

        SX1272LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );

        RFLRState = RFLR_STATE_TX_RUNNING;
        break;
    case RFLR_STATE_TX_RUNNING:
        if( SX1272ReadDio0( ) == 1 ) // TxDone
        {
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
            RFLRState = RFLR_STATE_TX_DONE;
        }
        if( SX1272ReadDio2( ) == 1 ) // FHSS Changed Channel
        {
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
                SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
        }
        break;
    case RFLR_STATE_TX_DONE:
        // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        RFLRState = RFLR_STATE_IDLE;
        result = RF_TX_DONE;
        break;
    case RFLR_STATE_CAD_INIT:
        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        SX1272LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    //RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                    //RFLR_IRQFLAGS_CADDETECTED;
        SX1272Write( REG_LR_IRQFLAGSMASK, SX1272LR->RegIrqFlagsMask );

                                    // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
        SX1272LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CAD Detected              ModeReady
        SX1272LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1272WriteBuffer( REG_LR_DIOMAPPING1, &SX1272LR->RegDioMapping1, 2 );

        SX1272LoRaSetOpMode( RFLR_OPMODE_CAD );
        RFLRState = RFLR_STATE_CAD_RUNNING;
        break;
    case RFLR_STATE_CAD_RUNNING:
        if( SX1272ReadDio3( ) == 1 ) //CAD Done interrupt
        {
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE  );
            if( SX1272ReadDio4( ) == 1 ) // CAD Detected interrupt
            {
                // Clear Irq
                SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
                // CAD detected, we have a LoRa preamble
                RFLRState = RFLR_STATE_RX_INIT;
                result = RF_CHANNEL_ACTIVITY_DETECTED;
            }
            else
            {
                // The device goes in Standby Mode automatically
                RFLRState = RFLR_STATE_IDLE;
                result = RF_CHANNEL_EMPTY;
            }
        }
        break;

    default:
        break;
    }
    return result;
}

void SX1272LoRaSetRFFrequency( uint32_t freq )
{
    LoRaSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1272LR->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1272LR->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1272LR->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    SX1272WriteBuffer( REG_LR_FRFMSB, &SX1272LR->RegFrfMsb, 3 );
}

uint32_t SX1272LoRaGetRFFrequency( void )
{
    SX1272ReadBuffer( REG_LR_FRFMSB, &SX1272LR->RegFrfMsb, 3 );
    LoRaSettings.RFFrequency = ( ( uint32_t )SX1272LR->RegFrfMsb << 16 ) | ( ( uint32_t )SX1272LR->RegFrfMid << 8 ) | ( ( uint32_t )SX1272LR->RegFrfLsb );
    LoRaSettings.RFFrequency = ( uint32_t )( ( double )LoRaSettings.RFFrequency * ( double )FREQ_STEP );

    return LoRaSettings.RFFrequency;
}

void SX1272LoRaSetRFPower( int8_t power )
{
    SX1272Read( REG_LR_PACONFIG, &SX1272LR->RegPaConfig );
    SX1272Read( REG_LR_PADAC, &SX1272LR->RegPaDac );
    
    if( ( SX1272LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1272LR->RegPaDac & 0x07 ) == 0x07 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1272LR->RegPaConfig = ( SX1272LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            SX1272LR->RegPaConfig = ( SX1272LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        SX1272LR->RegPaConfig = ( SX1272LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1272Write( REG_LR_PACONFIG, SX1272LR->RegPaConfig );
    LoRaSettings.Power = power;
}

int8_t SX1272LoRaGetRFPower( void )
{
    SX1272Read( REG_LR_PACONFIG, &SX1272LR->RegPaConfig );
    SX1272Read( REG_LR_PADAC, &SX1272LR->RegPaDac );

    if( ( SX1272LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1272LR->RegPaDac & 0x07 ) == 0x07 )
        {
            LoRaSettings.Power = 5 + ( SX1272LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
        else
        {
            LoRaSettings.Power = 2 + ( SX1272LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
    }
    else
    {
        LoRaSettings.Power = -1 + ( SX1272LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
    }
    return LoRaSettings.Power;
}

void SX1272LoRaSetSignalBandwidth( uint8_t bw )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 6 );
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
    LoRaSettings.SignalBw = bw;
}

uint8_t SX1272LoRaGetSignalBandwidth( void )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    LoRaSettings.SignalBw = ( SX1272LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_BW_MASK ) >> 6;
    return LoRaSettings.SignalBw;
}

void SX1272LoRaSetSpreadingFactor( uint8_t factor )
{

    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }
    
    if( factor == 6 )
    {
        SX1272LoRaSetNbTrigPeaks( 5 );
    }
    else
    {
        SX1272LoRaSetNbTrigPeaks( 3 );
    }

    SX1272Read( REG_LR_MODEMCONFIG2, &SX1272LR->RegModemConfig2 );    
    SX1272LR->RegModemConfig2 = ( SX1272LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    SX1272Write( REG_LR_MODEMCONFIG2, SX1272LR->RegModemConfig2 );    
    LoRaSettings.SpreadingFactor = factor;
}

uint8_t SX1272LoRaGetSpreadingFactor( void )
{
    SX1272Read( REG_LR_MODEMCONFIG2, &SX1272LR->RegModemConfig2 );   
    LoRaSettings.SpreadingFactor = ( SX1272LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SF_MASK ) >> 4;
    return LoRaSettings.SpreadingFactor;
}

void SX1272LoRaSetErrorCoding( uint8_t value )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 3 );
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = value;
}

uint8_t SX1272LoRaGetErrorCoding( void )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = ( SX1272LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_CODINGRATE_MASK ) >> 3;
    return LoRaSettings.ErrorCoding;
}

void SX1272LoRaSetPacketCrcOn( bool enable )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK ) | ( enable << 1 );
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
    LoRaSettings.CrcOn = enable;
}

void SX1272LoRaSetPreambleLength( uint16_t value )
{
    SX1272ReadBuffer( REG_LR_PREAMBLEMSB, &SX1272LR->RegPreambleMsb, 2 );

    SX1272LR->RegPreambleMsb = ( value >> 8 ) & 0x00FF;
    SX1272LR->RegPreambleLsb = value & 0xFF;
    SX1272WriteBuffer( REG_LR_PREAMBLEMSB, &SX1272LR->RegPreambleMsb, 2 );
}

uint16_t SX1272LoRaGetPreambleLength( void )
{
    SX1272ReadBuffer( REG_LR_PREAMBLEMSB, &SX1272LR->RegPreambleMsb, 2 );
    return ( ( SX1272LR->RegPreambleMsb & 0x00FF ) << 8 ) | SX1272LR->RegPreambleLsb;
}

bool SX1272LoRaGetPacketCrcOn( void )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    LoRaSettings.CrcOn = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_RXPAYLOADCRC_ON ) >> 1;
    return LoRaSettings.CrcOn;
}

void SX1272LoRaSetImplicitHeaderOn( bool enable )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable << 2 );
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = enable;
}

bool SX1272LoRaGetImplicitHeaderOn( void )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_ON ) >> 2;
    return LoRaSettings.ImplicitHeaderOn;
}

void SX1272LoRaSetRxSingleOn( bool enable )
{
    LoRaSettings.RxSingleOn = enable;
}

bool SX1272LoRaGetRxSingleOn( void )
{
    return LoRaSettings.RxSingleOn;
}

void SX1272LoRaSetFreqHopOn( bool enable )
{
    LoRaSettings.FreqHopOn = enable;
}

bool SX1272LoRaGetFreqHopOn( void )
{
    return LoRaSettings.FreqHopOn;
}

void SX1272LoRaSetHopPeriod( uint8_t value )
{
    SX1272LR->RegHopPeriod = value;
    SX1272Write( REG_LR_HOPPERIOD, SX1272LR->RegHopPeriod );
    LoRaSettings.HopPeriod = value;
}

uint8_t SX1272LoRaGetHopPeriod( void )
{
    SX1272Read( REG_LR_HOPPERIOD, &SX1272LR->RegHopPeriod );
    LoRaSettings.HopPeriod = SX1272LR->RegHopPeriod;
    return LoRaSettings.HopPeriod;
}

void SX1272LoRaSetTxPacketTimeout( uint32_t value )
{
    LoRaSettings.TxPacketTimeout = value;
}

uint32_t SX1272LoRaGetTxPacketTimeout( void )
{
    return LoRaSettings.TxPacketTimeout;
}

void SX1272LoRaSetRxPacketTimeout( uint32_t value )
{
    LoRaSettings.RxPacketTimeout = value;
}

uint32_t SX1272LoRaGetRxPacketTimeout( void )
{
    return LoRaSettings.RxPacketTimeout;
}

void SX1272LoRaSetPayloadLength( uint8_t value )
{
    SX1272LR->RegPayloadLength = value;
    SX1272Write( REG_LR_PAYLOADLENGTH, SX1272LR->RegPayloadLength );
    LoRaSettings.PayloadLength = value;
}

uint8_t SX1272LoRaGetPayloadLength( void )
{
    SX1272Read( REG_LR_PAYLOADLENGTH, &SX1272LR->RegPayloadLength );
    LoRaSettings.PayloadLength = SX1272LR->RegPayloadLength;
    return LoRaSettings.PayloadLength;
}

void SX1272LoRaSetPa20dBm( bool enale )
{
    SX1272Read( REG_LR_PADAC, &SX1272LR->RegPaDac );
    
    if( enale == true )
    {
        SX1272LR->RegPaDac = 0x87;
    }
    else
    {
        SX1272LR->RegPaDac = 0x84;
    }
    SX1272Write( REG_LR_PADAC, SX1272LR->RegPaDac );
}

bool SX1272LoRaGetPa20dBm( void )
{
    SX1272Read( REG_LR_PADAC, &SX1272LR->RegPaDac );
    
    return ( ( SX1272LR->RegPaDac & 0x07 ) == 0x07 ) ? true : false;
}

void SX1272LoRaSetPAOutput( uint8_t outputPin )
{
    SX1272Read( REG_LR_PACONFIG, &SX1272LR->RegPaConfig );
    SX1272LR->RegPaConfig = (SX1272LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    SX1272Write( REG_LR_PACONFIG, SX1272LR->RegPaConfig );
}

uint8_t SX1272LoRaGetPAOutput( void )
{
    SX1272Read( REG_LR_PACONFIG, &SX1272LR->RegPaConfig );
    return SX1272LR->RegPaConfig & ~RFLR_PACONFIG_PASELECT_MASK;
}
void SX1272LoRaSetPaRamp( uint8_t value )
{
    SX1272Read( REG_LR_PARAMP, &SX1272LR->RegPaRamp );
    SX1272LR->RegPaRamp = ( SX1272LR->RegPaRamp & RFLR_PARAMP_MASK ) | ( value & ~RFLR_PARAMP_MASK );
    SX1272Write( REG_LR_PARAMP, SX1272LR->RegPaRamp );
}

uint8_t SX1272LoRaGetPaRamp( void )
{
    SX1272Read( REG_LR_PARAMP, &SX1272LR->RegPaRamp );
    return SX1272LR->RegPaRamp & ~RFLR_PARAMP_MASK;
}

void SX1272LoRaSetSymbTimeout( uint16_t value )
{
    SX1272ReadBuffer( REG_LR_MODEMCONFIG2, &SX1272LR->RegModemConfig2, 2 );

    SX1272LR->RegModemConfig2 = ( SX1272LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1272LR->RegSymbTimeoutLsb = value & 0xFF;
    SX1272WriteBuffer( REG_LR_MODEMCONFIG2, &SX1272LR->RegModemConfig2, 2 );
}

uint16_t SX1272LoRaGetSymbTimeout( void )
{
    SX1272ReadBuffer( REG_LR_MODEMCONFIG2, &SX1272LR->RegModemConfig2, 2 );
    return ( ( SX1272LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) << 8 ) | SX1272LR->RegSymbTimeoutLsb;
}

void SX1272LoRaSetLowDatarateOptimize( bool enable )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) | enable;
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
}

bool SX1272LoRaGetLowDatarateOptimize( void )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    return ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_ON );
}

void SX1272LoRaSetNbTrigPeaks( uint8_t value )
{
    SX1272Read( 0x31, &SX1272LR->RegDetectOptimize );
    SX1272LR->RegDetectOptimize = ( SX1272LR->RegDetectOptimize & 0xF8 ) | value;
    SX1272Write( 0x31, SX1272LR->RegDetectOptimize );
}

uint8_t SX1272LoRaGetNbTrigPeaks( void )
{
    SX1272Read( 0x31, &SX1272LR->RegDetectOptimize );
    return ( SX1272LR->RegDetectOptimize & 0x07 );
}
