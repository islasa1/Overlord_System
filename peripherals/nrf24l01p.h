//*****************************************************************************
//! nrf24l01p.h - Prototypes for the nRF24L01+ 2.4GHz Transceiver driver.
//
//
//!  Created on: Mar 1, 2017
//!      Author: islasa1
//*****************************************************************************

#ifndef __PERIPHERALS_NRF24L01P_H__
#define __PERIPHERALS_NRF24L01P_H__

#include <stdint.h>
#include <stdbool.h>

#include "spim_drv.h"
#include "hw_nrf24l01p.h"

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Bit field used to store configuration options of nRF24L01+
//
//*****************************************************************************
typedef struct
{
    //
    //! Interrupt not reflected on IRQ pin.
    //
    unsigned int ui1MaskRxDR    : 1;

    //
    //! Interrupt not reflected on IRQ pin.
    //
    unsigned int ui1MaskTxDS    : 1;

    //
    //! Interrupt not reflected on IRQ pin.
    //
    unsigned int ui1MaskMaxRT   : 1;

    //
    //! Enabled CRC.
    //
    unsigned int ui1EnableCRC   : 1;

    //
    //! CRC encoding scheme.
    //! 0 : 1 byte
    //! 1 : 2 bytes
    //
    unsigned int ui1CRCScheme   : 1;

    //
    //! Power up state, or power down state.
    //
    unsigned int ui1PowerUp     : 1;

    //
    //! Device is PRX/PTX (Primary Receiver / Transmitter).
    //
    unsigned int ui1PrimaryRx   : 1;

} tNRF24L01P_CONFIG;

//*****************************************************************************
//
//! Bit field used to store pipe information of nRF24L01+
//
//*****************************************************************************
typedef struct
{
    //
    // Skip bit 1, no identifier
    //
    unsigned int                : 1;

    //
    //! Information for data pipe 5.
    //
    unsigned int ui1Pipe5       : 1;

    //
    //! Information for data pipe 4.
    //
    unsigned int ui1Pipe4       : 1;

    //
    //! Information for data pipe 3.
    //
    unsigned int ui1Pipe3       : 1;

    //
    //! Information for data pipe 2.
    //
    unsigned int ui1Pipe2       : 1;

    //
    //! Information for data pipe 1.
    //
    unsigned int ui1Pipe1       : 1;

    //
    //! Information for data pipe 0.
    //
    unsigned int ui1Pipe0       : 1;

} tNRF24L01P_Pipes;

//*****************************************************************************
//
//! Bit field used to store status information of nRF24L01+
//
//*****************************************************************************
typedef struct
{
    //
    // Skip bit 1, no identifier
    //
    unsigned int                : 1;

    //
    //! Data Ready RX FIFO interrupt.
    //
    unsigned int ui1RxDR        : 1;

    //
    //! Data Sent TX FIFO interrupt.
    //
    unsigned int ui1TxDS        : 1;

    //
    //! Max number of TX retransmits interrupt.
    //
    unsigned int ui1MaxRT       : 1;

    //
    //! Data pipe number for payload available for reading from RX FIFO.
    //
    unsigned int ui1RxPNo       : 3;

    //
    //! TX FIFO full flag.
    //
    unsigned int ui1TxFull      : 1;

} tNRF24L01P_STATUS;



//*****************************************************************************
//
//! The structure that defines the internal state of the nRF24L01+ driver
//
//*****************************************************************************
typedef struct
{
    //
    //! The pointer to the SPI master interface instance used to communicate
    //! with the nRF24L01+.
    //
    tSPIMInstance *psSPIInst;

    //
    //! The SPI Chip Select Port Base.
    //
    uint32_t ui32CSPort;

    //
    //! The SPI Chip Select Pin.
    //
    uint8_t ui8CSPin;

    //
    //! The operational frequency (in MHz).
    //
    uint16_t ui16Freq;

    //
    //! The RF output power when transmitting (in dBm).
    //
    int8_t i8Power;

    //
    //! The operational data rate (in Kbps).
    //
    uint16_t ui16AirDataRate;

    //
    //! Key configuration options of device.
    //
    tNRF24L01P_CONFIG sConfig;

    //
    //! Enabled Auto Acknowledge information per pipe.
    //
    tNRF24L01P_Pipes sEnabledAA;

    //
    //! Enabled RX Addresses.
    //
    tNRF24L01P_Pipes sEnabledRxAddr

    //
    //! The state of the state machine used while accessing the nRF24L01+.
    //
    uint8_t ui8State;

    //
    //! The data buffer used for sending/receiving data to/from the nRF24L01+.
    //
    uint8_t pui8Data[32];

    //
    //! The function that is called when the current request has completed
    //! processing.
    //
    tSPICallback *pfnCallback;

    //
    //! The callback data provided to the callback function.
    //
    void *pvCallbackData;

} tNRF24L01P;

extern uint_fast8_t NRF24L01PInit(tNRF24L01P *psInst, tSPIMInstance *psSPIInst,
                                  uint32_t ui32CSPort, uint8_t ui8CSPin,
                                  uint32_t ui32CEPort, uint8_t ui8CEPin,
                                  tSPICallback *pfnCallback,
                                  void *pvCallbackData);
extern void NRF24L01PSetFrequency(tNRF24L01P *psInst, uint16_t ui16Freq);
extern uint16_t NRF24L01PGetFrequency(tNRF24L01P *psInst);
extern void NRF24L01PSetRFOutputPower(tNRF24L01P *psInst, uint8_t ui8Power);
extern int8_t NRF24L01PGetRFOutputPower(tNRF24L01P *psInst);
extern void NRF24L01PSetAirDataRate(tNRF24L01P *psInst,
                                    uint16_t ui16AirDataRate);
extern int32_t NRF24L01PGetAirDataRate(tNRF24L01P *psInst);
extern void NRF24L01PSetCrcWidth(tNRF24L01P *psInst, uint8_t ui8Width);
extern int32_t NRF24L01PGetCrcWidth(tNRF24L01P *psInst);
extern void NRF24L01PSetRxAddress(tNRF24L01P *psInst, uint64_t ui64Address, uint8_t ui8Width, uint8_t ui8Pipe);
extern void NRF24L01PSetTxAddress(tNRF24L01P *psInst, uint64_t ui64Address, uint8_t ui8Width);
extern uint64_t NRF24L01PGetRxAddress(tNRF24L01P *psInst, uint8_t ui8Pipe);
extern uint64_t NRF24L01PGetTxAddress(tNRF24L01P *psInst);
extern void NRF24L01PSetTransferSize(tNRF24L01P *psInst, uint8_t ui8Size, uint8_t ui8Pipe );
extern int32_t NRF24L01PGetTransferSize(tNRF24L01P *psInst, uint8_t ui8Pipe);
extern bool NRF24L01PGetRPD(tNRF24L01P *psInst);
extern void NRF24L01PSetReceiveMode(tNRF24L01P *psInst);
extern void NRF24L01PSetTransmitMode(tNRF24L01P *psInst);
extern void NRF24L01PPowerUp(tNRF24L01P *psInst);
extern void NRF24L01PPowerDown(tNRF24L01P *psInst);
extern void NRF24L01PEnable(tNRF24L01P *psInst);
extern void NRF24L01PDisable(tNRF24L01P *psInst);
extern int32_t NRF24L01PWrite(tNRF24L01P *psInst, uint8_t ui8Pipe, uint8_t *ui8Data, uint8_t ui8Count);
extern int32_t NRF24L01PRead(tNRF24L01P *psInst, uint8_t ui8Pipe, uint8_t *ui8Data, uint8_t ui8Count);
extern bool NRF24L01PReadable(tNRF24L01P *psInst, uint8_t ui8Pipe);
extern void NRF24L01PDisableAllRxPipes(tNRF24L01P *psInst);
extern void NRF24L01PDisableAutoAcknowledge(tNRF24L01P *psInst);
extern void NRF24L01PEnableAutoAcknowledge(tNRF24L01P *psInst, uint8_t ui8Pipe);
extern void NRF24L01PDisableAutoRetransmit(tNRF24L01P *psInst);
extern void NRF24L01PEnableAutoRetransmit(tNRF24L01P *psInst, uint16_t ui16Delay, uint8_t ui8Count);

//****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __PERIPHERALS_NRF24L01P_H__
