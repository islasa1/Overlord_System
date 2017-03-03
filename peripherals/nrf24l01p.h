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

#include "spim_drv.h"
#include "hw_nrf24l01p.h"

///********************************************************************************************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
///********************************************************************************************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

///********************************************************************************************************************************************************
//
// The structure that defines the internal state of the nRF24L01+ driver
//
///********************************************************************************************************************************************************
typedef struct
{
    //
    //! The pointer to the SPI master interface instance used to communicate
    //! with the nRF24L01+.
    //
    tSPIMInstance *psSPIInst;

    //
    //! The SPI Chip Select Port Base
    //
    uint32_t ui32CSPort;

    //
    //! The SPI Chip Select Pin
    //
    uint8_t ui8CSPin;

    //
    //! The state of the state machine used while accessing the nRF24L01+
    //
    uint8_t ui8State;

    //
    //! The data buffer used for sending/receiving data to/from the nRF24L01+
    //
    uint8_t pui8Data[32];

    //
    //! The function that is called when the current request has completed
    //! processing
    //
    tSPICallback *pfnCallback;

    //
    //! The callback data provided to the callback function
    //
    void *pvCallbackData;

    union
    {
        //
        //! A buffer used to store the write portion of a register read.
        //
        uint8_t pui8Buffer[8];

        //
        //! The command state
        //
        tSPIMCommand sSPICmd;
    } uCommand;
} tNRF24L01P;

extern uint_fast8_t NRF24L01PInit(tNRF24L01P *psInst, tSPIMInstance *psSPIInst,
                                  uint32_t ui32CSPort, uint8_t ui8CSPin,
                                  tSPICallback *pfnCallback,
                                  void *pvCallbackData);
extern void NRF24L01PSetFrequency(tNRF24L01P *psInst, uint16_t ui16Freq);
extern uint16_t NRF24L01PGetFrequency(tNRF24L01P *psInst);
extern void NRF24L01PSetRFOutputPower(tNRF24L01P *psInst, uint8_t ui8Power);
extern uint8_t NRF24L01PGetRFOutputPower(tNRF24L01P *psInst);
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

///****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
///****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __PERIPHERALS_NRF24L01P_H__
