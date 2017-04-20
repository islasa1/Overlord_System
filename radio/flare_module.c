/*
 * flare_module.c
 *
 *  Created on: Jan 22, 2017
 *      Author: Anthony
 */

#include "../peripherals/nrf24l01p.h"

#include "flare_module.h"



//*****************************************************************************
//
// Global structure to NRF24L01P instance. Local to this file
//
//*****************************************************************************
static tNRF24L01P sg_sNRF24L01PInst;

//*****************************************************************************
//
// Global structure to SPI Master instance. Local to this file
//
//*****************************************************************************
static tSPIMInstance sg_sSPIMInst;

static void FlareAppCallback(void *pvData, uint_fast8_t ui8Status)
{
    // Do nothing right now
}

void FlareInit()
{
    NRF24L01PInit(&sg_sNRF24L01PInst, &sg_sSPIMInst, FlareAppCallback,
                  &sg_sNRF24L01PInst);
}

void FlareTest_1(void)
{
    int i;
    NRF24L01PGetFrequency(&sg_sNRF24L01PInst);
    NRF24L01PGetRFOutputPower(&sg_sNRF24L01PInst);
    NRF24L01PGetAirDataRate(&sg_sNRF24L01PInst);
    NRF24L01PGetCrcWidth(&sg_sNRF24L01PInst);
    NFR24L01PGetAddrWidth(&sg_sNRF24L01PInst);
    NRF24L01PGetTxAddress(&sg_sNRF24L01PInst);

    for(i = 0; i < NRF24L01P_PIPE_5; i++)
    {
        NRF24L01PGetRxAddress(&sg_sNRF24L01PInst, i);
        NRF24L01PGetTransferSize(&sg_sNRF24L01PInst, i);
    }

    NRF24L01PGetARC(&sg_sNRF24L01PInst);
    NRF24L01PGetARD(&sg_sNRF24L01PInst);
    NRF24L01PReadStatus(&sg_sNRF24L01PInst);
    NRF24L01PReadFIFOStatus(&sg_sNRF24L01PInst);
}
