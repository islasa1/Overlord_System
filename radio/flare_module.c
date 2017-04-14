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
// Global pointer to NRF24L01P instance. Local to this file
//
//*****************************************************************************
static tNRF24L01P *sg_psNRF24L01PInst;

//*****************************************************************************
//
// Global pointer to SPI Master instance. Local to this file
//
//*****************************************************************************
static tSPIMInstance *sg_psSPIMInst;

static void FlareAppCallback(void *pvData, uint_fast8_t ui8Status)
{
    // Do nothing right now
}

void FlareInit()
{
    NRF24L01PInit(sg_psNRF24L01PInst, sg_psSPIMInst, FlareAppCallback, sg_psNRF24L01PInst);

}

void FlareTest_1(void)
{

}
