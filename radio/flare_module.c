/*
 * flare_module.c
 *
 *  Created on: Jan 22, 2017
 *      Author: Anthony
 */

#include "../../peripherals/nrf24l01p.h"

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

void FlareInit(tNRF24L01P *psNRF24L01PInst, tSPIMInstance *psSPIMInst)
{
    //
    // Configure Instances
    //
    sg_psSPIMInst = psSPIMInst;
    sg_psNRF24L01PInst = psNRF24L01PInst;


}

void FlareTest_1(void)
{

}
