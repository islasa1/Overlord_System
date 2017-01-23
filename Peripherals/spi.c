/*
 * spi.c
 *
 *  Created on: Dec 25, 2016
 *      Author: Anthony
 */


#include "spi.h"

void SPIInit(tSPIInstance *psInst,
             uint8_t ui8CSPin, uint32_t ui32CSPort,
             uint32_t ui32Base, uint32_t ui32SSIClk,
             uint32_t ui32Protocol, uint32_t ui32Mode,
             uint32_t ui32BitRate, uint32_t ui32DataWidth)
{
    SSIConfigSetExpClk(ui32Base,
                       ui32SSIClk,
                       ui32Protocol,
                       ui32Mode,
                       ui32BitRate,
                       ui32DataWidth);

    psInst->ui8CSPin = ui8CSPin;
    psInst->ui32CSPort = ui32CSPort;
    psInst->ui32Base = ui32Base;
    psInst->ui32Clk = ui32SSIClk;
    psInst->ui32Mode = ui32Mode;
    psInst->ui32BitRate = ui32BitRate;
    psInst->ui32DataWidth = ui32DataWidth;

}

void SPIEnable(tSPIInstance *psInst)
{
    SSIEnable(psInst->ui32Base);
}

void SPIDisable(tSPIInstance *psInst)
{
    SSIDisable(psInst->ui32Base);
}

void SPIWrite(tSPIInstance *psInst, uint32_t ui32Data)
{
    SSIDataPut(psInst->ui32Base, ui32Data);
}
