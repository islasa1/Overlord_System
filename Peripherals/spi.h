/*
 * spi.h
 *
 *  Created on: Dec 25, 2016
 *      Author: Anthony
 */

#ifndef __PERIPHERALS_SPI_H__
#define __PERIPHERALS_SPI_H__

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"

typedef struct
{
    //
    // Chip Select Pin and Port
    //
    uint8_t ui8CSPin;
    uint32_t ui32CSPort;

    //
    // SSI/SPI Base
    //
    uint32_t ui32Base;

    //
    // Rate of the clock supplied
    //
    uint32_t ui32Clk;

    //
    // Mode of operation
    //
    uint32_t ui32Mode;

    //
    // The bit rate
    //
    uint32_t ui32BitRate;

    //
    // Number of bits transferred per frame
    //
    uint32_t ui32DataWidth;

} tSPIInstance;


extern void SPIInit(tSPIInstance *psInst,
                    uint8_t ui8CSPin, uint32_t ui32CSPort,
                    uint32_t ui32Base, uint32_t ui32SSIClk,
                    uint32_t ui32Protocol, uint32_t ui32Mode,
                    uint32_t ui32BitRate, uint32_t ui32DataWidth);

extern void SPIEnable(tSPIInstance *psInst);
extern void SPIDisable(tSPIInstance *psInst);
extern void SPIWrite(tSPIInstance *psInst, uint32_t ui32Data);

#endif /* __PERIPHERALS_SPI_H__ */
