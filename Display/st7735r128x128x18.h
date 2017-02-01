/*
 * st7735rb128x128x18.h
 *
 *  Created on: Jan 30, 2017
 *      Author: Anthony Islas
 */

#ifndef __ST7735RB128x128X18_H__
#define __ST7735RB128x128X18_H__

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "grlib/grlib.h"

//*****************************************************************************
//
// Prototypes for the globals exported by this driver.
//
//*****************************************************************************
extern void ST7735R128x128x18Init(void);
extern const tDisplay g_sST7735R128x128x18;

#endif /* PERIPHERALS_ST7735RB128x128X18_H_ */
