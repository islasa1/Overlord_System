/*
 * misc.h
 *
 *  Created on: Feb 17, 2015
 *      Author: Ryan
 */

#ifndef MISC_H_
#define MISC_H_

#include <stdbool.h>
#include <stdint.h>

// LED Colors
#define RED_LED     GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED   GPIO_PIN_3
#define YELLOW_LED  RED_LED   | GREEN_LED
#define MAGENTA_LED RED_LED   | BLUE_LED
#define CYAN_LED    GREEN_LED | BLUE_LED
#define WHITE_LED   RED_LED   | GREEN_LED | BLUE_LED

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

extern bool ButtonsPressed(uint8_t ui8Buttons);
extern void ConfigureUART(void);
extern void SysDelay(uint32_t ui32ms);
extern void FPUInit(void);
extern void LEDClear(void);
extern void LEDInit(void);
extern void LEDOn(uint8_t ui8Color);
extern void LEDOff(uint8_t ui8Color);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* MISC_H_ */
