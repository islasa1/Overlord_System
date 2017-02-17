/*
 * timer.h
 *
 *  Created on: Nov 8, 2015
 *      Author: Ryan
 */

#ifndef PERIPHERALS_TIMER_H_
#define PERIPHERALS_TIMER_H_

#include <stdbool.h>
#include <stdint.h>

uint32_t timerAPeriodicIntInit(uint32_t ui32Base, uint32_t ui32Hz, uint8_t ui8Priority, void (*pfnHandler)(void));
uint32_t timerAOneshotIntInit(uint32_t ui32Base, uint32_t ui32Hz, uint8_t ui8Priority, void (*pfnHandler)(void));

#endif /* PERIPHERALS_TIMER_H_ */
