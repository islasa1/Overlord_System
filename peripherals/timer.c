/*
 * timer.c
 *
 *  Created on: Nov 8, 2015
 *      Author: Ryan
 */

#include "timer.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

uint32_t timerAPeriodicIntInit(uint32_t ui32Base, uint32_t ui32Hz, uint8_t ui8Priority, void (*pfnHandler)(void)) {
  uint32_t ui32Peripheral;
  uint32_t ui32TimerInt;
  switch (ui32Base) {
    case TIMER0_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER0;
      ui32TimerInt = INT_TIMER0A;
      break;
    }
    case TIMER1_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER1;
      ui32TimerInt = INT_TIMER1A;
      break;
    }
    case TIMER2_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER2;
      ui32TimerInt = INT_TIMER2A;
      break;
    }
    case TIMER3_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER3;
      ui32TimerInt = INT_TIMER3A;
      break;
    }
    case TIMER4_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER4;
      ui32TimerInt = INT_TIMER4A;
      break;
    }
    case TIMER5_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER5;
      ui32TimerInt = INT_TIMER5A;
      break;
    }
  }
  //
  // Enable the peripherals used by the timer interrupts
  //
  MAP_SysCtlPeripheralEnable(ui32Peripheral);

  //
  // Configure the 32-bit periodic timer for the status code length define.
  //
  MAP_TimerConfigure(ui32Base, TIMER_CFG_PERIODIC);
  MAP_TimerLoadSet(ui32Base, TIMER_A, MAP_SysCtlClockGet() / ui32Hz);

  //
  // Setup the interrupts for the timer timeouts.
  //
  IntPrioritySet(ui32TimerInt, ui8Priority);
  TimerIntRegister(ui32Base, TIMER_A, pfnHandler);
  MAP_IntEnable(ui32TimerInt);
  MAP_TimerIntEnable(ui32Base, TIMER_TIMA_TIMEOUT);

  //
  // Enable the timer.
  //
  MAP_TimerEnable(ui32Base, TIMER_A);

  return ui32TimerInt;
}

uint32_t timerAOneshotIntInit(uint32_t ui32Base, uint32_t ui32Hz, uint8_t ui8Priority, void (*pfnHandler)(void)) {
  uint32_t ui32Peripheral;
  uint32_t ui32TimerInt;
  switch (ui32Base) {
    case TIMER0_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER0;
      ui32TimerInt = INT_TIMER0A;
      break;
    }
    case TIMER1_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER1;
      ui32TimerInt = INT_TIMER1A;
      break;
    }
    case TIMER2_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER2;
      ui32TimerInt = INT_TIMER2A;
      break;
    }
    case TIMER3_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER3;
      ui32TimerInt = INT_TIMER3A;
      break;
    }
    case TIMER4_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER4;
      ui32TimerInt = INT_TIMER4A;
      break;
    }
    case TIMER5_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER5;
      ui32TimerInt = INT_TIMER5A;
      break;
    }
  }
  //
  // Enable the peripherals used by the timer interrupts
  //
  MAP_SysCtlPeripheralEnable(ui32Peripheral);

  //
  // Configure the 32-bit periodic timer for the status code length define.
  //
  MAP_TimerConfigure(ui32Base, TIMER_CFG_ONE_SHOT);
  MAP_TimerLoadSet(ui32Base, TIMER_A, MAP_SysCtlClockGet() / ui32Hz);

  //
  // Setup the interrupts for the timer timeouts.
  //
  IntPrioritySet(ui32TimerInt, ui8Priority);
  TimerIntRegister(ui32Base, TIMER_A, pfnHandler);
  MAP_IntEnable(ui32TimerInt);
  MAP_TimerIntEnable(ui32Base, TIMER_TIMA_TIMEOUT);

  //
  // Enable the timer.
  //
  MAP_TimerEnable(ui32Base, TIMER_A);

  return ui32TimerInt;
}
