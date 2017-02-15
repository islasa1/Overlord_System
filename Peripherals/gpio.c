/*
 * gpio.c
 *
 *  Created on: Feb 22, 2015
 *      Author: Ryan
 */

#include "gpio.h"

#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

void
GPIOSysCtlInit(uint32_t ui32Port) {
  switch (ui32Port) {
    case GPIO_PORTA_BASE: {
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
      break;
    }
    case GPIO_PORTB_BASE: {
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
      break;
    }
    case GPIO_PORTC_BASE: {
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
      break;
    }
    case GPIO_PORTD_BASE: {
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
      break;
    }
    case GPIO_PORTE_BASE: {
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
      break;
    }
    case GPIO_PORTF_BASE: {
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
      break;
    }
  }
}

uint8_t
GPIOPinDisable(uint32_t ui32Port, uint8_t ui8Pins) {
  MAP_GPIOPinWrite(ui32Port, ui8Pins, 0x0);
  return MAP_GPIOPinRead(ui32Port, ui8Pins);
}

uint8_t
GPIOPinEnable(uint32_t ui32Port, uint8_t ui8Pins) {
  MAP_GPIOPinWrite(ui32Port, ui8Pins, ui8Pins);
  return MAP_GPIOPinRead(ui32Port, ui8Pins);
}

void
GPIOInputInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32PinType) {
  GPIOSysCtlInit(ui32Port);
  MAP_GPIOPinTypeGPIOInput(ui32Port, ui8Pins);
  MAP_GPIOPadConfigSet(ui32Port, ui8Pins, GPIO_STRENGTH_2MA, ui32PinType);
}

void
GPIOOutputInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32Strength, uint32_t ui32PinType) {
  GPIOSysCtlInit(ui32Port);
  MAP_GPIOPinTypeGPIOOutput(ui32Port, ui8Pins);
  MAP_GPIOPadConfigSet(ui32Port, ui8Pins, ui32Strength, ui32PinType);
}

void
gpioOutputODInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32Strength, uint32_t ui32PinType) {
  GPIOSysCtlInit(ui32Port);
  MAP_GPIOPinTypeGPIOOutputOD(ui32Port, ui8Pins);
  MAP_GPIOPadConfigSet(ui32Port, ui8Pins, ui32Strength, ui32PinType);
}

uint32_t
GPIOIntInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32IntType, int32_t ui32Priority, void (*pfnHandler)(void)) {
  uint32_t ui32IntFlags = 0, ui32Int;
  switch (ui32Port) {
    case GPIO_PORTA_BASE: {
      ui32Int = INT_GPIOA;
    }
    case GPIO_PORTB_BASE: {
      ui32Int = INT_GPIOB;
    }
    case GPIO_PORTC_BASE: {
      ui32Int = INT_GPIOC;
    }
    case GPIO_PORTD_BASE: {
      ui32Int = INT_GPIOD;
    }
    case GPIO_PORTE_BASE: {
      ui32Int = INT_GPIOE;
    }
    case GPIO_PORTF_BASE: {
      ui32Int = INT_GPIOF;
    }
    case GPIO_PORTG_BASE: {
      ui32Int = INT_GPIOG;
    }
    case GPIO_PORTH_BASE: {
      ui32Int = INT_GPIOH;
    }
  }

  if (ui8Pins & GPIO_PIN_0) {
    ui32IntFlags |= GPIO_INT_PIN_0;
  }
  if (ui8Pins & GPIO_PIN_1) {
    ui32IntFlags |= GPIO_INT_PIN_1;
  }
  if (ui8Pins & GPIO_PIN_2) {
    ui32IntFlags |= GPIO_INT_PIN_2;
  }
  if (ui8Pins & GPIO_PIN_3) {
    ui32IntFlags |= GPIO_INT_PIN_3;
  }
  if (ui8Pins & GPIO_PIN_4) {
    ui32IntFlags |= GPIO_INT_PIN_4;
  }
  if (ui8Pins & GPIO_PIN_5) {
    ui32IntFlags |= GPIO_INT_PIN_5;
  }
  if (ui8Pins & GPIO_PIN_6) {
    ui32IntFlags |= GPIO_INT_PIN_6;
  }
  if (ui8Pins & GPIO_PIN_7) {
    ui32IntFlags |= GPIO_INT_PIN_7;
  }
  GPIOIntRegister(ui32Port, pfnHandler);
  MAP_GPIOIntClear(ui32Port, 0xFF);
  MAP_GPIOIntEnable(ui32Port, ui32IntFlags);
  MAP_GPIOIntTypeSet(ui32Port, ui8Pins, ui32IntType);
  MAP_IntEnable(ui32Int);
  if (ui32Priority >= 0x00) MAP_IntPrioritySet(ui32Int, ui32Priority);
  return ui32IntFlags;
}

void
gpioUnlock(uint32_t ui32Port) {
  HWREG(ui32Port + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(ui32Port + GPIO_O_CR)  |= 0x01;
  HWREG(ui32Port + GPIO_O_LOCK) = 0;
}
