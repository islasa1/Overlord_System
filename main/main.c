/*
 * main.c
 *
 * Overlord Software for Axis Powers Team
 *
 * Created on: Jan 22, 2017
 *
 *
 *
 */
//*****************************************************************************
//
// Standard Includes
//
//*****************************************************************************
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "sensorlib/hw_ak8963.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8963.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/magneto.h"

#include "utils/uartstdio.h"
#include "utils/cmdline.h"

// Just in case we missed it.
#ifndef NULL
#define NULL 0
#endif

#include "../peripherals/bq27441.h"
#include "../peripherals/gpio.h"
#include "../peripherals/i2c.h"
#include "../peripherals/misc.h"
#include "../peripherals/hw_mpu9x50.h"
#include "../peripherals/mpu9x50.h"
#include "../peripherals/mpu9250_drv.h"

#include "../display/charter_module.h"
#include "../sensors/imu_tests.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


void ConsoleInit(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

int main(void)
{
    //
    // Set the clocking to run at 80MHz.
    //
    MAP_SysCtlClockSet(
            SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);

    //
    // Enable the FPU
    //
    FPUInit();

    ConsoleInit();
    UARTprintf("Regression Tests\r\n");
    UARTprintf("Initializing...\r\n");
    CharterInit();
    UARTprintf("Done\r\n");
    FlareInit();
    FlareTest_1();
    CharterTest_1();
}
