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

#include "../peripherals/bq27441.h"
#include "../peripherals/gpio.h"
#include "../peripherals/i2c.h"
#include "../peripherals/misc.h"
#include "../peripherals/hw_mpu9x50.h"
#include "../peripherals/mpu9x50.h"
#include "../peripherals/mpu9250_drv.h"

#include "../display/charter_module.h"
#include "../power/dips_module.h"
#include "../sensors/imu_tests.h"
#include "../sensors/windrose_module.h"


#define DELAY 0

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

// I2C Options
#define I2C_SPEED I2C_SPEED_400

// MPU9X50
#define MPU9X50_X_AXIS X_AXIS   // Array index that contains corrected x-axis when mounted
#define MPU9X50_Y_AXIS Z_AXIS   // Array index that contains corrected y-axis when mounted
#define MPU9X50_Z_AXIS Y_AXIS   // Array index that contains corrected z-axis when mounted
#define MPU9X50_X_FLIP 0        // bool value (0/1) that negates raw x-axis value
#define MPU9X50_Y_FLIP 1        // bool value (0/1) that negates raw y-axis value
#define MPU9X50_Z_FLIP 1        // bool value (0/1) that negates raw z-axis value
#define MPU9X50_ROLL    0
#define MPU9X50_PITCH   1
#define MPU9X50_YAW     2

float g_pfAccel[3] = { 0, 0, 0 };
float g_pfGyro[3] = { 0, 0, 0 };
float g_pfMag[3] = { 0, 0, 0 };
float g_fHead = 0;

// Global instance structures for data acquisition
tI2CMInstance g_sI2CMInst;
tMPU9X50 g_sMPU9X50Inst;
tAK8963 g_sAK8963Inst;

//
// Global flags to alert main that I2C transaction is complete, an error occured, or data is ready
//
volatile uint_fast8_t g_vui8MPU9X50DoneFlag = false, g_vui8MPU9X50ErrorFlag = 0;
volatile uint_fast8_t g_vui8AK8963DoneFlag = false, g_vui8AK8963ErrorFlag = 0;

#define M_PI       3.14159265358979323846
#define RAD_TO_DEG 57.2957795130823208767
#define DEG_TO_RAD 0.01745329251994329576

#define ALPHA 0.01

void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    IMUDataRead();

    IMUDataGetFloat(g_pfAccel, g_pfGyro, g_pfMag);
    UpdateHeading(g_pfGyro, g_pfMag);
    UpdatePosition(g_pfAccel);
}

void Timer1IntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    float x, y;

    GetPosition(&x, &y);
    CharterDrawPosition(x, y);
    g_fHead = GetRelativeHeading(0,0);
    CharterDrawHeading(g_fHead);
    CharterShowBattPercent( 0, 0);
    CharterFlush();
}

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
    UARTprintf("IMU Visualization Test\r\n");
    UARTprintf("Initializing...\r\n");
    IMUInit(&g_sMPU9X50Inst, &g_sAK8963Inst, &g_sI2CMInst);
    BatteryInit ();
    CharterInit(true);
    CharterClrScreen();
    UARTprintf("Done\r\n");

    bool result;

    result = IMUTest1(&g_sMPU9X50Inst, &g_sAK8963Inst);
    if (result)
    {
        IMUDataGetFloat(g_pfAccel, g_pfGyro, g_pfMag);
        ReadCalibrationData();
        InitPosition(g_pfAccel);
        InitHeading(g_pfMag);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
        TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
        TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 100);
        TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 10);

        IntMasterDisable();

        IntEnable(INT_TIMER0A);
        IntEnable(INT_TIMER1A);
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
        IntRegister(INT_TIMER0A, Timer0IntHandler);
        IntRegister(INT_TIMER1A, Timer1IntHandler);

        IntMasterEnable();

        TimerEnable(TIMER0_BASE, TIMER_A);
        TimerEnable(TIMER1_BASE, TIMER_A);
    }

    while(1)
    {
        //
        // Do nothing
        //
    }
}
