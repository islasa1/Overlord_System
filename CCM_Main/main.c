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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

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

#include "sensorlib/i2cm_drv.h"

#include "sensorlib/hw_ak8963.h"
#include "sensorlib/hw_ak8975.h"

#include "sensorlib/ak8963.h"
#include "sensorlib/ak8975.h"

#include "sensorlib/magneto.h"

#include "utils/uartstdio.h"
#include "utils/cmdline.h"

#include "../Peripherals/gpio.h"
#include "../Peripherals/i2c.h"
#include "../Peripherals/misc.h"

#include "../Sensors/hw_mpu9x50.h"
#include "../Sensors/imu.h"
#include "../Sensors/imu_tests.h"
#include "../Sensors/mpu9x50.h"


#define DELAY 0

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

// I2C Options
#define I2C_SPEED I2C_SPEED_400

// MPU9X50
#define MPU9X50_X_AXIS X_AXIS // Array index that contains corrected x-axis when mounted
#define MPU9X50_Y_AXIS Z_AXIS // Array index that contains corrected y-axis when mounted
#define MPU9X50_Z_AXIS Y_AXIS // Array index that contains corrected z-axis when mounted
#define MPU9X50_X_FLIP 0 // bool value (0/1) that negates raw x-axis value
#define MPU9X50_Y_FLIP 1 // bool value (0/1) that negates raw y-axis value
#define MPU9X50_Z_FLIP 1 // bool value (0/1) that negates raw z-axis value
#define MPU9X50_ROLL    0
#define MPU9X50_PITCH   1
#define MPU9X50_YAW     2

float g_pfAccel[3] = { 0 };
float g_pfGyro[3] = { 0 };
float g_pfMag[3] = { 0 };
float g_pfHead[3] = { 0 };

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

#include "../Display/charter_module.h"

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

float RadToDegrees(float rad)
{
    return rad * (180 / M_PI);
}

void CalculatePitchRoll(float x, float y, float z, float *roll,
                        float *pitch)
{
    static float aX = 0;
    static float aY = 0;
    static float aZ = 0;
    aX = x * ALPHA + (aX * (1.0 - ALPHA));
    aY = y * ALPHA + (aY * (1.0 - ALPHA));
    aZ = z * ALPHA + (aZ * (1.0 - ALPHA));
    *roll = atan2(aY, aZ) * RAD_TO_DEG;
    *pitch = atan2(-aX, sqrt(aY * aY + aZ * aZ)) * RAD_TO_DEG;
}


//*****************************************************************************
//
// MPU9X50 Sensor callback function.  Called at the end of MPU9X50 sensor
// driver transactions.
//
//*****************************************************************************
void MPU9X50DataAppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // MPU9X50 Transactions are complete. Check for success
    //
    g_vui8MPU9X50DoneFlag = true;
    if (ui8Status == I2CM_STATUS_SUCCESS)
    {
        MPU9X50DataAccelGetFloat((tMPU9X50*) pvCallbackData, &g_pfAccel[0],
                                 &g_pfAccel[1], &g_pfAccel[2]);
        MPU9X50DataGyroGetFloat((tMPU9X50*) pvCallbackData, &g_pfGyro[0],
                                &g_pfGyro[1], &g_pfGyro[2]);

        g_pfAccel[MPU9X50_X_AXIS] *= MPU9X50_X_FLIP ? -1 : 1;
        g_pfAccel[MPU9X50_Y_AXIS] *= MPU9X50_Y_FLIP ? -1 : 1;
        g_pfAccel[MPU9X50_Z_AXIS] *= MPU9X50_Z_FLIP ? -1 : 1;

        g_pfGyro[MPU9X50_X_AXIS] *= MPU9X50_X_FLIP ? -1 : 1;
        g_pfGyro[MPU9X50_Y_AXIS] *= MPU9X50_Y_FLIP ? -1 : 1;
        g_pfGyro[MPU9X50_Z_AXIS] *= MPU9X50_Z_FLIP ? -1 : 1;

        CalculatePitchRoll(g_pfAccel[MPU9X50_X_AXIS],
                           g_pfAccel[MPU9X50_Y_AXIS],
                           g_pfAccel[MPU9X50_Z_AXIS],
                           &g_pfHead[MPU9X50_ROLL],
                           &g_pfHead[MPU9X50_PITCH]);

        g_pfHead[MPU9X50_YAW] = MagnetoHeadingCompute(
                g_pfMag[MPU9X50_X_AXIS], g_pfMag[MPU9X50_Y_AXIS],
                g_pfMag[MPU9X50_Z_AXIS],
                g_pfHead[MPU9X50_ROLL] * DEG_TO_RAD,
                g_pfHead[MPU9X50_PITCH] * DEG_TO_RAD) * RAD_TO_DEG;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8MPU9X50ErrorFlag = ui8Status;
}

void AK8963DataAppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    g_vui8AK8963DoneFlag = true;
    if (ui8Status == I2CM_STATUS_SUCCESS)
    {
        AK8963DataMagnetoGetFloat((tAK8963*) pvCallbackData, &g_pfMag[0],
                                  &g_pfMag[1], &g_pfMag[2]);

        g_pfMag[MPU9X50_X_AXIS] *= MPU9X50_X_FLIP ? -1 : 1;
        g_pfMag[MPU9X50_Y_AXIS] *= MPU9X50_Y_FLIP ? -1 : 1;
        g_pfMag[MPU9X50_Z_AXIS] *= MPU9X50_Z_FLIP ? -1 : 1;

        //
        // convert mag data to micro-tesla for better human interpretation.
        //
        g_pfMag[MPU9X50_X_AXIS] *= 1e6;
        g_pfMag[MPU9X50_Y_AXIS] *= 1e6;
        g_pfMag[MPU9X50_Z_AXIS] *= 1e6;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8AK8963ErrorFlag = ui8Status;
    AK8963DataRead((tAK8963*) pvCallbackData, AK8963DataAppCallback,
                   pvCallbackData);
}

//*****************************************************************************
//
// Interrupt handler for Timer 0. Does math with IMU data.
//
//*****************************************************************************
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    IMUDataGetFloat(g_pfAccel, g_pfGyro, g_pfMag);
}

int main(void)
{
    //
    // Set the clocking to run at 80MHz.
    //
    MAP_SysCtlClockSet(
            SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 100);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntRegister(INT_TIMER0A, Timer0IntHandler);

    MAP_IntMasterEnable();

    //
    // Enable the FPU
    //
    FPUInit();

    ConsoleInit();
    UARTprintf("IMU Visualization Test\r\n");
    UARTprintf("Initializing...\r\n");
    IMUInit(&g_sMPU9X50Inst, &g_sAK8963Inst, &g_sI2CMInst);
    UARTprintf("Done\r\n");

    bool result;
    result = IMUTest1(&g_sMPU9X50Inst, &g_sAK8963Inst);
    if (result)
    {
        TimerEnable(TIMER0_BASE, TIMER_A);
    }

    while (true)
    {

    }
}
