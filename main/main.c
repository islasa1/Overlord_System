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

#include "../display/charter_module.h"

#include "../peripherals/bq27441.h"
#include "../peripherals/gpio.h"
#include "../peripherals/i2c.h"
#include "../peripherals/misc.h"
#include "../peripherals/hw_mpu9x50.h"
#include "../peripherals/mpu9x50.h"
#include "../peripherals/mpu9250_drv.h"

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


//*****************************************************************************
//
// Define BQ27441 I2C Address.
//
//*****************************************************************************
#define BQ27441_I2C_BASE         I2C1_BASE
#define BQ27441_I2C_INT          INT_I2C1
#define BQ27441_I2C_ADDRESS      0x55

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sBQ27441I2CInst;

//*****************************************************************************
//
// Global instance structure for the BQ27441 sensor driver.
//
//*****************************************************************************
tBQ27441 g_sBQ27441Inst;

//*****************************************************************************
//
// Global flags for the BQ27441 to indicate data ready and error state
//
//*****************************************************************************
typedef struct {
    float fVoltage;
    float fTemperature;
    int16_t i16NominalAvailableCapacity;
    int16_t i16FullAvailableCapacity;
    int16_t i16RemainingCapacity;
    int16_t i16FullChargeCapacity;
    int16_t i16AverageCurrent;
    int16_t i16StandbyCurrent;
    int16_t i16MaxLoadCurrent;
    int16_t i16AveragePower;
    int16_t i16StateOfCharge;
    float fInternalTemperature;
    int16_t i16StateOfHealth;
} tBatteryInfo;

tBatteryInfo g_sBatteryInfo;
volatile uint_fast8_t g_vui8CmdFlag, g_vui8DataFlag, g_vui8ErrorFlag;

//*****************************************************************************
//
// Global flags for the BQ27441 to indicate data ready and error state
//
//*****************************************************************************
float g_fTemperature, g_fVoltage, g_fNom;

//*****************************************************************************
//
// BQ27441 Sensor callback function.  Called at the end of BQ27441 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void BQ27441DataAppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        //
        // If I2C transaction is successful, set data ready flag.
        //
        g_vui8DataFlag = 1;

        BQ27441DataBatteryVoltageGetFloat (&g_sBQ27441Inst, &(g_sBatteryInfo.fVoltage));
        BQ27441DataBatteryTemperatureGetFloat (&g_sBQ27441Inst, &(g_sBatteryInfo.fTemperature));
        BQ27441DataNominalAvailableCapacityGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16NominalAvailableCapacity));
        BQ27441DataFullAvailableCapacityGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16FullAvailableCapacity));
        BQ27441DataRemainingCapacityGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16RemainingCapacity));
        BQ27441DataFullChargeCapacityGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16FullChargeCapacity));
        BQ27441DataAverageCurrentGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16AverageCurrent));
        BQ27441DataStandbyCurrentGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16StandbyCurrent));
        BQ27441DataMaxLoadCurrentGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16MaxLoadCurrent));
        BQ27441DataAveragePowerGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16AveragePower));
        BQ27441DataStateOfChargeGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16StateOfCharge));
        BQ27441DataInternalTemperatureGetFloat(&g_sBQ27441Inst, &(g_sBatteryInfo.fInternalTemperature));
        BQ27441DataStateOfHealthGetRaw(&g_sBQ27441Inst, &(g_sBatteryInfo.i16StateOfHealth));

        BQ27441DataRead(&g_sBQ27441Inst, BQ27441DataAppCallback, &g_sBQ27441Inst);
    }
    else
    {
        //
        // If I2C transaction fails, set error flag.
        //
        g_vui8ErrorFlag = ui8Status;
    }
}

void BQ27441AppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        //
        // If I2C transaction is successful, set data ready flag.
        //
        g_vui8CmdFlag = 1;
    }
    else
    {
        //
        // If I2C transaction fails, set error flag.
        //
        g_vui8ErrorFlag = ui8Status;
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C8 Interrupt. I2C8 is the I2C connection
// to the BQ27441 fuel guage.
//
// This handler is installed in the vector table for I2C8 by default.  To use
// the Fuel Tank on BoosterPack 1 interface change the startup file to place
// this interrupt in I2C7 vector location.
//
//*****************************************************************************
void BQ27441I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sBQ27441I2CInst);
}

//*****************************************************************************
//
// Function to wait for the BQ27441 transactions to complete.
//
//*****************************************************************************
uint_fast8_t BQ27441AppI2CWait(void)
{
    uint_fast8_t ui8RC = 0;

    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8CmdFlag == 0) && (g_vui8ErrorFlag ==0))
    {
        //
        // Wait for I2C Transactions to complete.
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag) ui8RC = g_vui8ErrorFlag;

    //
    // Clear the data flags for next use.
    //
    g_vui8CmdFlag = 0;
    g_vui8ErrorFlag = 0;

    //
    // Return original error code to caller.
    //
    return ui8RC;
}

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

    IMUDataRead();

    IMUDataGetFloat(g_pfAccel, g_pfGyro, g_pfMag);
    UpdateHeading(g_pfGyro, g_pfMag);
    UpdatePosition(g_pfAccel);
    g_pfHead[0] = GetRelativeHeading(0, 0);
    CharterDrawHeading(g_pfHead[0]);
    CharterShowBattPercent( 0, 0);
    CharterFlush();
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
    CharterInit(true);
    CharterClrScreen();
    UARTprintf("Done\r\n");

    bool result;

    result = IMUTest1(&g_sMPU9X50Inst, &g_sAK8963Inst);
    if (result)
    {
        IMUDataGetFloat(g_pfAccel, g_pfGyro, g_pfMag);
        InitPosition(g_pfAccel);
        InitHeading(g_pfMag);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
        TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 10);

        IntMasterDisable();

        IntEnable(INT_TIMER0A);
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        IntRegister(INT_TIMER0A, Timer0IntHandler);

        IntMasterEnable();

        TimerEnable(TIMER0_BASE, TIMER_A);
    }

    while(1)
    {
        //
        // Do nothing
        //
    }
}
