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

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>

#include <driverlib/debug.h>
#include <driverlib/gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/sysctl.h>

#include "sensorlib/i2cm_drv.h"

#include "../Power/BQ27441.h"
#include "../Peripherals/i2c.h"

//*****************************************************************************
//
// Define BQ27510G3 I2C Address.
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
// Global instance structure for the BQ27510G3 sensor driver.
//
//*****************************************************************************
tBQ27441 g_sBQ27441Inst;

//*****************************************************************************
//
// Global flags for the BQ27441 to indicate data ready and error state
//
//*****************************************************************************
volatile uint_fast8_t g_vui8CmdFlag, g_vui8DataFlag, g_vui8ErrorFlag;

//*****************************************************************************
//
// Global flags for the BQ27441 to indicate data ready and error state
//
//*****************************************************************************
float g_fTemperature, g_fVoltage;

//*****************************************************************************
//
// BQ27510G3 Sensor callback function.  Called at the end of BQ27510G3 sensor
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
        BQ27441DataBatteryVoltageGetFloat (&g_sBQ27441Inst, &g_fTemperature);
        BQ27441DataBatteryTemperatureGetFloat (&g_sBQ27441Inst, &g_fVoltage);
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

void BQ27441CmdAppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
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
// to the BQ27510G3 fuel guage.
//
// This handler is installed in the vector table for I2C8 by default.  To use
// the Fuel Tank on BoosterPack 1 interface change the startup file to place
// this interrupt in I2C7 vector location.
//
//*****************************************************************************
void
BQ27441I2CIntHandler(void)
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
// Function to wait for the BQ27510G3 transactions to complete.
//
//*****************************************************************************
uint_fast8_t
BQ27441AppI2CWait(void)
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

int main(void)
{
    g_sBQ27441Inst.psI2CInst = NULL;

    I2CInit(BQ27441_I2C_BASE, I2C_SPEED_400);
    I2CIntRegister(BQ27441_I2C_BASE, BQ27441I2CIntHandler);

    I2CMInit (&g_sBQ27441I2CInst, BQ27441_I2C_BASE, BQ27441_I2C_INT, 0xff, 0xff, ROM_SysCtlClockGet ());
    BQ27441Init (&g_sBQ27441Inst, &g_sBQ27441I2CInst, BQ27441_I2C_ADDRESS, BQ27441CmdAppCallback, &g_sBQ27441Inst);

    //
    // Wait for initialization callback to indicate reset request is complete.
    //
    if (BQ27441AppI2CWait()) while (true);

    BQ27441DataRead(&g_sBQ27441Inst, BQ27441DataAppCallback, &g_sBQ27441Inst);

    while (true) {
    }
}
