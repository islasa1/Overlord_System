/*
 * dips_module.c
 *
 *  Created on: Jan 22, 2017
 *      Author: Anthony
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "sensorlib/i2cm_drv.h"

#include "../peripherals/bq27441.h"
#include "../peripherals/gpio.h"
#include "../peripherals/i2c.h"
#include "../peripherals/timer.h"

#include "dips_module.h"

//*****************************************************************************
//
//! \addtogroup DIPS_Module_api
//! @{
//
//*****************************************************************************

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
// Global instance structure for the battery information
//
//*****************************************************************************
tBatteryInfo g_sBatteryInfo;

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
//
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

//*****************************************************************************
//
// Function to update all the values within battery information structure
//
//*****************************************************************************
void PeriodicTimerIntHandler (void)
{
    BQ27441DataRead(&g_sBQ27441Inst, BQ27441DataAppCallback, &g_sBQ27441Inst);
}

//*****************************************************************************
//
//! Initializes the battery.
//!
//! This function initializes the battery pack, including the I2C interrupts,
//! I2C master instance, the BQ27441, and the periodic interrupt timer.
//! This initializes the battery information structure and prepares for
//! future data reads based on when the periodic timer interrupts occur.
//!
//! \return None.
//
//*****************************************************************************
void BatteryInit (void)
{
    I2CInit(BQ27441_I2C_BASE, I2C_SPEED_400);
    I2CIntRegister(BQ27441_I2C_BASE, BQ27441I2CIntHandler);

    I2CMInit (&g_sBQ27441I2CInst, BQ27441_I2C_BASE, BQ27441_I2C_INT, 0xff, 0xff, MAP_SysCtlClockGet ());
    BQ27441Init (&g_sBQ27441Inst, &g_sBQ27441I2CInst, BQ27441_I2C_ADDRESS, BQ27441AppCallback, &g_sBQ27441Inst);

    //
    // Wait for initialization callback to indicate reset request is complete.
    //
    if (BQ27441AppI2CWait()) while (true);

    //
    // Initialize the periodic timer to refresh the battery information
    //
    timerAPeriodicIntInit(TIMER1_BASE, 2, 0xE0, PeriodicTimerIntHandler);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
