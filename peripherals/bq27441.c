/*
 * BQ27441.c
 *
 *  Created on: Feb 6, 2017
 *      Author: bergej
 */

#include "bq27441.h"

//*****************************************************************************
//
//! \addtogroup BQ27441_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// The constants used to calculate object temperature.
//
//*****************************************************************************
#define T_REF                   273

//*****************************************************************************
//
// The callback function that is called when I2C transactions to/from the
// BQ27441 have completed.
//
//*****************************************************************************
void
BQ27441Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    tBQ27441 *psInst;

    //
    // Convert the instance data into a pointer to a tBQ27441 structure.
    //
    psInst = pvCallbackData;

    //
    // If the I2C master driver encountered a failure, force the state machine
    // to the idle state (which will also result in a callback to propagate the
    // error).
    //
    if(ui8Status != I2CM_STATUS_SUCCESS)
    {
        psInst->ui8State = BQ27441_STATE_IDLE;
    }

    //
    // Determine the current state of the BQ27441 state machine.
    //
    switch(psInst->ui8State)
    {
        //
        // The first data read state, has finished setup and trigger data read
        // state 2.
        //
        case BQ27441_STATE_READ_DATA_1:
        {
            //
            // Move the state machine to the next read state.
            //
            psInst->ui8State = BQ27441_STATE_READ_DATA_2;

            //
            // Read the requested data from the BQ27441.
            //
            psInst->uCommand.pui8Buffer[0] = BQ27441_O_NOM_AV_CAP_LSB;
            I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                        psInst->uCommand.pui8Buffer, 1, psInst->pui8Data + 4,
                        14, BQ27441Callback, psInst);

            //
            // break
            //
            break;
        }

        //
        // The 2nd data read state, has finished setup and trigger data read
        // state 3.  Read state 3 is the final state and when done will return
        // to idle and trigger the application level callback.
        //
        case BQ27441_STATE_READ_DATA_2:
        {
            //
            // Move the state machine to the next read state.
            //
            psInst->ui8State = BQ27441_STATE_READ_DATA_3;

            //
            // Read the requested data from the BQ27441.
            //
            psInst->uCommand.pui8Buffer[0] = BQ27441_O_AVG_PWR_LSB;
            I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                        psInst->uCommand.pui8Buffer, 1, psInst->pui8Data + 18,
                        2, BQ27441Callback, psInst);

            //
            // break
            //
            break;
        }

        case BQ27441_STATE_READ_DATA_3:
        {
            //
            // Move the state machine to the next read state.
            //
            psInst->ui8State = BQ27441_STATE_READ_DATA_4;

            //
            // Read the requested data from the BQ27441.
            //
            psInst->uCommand.pui8Buffer[0] = BQ27441_O_STATE_OF_CHRG_LSB;
            I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                        psInst->uCommand.pui8Buffer, 1, psInst->pui8Data + 20,
                        6, BQ27441Callback, psInst);

            //
            // break
            //
            break;
        }
        //
        // All states that trivially transition to IDLE, and all unknown
        // states.
        //
        case BQ27441_STATE_INIT:
        case BQ27441_STATE_READ:
        case BQ27441_STATE_WRITE:
        case BQ27441_STATE_READ_DATA_4:
        case BQ27441_STATE_RMW:
        default:
        {
            //
            // The state machine is now idle.
            //
            psInst->ui8State = BQ27441_STATE_IDLE;

            //
            // Done.
            //
            break;
        }
    }

    //
    // See if the state machine is now idle and there is a callback function.
    //
    if((psInst->ui8State == BQ27441_STATE_IDLE) && psInst->pfnCallback)
    {
        //
        // Call the application-supplied callback function.
        //
        psInst->pfnCallback(psInst->pvCallbackData, ui8Status);
    }
}

//*****************************************************************************
//
//! Initializes the BQ27441 driver.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param psI2CInst is a pointer to the I2C driver instance data.
//! \param ui8I2CAddr is the I2C address of the BQ27441 device.
//! \param pfnCallback is the function to be called when the initialization has
//! completed (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function initializes the BQ27441 driver, preparing it for operation.
//!
//! \return Returns 1 if the BQ27441 driver was successfully initialized and
//! 0 if it was not.
//
//*****************************************************************************
uint_fast8_t
BQ27441Init(tBQ27441 *psInst, tI2CMInstance *psI2CInst,
              uint_fast8_t ui8I2CAddr, tSensorCallback *pfnCallback,
              void *pvCallbackData)
{
    //
    // Initialize the BQ27441 instance structure
    //
    psInst->psI2CInst = psI2CInst;
    psInst->ui8Addr = ui8I2CAddr;
    psInst->ui8State = BQ27441_STATE_IDLE;

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // The default settings are ok.  Return success and call the callback.
    //
    if(pfnCallback)
    {
        pfnCallback(pvCallbackData, I2CM_STATUS_SUCCESS);
    }

    //
    // Success
    //
    return(1);
}

//*****************************************************************************
//
//! Reads data from BQ27441 registers.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param ui8Reg is the first register to read.
//! \param pui16Data is a pointer to the location to store the data that is
//! read.
//! \param ui16Count the number of register values to read.
//! \param pfnCallback is the function to be called when data read is complete
//! (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function reads a sequence of data values from consecutive registers in
//! the BQ27441.
//!
//! \note The BQ27441 does not auto-increment the register pointer, so reads
//! of more than one value returns garbage for the subsequent values.
//!
//! \return Returns 1 if the write was successfully started and 0 if it was
//! not.
//
//*****************************************************************************
uint_fast8_t
BQ27441Read(tBQ27441 *psInst, uint_fast8_t ui8Reg, uint16_t *pui16Data,
           uint_fast16_t ui16Count, tSensorCallback *pfnCallback,
           void *pvCallbackData)
{
    //
    // Return a failure if the BQ27441 driver is not idle (in other words,
    // there is already an outstanding request to the BQ27441).
    //
    if(psInst->ui8State != BQ27441_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the wait for read state.
    //
    psInst->ui8State = BQ27441_STATE_READ;

    //
    // Read the requested registers from the BQ27441.
    //
    if(I2CMRead16BE(&(psInst->uCommand.sReadState), psInst->psI2CInst,
                    psInst->ui8Addr, ui8Reg, pui16Data, ui16Count,
                    BQ27441Callback, psInst) == 0)
    {
        //
        // The I2C write failed, so move to the idle state and return a
        // failure.
        //
        psInst->ui8State = BQ27441_STATE_IDLE;
        return(0);
    }

    //
    // Success.
    //
    return(1);
}

//*****************************************************************************
//
//! Writes data to BQ27441 registers.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param ui8Reg is the first register to write.
//! \param pui16Data is a pointer to the 16-bit register data to write.
//! \param ui16Count is the number of 16-bit registers to write.
//! \param pfnCallback is the function to be called when the data has been
//! written (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function writes a sequence of data values to consecutive registers in
//! the BQ27441.  The first value in the \e pui16Data buffer contains the
//! data to be written into the \e ui8Reg register, the second value contains
//! the data to be written into the next register, and so on.
//!
//! \note The BQ27441 does not auto-increment the register pointer, so writes
//! of more than one register are rejected by the BQ27441.
//!
//! \return Returns 1 if the write was successfully started and 0 if it was
//! not.
//
//*****************************************************************************
uint_fast8_t
BQ27441Write(tBQ27441 *psInst, uint_fast8_t ui8Reg,
               const uint16_t *pui16Data, uint_fast16_t ui16Count,
               tSensorCallback *pfnCallback, void *pvCallbackData)
{
    //
    // Return a failure if the BQ27441 driver is not idle (in other words,
    // there is already an outstanding request to the BQ27441).
    //
    if(psInst->ui8State != BQ27441_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the wait for write state.
    //
    psInst->ui8State = BQ27441_STATE_WRITE;

    //
    // Write the requested registers to the BQ27441.
    //
    if(I2CMWrite16BE(&(psInst->uCommand.sWriteState), psInst->psI2CInst,
                     psInst->ui8Addr, ui8Reg, pui16Data, ui16Count,
                     BQ27441Callback, psInst) == 0)
    {
        //
        // The I2C write failed, so move to the idle state and return a
        // failure.
        //
        psInst->ui8State = BQ27441_STATE_IDLE;
        return(0);
    }

    //
    // Success.
    //
    return(1);
}

//*****************************************************************************
//
//! Performs a read-modify-write of a BQ27441 register.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param ui8Reg is the register offset to read modify and write
//! \param ui16Mask is the bit mask that is ANDed with the current register
//! value.
//! \param ui16Value is the bit mask that is ORed with the result of the AND
//! operation.
//! \param pfnCallback is the function to be called when the data has been
//! changed (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function changes the value of a register in the BQ27441 via a
//! read-modify-write operation, allowing one of the fields to be changed
//! without disturbing the other fields.  The \e ui8Reg register is read, ANDed
//! with \e ui16Mask, ORed with \e ui16Value, and then written back to the
//! BQ27441.
//!
//! \return Returns 1 if the read-modify-write was successfully started and 0
//! if it was not.
//
//*****************************************************************************
uint_fast8_t
BQ27441ReadModifyWrite(tBQ27441 *psInst, uint_fast8_t ui8Reg,
                      uint_fast16_t ui16Mask, uint_fast16_t ui16Value,
                      tSensorCallback *pfnCallback, void *pvCallbackData)
{
    //
    // Return a failure if the BQ27441 driver is not idle (in other words,
    // there is already an outstanding request to the BQ27441).
    //
    if(psInst->ui8State != BQ27441_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the wait for read-modify-write state.
    //
    psInst->ui8State = BQ27441_STATE_RMW;

    //
    // Submit the read-modify-write request to the BQ27441.
    //
    if(I2CMReadModifyWrite16BE(&(psInst->uCommand.sReadModifyWriteState),
                               psInst->psI2CInst, psInst->ui8Addr, ui8Reg,
                               ui16Mask, ui16Value, BQ27441Callback,
                               psInst) == 0)
    {
        //
        // The I2C read-modify-write failed, so move to the idle state and
        // return a failure.
        //
        psInst->ui8State = BQ27441_STATE_IDLE;
        return(0);
    }

    //
    // Success.
    //
    return(1);
}

//*****************************************************************************
//
//! Performs a read of a BQ27441 data register.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pfnCallback is the function to be called when the data has been read
//! (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function initiates a read of the BQ27441 data registers.  When the
//! read has completed (as indicated by calling the callback function), the new
//! readings can be obtained via functions like:
//!
//! - BQ27441DataTCurrentInstantaneousGetRaw()
//! - BQ27441DataTCurrentInstantaneousGetFloat()
//!
//! \return Returns 1 if the read was successfully started and 0 if it was not.
//
//*****************************************************************************
uint_fast8_t
BQ27441DataRead(tBQ27441 *psInst, tSensorCallback *pfnCallback,
                  void *pvCallbackData)
{
    //
    // Return a failure if the BQ27441 driver is not idle (in other words,
    // there is already an outstanding request to the BQ27441).
    //
    if(psInst->ui8State != BQ27441_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the first read state.  Reads are done in three
    // parts based on address ranges of the information being read.
    //
    psInst->ui8State = BQ27441_STATE_READ_DATA_1;

    //
    // Read the requested data from the BQ27441.
    //
    psInst->uCommand.pui8Buffer[0] = BQ27441_O_TEMP_LSB;
    if(I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                psInst->uCommand.pui8Buffer, 1, psInst->pui8Data, 4,
                BQ27441Callback, psInst) == 0)
    {
        //
        // The I2C read failed, so move to the idle state and return a failure.
        //
        psInst->ui8State = BQ27441_STATE_IDLE;
        return(0);
    }

    //
    // Success.
    //

    return(1);
}

//*****************************************************************************
//
//! Gets the raw battery temperature from the most recent data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataBatteryTemperatureGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[1] << 8) | psInst->pui8Data[0];
}

//*****************************************************************************
//
//! Gets the battery temperature measurement data from the most recent  data
//! read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pfData is a pointer to the value into which the data is stored as
//! floating point.
//!
//! This function returns the data from the most recent data read,
//! converted into float value. Units are degrees Celsius.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataBatteryTemperatureGetFloat(tBQ27441 *psInst, float *pfData)
{
    int16_t i16Data;

    //
    // Get the raw readings.
    //
    BQ27441DataBatteryTemperatureGetRaw(psInst, &i16Data);

    //
    // Device returns the units as 0.1 degrees K.  Convert first to whole
    // degrees then from K to C
    //
    *pfData = (float)(i16Data);
    *pfData = *pfData / 10.0f;
    *pfData -= 272.15f;


}

//*****************************************************************************
//
//! Gets the raw battery voltage measurement data from the most recent data
//! read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataBatteryVoltageGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[3] << 8) | psInst->pui8Data[2];
}

//*****************************************************************************
//
//! Gets the battery voltage measurement from the most recent data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pfData is a pointer to the value into which the data is stored as
//! floating point.
//!
//! This function returns the data from the most recent data read,
//! converted into float value. Units are volts.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataBatteryVoltageGetFloat(tBQ27441 *psInst, float *pfData)
{
    int16_t i16Data;

    //
    // Get the raw readings.
    //
    BQ27441DataBatteryVoltageGetRaw(psInst, &i16Data);

    //
    // Covert to float.
    //
    *pfData = (float)(i16Data);
    *pfData = *pfData / 1000.0f;

}

//*****************************************************************************
//
//! Gets the raw nominal available capacity measurement from the most recent
//! data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataNominalAvailableCapacityGetRaw(tBQ27441 *psInst,
                                            int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[5] << 8) | psInst->pui8Data[4];
}

//*****************************************************************************
//
//! Gets the raw available capacity of a new battery from the most recent data
//! read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataFullAvailableCapacityGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[7] << 8) | psInst->pui8Data[6];
}

//*****************************************************************************
//
//! Gets the raw remaining capacity of measurement from the most recent data
//! read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataRemainingCapacityGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[9] << 8) | psInst->pui8Data[8];
}

//*****************************************************************************
//
//! Gets the raw full charge capacity from the most recent data
//! read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataFullChargeCapacityGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[11] << 8) | psInst->pui8Data[10];
}

//*****************************************************************************
//
//! Gets the raw average current measurement from the most recent data
//! read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataAverageCurrentGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[13] << 8) | psInst->pui8Data[12];
}

//*****************************************************************************
//
//! Gets the raw standby current from the most recent data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataStandbyCurrentGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[15] << 8) | psInst->pui8Data[14];
}

//*****************************************************************************
//
//! Gets the raw max load current from the most recent data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataMaxLoadCurrentGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[17] << 8) | psInst->pui8Data[16];
}

//*****************************************************************************
//
//! Gets the average power from the most recent data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataAveragePowerGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[19] << 8) | psInst->pui8Data[18];
}

//*****************************************************************************
//
//! Gets the raw charge state data from the most recent data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataStateOfChargeGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[21] << 8) | psInst->pui8Data[20];
}

//*****************************************************************************
//
//! Gets the raw internal temparature data from the most recent data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataInternalTemperatureGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[23] << 8) | psInst->pui8Data[22];
}

//*****************************************************************************
//
//! Gets the internal temperature data from the most recent data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pfData is a pointer to the value into which the data is stored as
//! floating point.
//!
//! This function returns the internal temperature from the most recent data
//! read.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataInternalTemperatureGetFloat(tBQ27441 *psInst, float *pfData)
{
    int16_t i16Data;

    //
    // Get the raw readings.
    //
    BQ27441DataInternalTemperatureGetRaw(psInst, &i16Data);

    //
    // Convert to floating point Kelvin, then Celsius.
    //
    *pfData = (float)(i16Data);
    *pfData = *pfData / 10.0f;
    *pfData -= 272.15f;
}

//*****************************************************************************
//
//! Gets the raw health data from the most recent data read.
//!
//! \param psInst is a pointer to the BQ27441 instance data.
//! \param pi16Data is a pointer to the value into which the raw data is
//! stored.
//!
//! This function returns the raw measurement data from the most recent data
//! read.  The data is not manipulated in any way by the driver.
//!
//! \return None.
//
//*****************************************************************************
void
BQ27441DataStateOfHealthGetRaw(tBQ27441 *psInst, int16_t *pi16Data)
{
    //
    // Return the raw data value.
    //
    *pi16Data = ((int16_t)psInst->pui8Data[25] << 8) | psInst->pui8Data[24];
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
