/*
 * IMU.c
 *
 *  Created on: Feb 8, 2017
 *      Author: moncadac
 */

#include "mpu9250_drv.h"

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"

#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_ak8963.h"
#include "sensorlib/ak8963.h"

#include "../peripherals/gpio.h"
#include "../peripherals/i2c.h"
#include "../peripherals/hw_mpu9x50.h"
#include "../peripherals/mpu9x50.h"

//*****************************************************************************
//
//! \addtogroup imu_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Global flags to alert main that I2C transaction is complete
//
//*****************************************************************************
volatile static uint_fast8_t sg_vui8MPUI2CDoneFlag, sg_vui8AKI2CDoneFlag;

//*****************************************************************************
//
// Global flags to alert main that I2C transaction error has occurred.
//
//*****************************************************************************
volatile static uint_fast8_t sg_vui8MPUErrorFlag, sg_vui8AKErrorFlag;

//*****************************************************************************
//
// Global pointer to MPU9X50 instance. Local to this file.
//
//*****************************************************************************
static tMPU9X50 *sg_psMPU9X50Inst;

//*****************************************************************************
//
// Global pointer to AK8963 instance. Local to this file.
//
//*****************************************************************************
static tAK8963 *sg_psAK8963Inst;

//*****************************************************************************
//
// Global pointer to AK8963 instance. Local to this file.
//
//*****************************************************************************
static tI2CMInstance *sg_psI2CMInst;

//*****************************************************************************
//
//! Waits for the MPU9X50 to finish an I2C transaction.
//!
//! \param pcFilename is a pointer to a char for use in an error handler. Not
//! used in this implementation.
//! \param ui32Line is an integer for use with an error handler. Not used in
//! this implementation.
//!
//! This function spin waits on the I2C bus for the MPU9X50 to finish its
//! transaction. Allows for I2C handling to be implemented.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9X50AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((sg_vui8MPUI2CDoneFlag == 0) && (sg_vui8MPUErrorFlag == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(sg_vui8MPUErrorFlag)
    {
        //MPU9X50AppErrorHandler(pcFilename, ui32Line);
        //UARTprintf("ERROR in MPU9X50AppErrorHandler\n");
    }

    //
    // clear the data flag for next use.
    //
    sg_vui8MPUI2CDoneFlag = 0;
}

//*****************************************************************************
//
//! Waits for the AK8963 to finish an I2C transaction.
//!
//! \param pcFilename is a pointer to a char for use in an error handler. Not
//! used in this implementation.
//! \param ui32Line is an integer for use with an error handler. Not used in
//! this implementation.
//!
//! This function spin waits on the I2C bus for the AK8963 to finish its
//! transaction. Allows for I2C handling to be implemented.
//!
//! \return None.
//
//*****************************************************************************
void
AK8963AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    while(!(sg_vui8AKI2CDoneFlag || sg_vui8AKErrorFlag))
    {
        //
        // Do nothing
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(sg_vui8AKErrorFlag)
    {
        //MPU9X50AppErrorHandler(pcFilename, ui32Line);
        //UARTprintf("ERROR in MPU9X50AppErrorHandler\n");
    }

    //
    // clear the data flag for next use.
    //
    sg_vui8AKI2CDoneFlag = 0;
}

//*****************************************************************************
//
//! MPU9X50 sensor callback function
//!
//! \param pvCallbackData is a pointer to data to be used by the callback
//! function. It is not used in this implementation.
//! \param ui8Status is an 8 bit integer that represents the status of the
//! transaction that this is a callback for.
//!
//! This function is called at the end of MPU9X50 I2C transactions and so is
//! used to check the status of those transactions. If ui8Status is not
//! success, the error flag is set to the error code. Otherwise, the done
//! flag is set.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9X50AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        sg_vui8MPUI2CDoneFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    sg_vui8MPUErrorFlag = ui8Status;
}

//*****************************************************************************
//
//! AK8963 sensor callback function
//!
//! \param pvCallbackData is a pointer to data to be used by the callback
//! function. It is not used in this implementation.
//! \param ui8Status is an 8 bit integer that represents the status of the
//! transaction that this is a callback for.
//!
//! This function is called at the end of AK8963 I2C transactions and so is
//! used to check the status of those transactions. If ui8Status is not
//! success, the error flag is set to the error code. Otherwise, the done
//! flag is set.
//!
//! \return None.
//
//*****************************************************************************
void
AK8963AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        sg_vui8AKI2CDoneFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    sg_vui8AKErrorFlag = ui8Status;
}

//*****************************************************************************
//
//! Called as a result of the MPU9X50 I2C interrupt.
//!
//! This function is a wrapper for the I2C master interrupt handler. It is
//! needed at the application layer in order to use the I2C master driver
//! instance. It is called as a result of the
//!
//! \return None.
//
//*****************************************************************************
void
MPU9X50I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(sg_psI2CMInst);
}

//*****************************************************************************
//
//! Called as a result of the MPU9X50 data ready interrupt.
//!
//! This function is called as a result of the MPU9X50 data ready interrupt.
//! It clears the interrupt and then Reads the data from the MPU9X50 and the
//! AK8963.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9X50IntHandler(void)
{
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(GPIO_PORTE_BASE, true);

    GPIOIntClear(GPIO_PORTE_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_0)
    {
        AK8963DataRead(sg_psAK8963Inst, AK8963AppCallback, sg_psAK8963Inst);
        MPU9X50DataRead(sg_psMPU9X50Inst, MPU9X50AppCallback, sg_psMPU9X50Inst);
    }
}

//*****************************************************************************
//
//! Gets the data from the MPU9X50 and AK8963 driver structures.
//!
//! \param pfAccel is a pointer to an array of floats where the accelerometer
//! data can be stored. Should be at least size 3.
//! \param pfGyro is a pointer to an array of floats where the gyroscope data
//! can be stored. Should be at least size 3.
//! \param pfMag is a pointer to an array of floats where the magnetometer data
//! can be stored. Should be at least size 3.
//!
//! This functions gets the data from the MPU9X50 and AK8963 driver structures
//! as floats and stores it in the provided arrays. This should be called after
//! calling the data read functions for both of the drivers.
//!
//! \return None.
//
//*****************************************************************************
void
IMUDataGetFloat(float *pfAccel, float *pfGyro, float *pfMag)
{
    MPU9X50DataAccelGetFloat(sg_psMPU9X50Inst, &pfAccel[0], &pfAccel[1], &pfAccel[2]);
    MPU9X50DataGyroGetFloat(sg_psMPU9X50Inst, &pfGyro[0], &pfGyro[1], &pfGyro[2]);
    AK8963DataMagnetoGetFloat(sg_psAK8963Inst, &pfMag[0], &pfMag[1], &pfMag[2]);
}

//*****************************************************************************
//
//! Initializes the MPU9X50, AK8963, and I2C master drivers.
//!
//! \param psMPU9X50Inst is a pointer to the MPU9X50 driver instance to be
//! initialized.
//! \param psAK8963Inst is a pointer to the AK8963 driver instance to be
//! initialized.
//! \param psI2CMInst is a pointer to the I2C master driver instance to be
//! initialized.
//!
//! This function initializes the drivers for the MPU9X50, AK8963, and I2C
//! master instances. The structures are passed in, although this could be
//! changed to simply use the static global pointers. Calls the specific
//! driver init functions as well as writing settings to the devices and
//! assigning interrupts.
//!
//! \return None.
//
//*****************************************************************************
void
IMUInit(tMPU9X50 *psMPU9X50Inst, tAK8963 *psAK8963Inst,
        tI2CMInstance *psI2CMInst)
{
    sg_psMPU9X50Inst = psMPU9X50Inst;
    sg_psAK8963Inst = psAK8963Inst;
    sg_psI2CMInst = psI2CMInst;

    //
    // Enable I2C & associated GPIO ports. Register their interrupts
    //
    I2CInit(MPU9X50_I2C_BASE, I2C_SPEED_400);
    I2CIntRegister(MPU9X50_I2C_BASE, MPU9X50I2CIntHandler);

    //
    // Initialize I2C driver library
    //
    I2CMInit(sg_psI2CMInst, MPU9X50_I2C_BASE, MPU9X50_I2C_INT, 0xff, 0xff, MAP_SysCtlClockGet());

    //
    // Initialize the MPU9X50 Driver.
    //
    MPU9X50Init(sg_psMPU9X50Inst, sg_psI2CMInst, MPU9X50_I2C_ADDRESS, MPU9X50AppCallback, sg_psMPU9X50Inst);

    //
    // Wait for transaction to complete
    //
    MPU9X50AppI2CWait(__FILE__, __LINE__);

    //
    // Write application specific sensor configuration such as filter settings
    // and sensor range settings.
    //
    sg_psMPU9X50Inst->pui8Data[0] = MPU9X50_CONFIG_DLPF_CFG_260_256;
    sg_psMPU9X50Inst->pui8Data[1] = MPU9X50_GYRO_CONFIG_FS_SEL_2000;
    sg_psMPU9X50Inst->pui8Data[2] = (MPU9X50_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                  MPU9X50_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9X50Write(sg_psMPU9X50Inst, MPU9X50_O_CONFIG, sg_psMPU9X50Inst->pui8Data, 3, MPU9X50AppCallback, sg_psMPU9X50Inst);

    //
    // Wait for transaction to complete
    //
    MPU9X50AppI2CWait(__FILE__, __LINE__);

    //
    // Kill the internal master mode
    //
    sg_psMPU9X50Inst->pui8Data[0] = 0x0;
    MPU9X50Write(sg_psMPU9X50Inst, MPU9X50_O_USER_CTRL,
                 sg_psMPU9X50Inst->pui8Data, 1, MPU9X50AppCallback,
                 sg_psMPU9X50Inst);

    MPU9X50AppI2CWait(__FILE__, __LINE__);

    //
    // Configure the data ready interrupt pin output of the MPU9X50.
    //
    sg_psMPU9X50Inst->pui8Data[0] = MPU9X50_INT_PIN_CFG_INT_LEVEL |
                                    MPU9X50_INT_PIN_CFG_INT_RD_CLEAR |
                                    MPU9X50_INT_PIN_CFG_LATCH_INT_EN |
                                    MPU9X50_INT_PIN_CFG_I2C_BYPASS_EN;
    sg_psMPU9X50Inst->pui8Data[1] = MPU9X50_INT_ENABLE_DATA_RDY_EN;
    MPU9X50Write(sg_psMPU9X50Inst, MPU9X50_O_INT_PIN_CFG,
                 sg_psMPU9X50Inst->pui8Data, 2, MPU9X50AppCallback,
                 sg_psMPU9X50Inst);

    //
    // Wait for transaction to complete
    //
    MPU9X50AppI2CWait(__FILE__, __LINE__);

    AK8963Init(sg_psAK8963Inst, sg_psI2CMInst, 0x0C, AK8963AppCallback, sg_psAK8963Inst);

    AK8963AppI2CWait(__FILE__, __LINE__);

    sg_psAK8963Inst->pui8Data[0] = AK8963_CNTL_MODE_CONT_2;
    AK8963Write(sg_psAK8963Inst, AK8963_O_CNTL,
                sg_psAK8963Inst->pui8Data, 1, AK8963AppCallback,
                sg_psAK8963Inst);

    AK8963AppI2CWait(__FILE__, __LINE__);

    //
    // Initialize the MPU9X50 Interrupt Pins
    //
    GPIOInputInit(MPU9X50_INT_BASE, MPU9X50_INT_PIN, GPIO_PIN_TYPE_STD);
    GPIOIntInit(MPU9X50_INT_BASE, MPU9X50_INT_PIN, GPIO_FALLING_EDGE, -1, MPU9X50IntHandler);

    //
    // MPU9X50 Transactions complete
    //
    sg_vui8MPUI2CDoneFlag = true;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

