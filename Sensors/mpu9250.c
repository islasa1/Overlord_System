/*
 * mpu9250.c
 *
 *  Created on: Jan 31, 2017
 *      Author: moncadac
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "hw_ak8963.h"
#include "hw_mpu9250.h"
#include "sensorlib/i2cm_drv.h"
#include "ak8963.h"
#include "mpu9250.h"

//*****************************************************************************
//
// The states of the MPU9250 state machine.
//
//*****************************************************************************
#define MPU9250_STATE_IDLE      0           // State machine is idle
#define MPU9250_STATE_LAST      1           // Last step in a sequence
#define MPU9250_STATE_READ      2           // Waiting for read
#define MPU9250_STATE_WRITE     3           // Waiting for write
#define MPU9250_STATE_RMW       4           // Waiting for read modify write
#define MPU9250_STATE_INIT_RESET                                              \
                                5           // reset request issued.
#define MPU9250_STATE_INIT_RESET_WAIT                                         \
                                6           // polling wait for reset complete
#define MPU9250_STATE_INIT_PWR_MGMT                                           \
                                7           // wake up the device.
#define MPU9250_STATE_INIT_USER_CTRL                                          \
                                8           // init user control
#define MPU9250_STATE_INIT_SAMPLE_RATE_CFG                                    \
                                9           // init the sensors and filters
#define MPU9250_STATE_INIT_I2C_SLAVE_DLY                                      \
                                10          // set the ak8975 polling delay
#define MPU9250_STATE_INIT_I2C_SLAVE_0                                        \
                                11          // config ak8975 automatic read
#define MPU9250_STATE_RD_DATA   12          // Waiting for data read

//*****************************************************************************
//
// The factors used to convert the acceleration readings from the MPU9250 into
// floating point values in meters per second squared.
//
// Values are obtained by taking the g conversion factors from the data sheet
// and multiplying by 9.81 (1 g = 9.81 m/s^2).
//
//*****************************************************************************
static const float g_fMPU9250AccelFactors[] =
{
    0.0005985482,                           // Range = +/- 2 g (16384 lsb/g)
    0.0011970964,                           // Range = +/- 4 g (8192 lsb/g)
    0.0023941928,                           // Range = +/- 8 g (4096 lsb/g)
    0.0047883855                            // Range = +/- 16 g (2048 lsb/g)
};

//*****************************************************************************
//
// The factors used to convert the acceleration readings from the MPU9250 into
// floating point values in radians per second.
//
// Values are obtained by taking the degree per second conversion factors
// from the data sheet and then converting to radians per sec (1 degree =
// 0.0174532925 radians).
//
//*****************************************************************************
static const float g_fMPU9250GyroFactors[] =
{
    1.3323124e-4,                           // Range = +/- 250 dps (131.0)
    2.6646248e-4,                           // Range = +/- 500 dps (65.5)
    5.3211258e-4,                           // Range = +/- 1000 dps (32.8)
    0.0010642252                            // Range = +/- 2000 dps (16.4)
};

//*****************************************************************************
//
// Converting sensor data to tesla (0.6 uT per LSB)
//
//*****************************************************************************
#define CONVERT_TO_TESLA        0.0000006


//*****************************************************************************
//
// The callback function that is called when I2C transations to/from the
// MPU9250 have completed.
//
//*****************************************************************************
static void
MPU9250Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    tMPU9250 *psInst;

    //
    // Convert the instance data into a pointer to a tMPU9250 structure.
    //
    psInst = pvCallbackData;

    //
    // If the I2C master driver encountered a failure, force the state machine
    // to the idle state (which will also result in a callback to propagate the
    // error). Except in the case that we are in the reset wait state and the
    // error is an address NACK.  This error is handled by the reset wait
    // state.
    //
    if((ui8Status != I2CM_STATUS_SUCCESS) &&
       !((ui8Status == I2CM_STATUS_ADDR_NACK) &&
         (psInst->ui8State == MPU9250_STATE_INIT_RESET_WAIT)))
    {
        psInst->ui8State = MPU9250_STATE_IDLE;
    }

    //
    // Determine the current state of the MPU9250 state machine.
    //
    switch(psInst->ui8State)
    {
        //
        // All states that trivially transition to IDLE, and all unknown
        // states.
        //
        case MPU9250_STATE_READ:
        case MPU9250_STATE_LAST:
        case MPU9250_STATE_RD_DATA:
        default:
        {
            //
            // The state machine is now idle.
            //
            psInst->ui8State = MPU9250_STATE_IDLE;

            //
            // Done.
            //
            break;
        }

        //
        // MPU9250 Device reset was issued
        //
        case MPU9250_STATE_INIT_RESET:
        {
            //
            // Issue a read of the status register to confirm reset is done.
            //
            psInst->uCommand.pui8Buffer[0] = MPU9250_O_PWR_MGMT_1;
            I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                     psInst->uCommand.pui8Buffer, 1, psInst->pui8Data, 1,
                     MPU9250Callback, psInst);

            psInst->ui8State = MPU9250_STATE_INIT_RESET_WAIT;
            SysCtlDelay(1000);
            break;
        }

        //
        // Status register was read, check if reset is done before proceeding.
        //
        case MPU9250_STATE_INIT_RESET_WAIT:
        {
            //
            // Check the value read back from status to determine if device
            // is still in reset or if it is ready.  Reset state for this
            // register is 0x01, which has sleep bit set. Device may also
            // respond with an address NACK during very early stages of the
            // its internal reset.  Keep polling until we verify device is
            // ready.
            //
            if((psInst->pui8Data[0] != MPU9250_PWR_MGMT_1_CLKSEL_XG) ||
               (ui8Status == I2CM_STATUS_ADDR_NACK))
            {
                //
                // Device still in reset so begin polling this register.
                //
                psInst->uCommand.pui8Buffer[0] = MPU9250_O_PWR_MGMT_1;
                I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                         psInst->uCommand.pui8Buffer, 1, psInst->pui8Data, 1,
                         MPU9250Callback, psInst);

                //
                // Intentionally stay in this state to create polling effect.
                //
            }
            else
            {
                //
                // Device is out of reset, bring it out of sleep mode.
                //
                psInst->uCommand.pui8Buffer[0] = MPU9250_O_PWR_MGMT_1;
                psInst->uCommand.pui8Buffer[1] = MPU9250_PWR_MGMT_1_CLKSEL_XG;
                I2CMWrite(psInst->psI2CInst, psInst->ui8Addr,
                          psInst->uCommand.pui8Buffer, 2, MPU9250Callback,
                          psInst);

                //
                // Update state to show we are modifing user control and
                // power management 1 regs.
                //
                SysCtlDelay(1000);
                psInst->ui8State = MPU9250_STATE_INIT_PWR_MGMT;
            }
            break;
        }

        //
        // Reset complete now take device out of sleep mode.
        //
        case MPU9250_STATE_INIT_PWR_MGMT:
        {
            psInst->uCommand.pui8Buffer[0] = MPU9250_O_USER_CTRL;
            psInst->uCommand.pui8Buffer[1] = 0x00;//MPU9250_USER_CTRL_I2C_MST_EN;
            I2CMWrite(psInst->psI2CInst, psInst->ui8Addr,
                      psInst->uCommand.pui8Buffer, 2, MPU9250Callback,
                      psInst);

            //
            // Update state to show we are modifying user control and
            // power management 1 regs.
            //
            psInst->ui8State = MPU9250_STATE_INIT_SAMPLE_RATE_CFG;
            SysCtlDelay(1000);

            break;
        }

        //
        // Change to power mode complete, device is ready for configuration.
        //
        case MPU9250_STATE_INIT_USER_CTRL:
        {
            //
            // Load index 0 with the sample rate register number.
            //
            psInst->uCommand.pui8Buffer[0] = MPU9250_O_SMPLRT_DIV;

            //
            // Set sample rate to 50 hertz.  1000 hz / (1 + 19)
            //
            psInst->uCommand.pui8Buffer[1] = 19;

            I2CMWrite(psInst->psI2CInst, psInst->ui8Addr,
                      psInst->uCommand.pui8Buffer, 2, MPU9250Callback, psInst);

            //
            // update state to show are in process of configuring sensors.
            //
            psInst->ui8State = MPU9250_STATE_LAST;
            SysCtlDelay(1000);
            break;
        }

        //
        // Sensor configuration is complete.
        //
        case MPU9250_STATE_INIT_SAMPLE_RATE_CFG:
        {
            //
            // Write the I2C Master delay control so we only sample the AK
            // every 5th time that we sample accel/gyro.  Delay Count itself
            // handled in next state.
            //
            psInst->uCommand.pui8Buffer[0] = MPU9250_O_I2C_MST_DELAY_CTRL;
            psInst->uCommand.pui8Buffer[1] =
                (MPU9250_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN |
                 MPU9250_I2C_MST_DELAY_CTRL_I2C_SLV4_DLY_EN);
            I2CMWrite(psInst->psI2CInst, psInst->ui8Addr,
                      psInst->uCommand.pui8Buffer, 2, MPU9250Callback, psInst);

            //
            // Update state to show we are configuring i2c slave delay between
            // slave events.  Slave 0 and Slave 4 transaction only occur every
            // 5th sample cycle.
            //
            psInst->ui8State = MPU9250_STATE_INIT_I2C_SLAVE_DLY;
            SysCtlDelay(1000);
            break;
        }

        //
        // Master slave delay configuration complete.
        //
        case MPU9250_STATE_INIT_I2C_SLAVE_DLY:
        {
            //
            // Write the configuration for I2C master control clock 400khz
            // and wait for external sensor before asserting data ready
            //
            psInst->uCommand.pui8Buffer[0] = MPU9250_O_I2C_MST_CTRL;
            psInst->uCommand.pui8Buffer[1] =
                (MPU9250_I2C_MST_CTRL_I2C_MST_CLK_400);/* |
                 MPU9250_I2C_MST_CTRL_WAIT_FOR_ES);*/

            //
            // Configure I2C Slave 0 for read of AK8963 (I2C Address 0x0C)
            // Start at AK8963 register status 1
            // Read 8 bytes and enable this slave transaction
            //
            psInst->uCommand.pui8Buffer[2] = MPU9250_I2C_SLV0_ADDR_RW | 0x0C;
            psInst->uCommand.pui8Buffer[3] = AK8963_O_ST1;
            psInst->uCommand.pui8Buffer[4] = MPU9250_I2C_SLV0_CTRL_EN | 0x08;
            I2CMWrite(psInst->psI2CInst, psInst->ui8Addr,
                      psInst->uCommand.pui8Buffer, 5, MPU9250Callback, psInst);

            //
            // Update state.  Now in process of configuring slave 0.
            //
            psInst->ui8State = MPU9250_STATE_INIT_I2C_SLAVE_0;
            SysCtlDelay(1000);
            break;
        }

        //
        // I2C slave 0 init complete.
        //
        case MPU9250_STATE_INIT_I2C_SLAVE_0:
        {
            //
            // Write the configuration for I2C Slave 4 transaction to AK8963
            // 0x0c is the AK8963 address on i2c bus.
            // we want to write the control register with the value for a
            // starting a single measurement.
            //
            psInst->uCommand.pui8Buffer[0] = MPU9250_O_I2C_SLV4_ADDR;
            psInst->uCommand.pui8Buffer[1] = 0X0C;
            psInst->uCommand.pui8Buffer[2] = AK8963_O_CNTL;
            psInst->uCommand.pui8Buffer[3] = AK8963_CNTL_MODE_SINGLE;

            //
            // Enable the SLV4 transaction and set the master delay to
            // 0x04 + 1.  This means the slave transactions with delay enabled
            // will run every fifth accel/gyro sample.
            //
            psInst->uCommand.pui8Buffer[4] = MPU9250_I2C_SLV4_CTRL_EN | 0x04;
            I2CMWrite(psInst->psI2CInst, psInst->ui8Addr,
                      psInst->uCommand.pui8Buffer, 5, MPU9250Callback, psInst);

            //
            // Update state.  Now in the final init state.
            //
            psInst->ui8State = MPU9250_STATE_LAST;
            SysCtlDelay(1000);
            break;
        }

        //
        // A write just completed
        //
        case MPU9250_STATE_WRITE:
        {
            //
            // Set the accelerometer and gyroscope ranges to the new values.
            // If the register was not modified, the values will be the same so
            // this has no effect.
            //
            psInst->ui8AccelAfsSel = psInst->ui8NewAccelAfsSel;
            psInst->ui8GyroFsSel = psInst->ui8NewGyroFsSel;

            //
            // The state machine is now idle.
            //
            psInst->ui8State = MPU9250_STATE_IDLE;

            //
            // Done.
            //
            break;
        }

        //
        // A read-modify-write just completed
        //
        case MPU9250_STATE_RMW:
        {
            //
            // See if the PWR_MGMT_1 register was just modified.
            //
            if(psInst->uCommand.sReadModifyWriteState.pui8Buffer[0] ==
               MPU9250_O_PWR_MGMT_1)
            {
                //
                // See if a soft reset has been issued.
                //
                if(psInst->uCommand.sReadModifyWriteState.pui8Buffer[1] &
                   MPU9250_PWR_MGMT_1_DEVICE_RESET)
                {
                    //
                    // Default range setting is +/- 2 g
                    //
                    psInst->ui8AccelAfsSel = 0;
                    psInst->ui8NewAccelAfsSel = 0;

                    //
                    // Default range setting is +/- 250 degrees/s
                    //
                    psInst->ui8GyroFsSel = 0;
                    psInst->ui8NewGyroFsSel = 0;
                }
            }

            //
            // See if the GYRO_CONFIG register was just modified.
            //
            if(psInst->uCommand.sReadModifyWriteState.pui8Buffer[0] ==
               MPU9250_O_GYRO_CONFIG)
            {
                //
                // Extract the FS_SEL from the GYRO_CONFIG register value.
                //
                psInst->ui8GyroFsSel =
                    ((psInst->uCommand.sReadModifyWriteState.pui8Buffer[1] &
                      MPU9250_GYRO_CONFIG_FS_SEL_M) >>
                     MPU9250_GYRO_CONFIG_FS_SEL_S);
            }

            //
            // See if the ACCEL_CONFIG register was just modified.
            //
            if(psInst->uCommand.sReadModifyWriteState.pui8Buffer[0] ==
               MPU9250_O_ACCEL_CONFIG)
            {
                //
                // Extract the FS_SEL from the ACCEL_CONFIG register value.
                //
                psInst->ui8AccelAfsSel =
                    ((psInst->uCommand.sReadModifyWriteState.pui8Buffer[1] &
                      MPU9250_ACCEL_CONFIG_AFS_SEL_M) >>
                     MPU9250_ACCEL_CONFIG_AFS_SEL_S);
            }

            //
            // The state machine is now idle.
            //
            psInst->ui8State = MPU9250_STATE_IDLE;

            //
            // Done.
            //
            break;
        }
    }

    //
    // See if the state machine is now idle and there is a callback function.
    //
    if((psInst->ui8State == MPU9250_STATE_IDLE) && psInst->pfnCallback)
    {
        //
        // Call the application-supplied callback function.
        //
        psInst->pfnCallback(psInst->pvCallbackData, ui8Status);
    }
}

//*****************************************************************************
//
//! Initializes the MPU9250 driver.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param psI2CInst is a pointer to the I2C master driver instance data.
//! \param ui8I2CAddr is the I2C address of the MPU9250 device.
//! \param pfnCallback is the function to be called when the initialization has
//! completed (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function initializes the MPU9250 driver, preparing it for operation.
//!
//! \return Returns 1 if the MPU9250 driver was successfully initialized and 0
//! if it was not.
//
//*****************************************************************************
uint_fast8_t
MPU9250Init(tMPU9250 *psInst, tI2CMInstance *psI2CInst,
            uint_fast8_t ui8I2CAddr, tSensorCallback *pfnCallback,
            void *pvCallbackData)
{
    //
    // Initialize the MPU9250 instance structure.
    //
    psInst->psI2CInst = psI2CInst;
    psInst->ui8Addr = ui8I2CAddr;

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Default range setting is +/- 2 g
    //
    psInst->ui8AccelAfsSel = (MPU9250_ACCEL_CONFIG_AFS_SEL_2G >>
                              MPU9250_ACCEL_CONFIG_AFS_SEL_S);
    psInst->ui8NewAccelAfsSel = (MPU9250_ACCEL_CONFIG_AFS_SEL_2G >>
                                 MPU9250_ACCEL_CONFIG_AFS_SEL_S);

    //
    // Default range setting is +/- 250 degrees/s
    //
    psInst->ui8GyroFsSel = (MPU9250_GYRO_CONFIG_FS_SEL_250 >>
                            MPU9250_GYRO_CONFIG_FS_SEL_S);
    psInst->ui8NewGyroFsSel = (MPU9250_GYRO_CONFIG_FS_SEL_250 >>
                               MPU9250_GYRO_CONFIG_FS_SEL_S);

    //
    // Set the state to show we are initiating a reset.
    //
    psInst->ui8State = MPU9250_STATE_INIT_RESET;

    //
    // Load the buffer with command to perform device reset
    //
    psInst->uCommand.pui8Buffer[0] = MPU9250_O_PWR_MGMT_1;
    psInst->uCommand.pui8Buffer[1] = MPU9250_PWR_MGMT_1_DEVICE_RESET;
    SysCtlDelay(10);
    if(I2CMWrite(psInst->psI2CInst, psInst->ui8Addr,
                 psInst->uCommand.pui8Buffer, 2, MPU9250Callback, psInst) == 0)
    {
        psInst->ui8State = MPU9250_STATE_IDLE;
        return(0);
    }

    //
    // Success
    //
    return(1);
}

//*****************************************************************************
//
//! Returns the pointer to the tAK8963 object
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//!
//! The MPU9250 contains in internal AK8963 magnetometer.  To access data from
//! that sensor, application should use this function to get a pointer to the
//! tAK8963 object, and then use the AK8963 APIs.
//!
//! \return Returns the pointer to the tAK8963 object
//
//*****************************************************************************
tAK8963 *
MPU9250MagnetoInstGet(tMPU9250 *psInst)
{
    return(&(psInst->sAK8963Inst));
}

//*****************************************************************************
//
//! Reads data from MPU9250 registers.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param ui8Reg is the first register to read.
//! \param pui8Data is a pointer to the location to store the data that is
//! read.
//! \param ui16Count is the number of data bytes to read.
//! \param pfnCallback is the function to be called when the data has been read
//! (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function reads a sequence of data values from consecutive registers in
//! the MPU9250.
//!
//! \return Returns 1 if the write was successfully started and 0 if it was
//! not.
//
//*****************************************************************************
uint_fast8_t
MPU9250Read(tMPU9250 *psInst, uint_fast8_t ui8Reg, uint8_t *pui8Data,
            uint_fast16_t ui16Count, tSensorCallback *pfnCallback,
            void *pvCallbackData)
{
    //
    // Return a failure if the MPU9250 driver is not idle (in other words,
    // there is already an outstanding request to the MPU9250).
    //
    if(psInst->ui8State != MPU9250_STATE_IDLE)
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
    psInst->ui8State = MPU9250_STATE_READ;

    //
    // Read the requested registers from the MPU9250.
    //
    psInst->uCommand.pui8Buffer[0] = ui8Reg;
    if(I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                psInst->uCommand.pui8Buffer, 1, pui8Data, ui16Count,
                MPU9250Callback, psInst) == 0)
    {
        //
        // The I2C write failed, so move to the idle state and return a
        // failure.
        //
        psInst->ui8State = MPU9250_STATE_IDLE;
        return(0);
    }

    //
    // Success.
    //
    return(1);
}

//*****************************************************************************
//
//! Writes data to MPU9250 registers.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param ui8Reg is the first register to write.
//! \param pui8Data is a pointer to the data to write.
//! \param ui16Count is the number of data bytes to write.
//! \param pfnCallback is the function to be called when the data has been
//! written (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function writes a sequence of data values to consecutive registers in
//! the MPU9250.  The first byte of the \e pui8Data buffer contains the value
//! to be written into the \e ui8Reg register, the second value contains the
//! data to be written into the next register, and so on.
//!
//! \return Returns 1 if the write was successfully started and 0 if it was
//! not.
//
//*****************************************************************************
uint_fast8_t
MPU9250Write(tMPU9250 *psInst, uint_fast8_t ui8Reg, const uint8_t *pui8Data,
             uint_fast16_t ui16Count, tSensorCallback *pfnCallback,
             void *pvCallbackData)
{
    //
    // Return a failure if the MPU9250 driver is not idle (in other words,
    // there is already an outstanding request to the MPU9250).
    //
    if(psInst->ui8State != MPU9250_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // See if the PWR_MGMT_1 register is being written.
    //
    if((ui8Reg <= MPU9250_O_PWR_MGMT_1) &&
       ((ui8Reg + ui16Count) > MPU9250_O_PWR_MGMT_1))
    {
        //
        // See if a soft reset is being requested.
        //
        if(pui8Data[ui8Reg - MPU9250_O_PWR_MGMT_1] &
           MPU9250_PWR_MGMT_1_DEVICE_RESET)
        {
            //
            // Default range setting is +/- 2 g.
            //
            psInst->ui8NewAccelAfsSel = 0;

            //
            // Default range setting is +/- 250 degrees/s.
            //
            psInst->ui8NewGyroFsSel = 0;
        }
    }

    //
    // See if the GYRO_CONFIG register is being written.
    //
    if((ui8Reg <= MPU9250_O_GYRO_CONFIG) &&
       ((ui8Reg + ui16Count) > MPU9250_O_GYRO_CONFIG))
    {
        //
        // Extract the FS_SEL from the GYRO_CONFIG register value.
        //
        psInst->ui8NewGyroFsSel = ((pui8Data[ui8Reg - MPU9250_O_GYRO_CONFIG] &
                                    MPU9250_GYRO_CONFIG_FS_SEL_M) >>
                                   MPU9250_GYRO_CONFIG_FS_SEL_S);
    }

    //
    // See if the ACCEL_CONFIG register is being written.
    //
    if((ui8Reg <= MPU9250_O_ACCEL_CONFIG) &&
       ((ui8Reg + ui16Count) > MPU9250_O_ACCEL_CONFIG))
    {
        //
        // Extract the AFS_SEL from the ACCEL_CONFIG register value.
        //
        psInst->ui8NewAccelAfsSel =
            ((pui8Data[ui8Reg - MPU9250_O_ACCEL_CONFIG] &
              MPU9250_ACCEL_CONFIG_AFS_SEL_M) >>
             MPU9250_ACCEL_CONFIG_AFS_SEL_S);
    }

    //
    // Move the state machine to the wait for write state.
    //
    psInst->ui8State = MPU9250_STATE_WRITE;

    //
    // Write the requested registers to the MPU9250.
    //
    if(I2CMWrite8(&(psInst->uCommand.sWriteState), psInst->psI2CInst,
                  psInst->ui8Addr, ui8Reg, pui8Data, ui16Count,
                  MPU9250Callback, psInst) == 0)
    {
        //
        // The I2C write failed, so move to the idle state and return a
        // failure.
        //
        psInst->ui8State = MPU9250_STATE_IDLE;
        return(0);
    }

    //
    // Success.
    //
    return(1);
}

//*****************************************************************************
//
//! Performs a read-modify-write of a MPU9250 register.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param ui8Reg is the register to modify.
//! \param ui8Mask is the bit mask that is ANDed with the current register
//! value.
//! \param ui8Value is the bit mask that is ORed with the result of the AND
//! operation.
//! \param pfnCallback is the function to be called when the data has been
//! changed (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function changes the value of a register in the MPU9250 via a
//! read-modify-write operation, allowing one of the fields to be changed
//! without disturbing the other fields.  The \e ui8Reg register is read, ANDed
//! with \e ui8Mask, ORed with \e ui8Value, and then written back to the
//! MPU9250.
//!
//! \return Returns 1 if the read-modify-write was successfully started and 0
//! if it was not.
//
//*****************************************************************************
uint_fast8_t
MPU9250ReadModifyWrite(tMPU9250 *psInst, uint_fast8_t ui8Reg,
                       uint_fast8_t ui8Mask, uint_fast8_t ui8Value,
                       tSensorCallback *pfnCallback, void *pvCallbackData)
{
    //
    // Return a failure if the MPU9250 driver is not idle (in other words,
    // there is already an outstanding request to the MPU9250).
    //
    if(psInst->ui8State != MPU9250_STATE_IDLE)
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
    psInst->ui8State = MPU9250_STATE_RMW;

    //
    // Submit the read-modify-write request to the MPU9250.
    //
    if(I2CMReadModifyWrite8(&(psInst->uCommand.sReadModifyWriteState),
                            psInst->psI2CInst, psInst->ui8Addr, ui8Reg,
                            ui8Mask, ui8Value, MPU9250Callback, psInst) == 0)
    {
        //
        // The I2C read-modify-write failed, so move to the idle state and
        // return a failure.
        //
        psInst->ui8State = MPU9250_STATE_IDLE;
        return(0);
    }

    //
    // Success.
    //
    return(1);
}

//*****************************************************************************
//
//! Reads the accelerometer and gyroscope data from the MPU9250 and the
//! magnetometer data from the on-chip aK8975.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param pfnCallback is the function to be called when the data has been read
//! (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function initiates a read of the MPU9250 data registers.  When the
//! read has completed (as indicated by calling the callback function), the new
//! readings can be obtained via:
//!
//! - MPU9250DataAccelGetRaw()
//! - MPU9250DataAccelGetFloat()
//! - MPU9250DataGyroGetRaw()
//! - MPU9250DataGyroGetFloat()
//! - MPU9250DataMagnetoGetRaw()
//! - MPU9250DataMagnetoGetFloat()
//!
//! \return Returns 1 if the read was successfully started and 0 if it was not.
//
//*****************************************************************************
uint_fast8_t
MPU9250DataRead(tMPU9250 *psInst, tSensorCallback *pfnCallback,
                void *pvCallbackData)
{
    //
    // Return a failure if the MPU9250 driver is not idle (in other words,
    // there is already an outstanding request to the MPU9250).
    //
    if(psInst->ui8State != MPU9250_STATE_IDLE)
    {
        return(0);
    }

    //
    // Save the callback information.
    //
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Move the state machine to the wait for data read state.
    //
    psInst->ui8State = MPU9250_STATE_RD_DATA;

    //
    // Read the data registers from the MPU9250.
    //
    // (ACCEL_XOUT_H(0x3B) -> GYRO_ZOUT_L(0x48) = 14 bytes
    // Grab Ext Sens Data as well for another 8 bytes.  ST1 + Mag Data + ST2
    //
    psInst->uCommand.pui8Buffer[0] = MPU9250_O_ACCEL_XOUT_H;
    if(I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                psInst->uCommand.pui8Buffer, 1, psInst->pui8Data, 22,
                MPU9250Callback, psInst) == 0)
    {
        //
        // The I2C read failed, so move to the idle state and return a failure.
        //
        psInst->ui8State = MPU9250_STATE_IDLE;
        return(0);
    }

    //
    // Success.
    //
    return(1);
}

//*****************************************************************************
//
//! Gets the raw accelerometer data from the most recent data read.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param pui16AccelX is a pointer to the value into which the raw X-axis
//! accelerometer data is stored.
//! \param pui16AccelY is a pointer to the value into which the raw Y-axis
//! accelerometer data is stored.
//! \param pui16AccelZ is a pointer to the value into which the raw Z-axis
//! accelerometer data is stored.
//!
//! This function returns the raw accelerometer data from the most recent data
//! read.  The data is not manipulated in any way by the driver.  If any of the
//! output data pointers are \b NULL, the corresponding data is not provided.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9250DataAccelGetRaw(tMPU9250 *psInst, uint_fast16_t *pui16AccelX,
                       uint_fast16_t *pui16AccelY, uint_fast16_t *pui16AccelZ)
{
    //
    // Return the raw accelerometer values.
    //
    if(pui16AccelX)
    {
        *pui16AccelX = (psInst->pui8Data[0] << 8) | psInst->pui8Data[1];
    }
    if(pui16AccelY)
    {
        *pui16AccelY = (psInst->pui8Data[2] << 8) | psInst->pui8Data[3];
    }
    if(pui16AccelZ)
    {
        *pui16AccelZ = (psInst->pui8Data[4] << 8) | psInst->pui8Data[5];
    }
}

//*****************************************************************************
//
//! Gets the accelerometer data from the most recent data read.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param pfAccelX is a pointer to the value into which the X-axis
//! accelerometer data is stored.
//! \param pfAccelY is a pointer to the value into which the Y-axis
//! accelerometer data is stored.
//! \param pfAccelZ is a pointer to the value into which the Z-axis
//! accelerometer data is stored.
//!
//! This function returns the accelerometer data from the most recent data
//! read, converted into meters per second squared (m/s^2).  If any of the
//! output data pointers are \b NULL, the corresponding data is not provided.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9250DataAccelGetFloat(tMPU9250 *psInst, float *pfAccelX, float *pfAccelY,
                         float *pfAccelZ)
{
    float fFactor;

    //
    // Get the acceleration conversion factor for the current data format.
    //
    fFactor = g_fMPU9250AccelFactors[psInst->ui8AccelAfsSel];

    //
    // Convert the accelerometer values into m/sec^2
    //
    if(pfAccelX)
    {
        *pfAccelX = ((float)(int16_t)((psInst->pui8Data[0] << 8) |
                                      psInst->pui8Data[1]) * fFactor);
    }
    if(pfAccelY)
    {
        *pfAccelY = ((float)(int16_t)((psInst->pui8Data[2] << 8) |
                                      psInst->pui8Data[3]) * fFactor);
    }
    if(pfAccelZ)
    {
        *pfAccelZ = ((float)(int16_t)((psInst->pui8Data[4] << 8) |
                                      psInst->pui8Data[5]) * fFactor);
    }
}

//*****************************************************************************
//
//! Gets the raw gyroscope data from the most recent data read.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param pui16GyroX is a pointer to the value into which the raw X-axis
//! gyroscope data is stored.
//! \param pui16GyroY is a pointer to the value into which the raw Y-axis
//! gyroscope data is stored.
//! \param pui16GyroZ is a pointer to the value into which the raw Z-axis
//! gyroscope data is stored.
//!
//! This function returns the raw gyroscope data from the most recent data
//! read.  The data is not manipulated in any way by the driver.  If any of the
//! output data pointers are \b NULL, the corresponding data is not provided.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9250DataGyroGetRaw(tMPU9250 *psInst, uint_fast16_t *pui16GyroX,
                      uint_fast16_t *pui16GyroY, uint_fast16_t *pui16GyroZ)
{
    //
    // Return the raw gyroscope values.
    //
    if(pui16GyroX)
    {
        *pui16GyroX = (psInst->pui8Data[8] << 8) | psInst->pui8Data[9];
    }
    if(pui16GyroY)
    {
        *pui16GyroY = (psInst->pui8Data[10] << 8) | psInst->pui8Data[11];
    }
    if(pui16GyroZ)
    {
        *pui16GyroZ = (psInst->pui8Data[12] << 8) | psInst->pui8Data[13];
    }
}

//*****************************************************************************
//
//! Gets the gyroscope data from the most recent data read.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param pfGyroX is a pointer to the value into which the X-axis
//! gyroscope data is stored.
//! \param pfGyroY is a pointer to the value into which the Y-axis
//! gyroscope data is stored.
//! \param pfGyroZ is a pointer to the value into which the Z-axis
//! gyroscope data is stored.
//!
//! This function returns the gyroscope data from the most recent data read,
//! converted into radians per second.  If any of the output data pointers are
//! \b NULL, the corresponding data is not provided.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9250DataGyroGetFloat(tMPU9250 *psInst, float *pfGyroX, float *pfGyroY,
                        float *pfGyroZ)
{
    float fFactor;
    int16_t i16Temp;

    //
    // Get the gyroscope conversion factor for the current data format.
    //
    fFactor = g_fMPU9250GyroFactors[psInst->ui8GyroFsSel];

    //
    // Convert the gyroscope values into rad/sec
    //
    if(pfGyroX)
    {
        i16Temp = (int16_t)psInst->pui8Data[8];
        i16Temp <<= 8;
        i16Temp += psInst->pui8Data[9];
        *pfGyroX = (float)i16Temp;
        *pfGyroX *= fFactor;
    }
    if(pfGyroY)
    {
        i16Temp = (int16_t)psInst->pui8Data[10];
        i16Temp <<= 8;
        i16Temp += psInst->pui8Data[11];
        *pfGyroY = (float)i16Temp;
        *pfGyroY *= fFactor;
    }
    if(pfGyroZ)
    {
        i16Temp = (int16_t)psInst->pui8Data[12];
        i16Temp <<= 8;
        i16Temp += psInst->pui8Data[13];
        *pfGyroZ = (float)i16Temp;
        *pfGyroZ *= fFactor;
    }
}

//*****************************************************************************
//
//! Gets the raw magnetometer data from the most recent data read.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param pui16MagnetoX is a pointer to the value into which the raw X-axis
//! magnetometer data is stored.
//! \param pui16MagnetoY is a pointer to the value into which the raw Y-axis
//! magnetometer data is stored.
//! \param pui16MagnetoZ is a pointer to the value into which the raw Z-axis
//! magnetometer data is stored.
//!
//! This function returns the raw magnetometer data from the most recent data
//! read.  The data is not manipulated in any way by the driver.  If any of the
//! output data pointers are \b NULL, the corresponding data is not provided.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9250DataMagnetoGetRaw(tMPU9250 *psInst, uint_fast16_t *pui16MagnetoX,
                         uint_fast16_t *pui16MagnetoY,
                         uint_fast16_t *pui16MagnetoZ)
{
    uint8_t *pui8ExtSensData;

    pui8ExtSensData = &(psInst->pui8Data[14]);

    //
    // Return the raw magnetometer values.
    //
    if(pui16MagnetoX)
    {
        *pui16MagnetoX = (pui8ExtSensData[2] << 8) | pui8ExtSensData[1];
    }
    if(pui16MagnetoY)
    {
        *pui16MagnetoY = (pui8ExtSensData[4] << 8) | pui8ExtSensData[3];
    }
    if(pui16MagnetoZ)
    {
        *pui16MagnetoZ = (pui8ExtSensData[6] << 8) | pui8ExtSensData[5];
    }
}

//*****************************************************************************
//
//! Gets the magnetometer data from the most recent data read.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param pfMagnetoX is a pointer to the value into which the X-axis
//! magnetometer data is stored.
//! \param pfMagnetoY is a pointer to the value into which the Y-axis
//! magnetometer data is stored.
//! \param pfMagnetoZ is a pointer to the value into which the Z-axis
//! magnetometer data is stored.
//!
//! This function returns the magnetometer data from the most recent data read,
//! converted into tesla.  If any of the output data pointers are
//! \b NULL, the corresponding data is not provided.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9250DataMagnetoGetFloat(tMPU9250 *psInst, float *pfMagnetoX,
                           float *pfMagnetoY, float *pfMagnetoZ)
{
    int16_t *pi16Data;

    pi16Data = (int16_t *)(psInst->pui8Data + 15);

    //
    // Convert the magnetometer values into floating-point tesla values.
    //
    if(pfMagnetoX)
    {
        *pfMagnetoX = (float)pi16Data[0];
        *pfMagnetoX *= CONVERT_TO_TESLA;
    }
    if(pfMagnetoY)
    {
        *pfMagnetoY = (float)pi16Data[1];
        *pfMagnetoY *= CONVERT_TO_TESLA;
    }
    if(pfMagnetoZ)
    {
        *pfMagnetoZ = (float)pi16Data[2];
        *pfMagnetoZ *= CONVERT_TO_TESLA;
    }
}

//*****************************************************************************
//
//! Gets the thermometer data from the most recent data read.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param pui16Temp is a pointer to where to store the temperature data.
//!
//! This function returns the thermometer data from the most recent data read.
//!
//! \return None.
//
//*****************************************************************************
void
MPU9250DataTempGetRaw(tMPU9250 *psInst, uint_fast16_t *pui16Temp)
{
    uint8_t *pui8Data;

    pui8Data = &(psInst->pui8Data[6]);

    //
    // Return the raw thermometer value.
    //
    if(pui16Temp)
    {
        *pui16Temp = (pui8Data[0] << 8) | pui8Data[1];
    }
}

//*****************************************************************************
//
//! Gets the thermometer data from the most recent data read.
//!
//! \param psInst is a pointer to the MPU9250 instance data.
//! \param pui16Temp is a pointer to where to store the temperature data.
//!
//! This function returns the thermometer data from the most recent data read,
//! converted into degrees Celsius.
//!
//! \return None.
//
//*****************************************************************************

void
MPU9250DataTempGetFloat(tMPU9250 *psInst, float *pfTemp)
{
    int16_t *pi16Data;

        pi16Data = (int16_t *)(psInst->pui8Data + 5);

        //
        // Convert the thermometer values into floating-point degrees values.
        //
        if(pfTemp)
        {
            *pfTemp = (float)pi16Data[0];
            *pfTemp = (*pfTemp / 331.87) + 21.0;
        }
}
