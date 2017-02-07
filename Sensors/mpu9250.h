/*
 * mpu9250.h
 *
 *  Created on: Jan 31, 2017
 *      Author: moncadac
 */

#ifndef DRIVERS_MPU9250_H_
#define DRIVERS_MPU9250_H_

//*****************************************************************************
//
// Structure that defines the internal state of the MPU9250
//
//*****************************************************************************
typedef struct
{
    //
        // The pointer to the I2C master interface instance used to communicate
        // with the MPU9250.
        //
        tI2CMInstance *psI2CInst;

        //
        // The AK8963 inst that used to access the on-chip AK8975 magnetometer
        //
        tAK8963 sAK8963Inst;

        //
        // The I2C address of the MPU9250.
        //
        uint8_t ui8Addr;

        //
        // The state of the state machine used while accessing the MPU9250.
        //
        uint8_t ui8State;

        //
        // The current accelerometer afs_sel setting
        //
        uint8_t ui8AccelAfsSel;

        //
        // The new accelerometer afs_sel setting, which is used when a register
        // write succeeds.
        //
        uint8_t ui8NewAccelAfsSel;

        //
        // The current gyroscope fs_sel setting
        //
        uint8_t ui8GyroFsSel;

        //
        // The new gyroscope fs_sel setting, which is used when a register write
        // succeeds.
        //
        uint8_t ui8NewGyroFsSel;

        //
        // The data buffer used for sending/receiving data to/from the MPU9250.
        //
        uint8_t pui8Data[24];

        //
        // The function that is called when the current request has completed
        // processing.
        //
        tSensorCallback *pfnCallback;

        //
        // The callback data provided to the callback function.
        //
        void *pvCallbackData;

        //
        // A union of structures that are used for read, write and
        // read-modify-write operations.  Since only one operation can be active at
        // a time, it is safe to re-use the memory in this manner.
        //
        union
        {
            //
            // A buffer used to store the write portion of a register read.
            //
            uint8_t pui8Buffer[6];

            //
            // The write state used to write register values.
            //
            tI2CMWrite8 sWriteState;

            //
            // The read-modify-write state used to modify register values.
            //
            tI2CMReadModifyWrite8 sReadModifyWriteState;
        }
        uCommand;
} tMPU9250;

//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
extern uint_fast8_t MPU9250Init(tMPU9250 *psInst, tI2CMInstance *psI2CInst,
                               uint_fast8_t ui8I2CAddr,
                               tSensorCallback *pfnCallback,
                               void *pvCallbackData);
extern tAK8963 *MPU9250InstAK8975Get(tMPU9250 *psInst);
extern uint_fast8_t MPU9250Read(tMPU9250 *psInst, uint_fast8_t ui8Reg,
                               uint8_t *pui8Data, uint_fast16_t ui16Count,
                               tSensorCallback *pfnCallback,
                               void *pvCallbackData);
extern uint_fast8_t MPU9250Write(tMPU9250 *psInst, uint_fast8_t ui8Reg,
                                const uint8_t *pui8Data,
                                uint_fast16_t ui16Count,
                                tSensorCallback *pfnCallback,
                                void *pvCallbackData);
extern uint_fast8_t MPU9250ReadModifyWrite(tMPU9250 *psInst,
                                          uint_fast8_t ui8Reg,
                                          uint_fast8_t ui8Mask,
                                          uint_fast8_t ui8Value,
                                          tSensorCallback *pfnCallback,
                                          void *pvCallbackData);
extern uint_fast8_t MPU9250DataRead(tMPU9250 *psInst,
                                   tSensorCallback *pfnCallback,
                                   void *pvCallbackData);
extern void MPU9250DataAccelGetRaw(tMPU9250 *psInst,
                                  uint_fast16_t *pui16AccelX,
                                  uint_fast16_t *pui16AccelY,
                                  uint_fast16_t *pui16AccelZ);
extern void MPU9250DataAccelGetFloat(tMPU9250 *psInst,
                                     float *pfAccelX,
                                     float *pfAccelY,
                                     float *pfAccelZ);
extern void MPU9250DataGyroGetRaw(tMPU9250 *psInst, uint_fast16_t *pui16GyroX,
                                 uint_fast16_t *pui16GyroY,
                                 uint_fast16_t *pui16GyroZ);
extern void MPU9250DataGyroGetFloat(tMPU9250 *psInst,
                                    float *pfGyroX,
                                    float *pfGyroY,
                                    float *pfGyroZ);
extern void MPU9250DataMagnetoGetRaw(tMPU9250 *psInst,
                                    uint_fast16_t *pui16MagnetoX,
                                    uint_fast16_t *pui16MagnetoY,
                                    uint_fast16_t *pui16MagnetoZ);
extern void MPU9250DataMagnetoGetFloat(tMPU9250 *psZInst,
                                       float *pfMagnetoX,
                                       float *pfMagnetoY,
                                       float *pfMagnetoZ);
extern void MPU9250DataTempGetRaw(tMPU9250 *psInst, uint_fast16_t *pui16Temp);
extern void MPU9250DataTempGetFloat(tMPU9250 *psInst, float *pfTemp);

#endif /* DRIVERS_MPU9250_H_ */

