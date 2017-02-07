/*
 * ak8963.h
 *
 *  Created on: Jan 31, 2017
 *      Author: moncadac
 */

#ifndef DRIVERS_AK8963_H_
#define DRIVERS_AK8963_H_

//*****************************************************************************
//
// The structure that defines the internal state of the AK8963 driver.
//
//*****************************************************************************
typedef struct
{
    //
    // The pointer to the I2C master interface instance used to communicate
    // with the AK8963.
    //
    tI2CMInstance *psI2CInst;

    //
    // The I2C address of the AK8963.
    //
    uint8_t ui8Addr;

    //
    // The state of the state machine used while accessing the AK8963.
    //
    uint8_t ui8State;

    //
    // The data buffer used for sending/receiving data to/from the AK8963.
    //
    uint8_t pui8Data[8];

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
        uint8_t pui8Buffer[2];

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
}
tAK8963;

//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
extern uint_fast8_t AK8963Init(tAK8963 *psInst, tI2CMInstance *psI2CInst,
                               uint_fast8_t ui8I2CAddr,
                               tSensorCallback *pfnCallback,
                               void *pvCallbackData);
extern uint_fast8_t AK8963Read(tAK8963 *psInst, uint_fast8_t ui8Reg,
                               uint8_t *pui8Data, uint_fast16_t ui16Count,
                               tSensorCallback *pfnCallback,
                               void *pvCallbackData);
extern uint_fast8_t AK8963Write(tAK8963 *psInst, uint_fast8_t ui8Reg,
                                uint8_t *pui8Data, uint_fast16_t ui16Count,
                                tSensorCallback *pfnCallback,
                                void *pvCallbackData);
extern uint_fast8_t AK8963ReadModifyWrite(tAK8963 *psInst,
                                          uint_fast8_t ui8Reg,
                                          uint_fast8_t ui8Mask,
                                          uint_fast8_t ui8Value,
                                          tSensorCallback *pfnCallback,
                                          void *pvCallbackData);
extern uint_fast8_t AK8963DataRead(tAK8963 *psInst,
                                   tSensorCallback *pfnCallback,
                                   void *pvCallbackData);
extern void AK8963DataMagnetoGetRaw(tAK8963 *psInst,
                                    uint_fast16_t *pui16MagnetoX,
                                    uint_fast16_t *pui16MagnetoY,
                                    uint_fast16_t *pui16MagnetoZ);
extern void AK8963DataMagnetoGetFloat(tAK8963 *psInst,
                                      float *pfMagnetoX,
                                      float *pfMagnetoY,
                                      float *pfMagnetoZ);
extern void AK8963DataGetStatus(tAK8963 *psInst, uint_fast8_t *pui8Status1,
                                uint_fast8_t *pui8Status2);



#endif /* DRIVERS_AK8963_H_ */
