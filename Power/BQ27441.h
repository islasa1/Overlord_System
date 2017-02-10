/*
 * BQ27441.h
 *
 *  Created on: Feb 6, 2017
 *      Author: bergej
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "HW_BQ27441.h"
#include "sensorlib/i2cm_drv.h"

#ifndef __POWER_BQ27441_H__
#define __POWER_BQ27441_H__

//*****************************************************************************
//
// The structure that defines the internal state of the BQ27441 driver.
//
//*****************************************************************************
typedef struct
{
    //
    // The pointer to the I2C master interface instance used to communicate
    // with the BQ27441.
    //
    tI2CMInstance *psI2CInst;

    //
    // The I2C address of the BQ27441.
    //
    uint8_t ui8Addr;

    //
    // The state of the state machine used while accessing the BQ27441.
    //
    uint8_t ui8State;

    //
    // The data buffer used for sending/receiving data to/from the BQ27441.
    //
    uint8_t pui8Data[32];

    //
    // The function that is called when the current request has completed
    // processing.
    //
    tSensorCallback *pfnCallback;

    //
    // The pointer provided to the callback function.
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
        uint8_t pui8Buffer[4];

        //
        // The read state used to read register values.
        //
        tI2CMRead16BE sReadState;

        //
        // The write state used to write register values.
        //
        tI2CMWrite16BE sWriteState;

        //
        // The read-modify-write state used to modify register values.
        //
        tI2CMReadModifyWrite16 sReadModifyWriteState;
    }
    uCommand;
}
tBQ27441;

//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
extern void BQ27441Callback(void *pvCallbackData, uint_fast8_t ui8Status);
extern void AppCallback(void *pvCallbackData, uint_fast8_t ui8Status);
extern uint_fast8_t BQ27441Init(tBQ27441 *psInst, tI2CMInstance *psI2CInst,
                                  uint_fast8_t ui8I2CAddr,
                                  tSensorCallback *pfnCallback,
                                  void *pvCallbackData);
extern uint_fast8_t BQ27441Read(tBQ27441 *psInst, uint_fast8_t ui8Reg,
                                  uint16_t *pui16Data, uint_fast16_t ui16Count,
                                  tSensorCallback *pfnCallback,
                               void *pvCallbackData);
extern uint_fast8_t BQ27441Write(tBQ27441 *psInst, uint_fast8_t ui8Reg,
                                   const uint16_t *pui16Data,
                                   uint_fast16_t ui16Count,
                                   tSensorCallback *pfnCallback,
                                   void *pvCallbackData);
extern uint_fast8_t BQ27441ReadModifyWrite(tBQ27441 *psInst,
                                             uint_fast8_t ui8Reg,
                                             uint_fast16_t ui16Mask,
                                             uint_fast16_t ui16Value,
                                             tSensorCallback *pfnCallback,
                                             void *pvCallbackData);
extern uint_fast8_t BQ27441DataRead(tBQ27441 *psInst,
                                      tSensorCallback *pfnCallback,
                                      void *pvCallbackData);

extern void BQ27441DataBatteryTemperatureGetRaw(tBQ27441 *psInst, int16_t *pui16Data);
extern void BQ27441DataBatteryTemperatureGetFloat(tBQ27441 *psInst, float *pfData);

extern void BQ27441DataBatteryVoltageGetRaw(tBQ27441 *psInst, int16_t *pui16Data);
extern void BQ27441DataBatteryVoltageGetFloat(tBQ27441 *psInst, float *pfData);

extern void BQ27441DataNominalAvailableCapacityGetRaw(tBQ27441 *psInst, int16_t *pui16Data);

extern void BQ27441DataFullAvailableCapacityGetRaw(tBQ27441 *psInst, int16_t *pui16Data);

extern void BQ27441DataRemainingCapacityGetRaw(tBQ27441 *psInst, int16_t *pui16Data);

extern void BQ27441DataFullChargeCapacityGetRaw(tBQ27441 *psInst, int16_t *pui16Data);

extern void BQ27441DataAverageCurrentGetRaw(tBQ27441 *psInst, int16_t *pui16Data);

extern void BQ27441DataStandbyCurrentGetRaw(tBQ27441 *psInst, int16_t *pui16Data);

extern void BQ27441DataMaxLoadCurrentGetRaw(tBQ27441 *psInst, int16_t *pui16Data);

extern void BQ27441DataAveragePowerGetRaw(tBQ27441 *psInst, int16_t *pui16Data);

extern void BQ27441DataStateOfChargeGetRaw(tBQ27441 *psInst, int16_t *pui16Data);

extern void BQ27441DataInternalTemperatureGetRaw(tBQ27441 *psInst, int16_t *pui16Data);
extern void BQ27441DataInternalTemperatureGetFloat(tBQ27441 *psInst, float *pfData);

extern void BQ27441DataStateOfHealthGetRaw(tBQ27441 *psInst, int16_t *pui16Data);


#endif /* __POWER_BQ27441_H__ */
