/*
 * IMU.h
 *
 *  Created on: Feb 8, 2017
 *      Author: moncadac
 */

#ifndef SENSORS_MPU9250_DRV_H_
#define SENSORS_MPU9250_DRV_H_

#define AK8963_I2C_ADDRESS 0x68
#define MPU9X50_I2C_ADDRESS 0x68
#define MPU9X50_INT_BASE GPIO_PORTE_BASE
#define MPU9X50_INT_PIN  GPIO_PIN_0
#define MPU9X50_I2C_BASE  I2C2_BASE
#define MPU9X50_I2C_INT   INT_I2C2

#include <stdint.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_ak8963.h"
#include "sensorlib/ak8963.h"

#include "../peripherals/hw_mpu9x50.h"
#include "../peripherals/mpu9x50.h"

extern void MPU9X50AppI2CWait(char *pcFilename, uint_fast32_t ui32Line);
extern void AK8963AppI2CWait(char *pcFilename, uint_fast32_t ui32Line);
extern void MPU9X50AppCallback(void *pvCallbackData, uint_fast8_t ui8Status);
extern void AK8963AppCallback(void *pvCallbackData, uint_fast8_t ui8Status);
extern void MPU9X50I2CIntHandler(void);
extern void GPIOeIntHandler(void);
extern void IMUInit(tMPU9X50 *psMPU9X50Inst, tAK8963 *psAK8963Inst,
             tI2CMInstance *psI2CInst);
extern void IMUDataRead(void);
extern void IMUDataGetFloat(float *pfAccel, float *pfGyro, float *pfMag);

#endif /* SENSORS_MPU9250_DRV_H_ */
