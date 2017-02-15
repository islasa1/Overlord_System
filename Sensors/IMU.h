/*
 * IMU.h
 *
 *  Created on: Feb 8, 2017
 *      Author: moncadac
 */

#ifndef SENSORS_IMU_H_
#define SENSORS_IMU_H_

#define AK8963_I2C_ADDRESS 0x68
#define MPU9X50_I2C_ADDRESS 0x68
#define MPU9X50_INT_BASE GPIO_PORTE_BASE
#define MPU9X50_INT_PIN  GPIO_PIN_0
#define MPU9X50_I2C_BASE  I2C2_BASE
#define MPU9X50_I2C_INT   INT_I2C2

#include <stdint.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_ak8963.h"
#include "hw_mpu9x50.h"
#include "sensorlib/ak8963.h"
#include "mpu9x50.h"

void MPU9X50AppI2CWait(char *pcFilename, uint_fast32_t ui32Line);
void AK8963AppI2CWait(char *pcFilename, uint_fast32_t ui32Line);
void MPU9X50AppCallback(void *pvCallbackData, uint_fast8_t ui8Status);
void AK8963AppCallback(void *pvCallbackData, uint_fast8_t ui8Status);
void MPU9X50I2CIntHandler(void);
void GPIOeIntHandler(void);
void IMUInit(tMPU9X50 *psMPU9X50Inst, tAK8963 *psAK8963Inst,
             tI2CMInstance *psI2CInst);
void IMUDataGetFloat(float *pfAccel, float *pfGyro, float *pfMag);

#endif /* SENSORS_IMU_H_ */
