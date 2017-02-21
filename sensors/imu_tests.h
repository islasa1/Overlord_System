/*
 * IMUTests.h
 *
 *  Created on: Feb 8, 2017
 *      Author: moncadac
 */

#ifndef SENSORS_IMU_TESTS_H_
#define SENSORS_IMU_TESTS_H_

#include <stdbool.h>
#include <stdint.h>

#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8963.h"

#include "../peripherals/mpu9x50.h"

bool IMUTest1(tMPU9X50 *psMPU9X50Inst, tAK8963 *psAK8963Inst);

#endif /* SENSORS_IMU_TESTS_H_ */
