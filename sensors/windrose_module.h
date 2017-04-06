/*
 * windrose_module.h
 *
 *  Created on: Jan 22, 2017
 *      Author: Anthony
 */

#ifndef __SENSORS_WINDROSE_MODULE_H__
#define __SENSORS_WINDROSE_MODULE_H__

extern void ReadCalibrationData(void);
extern void ResetCalibrationData(void);
extern void InitPosition(float *pfAccel);
extern void InitHeading(float *pfMag);
extern void UpdateHeading(float *pfGyro, float *pfMag);
extern void UpdatePosition(float *pfAccel);
extern float GetRelativeHeading(float x, float y);
extern void GetPosition(float *x, float *y);

#endif /* __SENSORS_WINDROSE_MODULE_H__ */
