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
extern void ApplyCalibration(float pfAccel[3], float pfGyro[3]);
extern void InitPosition();
extern void InitHeading(float pfAccel[3], float pfMag[3]);
extern void UpdateHeading(float pfAccel[3], float pfGyro[3], float pfMag[3]);
extern void UpdatePosition(float pfAccel[3]);
extern float GetRelativeHeading(float x, float y);
extern float GetHeading();

#endif /* __SENSORS_WINDROSE_MODULE_H__ */
