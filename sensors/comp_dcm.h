/*
 * comp_dcm.h
 *
 *  Created on: Apr 3, 2017
 *      Author: Christian
 */

#ifndef SENSORS_COMP_DCM_H
#define SENSORS_COMP_DCM_H

//*****************************************************************************
//
// The structure that defines the internal state of the complementary filter
// DCM algorithm.
//
//*****************************************************************************
typedef struct
{
    //
    // The state of the direction cosine matrix.
    //
    float ppfDCM[3][3];

    //
    // The time delta between updates to the DCM.
    //
    float fDeltaT;

    //
    // The scaling factor for the DCM update based on the accelerometer
    // reading.
    //
    float fScaleA;

    //
    // The scaling factor for the DCM update based on the gyroscope reading.
    //
    float fScaleG;

    //
    // The scaling factor for the DCM update based on the magnetometer reading.
    //
    float fScaleM;
}
tCompDCM;

extern void CompDCMInit(tCompDCM *psDCM, float fDeltaT, float fScaleA,
                        float fScaleG, float fScaleM);
extern void CompDCMStart(tCompDCM *psDCM, float pfAccel[3], float pfMag[3]);
extern void CompDCMUpdate(tCompDCM *psDCM, float pfAccel[3], float pfGyro[3], float pfMag[3]);

#endif /* SENSORS_COMP_DCM_H */
