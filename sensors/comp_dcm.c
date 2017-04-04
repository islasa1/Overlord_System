/*
 * comp_dcm.c
 *
 *  Created on: Apr 3, 2017
 *      Author: Christian
 */

#include <stdbool.h>
#include <math.h>

#include "sensorlib/vector.h"

#include "../sensors/comp_dcm.h"

//*****************************************************************************
//
//! Initializes the complementary filter DCM attitude estimation state.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param fDeltaT is the amount of time between DCM updates, in seconds.
//! \param fScaleA is the weight of the accelerometer reading in determining
//! the updated attitude estimation.
//! \param fScaleG is the weight of the gyroscope reading in determining the
//! updated attitude estimation.
//! \param fScaleM is the weight of the magnetometer reading in determining the
//! updated attitude estimation.
//!
//! This function initializes the complementary filter DCM attitude estimation
//! state, and must be called prior to performing any attitude estimation.
//!
//! New readings must be supplied to the complementary filter DCM attitude
//! estimation algorithm at the rate specified by the \e fDeltaT parameter.
//! Failure to provide new readings at this rate results in inaccuracies in the
//! attitude estimation.
//!
//! The \e fScaleA, \e fScaleG, and \e fScaleM weights must sum to one.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMInit(tCompDCM *psDCM, float fDeltaT, float fScaleA, float fScaleG,
            float fScaleM)
{
    //
    // Initialize the DCM matrix to the identity matrix.
    //
    psDCM->ppfDCM[0][0] = 1.0;
    psDCM->ppfDCM[0][1] = 0.0;
    psDCM->ppfDCM[0][2] = 0.0;
    psDCM->ppfDCM[1][0] = 0.0;
    psDCM->ppfDCM[1][1] = 1.0;
    psDCM->ppfDCM[1][2] = 0.0;
    psDCM->ppfDCM[2][0] = 0.0;
    psDCM->ppfDCM[2][1] = 0.0;
    psDCM->ppfDCM[2][2] = 1.0;

    //
    // Save the time delta between DCM updates.
    //
    psDCM->fDeltaT = fDeltaT;

    //
    // Save the scaling factors that are applied to the accelerometer,
    // gyroscope, and magnetometer readings.
    //
    psDCM->fScaleA = fScaleA;
    psDCM->fScaleG = fScaleG;
    psDCM->fScaleM = fScaleM;
}

//*****************************************************************************
//
//! Starts the complementary filter DCM attitude estimation from an initial
//! sensor reading.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param pfAccel is a pointer to the accelerometer data to use to calculate
//! the initial attitude.
//! \param pfMag is a pointer to the magnetometer data to use to calculate the
//! initial attitude.
//!
//! This function computes the initial complementary filter DCM attitude
//! estimation state based on the initial accelerometer and magnetometer
//! reading.  While not necessary for the attitude estimation to converge,
//! using an initial state based on sensor readings results in quicker
//! convergence.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMStart(tCompDCM *psDCM, float pfAccel[3], float pfMag[3])
{
    float pfI[3], pfJ[3], pfK[3];

    //
    // The magnetometer reading forms the initial I vector, pointing north.
    //
    pfI[0] = pfMag[0];
    pfI[1] = pfMag[1];
    pfI[2] = pfMag[2];

    //
    // The accelerometer reading forms the initial K vector, pointing down.
    //
    pfK[0] = pfAccel[0];
    pfK[1] = pfAccel[1];
    pfK[2] = pfAccel[2];

    //
    // Compute the initial J vector, which is the cross product of the K and I
    // vectors.
    //
    VectorCrossProduct(pfJ, pfK, pfI);

    //
    // Recompute the I vector from the cross product of the J and K vectors.
    // This makes it fully orthogonal, which it wasn't before since magnetic
    // north points inside the Earth in many places.
    //
    VectorCrossProduct(pfI, pfJ, pfK);

    //
    // Normalize the I, J, and K vectors.
    //
    VectorScale(pfI, pfI, 1 / sqrtf(VectorDotProduct(pfI, pfI)));
    VectorScale(pfJ, pfJ, 1 / sqrtf(VectorDotProduct(pfJ, pfJ)));
    VectorScale(pfK, pfK, 1 / sqrtf(VectorDotProduct(pfK, pfK)));

    //
    // Initialize the DCM matrix from the I, J, and K vectors.
    //
    psDCM->ppfDCM[0][0] = pfI[0];
    psDCM->ppfDCM[0][1] = pfI[1];
    psDCM->ppfDCM[0][2] = pfI[2];
    psDCM->ppfDCM[1][0] = pfJ[0];
    psDCM->ppfDCM[1][1] = pfJ[1];
    psDCM->ppfDCM[1][2] = pfJ[2];
    psDCM->ppfDCM[2][0] = pfK[0];
    psDCM->ppfDCM[2][1] = pfK[1];
    psDCM->ppfDCM[2][2] = pfK[2];
}

//*****************************************************************************
//
//! Updates the complementary filter DCM attitude estimation based on an
//! updated set of sensor readings.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param pfAccel is a pointer to the accelerometer data to use to update the
//! DCM attitude estimation.
//! \param pfGyro is a pointer to the gyroscope data to use to update the
//! DCM attitude estimation.
//! \param pfMag is a pointer to the magnetometer data to use to update the
//! DCM attitude estimation.
//!
//! This function updates the complementary filter DCM attitude estimation
//! state based on the current sensor readings.  This function must be called
//! at the rate specified to CompDCMInit(), with new readings supplied at an
//! appropriate rate (for example, magnetometers typically sample at a much
//! slower rate than accelerometers and gyroscopes).
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMUpdate(tCompDCM *psDCM, float pfAccel[3], float pfGyro[3], float pfMag[3])
{
    float pfI[3], pfJ[3], pfK[3], pfDelta[3], pfTemp[3], fError, fMagnitude;
    bool bUseAccel = true;

    //
    // If the magnitude is or less than a gravity with some range for noise,
    // don't use the accelerometer to estimate the new attitude.
    //
    fMagnitude = sqrtf(VectorDotProduct(pfAccel, pfAccel));
    if(fMagnitude > 9.85 || fMagnitude < 9.73)
    {
        bUseAccel = false;
    }

    //if(bUseAccel)
    //{
        //
        // The magnetometer reading forms the new Im vector, pointing north.
        //
        pfI[0] = pfMag[0];
        pfI[1] = pfMag[1];
        pfI[2] = pfMag[2];

        //
        // The accelerometer reading forms the new Ka vector, pointing down.
        //
        pfK[0] = pfAccel[0];
        pfK[1] = pfAccel[1];
        pfK[2] = pfAccel[2];

        //
        // Compute the new J vector, which is the cross product of the Ka and Im
        // vectors.
        //
        VectorCrossProduct(pfJ, pfK, pfI);

        //
        // Recompute the Im vector from the cross product of the J and Ka vectors.
        // This makes it fully orthogonal, which it wasn't before since magnetic
        // north points inside the Earth in many places.
        //
        VectorCrossProduct(pfI, pfJ, pfK);

        //
        // Normalize the Im and Ka vectors.
        //
        VectorScale(pfI, pfI, 1 / sqrtf(VectorDotProduct(pfI, pfI)));
        VectorScale(pfK, pfK, 1 / sqrtf(VectorDotProduct(pfK, pfK)));

        //
        // Compute and scale the rotation as inferred from the accelerometer,
        // storing it in the rotation accumulator.
        //
        VectorCrossProduct(pfTemp, psDCM->ppfDCM[2], pfK);
        VectorScale(pfDelta, pfTemp, psDCM->fScaleA);

        //
        // Compute and scale the rotation as measured by the gyroscope, adding it
        // to the rotation accumulator.
        //
        pfTemp[0] = pfGyro[0] * psDCM->fDeltaT * psDCM->fScaleG;
        pfTemp[1] = pfGyro[1] * psDCM->fDeltaT * psDCM->fScaleG;
        pfTemp[2] = pfGyro[2] * psDCM->fDeltaT * psDCM->fScaleG;
        VectorAdd(pfDelta, pfDelta, pfTemp);

        //
        // Compute and scale the rotation as inferred from the magnetometer, adding
        // it to the rotation accumulator.
        //
        VectorCrossProduct(pfTemp, psDCM->ppfDCM[0], pfI);
        VectorScale(pfTemp, pfTemp, psDCM->fScaleM);
        VectorAdd(pfDelta, pfDelta, pfTemp);

        //
        // Rotate the I vector from the DCM matrix by the scaled rotation.
        //
        VectorCrossProduct(pfI, pfDelta, psDCM->ppfDCM[0]);
        VectorAdd(psDCM->ppfDCM[0], psDCM->ppfDCM[0], pfI);

        //
        // Rotate the K vector from the DCM matrix by the scaled rotation.
        //
        VectorCrossProduct(pfK, pfDelta, psDCM->ppfDCM[2]);
        VectorAdd(psDCM->ppfDCM[2], psDCM->ppfDCM[2], pfK);

        //
        // Compute the orthogonality error between the rotated I and K vectors and
        // adjust each by half the error, bringing them closer to orthogonality.
        //
        fError = VectorDotProduct(psDCM->ppfDCM[0], psDCM->ppfDCM[2]) / -2.0;
        VectorScale(pfI, psDCM->ppfDCM[0], fError);
        VectorScale(pfK, psDCM->ppfDCM[2], fError);
        VectorAdd(psDCM->ppfDCM[0], psDCM->ppfDCM[0], pfK);
        VectorAdd(psDCM->ppfDCM[2], psDCM->ppfDCM[2], pfI);

        //
        // Normalize the I and K vectors.
        //
        VectorScale(psDCM->ppfDCM[0], psDCM->ppfDCM[0],
                    0.5 * (3.0 - VectorDotProduct(psDCM->ppfDCM[0],
                                                  psDCM->ppfDCM[0])));
        VectorScale(psDCM->ppfDCM[2], psDCM->ppfDCM[2],
                    0.5 * (3.0 - VectorDotProduct(psDCM->ppfDCM[2],
                                                  psDCM->ppfDCM[2])));

        //
        // Compute the rotated J vector from the cross product of the rotated,
        // corrected K and I vectors.
        //
        VectorCrossProduct(psDCM->ppfDCM[1], psDCM->ppfDCM[2], psDCM->ppfDCM[0]);
    /*}
    else
    {
        float fGyroScale, fMagScale;
        fGyroScale = psDCM->fScaleG + psDCM->fScaleA *
                    (psDCM->fScaleG / (psDCM->fScaleG + psDCM->fScaleM));
        fMagScale = psDCM->fScaleM + psDCM->fScaleA *
                    (psDCM->fScaleM / (psDCM->fScaleG + psDCM->fScaleM));

        //
        // The magnetometer reading forms the new Im vector, pointing north.
        //
        pfI[0] = pfMag[0];
        pfI[1] = pfMag[1];
        pfI[2] = pfMag[2];

        //
        // Compute the new J vector, which is the cross product of the Ka and Im
        // vectors.
        //
        VectorCrossProduct(pfJ, pfK, pfI);

        //
        // Recompute the Im vector from the cross product of the J and Ka vectors.
        // This makes it fully orthogonal, which it wasn't before since magnetic
        // north points inside the Earth in many places.
        //
        VectorCrossProduct(pfI, pfJ, pfK);

        //
        // Normalize the Im and Ka vectors.
        //
        VectorScale(pfI, pfI, 1 / sqrtf(VectorDotProduct(pfI, pfI)));
        VectorScale(pfK, pfK, 1 / sqrtf(VectorDotProduct(pfK, pfK)));

        //
        // Compute and scale the rotation as measured by the gyroscope, adding it
        // to the rotation accumulator.
        //
        pfTemp[0] = pfGyro[0] * psDCM->fDeltaT * fGyroScale;
        pfTemp[1] = pfGyro[1] * psDCM->fDeltaT * fGyroScale;
        pfTemp[2] = pfGyro[2] * psDCM->fDeltaT * fGyroScale;
        VectorAdd(pfDelta, pfDelta, pfTemp);

        //
        // Compute and scale the rotation as inferred from the magnetometer, adding
        // it to the rotation accumulator.
        //
        VectorCrossProduct(pfTemp, psDCM->ppfDCM[0], pfI);
        VectorScale(pfTemp, pfTemp, fMagScale);
        VectorAdd(pfDelta, pfDelta, pfTemp);

        //
        // Rotate the I vector from the DCM matrix by the scaled rotation.
        //
        VectorCrossProduct(pfI, pfDelta, psDCM->ppfDCM[0]);
        VectorAdd(psDCM->ppfDCM[0], psDCM->ppfDCM[0], pfI);

        //
        // Rotate the K vector from the DCM matrix by the scaled rotation.
        //
        VectorCrossProduct(pfK, pfDelta, psDCM->ppfDCM[2]);
        VectorAdd(psDCM->ppfDCM[2], psDCM->ppfDCM[2], pfK);

        //
        // Compute the orthogonality error between the rotated I and K vectors and
        // adjust each by half the error, bringing them closer to orthogonality.
        //
        fError = VectorDotProduct(psDCM->ppfDCM[0], psDCM->ppfDCM[2]) / -2.0;
        VectorScale(pfI, psDCM->ppfDCM[0], fError);
        VectorScale(pfK, psDCM->ppfDCM[2], fError);
        VectorAdd(psDCM->ppfDCM[0], psDCM->ppfDCM[0], pfK);
        VectorAdd(psDCM->ppfDCM[2], psDCM->ppfDCM[2], pfI);

        //
        // Normalize the I and K vectors.
        //
        VectorScale(psDCM->ppfDCM[0], psDCM->ppfDCM[0],
                    0.5 * (3.0 - VectorDotProduct(psDCM->ppfDCM[0],
                                                  psDCM->ppfDCM[0])));
        VectorScale(psDCM->ppfDCM[2], psDCM->ppfDCM[2],
                    0.5 * (3.0 - VectorDotProduct(psDCM->ppfDCM[2],
                                                  psDCM->ppfDCM[2])));

        //
        // Compute the rotated J vector from the cross product of the rotated,
        // corrected K and I vectors.
        //
        VectorCrossProduct(psDCM->ppfDCM[1], psDCM->ppfDCM[2], psDCM->ppfDCM[0]);
    }*/
}
