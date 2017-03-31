/*
 * windrose_module.c
 *
 *  Created on: Jan 22, 2017
 *      Author: Anthony
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "driverlib/eeprom.h"
#include "driverlib/sysctl.h"

#define TIMESTEP .1
#define CALIBRATION_DATA_ADDRESS 0
#define CALIBRATION_DATA_SIZE sizeof(tCalibrationData)

#ifndef M_PI
#define M_PI                    3.14159265358979323846
#endif

#define G 9.79193

//*****************************************************************************
//
// Define calibration data struct.
//
//*****************************************************************************
typedef struct
{
    float fXAccelBias;
    float fYAccelBias;
    float fZAccelBias;
    float fXAccelScale;
    float fYAccelScale;
    float fZAccelScale;
    float fYXAccelAlignment;
    float fZXAccelAlignment;
    float fXYAccelAlignment;
    float fZYAccelAlignment;
    float fXZAccelAlignment;
    float fYZAccelAlignment;
    float fXGyroBias;
    float fYGyroBias;
    float fZGyroBias;
    float fXGyroScale;
    float fYGyroScale;
    float fZGyroScale;
    float fYXGyroAlignment;
    float fZXGyroAlignment;
    float fXYGyroAlignment;
    float fZYGyroAlignment;
    float fXZGyroAlignment;
    float fYZGyroAlignment;
} tCalibrationData;

tCalibrationData g_sCalData;

//*****************************************************************************
//
// Globals for absolute position and heading.
//
//*****************************************************************************
float g_fAbsoluteX;
float g_fAbsoluteY;
float g_fCurrentHeading;
float g_pfCurrentVelocity[2];

//*****************************************************************************
//
//! Resets the calibration data struct.
//!
//! This function resets the global calibration data struct to values that do
//! not affect the measurements (i.e. keep the uncalibrated data). This is
//! meant to be used when the EEPROM fails to be read.
//!
//! \return None.
//
//*****************************************************************************
void
ResetCalibrationData(void)
{
    g_sCalData.fXAccelBias = 0;
    g_sCalData.fYAccelBias = 0;
    g_sCalData.fZAccelBias = 0;
    g_sCalData.fXAccelScale = 1;
    g_sCalData.fYAccelScale = 1;
    g_sCalData.fZAccelScale = 1;
    g_sCalData.fYXAccelAlignment = 0;
    g_sCalData.fZXAccelAlignment = 0;
    g_sCalData.fXYAccelAlignment = 0;
    g_sCalData.fZYAccelAlignment = 0;
    g_sCalData.fXZAccelAlignment = 0;
    g_sCalData.fYZAccelAlignment = 0;
    g_sCalData.fXGyroBias = 0;
    g_sCalData.fYGyroBias = 0;
    g_sCalData.fZGyroBias = 0;
    g_sCalData.fXGyroScale = 1;
    g_sCalData.fYGyroScale = 1;
    g_sCalData.fZGyroScale = 1;
    g_sCalData.fYXGyroAlignment = 0;
    g_sCalData.fZXGyroAlignment = 0;
    g_sCalData.fXYGyroAlignment = 0;
    g_sCalData.fZYGyroAlignment = 0;
    g_sCalData.fXZGyroAlignment = 0;
    g_sCalData.fYZGyroAlignment = 0;
}

//*****************************************************************************
//
//! Retrieves the calibration data from EEPROM.
//!
//! This function retrieves the IMU calibration data from EEPROM and stores it
//! in the global calibration data struct. The calibration data must be
//! written to EEPROM prior to operation. This data is then used to improve
//! the accuracy of the tracking algorithm.
//!
//! \return None.
//
//*****************************************************************************
void
ReadCalibrationData(void)
{
    //
    // Initialize EEPROM and peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    if(EEPROMInit() != EEPROM_INIT_OK)
    {
        ResetCalibrationData();
        return;
    }

    //
    // Read the data from EEPROM
    //
    EEPROMRead((uint32_t *)&g_sCalData, CALIBRATION_DATA_ADDRESS,
               CALIBRATION_DATA_SIZE);
}

//*****************************************************************************
//
//! Initialize the absolute position.
//!
//! \param pfAccel is an array of floats representing the initial accelerometer
//! readings. It is used in the 2 dimensional analysis to find gravity.
//!
//! This function initializes the absolute position coordinates to (0,0). In
//! the 2 dimensional algorithm, it also finds the distribution of gravity to
//! subtract from future readings.
//!
//! \return None.
//
//*****************************************************************************
void
InitPosition(float *pfAccel)
{
    g_fAbsoluteX = 0;
    g_fAbsoluteY = 0;
    g_pfCurrentVelocity[0] = 0;
    g_pfCurrentVelocity[1] = 0;
}

//*****************************************************************************
//
//! Initializes the absolute heading.
//!
//! \param pfMag is an array of floats representing the initial magnetometer
//! readings. The readings are used to determine the initial absolute heading.
//!
//! This function initializes the absolute heading using the initial
//! magnetometer readings. This value is then saved in a global for use in
//! the tracking algorithm.
//!
//! \return None.
//
//*****************************************************************************
void
InitHeading(float *pfMag)
{
    g_fCurrentHeading = atan2(-pfMag[1], pfMag[0]);
}

//*****************************************************************************
//
//! Updates the current absolute heading.
//!
//! \param pfGyro is an array of floats representing the current gyroscope
//! readings.
//! \param pfMag is an array of floats representing the current magnetometer
//! readings.
//!
//! This function updates the current absolute heading. This is
//! calculated by weighting change according to the gyroscope readings and
//! the magnetometer readings and then summing the weighted values.
//!
//! \return None.
//
//*****************************************************************************
void
UpdateHeading(float *pfGyro, float *pfMag)
{
    pfGyro[0] -= g_sCalData.fXGyroBias;
    pfGyro[1] -= g_sCalData.fYGyroBias;
    pfGyro[2] -= g_sCalData.fZGyroBias;

    pfGyro[0] = pfGyro[0] * g_sCalData.fXGyroScale + pfGyro[1] * g_sCalData.fYXGyroAlignment
               + pfGyro[2] * g_sCalData.fZXGyroAlignment;
    pfGyro[1] = pfGyro[1] * g_sCalData.fYGyroScale + pfGyro[0] * g_sCalData.fXYGyroAlignment
               + pfGyro[2] * g_sCalData.fZYGyroAlignment;
    pfGyro[2] = pfGyro[2] * g_sCalData.fZGyroScale + pfGyro[0] * g_sCalData.fXZGyroAlignment
               + pfGyro[1] * g_sCalData.fYZGyroAlignment;

    float fHeadingGyro = g_fCurrentHeading + pfGyro[2] * TIMESTEP;
    float fHeadingMag = atan2(-pfMag[1], pfMag[0]);
    float fGyroWeight = .75;
    //
    // Correction to avoid negative values.
    //
    if(fHeadingMag < 0)
    {
        fHeadingMag += 2 * M_PI;
    }
    if((fHeadingMag - fHeadingGyro) < -M_PI)
    {
        fHeadingMag += 2 * M_PI;
    }
    else if((fHeadingMag - fHeadingGyro) > M_PI)
    {
        fHeadingGyro += 2 * M_PI;
    }

    g_fCurrentHeading = fHeadingGyro * fGyroWeight +
                        fHeadingMag * (1 - fGyroWeight);
}

//*****************************************************************************
//
//! Updates the current absolute position.
//!
//! \param pfAccel is an array of floats representing the current accelerometer
//! readings.
//!
//! This function updates the current absolute position which is the
//! coordinates relative to the initial position in meters. The x-axis of the
//! coordinate system is North. Before integration, calibration is applied to
//! the accelerometer readings.
//!
//! \return None.
//
//*****************************************************************************
void
UpdatePosition(float *pfAccel)
{
    //
    // Apply calibration
    //
    pfAccel[0] -= g_sCalData.fXAccelBias;
    pfAccel[1] -= g_sCalData.fYAccelBias;
    pfAccel[2] -= g_sCalData.fZAccelBias;

    pfAccel[0] = pfAccel[0] * g_sCalData.fXAccelScale + pfAccel[1] * g_sCalData.fYXAccelAlignment
               + pfAccel[2] * g_sCalData.fZXAccelAlignment;
    pfAccel[1] = pfAccel[1] * g_sCalData.fYAccelScale + pfAccel[0] * g_sCalData.fXYAccelAlignment
               + pfAccel[2] * g_sCalData.fZYAccelAlignment;
    pfAccel[2] = pfAccel[2] * g_sCalData.fZAccelScale + pfAccel[0] * g_sCalData.fXZAccelAlignment
               + pfAccel[1] * g_sCalData.fYZAccelAlignment;

    //
    // Rotate acceleration data
    //
    pfAccel[0] = cos(g_fCurrentHeading)*pfAccel[0] - sin(g_fCurrentHeading)*pfAccel[1];
    pfAccel[1] = sin(g_fCurrentHeading)*pfAccel[0] + cos(g_fCurrentHeading)*pfAccel[1];

    //
    // Update the position
    //
    g_fAbsoluteX += g_pfCurrentVelocity[0] * TIMESTEP + pfAccel[0] * TIMESTEP * TIMESTEP / 2;
    g_fAbsoluteY += g_pfCurrentVelocity[1] * TIMESTEP + pfAccel[1] * TIMESTEP * TIMESTEP / 2;

    //
    // Update the velocities
    //
    g_pfCurrentVelocity[0] += pfAccel[0] * TIMESTEP;
    g_pfCurrentVelocity[1] += pfAccel[1] * TIMESTEP;
}

//*****************************************************************************
//
//! Calculates the relative heading to the absolute position supplied.
//!
//! \param x is the x coordinate of the other absolute position.
//! \param y is the y coordinate of the other absolute position.
//!
//! This function calculates the relative heading to the supplied absolute
//! position from the current absolute position.
//!
//! \return The relative heading between the two points in radians.
//
//*****************************************************************************
float
GetRelativeHeading(float x, float y)
{
    float fRelativeX = x - g_fAbsoluteX;
    float fRelativeY = y - g_fAbsoluteY;

    return atan2(-fRelativeY, fRelativeX) + g_fCurrentHeading;
}
















