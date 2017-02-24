/*
 * dips_module.h
 *
 *  Created on: Jan 22, 2017
 *      Author: Anthony
 */


#ifndef __DIPS_MODULE_DIPS_MODULE_H__
#define __DIPS_MODULE_DIPS_MODULE_H__

//*****************************************************************************
//
// Structure containing all the information about the battery
//
//*****************************************************************************
typedef struct {
    float fVoltage;
    float fTemperature;
    int16_t i16NominalAvailableCapacity;
    int16_t i16FullAvailableCapacity;
    int16_t i16RemainingCapacity;
    int16_t i16FullChargeCapacity;
    int16_t i16AverageCurrent;
    int16_t i16StandbyCurrent;
    int16_t i16MaxLoadCurrent;
    int16_t i16AveragePower;
    int16_t i16StateOfCharge;
    float fInternalTemperature;
    int16_t i16StateOfHealth;
} tBatteryInfo;

//*****************************************************************************
//
// Function prototype.
//
//*****************************************************************************
void BatteryInit (void);

#endif /* __DIPS_MODULE_DIPS_MODULE_H__ */
