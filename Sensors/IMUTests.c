/*
 * IMUTests.c
 *
 *  Created on: Feb 8, 2017
 *      Author: moncadac
 */

#include "IMUTests.h"
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "IMU.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8963.h"
#include "mpu9x50.h"

//*****************************************************************************
//
// Tests to make sure that we are getting data from the IMU. First checks to
// make sure WHOAMI registers return the correct values from both the AK8963
// and MPU9X50. Then makes sure that we receive several different values from
// the sensors to ensure that we are getting actual data.
//
//*****************************************************************************
bool
IMUTest1(tMPU9X50 *psMPU9X50Inst, tAK8963 *psAK8963Inst)
{
    uint8_t ui8WhoAmI;
    int iDataChangeCount = 0, i;
    float pfData[6], pfOldData[6];

    //
    // Check the MPU9X50 Who Am I register. Return fail if we do not read
    // 0x71.
    //
    MPU9X50Read(psMPU9X50Inst, MPU9X50_O_WHO_AM_I, &ui8WhoAmI, 1,
                MPU9X50AppCallback, psMPU9X50Inst);
    SysCtlDelay(SysCtlClockGet() / 100);
    while(!(ui8WhoAmI == 0x71))
    {
        MPU9X50Read(psMPU9X50Inst, MPU9X50_O_WHO_AM_I, &ui8WhoAmI, 1,
                    MPU9X50AppCallback, psMPU9X50Inst);
        SysCtlDelay(SysCtlClockGet() / 100);
    }

    //
    // Check the AK8963 Who Am I register. Return fail if we do not read
    // 0x48.
    //
    AK8963Read(psAK8963Inst, AK8963_O_WIA, &ui8WhoAmI, 1,
               AK8963AppCallback, psAK8963Inst);
    SysCtlDelay(SysCtlClockGet() / 100);
    while(!(ui8WhoAmI == 0x48))
    {
        AK8963Read(psAK8963Inst, AK8963_O_WIA, &ui8WhoAmI, 1,
                   AK8963AppCallback, psAK8963Inst);
        SysCtlDelay(SysCtlClockGet() / 100);

    }

    //
    // Make sure we are getting data from the MPU9X50.
    //
    while(iDataChangeCount < 5)
    {
        MPU9X50DataRead(psMPU9X50Inst, MPU9X50AppCallback, psMPU9X50Inst);
        MPU9X50DataAccelGetFloat(psMPU9X50Inst, &pfData[0], &pfData[1],
                                 &pfData[2]);
        MPU9X50DataGyroGetFloat(psMPU9X50Inst, &pfData[3], &pfData[4],
                                &pfData[5]);

        for(i = 0; i < 6; i++)
        {
            if(pfData[i] != pfOldData[i])
            {
                iDataChangeCount++;
                break;
            }
        }
        for(i = 0; i < 6; i++)
        {
            pfOldData[i] = pfData[i];
        }
    }

    iDataChangeCount = 0;

    //
    // Make sure we are getting data from the AK8963.
    //
    while(iDataChangeCount < 5)
    {
        AK8963DataRead(psAK8963Inst, AK8963AppCallback, psAK8963Inst);
        AK8963DataMagnetoGetFloat(psAK8963Inst, &pfData[0],
                                  &pfData[1], &pfData[2]);

        for(i = 0; i < 3; i++)
        {
            if(pfData[i] != pfOldData[i])
            {
                iDataChangeCount++;
                break;
            }
        }
        for(i = 0; i < 3; i++)
        {
            pfOldData[i] = pfData[i];
        }
    }

    return true;
}
