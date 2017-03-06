/*
 * main.c
 *
 * Overlord Software for Axis Powers Team
 *
 * Created on: Jan 22, 2017
 *
 *
 *
 */
//*****************************************************************************
//
// Standard Includes
//
//*****************************************************************************
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "sensorlib/hw_ak8963.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8963.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/magneto.h"

#include "utils/uartstdio.h"
#include "utils/cmdline.h"

#include "../peripherals/gpio.h"
#include "../peripherals/i2c.h"
#include "../peripherals/misc.h"
#include "../peripherals/hw_mpu9x50.h"
#include "../peripherals/mpu9x50.h"
#include "../peripherals/mpu9250_drv.h"

#include "../display/charter_module.h"
#include "../power/dips_module.h"
#include "../sensors/imu_tests.h"

int main(void)
{

    BatteryInit ();
    CharterBatteryTest ();
}
