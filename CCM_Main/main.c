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

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/debug.h>
#include <driverlib/i2c.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include "../Power/BQ27441.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"

void BogusHandler (void)
{
    I2CMasterIntClear (INT_I2C1_TM4C123);
}

int main(void)
{
tI2CMInstance i2c_master;
tBQ27441 BQ_inst;
volatile uint_fast8_t success=10;
int16_t first=0;
int16_t second=0;
float temp=0.0;
float temp2=0.0;

SysCtlPeripheralEnable (SYSCTL_PERIPH_I2C1);
SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);

SysCtlDelay (1000);

GPIOPinConfigure (GPIO_PA6_I2C1SCL);
GPIOPinConfigure (GPIO_PA7_I2C1SDA);

SysCtlDelay (1000);

GPIOPinTypeI2C (GPIO_PORTA_BASE, GPIO_PIN_7);
GPIOPinTypeI2CSCL (GPIO_PORTA_BASE, GPIO_PIN_6);

SysCtlDelay (1000);

I2CMInit (&i2c_master, I2C1_BASE, INT_I2C1_TM4C123, 0xff, 0xff, ROM_SysCtlClockGet ());
BQ27441Init (&BQ_inst, &i2c_master, I2C1_BASE, AppCallback, &BQ_inst);
while (1)
{
    success = BQ27441DataRead (&BQ_inst, BQ27441Callback, 0);
    SysCtlDelay (ROM_SysCtlClockGet ()/6);
    BQ27441DataBatteryVoltageGetRaw (&BQ_inst, &first);
    BQ27441DataBatteryVoltageGetFloat (&BQ_inst, &temp);
    BQ27441DataBatteryTemperatureGetRaw (&BQ_inst, &second);
    BQ27441DataBatteryTemperatureGetFloat (&BQ_inst, &temp2);
    SysCtlDelay (ROM_SysCtlClockGet ()/6);
}
	return 0;
}
