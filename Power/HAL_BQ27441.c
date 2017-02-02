/*
 * HAL_BQ27441.c
 *
 *  Created on: Jan 30, 2017
 *      Author: bergej
 */

#include </git/Overlord_System/Power/i2c_read_write.c>
#include <stdio.h>
#include <stdint.h>
#include <driverlib/i2c.h>
#include <driverlib/sysctl.h>
#include <driverlib/rom.h>
#include <inc/hw_memmap.h>
#include "HAL_BQ27441.h"

bool BQ27441_initConfig ()
{
    unsigned char result =0;

    I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
    I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
    i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, FLAGS, &result);

    /* Check if ITPOR bit is set in FLAGS */
    if (result & 0x0020)
    {
        /* Instructs fuel gauge to enter CONFIG UPDATE mode. */
        if (!BQ27441_control (SET_CFGUPDATE))
            return 0;

        SysCtlDelay (1000000);

        result =0;
        /* Check if CFGUPMODE bit is set in FLAGS */
        while (!(result & 0x0010))
        {
            I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
            I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
            i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, FLAGS, &result);
        }

        /* Enable Block data memory control */
        if (!BQ27441_command(BLOCK_DATA_CONTROL))
            return 0;

        /* Set the data class to be accessed */
        if (!BQ27441_command(DATA_CLASS))
            return 0;

        /* Write the block offset location */
        if (!BQ27441_command(DATA_BLOCK))
            return 0;

        SysCtlDelay (1000000);

        unsigned char old_chksum = 0;
        unsigned char new_chksum = 0;
        unsigned char tmp_chksum = 0;
        unsigned char chksum = 0;
        do
        {
            /* Read Block Data Checksum */
            if (!BQ27441_readChecksum(&old_chksum))
                return 0;

            SysCtlDelay (1000000);

            /* Checksum calculation */
            tmp_chksum = old_chksum;

            unsigned char old_designCapacity = 0;
            unsigned char old_designEnergy = 0;
            unsigned char old_terminateVoltage = 0;
            unsigned char old_taperRate = 0;

            unsigned char read_data = 0x4A;
            /* Read old design capacity */
            I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
            I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
            i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, read_data, &old_designCapacity);

            tmp_chksum = computeCheckSum(tmp_chksum, old_designCapacity, CONF_DESIGN_CAPACITY);

            read_data = 0x4C;

            /* Read old design energy */
            I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
            I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
            i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, read_data, &old_designEnergy);

            tmp_chksum = computeCheckSum(tmp_chksum, old_designEnergy, CONF_DESIGN_ENERGY);

            read_data = 0x50;

            /* Read old terminate voltage */
            I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
            I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
            i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, read_data, &old_terminateVoltage);

            tmp_chksum = computeCheckSum(tmp_chksum, old_terminateVoltage, CONF_TERMINATE_VOLTAGE);

            read_data = 0x5B;

            /* Read old taper rate */
            I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
            I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
            i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, read_data, &old_taperRate);

            /* Checksum calculation */
            tmp_chksum = computeCheckSum(tmp_chksum, old_taperRate, CONF_TAPER_RATE);

            unsigned char write_data = swapMSB_LSB(CONF_DESIGN_CAPACITY);

            /* Write new design capacity */
            i2c_write (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, &write_data);

            /* Write new design energy */
            write_data = swapMSB_LSB(CONF_DESIGN_ENERGY);
            i2c_write (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, &write_data);

            /* Write new terminate voltage */
            write_data = swapMSB_LSB(CONF_TERMINATE_VOLTAGE);
            i2c_write (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, &write_data);
            /* Write new taper rate */
            write_data = swapMSB_LSB(CONF_TAPER_RATE);
            i2c_write (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, &write_data);

            /* Checksum calculation */
            new_chksum = tmp_chksum;

            /* Write new checksum */
            if (!BQ27441_command(BLOCK_DATA_CHECKSUM))
                return 0;

            SysCtlDelay (1000000);

            /* Read Block Data Checksum */
            if (!BQ27441_readChecksum(&chksum))
                return 0;

            SysCtlDelay (1000000);
        }
        while (new_chksum != chksum);

        if (!BQ27441_control(SOFT_RESET))
            return 0;

        SysCtlDelay (1000000);

        result = 0;
        /* Check if CFGUPMODE bit is cleared in FLAGS */
        while(result & 0x0010)
        {
            I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
            I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
            i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, FLAGS, &result);
        }
    }
    return 1;
}

bool BQ27441_initOpConfig ()
{
    unsigned char result = 0;

    /* Instructs fuel gauge to enter CONFIG UPDATE mode. */
    if (!BQ27441_control(SET_CFGUPDATE))
        return 0;

    SysCtlDelay (1000000);

    result = 0;
    /* Check if CFGUPMODE bit is set in FLAGS */
    while(!(result & 0x0010))
    {
        I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
        I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
        i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, FLAGS, &result);
    }

    /* Enable Block data memory control */
    if (!BQ27441_command(BLOCK_DATA_CONTROL))
        return 0;

    /* Set the data class to be accessed */
    if (!BQ27441_command(DATA_CLASS))
        return 0;

    /* Write the block offset location */
    if (!BQ27441_command(DATA_BLOCK))
        return 0;

    SysCtlDelay (1000000);

    unsigned char old_chksum = 0;
    unsigned char new_chksum = 0;
    unsigned char tmp_chksum = 0;
    unsigned char chksum = 0;
    do
    {
        /* Read Block Data Checksum */
        if (!BQ27441_readChecksum(&old_chksum))
            return 0;

        SysCtlDelay (1000000);

        /* Checksum calculation */
        tmp_chksum = old_chksum;

        unsigned char old_opconfig= 0;

        unsigned char read_data = 0x40;
        /* Read old opconfig */
        I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
        I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
        i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, read_data, &old_opconfig);

        tmp_chksum = computeCheckSum(tmp_chksum, old_opconfig, 0x05F8);

        /* Write new opconfig */
        unsigned char write_data = swapMSB_LSB(0x05F8);
        i2c_write (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, &write_data);

        /* Checksum calculation */
        new_chksum = tmp_chksum;

        /* Write new checksum */
        if (!BQ27441_command(BLOCK_DATA_CHECKSUM))
            return 0;

        SysCtlDelay (1000000);

        /* Read Block Data Checksum */
        if (!BQ27441_readChecksum(&chksum))
            return 0;

        SysCtlDelay (1000000);
    }
    while(new_chksum != chksum);

    /* Send SOFT_RESET control command */
    if (!BQ27441_control(SOFT_RESET))
        return 0;

    SysCtlDelay (1000000);

    result = 0;
    /* Check if CFGUPMODE bit is cleared in FLAGS */
    while(result & 0x0010)
    {
        I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
        I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
        i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, FLAGS, &result);
    }

    // Read the Operation Config
    unsigned char result16 = 0;

    I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
    I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
    i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, OP_CONFIG, &result16);

    // Check if BIE is cleared in Operation Config
    if (result16 & 0x2000)
        return 0;

    return 1;
}

bool BQ27441_control (unsigned char subcommand/*, unsigned int timeout*/)
{
    I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);

    I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);

    i2c_write (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, &subcommand);

    return 1;
}

bool BQ27441_controlRead (unsigned char subcommand, unsigned char* result/*, unsigned int timeout*/)
{
    if (!BQ27441_control (subcommand))
        return 0;

    SysCtlDelay (1000000);

    I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);
    I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);
    i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 16, CONTROL, result);

    return 1;
}

bool BQ27441_command (unsigned char command/*, char data, unsigned int timeout*/)
{
    I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);

    i2c_write (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 8, &command);

    return 1;
}

bool BQ27441_readChecksum (unsigned char *result/*, unsigned int timeout*/)
{
    I2CMasterInitExpClk (I2C1_BASE, ROM_SysCtlClockGet (), false);

    I2CSlaveAddressSet (I2C1_BASE, 0, BQ27441_SLAVE_ADDRESS);

    i2c_read (I2C1_BASE, BQ27441_SLAVE_ADDRESS, 8, BLOCK_DATA_CHECKSUM, result);

    return 1;
}

static unsigned char computeCheckSum (unsigned char oldCheckSum, int oldData, int newData)
{
    unsigned char tmpCheckSum = 0xFF - oldCheckSum - ( unsigned char )oldData - ( unsigned char )( oldData >> 8 );
    unsigned char newCheckSum = 0xFF - (  tmpCheckSum + ( unsigned char )newData + ( unsigned char )( newData >> 8 ) );
    return newCheckSum;
}

static unsigned char swapMSB_LSB (int data)
{
    unsigned char tmp = ( unsigned char )data;
    tmp = tmp << 8;
    tmp += ( unsigned char )( data >> 8 );
    return tmp;
}

