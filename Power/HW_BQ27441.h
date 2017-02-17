/*
 * HW_BQ27441.h
 *
 *  Created on: Feb 6, 2017
 *      Author: bergej
 */

#ifndef __POWER_HW_BQ27441_H__
#define __POWER_HW_BQ27441_H__

//*****************************************************************************
//
// The following are defines for the BQ27441 register addresses
//
//*****************************************************************************
#define BQ27441_O_CNTL_LSB    0x00        // System Control Register LSB
#define BQ27441_O_CNTL_MSB    0x01        // System Control Register MSB

#define BQ27441_O_TEMP_LSB    0x02        // Battery temperature LSB
#define BQ27441_O_TEMP_MSB    0x03        // Battery temperature MSB

#define BQ27441_O_VOLT_LSB    0x04        // Battery cell-pack voltage LSB
#define BQ27441_O_VOLT_MSB    0x05        // Battery cell-pack voltage MSB

#define BQ27441_O_FLAGS_LSB   0x06        // Holds various operating status value of gas-gauge LSB
#define BQ27441_O_FLAGS_MSB   0x07        // Holds various operating status value of gas-gauge MSB

#define BQ27441_O_NOM_AV_CAP_LSB  0x08        // Uncompensated battery capacity remaining LSB
#define BQ27441_O_NOM_AV_CAP_MSB  0x09        // Uncompensated battery capacity remaining MSB

#define BQ27441_O_FULL_AV_CAP_LSB  0x0A        // Uncompensated capacity of fully charged battery LSB
#define BQ27441_O_FULL_AV_CAP_MSB  0x0B        // Uncompensated capacity of fully charged battery MSB

#define BQ27441_O_REM_CAP_LSB 0x0C        // Compensated battery capacity remaining LSB
#define BQ27441_O_REM_CAP_MSB 0x0D        // Compensated battery capacity remaining MSB

#define BQ27441_O_FULL_CHRG_CAP_LSB  0x0E        // Compensated battery capacity when fully charged LSB
#define BQ27441_O_FULL_CHRG_CAP_MSB  0x0F        // Compensated battery capacity when fully charged MSB

#define BQ27441_O_AVG_I_LSB   0x10        // Average current flow through sense resistor LSB
#define BQ27441_O_AVG_I_MSB   0x11        // Average current flow through sense resistor MSB

#define BQ27441_O_STBY_I_LSB  0x12        // Standby current through sense resistor LSB
#define BQ27441_O_STBY_I_MSB  0x13        // Standby current through sense resistor MSB

#define BQ27441_O_MAX_LOAD_I_LSB  0x14        // Max load current through sense resistor LSB
#define BQ27441_O_MAX_LOAD_I_MSB  0x15        // Max load current through sense resistor MSB

#define BQ27441_O_AVG_PWR_LSB  0x18        // Average power draw LSB
#define BQ27441_O_AVG_PWR_MSB  0x19        // Average power draw MSB

#define BQ27441_O_STATE_OF_CHRG_LSB  0x1C        // State of charge (percent) LSB
#define BQ27441_O_STATE_OF_CHRG_MSB  0x1D        // State of charge (percent) MSB

#define BQ27441_O_INT_TEMP_LSB  0x1E        // Internal temperature LSB
#define BQ27441_O_INT_TEMP_MSB  0x1F        // Internal temperature MSB

#define BQ27441_O_STATE_OF_HEALTH_LSB  0x20        // State of health (percent) LSB
#define BQ27441_O_STATE_OF_HEALTH_MSB  0x21        // State of health (percent) MSB

#define BQ27441_O_REM_CAP_UNFIL_LSB  0x28        // Remaining capacity unfiltered LSB
#define BQ27441_O_REM_CAP_UNFIL_MSB  0x29        // Remaining capacity unfiltered MSB

#define BQ27441_O_REM_CAP_FIL_LSB  0x2A        // Remaining capacity filtered LSB
#define BQ27441_O_REM_CAP_FIL_MSB  0x2B        // Remaining capacity filtered MSB

#define BQ27441_O_FULL_CAP_UNFIL_LSB  0x2C        // Full capacity unfiltered LSB
#define BQ27441_O_FULL_CAP_UNFIL_MSB  0x2D        // Full capacity unfiltered MSB

#define BQ27441_O_FULL_CAP_FIL_LSB  0x2E        // Full capacity filtered LSB
#define BQ27441_O_FULL_CAP_FIL_MSB  0x2F        // Full capacity filtered MSB

#define BQ27441_O_STATE_OF_CHARGE_LSB  0x30        // State of charge (mAh) LSB
#define BQ27441_O_STATE_OF_CAHRGE_MSB  0x31        // State of charge (mAh) MSB

//*****************************************************************************
//
// The following are defines for the bit fields in the BQ27441_O_CNTL_LSB
// register.
//
//*****************************************************************************
#define BQ27441_CNTL_LSB_FUNC_M                                             \
                                0xFF        // Functions
#define BQ27441_CNTL_LSB_FUNC_STATUS                                        \
                                0x00        // reports DF checksum, hibernate,
                                            // IT, etc
#define BQ27441_CNTL_LSB_FUNC_DEVTYPE                                       \
                                0x01        // reports device type (for
                                            // example: 0x0520)
#define BQ27441_CNTL_LSB_FUNC_FWVER                                         \
                                0x02        // reports firmware version on the
                                            // device type
#define BQ27441_CNTL_LSB_FUNC_PREVCMD                                       \
                                0x07        // reports previous Control()
                                            // subcommand code
#define BQ27441_CNTL_LSB_FUNC_RESET                                         \
                                0x41        // forces a full reset of the fuel
                                            // gauge
#define BQ27441_CNTL_LSB_FUNC_S                                             \
                                0

#endif /* __POWER_HW_BQ27441_H__ */
