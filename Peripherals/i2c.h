/*
 * i2c.h
 *
 *  Created on: Feb 17, 2015
 *      Author: Ryan
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdbool.h>
#include <stdint.h>

#define I2C_MODE_WRITE false
#define I2C_MODE_READ  true
#define I2C_SPEED_100  false
#define I2C_SPEED_400  true

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
// Module function prototypes.
//
//*****************************************************************************
void I2CInit(uint32_t ui32Base, bool bSpeed);
bool I2CBurstRead(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t* ui8ptrReadData, uint32_t ui32Size);
bool I2CBurstWrite(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t ui8SendData[], uint32_t ui32Size);
bool I2CRead(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t* ui8ptrData);
bool I2CWrite(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t ui8Data);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* I2C_H_ */
