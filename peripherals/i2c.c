/*
 * i2c.c
 *
 *  Created on: Feb 17, 2015
 *      Author: Ryan
 */

#include "i2c.h"

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

void I2CInit(uint32_t ui32Base, bool bSpeed)
{
  switch(ui32Base)
  {
    case I2C0_BASE:
    {
      //
      // Enable Peripherals used by I2C0
      //
      MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

      //
      // Enable pin PB2 for I2C0 I2C0SCL
      //
      MAP_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
      MAP_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);

      //
      // Enable pin PB3 for I2C0 I2C0SDA
      //
      MAP_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
      MAP_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

      break;
    }
    case I2C1_BASE:
    {
      //
      // Enable Peripherals used by I2C1
      //
      MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

      //
      // Enable pin PA6 for I2C1 I2C1SCL
      //
      MAP_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
      MAP_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);

      //
      // Enable pin PA7 for I2C1 I2C1SDA
      //
      MAP_GPIOPinConfigure(GPIO_PA7_I2C1SDA);
      MAP_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

      break;
    }
    case I2C2_BASE:
    {
      //
      // Enable Peripherals used by I2C2
      //
      MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

      //
      // Enable pin PE4 for I2C2 I2C2SCL
      //
      MAP_GPIOPinConfigure(GPIO_PE4_I2C2SCL);
      MAP_GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);

      //
      // Enable pin PE5 for I2C2 I2C2SDA
      //
      MAP_GPIOPinConfigure(GPIO_PE5_I2C2SDA);
      MAP_GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

      break;
    }
    case I2C3_BASE:
    {
      //
      // Enable Peripherals used by I2C3
      //
      MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

      //
      // Enable pin PD1 for I2C3 I2C3SDA
      //
      MAP_GPIOPinConfigure(GPIO_PD1_I2C3SDA);
      MAP_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

      //
      // Enable pin PD0 for I2C3 I2C3SCL
      //
      MAP_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
      MAP_GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);

      break;
    }
#if defined(GPIO_PG2_I2C4SCL) && defined(GPIO_PG3_I2C4SDA)
    case I2C4_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOG;
      ui32PinConfigureSCL = GPIO_PG2_I2C4SCL;
      ui32PinConfigureSDA = GPIO_PG3_I2C4SDA;
      ui32GPIOBase = GPIO_PORTG_BASE;
      ui8GPIOPinSCL = GPIO_PIN_2;
      ui8GPIOPinSDA = GPIO_PIN_3;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C4;
      break;
    }
#endif
#if defined(GPIO_PG6_I2C5SCL) && defined(GPIO_PG7_I2C5SDA)
    case I2C5_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOG;
      ui32PinConfigureSCL = GPIO_PG6_I2C5SCL;
      ui32PinConfigureSDA = GPIO_PG7_I2C5SDA;
      ui32GPIOBase = GPIO_PORTG_BASE;
      ui8GPIOPinSCL = GPIO_PIN_6;
      ui8GPIOPinSDA = GPIO_PIN_7;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C5;
      break;
    }
#endif
#if defined(GPIO_PA6_I2C6SCL) && defined(GPIO_PA7_I2C6SDA)
    case I2C6_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOA;
      ui32PinConfigureSCL = GPIO_PA6_I2C6SCL;
      ui32PinConfigureSDA = GPIO_PA7_I2C6SDA;
      ui32GPIOBase = GPIO_PORTA_BASE;
      ui8GPIOPinSCL = GPIO_PIN_6;
      ui8GPIOPinSDA = GPIO_PIN_7;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C6;
      break;
    }
#endif
#if defined(GPIO_PD0_I2C7SCL) && defined(GPIO_PD1_I2C7SDA)
    case I2C7_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOD;
      ui32PinConfigureSCL = GPIO_PD0_I2C7SCL;
      ui32PinConfigureSDA = GPIO_PD1_I2C7SDA;
      ui32GPIOBase = GPIO_PORTD_BASE;
      ui8GPIOPinSCL = GPIO_PIN_0;
      ui8GPIOPinSDA = GPIO_PIN_1;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C7;
      break;
    }
#endif
#if defined(GPIO_PD2_I2C8SCL) && defined(GPIO_PD3_I2C8SDA)
    case I2C8_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOD;
      ui32PinConfigureSCL = GPIO_PD2_I2C8SCL;
      ui32PinConfigureSDA = GPIO_PD3_I2C8SDA;
      ui32GPIOBase = GPIO_PORTD_BASE;
      ui8GPIOPinSCL = GPIO_PIN_2;
      ui8GPIOPinSDA = GPIO_PIN_3;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C8;
      break;
    }
#endif
#if defined(GPIO_PA0_I2C9SCL) && defined(GPIO_PA1_I2C9SDA)
    case I2C9_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOA;
      ui32PinConfigureSCL = GPIO_PA0_I2C9SCL;
      ui32PinConfigureSDA = GPIO_PA1_I2C9SDA;
      ui32GPIOBase = GPIO_PORTA_BASE;
      ui8GPIOPinSCL = GPIO_PIN_0;
      ui8GPIOPinSDA = GPIO_PIN_1;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C9;
      break;
    }
#endif
    default: {
      return;
    }
  }

  //
  // Enable the I2C Base Master Block and Clock
  //
  MAP_I2CMasterInitExpClk(ui32Base, MAP_SysCtlClockGet(), bSpeed);
}

bool I2CBurstRead(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t* ui8ptrReadData, uint32_t ui32Size)
{
  //
  // Use I2C single read if theres only 1 item to receive
  //
  if (ui32Size == 1)
    return I2CRead(ui32Base, ui8SlaveAddr, &ui8ptrReadData[0]);

  uint32_t ui32ByteCount;        // local variable used for byte counting/state determination
  uint32_t MasterOptionCommand; // used to assign the control commands

  //
  // Tell the master module what address it will place on the bus when
  // reading from the slave.
  //
  MAP_I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, I2C_MODE_READ);

  //
  // Start with BURST with more than one byte to read
  //
  MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_START;

  for(ui32ByteCount = 0; ui32ByteCount < ui32Size; ui32ByteCount++)
  {
    //
    // The second and intermittent byte has to be read with CONTINUE control word
    //
    if(ui32ByteCount == 1)
      MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_CONT;

    //
    // The last byte has to be send with FINISH control word
    //
    if(ui32ByteCount == ui32Size - 1)
      MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;

    //
    // Disable Interrupts to prevent I2C comm failure
    //
    MAP_IntMasterDisable();

    //
    // Initiate read of data from the slave.
    //
    MAP_I2CMasterControl(ui32Base, MasterOptionCommand);

    //
    // Wait until master module is done reading.
    //
    while(!MAP_I2CMasterBusy(ui32Base)) {}
    while(MAP_I2CMasterBusy(ui32Base)) {}

    //
    // Clear any interrupts that may have been generated
    //
    MAP_I2CMasterIntClear(ui32Base);

    //
    // Reenable Interrupts
    //
    MAP_IntMasterEnable();

    //
    // Check for errors.
    //
    if (MAP_I2CMasterErr(ui32Base) != I2C_MASTER_ERR_NONE)
      return false;

    //
    // Move byte from register
    //
    ui8ptrReadData[ui32ByteCount] = (uint8_t) MAP_I2CMasterDataGet(ui32Base);
  }

  //
  // Return 1 if there is no error.
  //
  return true;
}

bool I2CBurstWrite(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t ui8SendData[], uint32_t ui32Size)
{
  //
  // Use I2C single write if theres only 1 item to send
  //
  if (ui32Size == 1)
    return I2CWrite(ui32Base, ui8SlaveAddr, ui8SendData[0]);

  uint32_t uiByteCount;         // local variable used for byte counting/state determination
  uint32_t MasterOptionCommand; // used to assign the control commands

  //
  // Tell the master module what address it will place on the bus when
  // writing to the slave.
  //
  MAP_I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, I2C_MODE_WRITE);

  //
  // The first byte has to be sent with the START control word
  //
  MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_START;

  for(uiByteCount = 0; uiByteCount < ui32Size; uiByteCount++)
  {
    //
    // The second and intermittent byte has to be send with CONTINUE control word
    //
    if(uiByteCount == 1)
      MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_CONT;

    //
    // The last byte has to be send with FINISH control word
    //
    if(uiByteCount == ui32Size - 1)
      MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_FINISH;

    //
    // Send data byte
    //
    MAP_I2CMasterDataPut(ui32Base, ui8SendData[uiByteCount]);

    //
    // Disable Interrupts to prevent I2C comm failure
    //
    MAP_IntMasterDisable();

    //
    //
    // Initiate send of data from the master.
    //
    MAP_I2CMasterControl(ui32Base, MasterOptionCommand);

    //
    //
    // Wait until master module is done transferring.
    //
    while(!MAP_I2CMasterBusy(ui32Base)) {}
    while(MAP_I2CMasterBusy(ui32Base)) {}

    //
    // Clear any interrupts that may have been generated
    //
    MAP_I2CMasterIntClear(ui32Base);

    //
    // Reenable Interrupts
    //
    MAP_IntMasterEnable();

    //
    // Check for errors.
    //
    if(MAP_I2CMasterErr(ui32Base) != I2C_MASTER_ERR_NONE) return false;
  }

  //
  // Return 1 if there is no error.
  //
  return true;
}

bool I2CRead(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t* ui8ptrData)
{
  //
  // Tell the master module what address it will place on the bus when
  // reading from the slave.
  //
  MAP_I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, I2C_MODE_READ);

  //
  // Disable Interrupts to prevent I2C comm failure
  //
  MAP_IntMasterDisable();

  //
  // Tell the master to read data.
  //
  MAP_I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_RECEIVE);

  //
  // Wait until master module is done receiving.
  //
  while(!MAP_I2CMasterBusy(ui32Base)) {}
  while(MAP_I2CMasterBusy(ui32Base)) {}

  //
  // Clear any interrupts that may have been generated
  //
  MAP_I2CMasterIntClear(ui32Base);

  //
  // Reenable Interrupts
  //
  MAP_IntMasterEnable();

  //
  // Check for errors.
  //
  if(MAP_I2CMasterErr(ui32Base) != I2C_MASTER_ERR_NONE)
    return false;

  //
  // Get data
  //
  *ui8ptrData = (uint8_t) MAP_I2CMasterDataGet(ui32Base);

  //
  // return the data from the master.
  //
  return true;
}

bool I2CWrite(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t ui8SendData)
{
  //
  //
  // Tell the master module what address it will place on the bus when
  // writing to the slave.
  //
  MAP_I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, I2C_MODE_WRITE);

  //
  // Place the command to be sent in the data register.
  //
  MAP_I2CMasterDataPut(ui32Base, ui8SendData);

  //
  // Disable Interrupts to prevent I2C comm failure
  //
  MAP_IntMasterDisable();

  //
  // Initiate send of data from the master.
  //
  MAP_I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_SEND);

  //
  // Wait until master module is done transferring.
  //
  while(!MAP_I2CMasterBusy(ui32Base)) {}
  while(MAP_I2CMasterBusy(ui32Base)) {}

  //
  // Clear any interrupts that may have been generated
  //
  MAP_I2CMasterIntClear(ui32Base);

  //
  // Reenable Interrupts
  //
  MAP_IntMasterEnable();

  //
  // Check for errors.
  //
  if(MAP_I2CMasterErr(ui32Base) != I2C_MASTER_ERR_NONE)
    return false;

  //
  // Return 1 if there is no error.
  //
  return true;
}


bool I2CClearBus(uint32_t ui32Base)
{
  uint8_t clkPulseIndex;
  uint32_t ui32GPIOBase;
  uint8_t  ui8GPIOPinSCL, ui8GPIOPinSDA;
   
  switch(ui32Base) {
    case I2C0_BASE: {
      ui32GPIOBase = GPIO_PORTB_BASE; 
      ui8GPIOPinSCL = GPIO_PIN_2;
      ui8GPIOPinSDA = GPIO_PIN_3;
      break;
    }
    case I2C1_BASE: {
      ui32GPIOBase = GPIO_PORTA_BASE; 
      ui8GPIOPinSCL = GPIO_PIN_6;
      ui8GPIOPinSDA = GPIO_PIN_7;
      break;
    }
    case I2C2_BASE: {
      ui32GPIOBase = GPIO_PORTE_BASE; 
      ui8GPIOPinSCL = GPIO_PIN_4;
      ui8GPIOPinSDA = GPIO_PIN_5;
      break;
    }
    case I2C3_BASE: {
      ui32GPIOBase = GPIO_PORTD_BASE; 
      ui8GPIOPinSCL = GPIO_PIN_0;
      ui8GPIOPinSDA = GPIO_PIN_1;
      break;
    }
    case I2C4_BASE: {
      ui32GPIOBase = GPIO_PORTG_BASE;
      ui8GPIOPinSCL = GPIO_PIN_2;
      ui8GPIOPinSDA = GPIO_PIN_3;
      break;
    }
    case I2C5_BASE: {
      ui32GPIOBase = GPIO_PORTG_BASE;
      ui8GPIOPinSCL = GPIO_PIN_6;
      ui8GPIOPinSDA = GPIO_PIN_7;
      break;
    }
    case I2C6_BASE: {
      ui32GPIOBase = GPIO_PORTA_BASE;
      ui8GPIOPinSCL = GPIO_PIN_6;
      ui8GPIOPinSDA = GPIO_PIN_7;
      break;
    }
    case I2C7_BASE: {
      ui32GPIOBase = GPIO_PORTD_BASE;
      ui8GPIOPinSCL = GPIO_PIN_0;
      ui8GPIOPinSDA = GPIO_PIN_1;
      break;
    }
    case I2C8_BASE: {
      ui32GPIOBase = GPIO_PORTD_BASE;
      ui8GPIOPinSCL = GPIO_PIN_2;
      ui8GPIOPinSDA = GPIO_PIN_3;
      break;
    }
    case I2C9_BASE: {
      ui32GPIOBase = GPIO_PORTA_BASE;
      ui8GPIOPinSCL = GPIO_PIN_0;
      ui8GPIOPinSDA = GPIO_PIN_1;
      break;
    }
  }

  if (MAP_GPIOPinRead(ui32GPIOBase, ui8GPIOPinSDA)) return true;

  //
  // Take over the SCL line
  //
  MAP_GPIODirModeSet(ui32GPIOBase, ui8GPIOPinSCL, GPIO_DIR_MODE_OUT);
  
  //
  // Clock through the bus hold up
  //
  for(clkPulseIndex = 0; clkPulseIndex < 10; clkPulseIndex++) { // 9nth cycle acts as a NACK
    // Master tries to assert a logic 1 on the SDA line
    // Check if the SDA line is high
    if (MAP_GPIOPinRead(ui32GPIOBase, ui8GPIOPinSDA)) {
      // The SDA line has been released.
      break;
    } else {
      // Master still sees a Logic 0 on SDA
      // Generate a clock pulse on SCL (1-0-1 transition) at ~100 kHz
      MAP_GPIOPinWrite(ui32GPIOBase, ui8GPIOPinSCL, ui8GPIOPinSCL); // 1
      MAP_SysCtlDelay(MAP_SysCtlClockGet() / 200000);               // 5uS pause
      MAP_GPIOPinWrite(ui32GPIOBase, ui8GPIOPinSCL, 0x0);           // 0
      MAP_SysCtlDelay(MAP_SysCtlClockGet() / 200000);               // 5uS pause
      MAP_GPIOPinWrite(ui32GPIOBase, ui8GPIOPinSCL, ui8GPIOPinSCL); // 1
    }
  }
  
  //
  // Reset the pins for I2C operation
  //
  MAP_GPIODirModeSet(ui32GPIOBase, ui8GPIOPinSCL, GPIO_DIR_MODE_HW);
    
  return (bool) MAP_GPIOPinRead(ui32GPIOBase, ui8GPIOPinSDA);

}
