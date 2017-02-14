//*****************************************************************************
//
// Standard Includes
//
//*****************************************************************************
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

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

#include "sensorlib/i2cm_drv.h"

#include "sensorlib/hw_ak8963.h"
#include "sensorlib/hw_ak8975.h"

#include "sensorlib/ak8963.h"
#include "sensorlib/ak8975.h"

#include "sensorlib/magneto.h"

#include "utils/uartstdio.h"
#include "utils/cmdline.h"

#include "../Peripherals/gpio.h"
#include "../Peripherals/i2c.h"
#include "../Peripherals/misc.h"

#include "../Sensors/hw_mpu9x50.h"
#include "../Sensors/mpu9x50.h"

#define DELAY 0

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

// I2C Options
#define I2C_SPEED I2C_SPEED_400

// MPU9X50
#define MPU9X50_I2C_BASE  I2C2_BASE
#define MPU9X50_I2C_INT   INT_I2C2
#define MPU9X50_I2C_ADDRESS 0x68
#define MPU9X50_INT_BASE GPIO_PORTE_BASE
#define MPU9X50_INT_PIN  GPIO_PIN_0
#define MPU9X50_X_AXIS X_AXIS // Array index that contains corrected x-axis when mounted
#define MPU9X50_Y_AXIS Z_AXIS // Array index that contains corrected y-axis when mounted
#define MPU9X50_Z_AXIS Y_AXIS // Array index that contains corrected z-axis when mounted
#define MPU9X50_X_FLIP 0 // bool value (0/1) that negates raw x-axis value
#define MPU9X50_Y_FLIP 1 // bool value (0/1) that negates raw y-axis value
#define MPU9X50_Z_FLIP 1 // bool value (0/1) that negates raw z-axis value
#define MPU9X50_ROLL    0
#define MPU9X50_PITCH   1
#define MPU9X50_YAW     2

float g_fAccel[3] = {0};
float g_fGyro[3] = {0};
float g_fMag[3] = {0};
float g_fHead[3] = {0};

#define M_PI       3.14159265358979323846
#define RAD_TO_DEG 57.2957795130823208767
#define DEG_TO_RAD 0.01745329251994329576

// Global instance structures for data acquisition
tI2CMInstance g_sMPU9X50I2CInst;
tMPU9X50 g_sMPU9X50Inst;
tAK8963 g_sAK8963Inst;

//
// Global flags to alert main that I2C transaction is complete, an error occured, or data is ready
//
volatile uint_fast8_t g_vui8MPU9X50DoneFlag = false, g_vui8MPU9X50ErrorFlag = 0;
volatile uint_fast8_t g_vui8AK8963DoneFlag = false, g_vui8AK8963ErrorFlag = 0;


void
ConsoleInit(void) {
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

float
RadToDegrees(float rad) {
  return rad * (180 / M_PI);
}

#define ALPHA 0.01

void
CalculatePitchRoll(float x, float y, float z, float *roll, float *pitch) {
  static float aX = 0;
  static float aY = 0;
  static float aZ = 0;
  aX = x * ALPHA + (aX * (1.0 - ALPHA));
  aY = y * ALPHA + (aY * (1.0 - ALPHA));
  aZ = z * ALPHA + (aZ * (1.0 - ALPHA));
  *roll  = atan2(aY, aZ) * RAD_TO_DEG;
  *pitch = atan2(-aX, sqrt(aY*aY + aZ*aZ)) * RAD_TO_DEG;
}

void
MPU9X50AppCallback(void *pvCallbackData, uint_fast8_t ui8Status) {
//*****************************************************************************
//
// MPU9X50 Sensor callback function.  Called at the end of MPU9X50 sensor
// driver transactions.
//
//*****************************************************************************
  //
  // MPU9X50 Transactions are complete
  //
  g_vui8MPU9X50DoneFlag = true;

  //
  // Store the most recent status in case it was an error condition
  //
  g_vui8MPU9X50ErrorFlag = ui8Status;
}

void
AK8963AppCallback(void *pvCallbackData, uint_fast8_t ui8Status) {
//*****************************************************************************
//
// MPU9X50 Sensor callback function.  Called at the end of MPU9X50 sensor
// driver transactions.
//
//*****************************************************************************
  //
  // MPU9X50 Transactions are complete
  //
  g_vui8AK8963DoneFlag = true;

  //
  // Store the most recent status in case it was an error condition
  //
  g_vui8AK8963ErrorFlag = ui8Status;
}

void
MPU9X50DataAppCallback(void *pvCallbackData, uint_fast8_t ui8Status) {
//*****************************************************************************
//
// MPU9X50 Sensor callback function.  Called at the end of MPU9X50 sensor
// driver transactions.
//
//*****************************************************************************
  //
  // MPU9X50 Transactions are complete. Check for success
  //
  g_vui8MPU9X50DoneFlag = true;
  if(ui8Status == I2CM_STATUS_SUCCESS) {
    MPU9X50DataAccelGetFloat((tMPU9X50*) pvCallbackData, &g_fAccel[0], &g_fAccel[1], &g_fAccel[2]);
    MPU9X50DataGyroGetFloat((tMPU9X50*) pvCallbackData, &g_fGyro[0], &g_fGyro[1], &g_fGyro[2]);

    g_fAccel[MPU9X50_X_AXIS] *= MPU9X50_X_FLIP ? -1 : 1;
    g_fAccel[MPU9X50_Y_AXIS] *= MPU9X50_Y_FLIP ? -1 : 1;
    g_fAccel[MPU9X50_Z_AXIS] *= MPU9X50_Z_FLIP ? -1 : 1;

    g_fGyro[MPU9X50_X_AXIS] *= MPU9X50_X_FLIP ? -1 : 1;
    g_fGyro[MPU9X50_Y_AXIS] *= MPU9X50_Y_FLIP ? -1 : 1;
    g_fGyro[MPU9X50_Z_AXIS] *= MPU9X50_Z_FLIP ? -1 : 1;

    CalculatePitchRoll(g_fAccel[MPU9X50_X_AXIS], g_fAccel[MPU9X50_Y_AXIS], g_fAccel[MPU9X50_Z_AXIS], &g_fHead[MPU9X50_ROLL], &g_fHead[MPU9X50_PITCH]);

    g_fHead[MPU9X50_YAW] = MagnetoHeadingCompute(g_fMag[MPU9X50_X_AXIS], g_fMag[MPU9X50_Y_AXIS], g_fMag[MPU9X50_Z_AXIS], g_fHead[MPU9X50_ROLL]*DEG_TO_RAD, g_fHead[MPU9X50_PITCH]*DEG_TO_RAD) * RAD_TO_DEG;
  }

  //
  // Store the most recent status in case it was an error condition
  //
  g_vui8MPU9X50ErrorFlag = ui8Status;
}

void
AK8963DataAppCallback(void *pvCallbackData, uint_fast8_t ui8Status) {
    g_vui8AK8963DoneFlag = true;
    if(ui8Status == I2CM_STATUS_SUCCESS) {
        AK8963DataMagnetoGetFloat((tAK8963*) pvCallbackData, &g_fMag[0], &g_fMag[1], &g_fMag[2]);

        g_fMag[MPU9X50_X_AXIS] *= MPU9X50_X_FLIP ? -1 : 1;
        g_fMag[MPU9X50_Y_AXIS] *= MPU9X50_Y_FLIP ? -1 : 1;
        g_fMag[MPU9X50_Z_AXIS] *= MPU9X50_Z_FLIP ? -1 : 1;

        //
        // convert mag data to micro-tesla for better human interpretation.
        //
        g_fMag[MPU9X50_X_AXIS] *= 1e6;
        g_fMag[MPU9X50_Y_AXIS] *= 1e6;
        g_fMag[MPU9X50_Z_AXIS] *= 1e6;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8AK8963ErrorFlag = ui8Status;
    AK8963DataRead((tAK8963*) pvCallbackData, AK8963DataAppCallback, pvCallbackData);
}

void
MPU9X50DataRdyIntHandler(void) {
//*****************************************************************************
//
// Called by the NVIC as a result of a MPU9X50 interrupt event
//
//*****************************************************************************
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(MPU9X50_INT_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(MPU9X50_INT_BASE, ulStatus);

    if(ulStatus & MPU9X50_INT_PIN) {
      //
      // MPU9X50 Data is ready for retrieval and processing.
      //
      g_vui8MPU9X50DoneFlag = false;
      MPU9X50DataRead(&g_sMPU9X50Inst, MPU9X50DataAppCallback, &g_sMPU9X50Inst);
    }
}

void
MPU9X50IntHandler(void) {
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sMPU9X50I2CInst);
}

void
AK8963AppI2CWait(char *pcFilename, uint_fast32_t ui32Line) {
//*****************************************************************************
//
// Function to wait for the MPU9X50 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while (!(g_vui8AK8963DoneFlag || g_vui8AK8963ErrorFlag)) {}

    //
    // If an error occurred call the error handler immediately.
    //
    if (g_vui8AK8963ErrorFlag) {
      UARTprintf("AK8963 Error: %d!\r\n", g_vui8AK8963ErrorFlag);
      //MPU9X50AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8AK8963DoneFlag = false;
}

void
MPU9X50AppI2CWait(char *pcFilename, uint_fast32_t ui32Line) {
//*****************************************************************************
//
// Function to wait for the MPU9X50 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while (!(g_vui8MPU9X50DoneFlag || g_vui8MPU9X50ErrorFlag)) {}

    //
    // If an error occurred call the error handler immediately.
    //
    if (g_vui8MPU9X50ErrorFlag) {
      UARTprintf("MPU9X50 Error: %d!\r\n", g_vui8MPU9X50ErrorFlag);
      //MPU9X50AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8MPU9X50DoneFlag = false;
}

void
DeviceI2CInit(void) {
  uint8_t ui8Int;

  //
  // Enable I2C & associated GPIO ports. Register their interrupts
  //
  I2CInit(MPU9X50_I2C_BASE, I2C_SPEED_400);
  I2CIntRegister(MPU9X50_I2C_BASE, MPU9X50IntHandler);

  //
  // Initialize I2C driver library
  //
  I2CMInit(&g_sMPU9X50I2CInst, MPU9X50_I2C_BASE, MPU9X50_I2C_INT, 0xff, 0xff, MAP_SysCtlClockGet());

  //
  // Initialize the MPU9X50 Driver.
  //
  MPU9X50Init(&g_sMPU9X50Inst, &g_sMPU9X50I2CInst, MPU9X50_I2C_ADDRESS, MPU9X50AppCallback, &g_sMPU9X50Inst);

  //
  // Wait for transaction to complete
  //
  MPU9X50AppI2CWait(__FILE__, __LINE__);

  //
  // Write application specific sensor configuration such as filter settings
  // and sensor range settings.
  //
  g_sMPU9X50Inst.pui8Data[0] = MPU9X50_CONFIG_DLPF_CFG_260_256;
  g_sMPU9X50Inst.pui8Data[1] = MPU9X50_GYRO_CONFIG_FS_SEL_2000;
  g_sMPU9X50Inst.pui8Data[2] = (MPU9X50_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                MPU9X50_ACCEL_CONFIG_AFS_SEL_2G);
  MPU9X50Write(&g_sMPU9X50Inst, MPU9X50_O_CONFIG, g_sMPU9X50Inst.pui8Data, 3, MPU9X50AppCallback, &g_sMPU9X50Inst);

  //
  // Wait for transaction to complete
  //
  MPU9X50AppI2CWait(__FILE__, __LINE__);

  //
  // Kill the internal master mode
  //
  g_sMPU9X50Inst.pui8Data[0] = 0x0;
  MPU9X50Write(&g_sMPU9X50Inst, MPU9X50_O_USER_CTRL,
               g_sMPU9X50Inst.pui8Data, 1, MPU9X50AppCallback,
               &g_sMPU9X50Inst);

  MPU9X50AppI2CWait(__FILE__, __LINE__);

  //
  // Configure the data ready interrupt pin output of the MPU9X50.
  //
  g_sMPU9X50Inst.pui8Data[0] = MPU9X50_INT_PIN_CFG_INT_LEVEL |
                                  MPU9X50_INT_PIN_CFG_INT_RD_CLEAR |
                                  MPU9X50_INT_PIN_CFG_LATCH_INT_EN |
                                  MPU9X50_INT_PIN_CFG_I2C_BYPASS_EN;
  g_sMPU9X50Inst.pui8Data[1] = MPU9X50_INT_ENABLE_DATA_RDY_EN;
  MPU9X50Write(&g_sMPU9X50Inst, MPU9X50_O_INT_PIN_CFG,
               g_sMPU9X50Inst.pui8Data, 2, MPU9X50AppCallback,
               &g_sMPU9X50Inst);

  //
  // Wait for transaction to complete
  //
  MPU9X50AppI2CWait(__FILE__, __LINE__);

  AK8963Init(&g_sAK8963Inst, &g_sMPU9X50I2CInst, 0x0C, AK8963AppCallback, &g_sAK8963Inst);

  AK8963AppI2CWait(__FILE__, __LINE__);

  g_sAK8963Inst.pui8Data[0] = AK8963_CNTL_MODE_CONT_2;
  AK8963Write(&g_sAK8963Inst, AK8975_O_CNTL,
              g_sAK8963Inst.pui8Data, 1, AK8963AppCallback,
              &g_sAK8963Inst);

  AK8963AppI2CWait(__FILE__, __LINE__);

  AK8963DataRead(&g_sAK8963Inst, AK8963DataAppCallback, &g_sAK8963Inst);

  //
  // Initialize the MPU9X50 Interrupt Pins
  //
  GPIOInputInit(MPU9X50_INT_BASE, MPU9X50_INT_PIN, GPIO_PIN_TYPE_STD);
  GPIOIntInit(MPU9X50_INT_BASE, MPU9X50_INT_PIN, GPIO_FALLING_EDGE, -1, MPU9X50DataRdyIntHandler);

  //
  // MPU9X50 Transactions complete
  //
  g_vui8MPU9X50DoneFlag = true;
}

int
main(void) {
  char str[256];
  uint64_t oldTime, newTime;

  //
  // Set the clocking to run at 80MHz.
  //
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  // set up a timer for the fine resolution clock
  // configure Timer as a 64 bit timer
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
  // ensure timer is disabled before configuring
  TimerDisable(WTIMER0_BASE, TIMER_BOTH);
  // timer overflows and restarts
  TimerConfigure(WTIMER0_BASE, TIMER_CFG_A_PERIODIC_UP);
  // Set timer clock source to be system clock
  TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_SYSTEM);
  // Set the overrun value (largely ignored, must be above max expected value)
  TimerLoadSet64(WTIMER0_BASE, 0xFFFFFFFFFFFFFFFF);
  // set the cause of interrupt
  TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
  // timer is free running
  TimerEnable( WTIMER0_BASE, TIMER_A);

  MAP_IntMasterEnable();

  //
  // Enable the FPU
  //
  FPUInit();

  ConsoleInit();
  UARTprintf("IMU Visualization Test\r\n");
  UARTprintf("Initializing...\r\n");
  DeviceI2CInit();
  UARTprintf("Done\r\n");

  int32_t count = 0;

  while (true) {
    newTime = 1000 * TimerValueGet64(WTIMER0_BASE) / MAP_SysCtlClockGet();
    if (newTime - oldTime > DELAY) {
      oldTime = newTime;
      count++;
      while(true) {
          uint8_t data[127] = {0};
          MPU9X50Read(&g_sMPU9X50Inst, 0x0, data, sizeof(data), MPU9X50AppCallback, &g_sMPU9X50Inst);

          MPU9X50AppI2CWait(__FILE__, __LINE__);

          int i = 0;
          for (i = 0; i < sizeof(data); i++) {
              UARTprintf("%d : %02X\r\n", i, data[i]);
          }
          SysCtlDelay(SysCtlClockGet() * 10 / 3);
      }
      /*
      sprintf(str,  "Data Set: %d\tTime: %llu\r\n"
                    "Int Status: %d\r\n"
                    "Acceleration: %f %f %f\r\n"
                    "Magnetometer: %f %f %f\r\n"
                    "Orientation: %f %f %f\r\n",
                    count, oldTime,
                    (!GPIOPinRead(MPU9X50_INT_BASE, MPU9X50_INT_PIN)),
                    g_fAccel[MPU9X50_X_AXIS], g_fAccel[MPU9X50_Y_AXIS], g_fAccel[MPU9X50_Z_AXIS],
                    g_fMag[MPU9X50_X_AXIS], g_fMag[MPU9X50_Y_AXIS], g_fMag[MPU9X50_Z_AXIS],
                    g_fHead[MPU9X50_ROLL], g_fHead[MPU9X50_PITCH], g_fHead[MPU9X50_YAW]);
     UARTprintf("%s", str);
     */
      //printf("%s", str);
    }
  }
}
