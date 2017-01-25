/*
 * charter_module.c
 *

 *  Created on: Dec 26, 2016
 *      Author: Anthony
 */

#include "charter_module.h"

static tST7735RDisplayInstance g_Disp;
static tSPIInstance g_SPI;

void CharterInit(void)
{
    MAP_SysCtlPeripheralEnable(SPI_SSI_SYS_PERIPH);
    MAP_SysCtlPeripheralEnable(SPI_GPIO_SYS_PERIPH);
#ifdef PWM_DISPLAY
    MAP_SysCtlPeripheralEnable(PWM_SYS_PERIPH);

    //
    // PWM Backlight
    // GPIO Pin Mux for PC4 for M0PWM6
    //
    MAP_GPIOPinConfigure(PWM_GPIO_CONFIG);
    MAP_GPIOPinTypePWM(PWM_GPIO_PORT, PWM_GPIO_PIN);
#endif

    //
    // Data/Command Pin
    // GPIO Pin Mux for PA4 for GPIO_PA4
    //
    MAP_GPIOPinTypeGPIOOutput(SPI_DC_PORT, SPI_DC_PIN);

    //
    // Chip Select
    // GPIO Pin Mux for PA3 for GPIO_PA3
    //
    MAP_GPIOPinTypeGPIOOutput(SPI_CS_PORT, SPI_CS_PIN);

    //
    // MOSI
    // GPIO Pin Mux for PA5 for SSI0TX
    //
    MAP_GPIOPinConfigure(MOSI_GPIO_CONFIG);
    MAP_GPIOPinTypeSSI(SPI_MOSI_PORT, SPI_MOSI_PIN);

    //
    // SCLK
    // GPIO Pin Mux for PA2 for SSI0CLK
    //
    MAP_GPIOPinConfigure(SCLK_GPIO_CONFIG);
    MAP_GPIOPinTypeSSI(SPI_SCLK_PORT, SPI_SCLK_PIN);

    // Initialize SPI
    SPIInit(&g_SPI,                     // Main Struct
            SPI_CS_PIN, SPI_CS_PORT,    // CS Port and Pin
            SSI0_BASE,                  // SSI Base
            SPI_CLK_RATE,               // SSI Clock
            SSI_FRF_MOTO_MODE_0,        // Protocol
            SSI_MODE_MASTER,            // Mode
            SPI_BIT_RATE,               // Bit Rate
            SPI_PKT_SIZE);              // Data Width
    // Enable the SPI Bus
    SPIEnable(&g_SPI);

    // Initialize ST7735R TFT
    ST7735RInit(&g_Disp, &g_SPI,
                SPI_DC_PIN, SPI_DC_PORT,
                INITR_18GREENTAB);
}

void CharterTest(void)
{
    CharterInit();
    // Draw something
    fillScreen(&g_Disp, ST7735_GREEN);
}
