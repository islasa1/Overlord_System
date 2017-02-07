/*
 * charter_module.c
 *

 *  Created on: Dec 26, 2016
 *      Author: Anthony
 */

#include "charter_module.h"
#include "neural.h"
#include "AxisLogo128x128.h"
#include "BatteryIcon.h"

// Display defines
#define X_MAX   (GrContextDpyWidthGet(&g_sContext) - 1)
#define Y_MAX   (GrContextDpyHeightGet(&g_sContext) - 1)

tContext g_sContext;
tRectangle g_sBattRect;

//*****************************************************************************
//! Initialize the module
//!
//! This function begins the initialization sequence for the Charter Module.
//!
//! \return None.
//
//*****************************************************************************
void CharterInit(void)
{
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
    // Initialize the display driver
    //
    ST7735R128x128x18Init();

    //
    // Initialize the graphics context
    //
    GrContextInit(&g_sContext, &g_sST7735R128x128x18);

    //
    // Set Battery drawing rectangle
    //

    // Image draw corner plus 1 pixel offset
    g_sBattRect.i16YMin = 5 + BATT_ICON_YMIN_OFFSET;

    // Image draw corner plus image height minus 1 pixel offset
    g_sBattRect.i16YMax = 5 + g_pui8BatteryIcon[3] - BATT_ICON_YMAX_OFFSET;

    // Image draw corner plus image width minus 1 pixel offset
    g_sBattRect.i16XMax = X_MAX - 20 + g_pui8BatteryIcon[1] -
            BATT_ICON_XMAX_OFFSET;
}

//*****************************************************************************
//! Clear the screen
//!
//! This function fills the screen with a black background (default background
//! color.
//!
//! \return None.
//
//*****************************************************************************
void CharterClrScreen(void)
{
    tRectangle sRect;

    //
    // Draw pixels to the screen
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = X_MAX;
    sRect.i16YMax = Y_MAX;

    GrContextForegroundSet(&g_sContext, ClrBlack);
    GrRectFill(&g_sContext, &sRect);
}

//*****************************************************************************
//! Display Team splash screen
//!
//! This function displays a predefined PPM image from a C struct with an
//! appropriate TI grlib image preamble (can be created using TI's pnmtoc tool)
//! for ~1.5 seconds
//!
//! \return None.
//
//*****************************************************************************
void CharterSplashScreen(void)
{
    GrImageDraw(&g_sContext, g_pui8AxisLogo, 0, 0);
    SysCtlDelay(SysCtlClockGet() * 2);
}

//*****************************************************************************
//! Display battery bar
//!
//! \param percentage is the integer value of the current percentage to display
//! (0 - 100).
//!
//! This function will draw the given percentage with a battery symbol overlay
//! onto the display screen
//!
//! \return None.
//
//*****************************************************************************
void CharterShowBattPercent(uint8_t percentage)
{
    GrContextBackgroundSet(&g_sContext, ClrBlack);
    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrImageDraw(&g_sContext, g_pui8BatteryIcon, X_MAX - 20, 5);

    uint32_t ui32Status;

    if(percentage > 65)
    {
        ui32Status = ClrGreen;
    }
    else if(percentage > 20)
    {
        ui32Status = ClrYellow;
    }
    else
    {
        ui32Status = ClrRed;
    }

    GrContextForegroundSet(&g_sContext, ui32Status);
    g_sBattRect.i16XMin = g_sBattRect.i16XMax -
            (int16_t)((float) percentage / 100 * BATT_ICON_DRAW_WIDTH);
    GrRectFill(&g_sContext, &g_sBattRect);
}


//*****************************************************************************
//! Test 1 for the Charter Module
//!
//! This function will run through the ST7735R 128x128 18-bit driver
//! initialization sequence, paint the screen red momentarily, and then display
//! the team logo. This should only be used in VnV testing
//!
//! \return None.
//
//*****************************************************************************
void CharterTest_1(void)
{
    tRectangle sRect;

    CharterInit();

    //
    // Draw pixels to the screen
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = X_MAX;
    sRect.i16YMax = Y_MAX;

    GrContextForegroundSet(&g_sContext, ClrRed);
    GrRectFill(&g_sContext, &sRect);

    SysCtlDelay(SysCtlClockGet() * 1.5);

    CharterSplashScreen();
    CharterClrScreen();

    uint8_t percent = 0;
    while(1)
    {
        CharterShowBattPercent(percent);
        SysCtlDelay(SysCtlClockGet() * 0.1);
        percent += 5;
        percent %= 100;
    }
}

//*****************************************************************************
//! Test 2 for the Charter Module
//!
//! This function will run through the ST7735R 128x128 18-bit driver
//! initialization sequence, and then test the capabilities of the Stellaris
//! Graphics Library (grlib) with the display driver by drawing a set of
//! images continuously.
//!
//! \return None.
//
//*****************************************************************************
void CharterTest_2(void)
{
    CharterInit();

    uint32_t index;
        while (1) {
            for (index = 0; g_pui8Neural[index] != 0x0; index++) {
                GrImageDraw(&g_sContext, g_pui8Neural[index], 0, 0);
                SysCtlDelay(SysCtlClockGet() * 0.1 / 10.0);
            }
        }
}

//*****************************************************************************
//! Test 3 for the Charter Module
//!
//! This function will run through the ST7735R 128x128 18-bit driver
//! initialization sequence, and then test the capabilities of the Stellaris
//! Graphics Library (grlib) with the display driver by using built-in fonts.
//!
//! \return None.
//
//*****************************************************************************
void CharterTest_3(void)
{
    char *testStr = "CHARTER TEST 3";
    CharterInit();

    //
    // Change foreground for white text.
    //
    GrContextForegroundSet(&g_sContext, ClrWhite);

    GrContextFontSet(&g_sContext, g_psFontFixed6x8);
    GrStringDrawCentered(&g_sContext, testStr, -1, X_MAX/2, Y_MAX / 2, false);
}
