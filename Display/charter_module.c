/*
 * charter_module.c
 *

 *  Created on: Dec 26, 2016
 *      Author: Anthony
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "utils/ustdlib.h"
#include "utils/sine.h"
#include "charter_module.h"

// Icons
#include "neural.h"
#include "AxisLogo128x128.h"
#include "BatteryIcon.h"
#include "ChargeIcon.h"

// Display defines
#define X_MAX               (GrContextDpyWidthGet(&g_sContext) - 1)
#define Y_MAX               (GrContextDpyHeightGet(&g_sContext) - 1)

#define HEADING_ARROW_HEIGHT    17
#define HEADING_ARROW_WIDTH     7
#define HEADING_LENGTH          38
#define HEADING_CENTER_X        (X_MAX / 2)
#define HEADING_CENTER_Y        (Y_MAX / 2)

#define HEADING_XOFFSET(radius, theta)      (int8_t) ((float)(cosf(theta) * \
                                            (radius)) + 0.5);
#define HEADING_YOFFSET(radius, theta)      (int8_t) ((float)(sinf(theta) * \
                                            (radius)) + 0.5);

#ifndef M_PI
#define M_PI            3.14159265358979323846
#endif
#define SENSORS_DEGREES_TO_RADIANS(d)           ((d) * M_PI / 180.0)           /**< Degrees to Radians */
#define SENSORS_RADIANS_TO_DEGREES(r)           ((r) * 180.0 / M_PI)           /**< Radians to Degrees */

tContext g_sContext;
tRectangle g_sBattImageRect;
tRectangle g_sBattPercentRect;

typedef struct
{
    //
    //! The farthest X offset from the heading center
    //
    int8_t i8XHiOffset;

    //
    //! The farthest Y offset from the heading center
    //
    int8_t i8YHiOffset;


    //
    //! The closest X offset (\em radius - 3)
    //
    int8_t i8XBaseOffset;

    //
    //! The closest Y offset (\em radius - 3)
    //
    int8_t i8YBaseOffset;

    //
    //! The X offset from the base level to draw arrow line orthogonal
    //
    int8_t i8XBaseArrowOffset;

    //
    //! The Y offset from the base level to draw arrow line orthogonal
    //
    int8_t i8YBaseArrowOffset;


} tHeadingPosition;

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
    g_sBattImageRect.i16YMin = 4 + BATT_ICON_YMIN_OFFSET;

    // Image draw corner plus image height minus 1 pixel offset
    g_sBattImageRect.i16YMax = 4 + g_pui8BatteryIcon[3] - BATT_ICON_YMAX_OFFSET;

    // Image draw corner plus image width minus 1 pixel offset
    g_sBattImageRect.i16XMax = X_MAX - 20 + g_pui8BatteryIcon[1] -
            BATT_ICON_XMAX_OFFSET;

    //
    // Battery text rectangle to clear it
    //
    g_sBattPercentRect.i16XMax = X_MAX - 22;
    g_sBattPercentRect.i16XMin = X_MAX - 45;
    g_sBattPercentRect.i16YMin = 0;
    g_sBattPercentRect.i16YMax = 12;

    //
    // Change foreground for white text.
    //
    GrContextForegroundSet(&g_sContext, ClrWhite);

    GrContextFontSet(&g_sContext, g_psFontFixed6x8);
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
void CharterClrScreen(tContext *psContext)
{
    tRectangle sRect;

    //
    // Draw pixels to the screen
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = X_MAX;
    sRect.i16YMax = Y_MAX;

    uint32_t ui32OrigColor = psContext->ui32Foreground;
    GrContextForegroundSet(psContext, ClrBlack);
    GrRectFill(psContext, &sRect);
    psContext->ui32Foreground = ui32OrigColor;
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
//! Display battery bar.
//!
//! \param percentage is the integer value of the current percentage to display
//! [0 - 100].
//!
//! This function will draw the given percentage with a battery symbol overlay
//! onto the display screen
//!
//! \return None.
//
//*****************************************************************************
void CharterShowBattPercent(uint8_t percentage, bool isCharging)
{
    char percentStr[6];

    //
    // Put the percent string
    //
    usprintf(percentStr, "%u%%", percentage);
    GrContextForegroundSet(&g_sContext, ClrBlack);
    GrRectFill(&g_sContext, &g_sBattPercentRect);

    GrContextBackgroundSet(&g_sContext, ClrBlack);
    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrImageDraw(&g_sContext, g_pui8BatteryIcon, X_MAX - 20, 5);

    GrStringDrawCentered(&g_sContext, (const char*)percentStr, -1, X_MAX - 32, 10, false);

    uint32_t ui32Status;

    if(percentage > 85)
    {
        ui32Status = ClrGreen;
    }
    else if(percentage > 65)
    {
        ui32Status = ClrOrange;
    }
    else if(percentage > 20)
    {
        ui32Status = ClrOrangeRed;
    }
    else
    {
        ui32Status = ClrRed;
    }

    GrContextForegroundSet(&g_sContext, ui32Status);
    g_sBattImageRect.i16XMin = g_sBattImageRect.i16XMax -
            (int16_t)((float) percentage / 100 * BATT_ICON_DRAW_WIDTH);
    GrRectFill(&g_sContext, &g_sBattImageRect);

    if(isCharging)
    {
        GrContextBackgroundSet(&g_sContext, ClrBlack);
        GrContextForegroundSet(&g_sContext, ClrWhite);
        //
        // GrTransparentImageDraw color takes index of color from color palette to
        // set as transparent
        //
        GrTransparentImageDraw(&g_sContext, g_pui8BatteryChargeIcon, X_MAX - 20 - 1, 5, 2);
    }

    //
    // Reset the foreground color
    //
    GrContextForegroundSet(&g_sContext, ClrWhite);
}


void CharterDrawHeadingArrow(tContext *psContext, tHeadingPosition *psHeading)
{
    //
    // Lines
    //
    GrLineDraw(psContext, HEADING_CENTER_X + psHeading->i8XHiOffset,
                          HEADING_CENTER_Y + psHeading->i8YHiOffset,
                          HEADING_CENTER_X + psHeading->i8XBaseOffset +
                              psHeading->i8XBaseArrowOffset,
                          HEADING_CENTER_Y + psHeading->i8YBaseOffset +
                              psHeading->i8YBaseArrowOffset);

    GrLineDraw(psContext, HEADING_CENTER_X + psHeading->i8XHiOffset,
                          HEADING_CENTER_Y + psHeading->i8YHiOffset,
                          HEADING_CENTER_X + psHeading->i8XBaseOffset -
                              psHeading->i8XBaseArrowOffset,
                          HEADING_CENTER_Y + psHeading->i8YBaseOffset -
                              psHeading->i8YBaseArrowOffset);

    GrLineDraw(psContext, HEADING_CENTER_X + psHeading->i8XBaseOffset -
                              psHeading->i8XBaseArrowOffset,
                          HEADING_CENTER_Y + psHeading->i8YBaseOffset -
                              psHeading->i8YBaseArrowOffset,
                          HEADING_CENTER_X + psHeading->i8XBaseOffset +
                              psHeading->i8XBaseArrowOffset,
                          HEADING_CENTER_Y + psHeading->i8YBaseOffset +
                              psHeading->i8YBaseArrowOffset);
}

void CharterDrawHeadingLine(tContext *psContext, tHeadingPosition *psHeading)
{
    //
    // Main Line
    //
    GrLineDraw(psContext, HEADING_CENTER_X - psHeading->i8XHiOffset,
                          HEADING_CENTER_Y - psHeading->i8YHiOffset,
                          HEADING_CENTER_X + psHeading->i8XHiOffset,
                          HEADING_CENTER_Y + psHeading->i8YHiOffset);
}

//*****************************************************************************
//! Display angle to the screen.
//!
//! \param angle is a float value in degrees [0 - 360)
//!
//! This function will draw an angle heading on the screen \em relative \em to
//! the X-axis 0° from the display orientation.
//!
//! \return None.
//
//*****************************************************************************
void CharterDrawHeading(tContext *psContext, float angle)
{
    static tHeadingPosition sPrevHeading;
    tHeadingPosition sHeading;

    angle = -SENSORS_DEGREES_TO_RADIANS(angle);

    //
    // Find tip location
    //
    sHeading.i8XHiOffset = HEADING_XOFFSET(HEADING_LENGTH, angle);
    sHeading.i8YHiOffset = HEADING_YOFFSET(HEADING_LENGTH, angle);

    //
    // Last position
    //
    sHeading.i8XBaseOffset = HEADING_XOFFSET(HEADING_LENGTH -
                                             HEADING_ARROW_HEIGHT, angle);
    sHeading.i8YBaseOffset = HEADING_YOFFSET(HEADING_LENGTH -
                                             HEADING_ARROW_HEIGHT, angle);


    //
    // Calculate arrow offsets
    //
    sHeading.i8XBaseArrowOffset = HEADING_XOFFSET(HEADING_ARROW_WIDTH,
                                                 (angle + M_PI / 2.0));
    sHeading.i8YBaseArrowOffset = HEADING_YOFFSET(HEADING_ARROW_WIDTH,
                                                 (angle + M_PI / 2.0));

    //
    // Save the previous foreground
    //
    uint32_t ui32OrigColor = psContext->ui32Foreground;

    //
    // Destroy previous heading
    //
    GrContextForegroundSet(psContext, ClrBlack);
    CharterDrawHeadingArrow(psContext, &sPrevHeading);
    CharterDrawHeadingLine(psContext, &sPrevHeading);

    //
    // Draw new heading in original color
    //
    psContext->ui32Foreground = ui32OrigColor;
    CharterDrawHeadingArrow(psContext, &sHeading);
    CharterDrawHeadingLine(psContext, &sHeading);
    //
    // Save previous state
    //
    sPrevHeading = sHeading;
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

    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrRectFill(&g_sContext, &sRect);

    SysCtlDelay(SysCtlClockGet() / 2 / 3);

    CharterSplashScreen();

    CharterClrScreen(&g_sContext);

    float percent = 0;
    bool charging = false;
    while(1)
    {
        CharterShowBattPercent((uint8_t) percent, charging);
        CharterDrawHeading(&g_sContext, percent * 360 / 100);
        SysCtlDelay(SysCtlClockGet() * 0.05);
        percent += 2.5;
        percent = percent > 100 ? percent - 100 : percent;
        charging = !charging;
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
