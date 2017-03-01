/*
 * charter_module.c
 *

 *  Created on: Dec 26, 2016
 *      Author: Anthony
 */

#include "charter_module.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "grlib/grlib.h"
#include "grlib/offscr8bpp.c"

#include "utils/ustdlib.h"
#include "utils/sine.h"

#include "axis_logo128x128.h"
#include "battery_icon.h"
#include "charge_icon.h"
#include "neural.h"

// Display defines
#define X_MAX                           (GrContextDpyWidthGet(&g_sTFTContext) - 1)
#define Y_MAX                           (GrContextDpyHeightGet(&g_sTFTContext) - 1)

#define HEADING_ARROW_HEIGHT            17
#define HEADING_ARROW_WIDTH             7
#define HEADING_LENGTH                  38
#define HEADING_CENTER_X                (X_MAX / 2)
#define HEADING_CENTER_Y                (Y_MAX / 2)

#define HEADING_XOFFSET(radius, theta)  (int8_t) ((float)(cosf(theta) * \
                                        (radius)) + 0.5);
#define HEADING_YOFFSET(radius, theta)  (int8_t) ((float)(sinf(theta) * \
                                        (radius)) + 0.5);

#ifndef M_PI
#define M_PI                            3.14159265358979323846
#endif
#define SENSORS_DEGREES_TO_RADIANS(d)   ((d) * M_PI / 180.0)           /**< Degrees to Radians */
#define SENSORS_RADIANS_TO_DEGREES(r)   ((r) * 180.0 / M_PI)           /**< Radians to Degrees */

#define OffScreenFlush(psContext)       GrImageDraw(psContext, \
                                                    g_sOffScreenContext.\
                                                    psDisplay->pvDisplayData,\
                                                    0, 0)


//*****************************************************************************
//
// Graphics context used for the actual physical display
//
//*****************************************************************************
tContext g_sTFTContext;

//*****************************************************************************
//
// Graphics contexts and display for an OffScreen buffer
//
//*****************************************************************************
#define OFFSCREEN_BUF_SIZE      GrOffScreen8BPPSize(128,128)
uint8_t g_pui8OffScreenBuf[OFFSCREEN_BUF_SIZE];

tContext g_sOffScreenContext;
tDisplay g_sOffScreenDisplay;

bool g_bOffscreen;

//*****************************************************************************
//
// Create a palette for the off-screen buffer
//
//*****************************************************************************
uint32_t g_pui32Palette[] =
{
    ClrGreen,
    ClrBlue,
    ClrRed,
    ClrWhite,
    ClrBlack,
    ClrOrange,
    ClrOrangeRed,
    ClrDarkSlateGray,
};
#define NUM_PALETTE_ENTRIES     (sizeof(g_pui32Palette) / sizeof(uint32_t))

//*****************************************************************************
//
// Simplify draw methods
//
//*****************************************************************************
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
//! \param offscreen is true for utilizing the display driver's offscreen
//! capabilities.
//!
//! \return None.
//
//*****************************************************************************
void CharterInit(bool bOffScreen)
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

    g_bOffscreen = bOffScreen;
    //
    // Initialize the display driver
    //
    ST7735R128x128x18Init();

    if(g_bOffscreen)
    {
        //
        // Initialize the OffScreen display buffer and assign palette
        //
        GrOffScreen8BPPInit(&g_sOffScreenDisplay, g_pui8OffScreenBuf,
                            g_sST7735R128x128x18.ui16Width,
                            g_sST7735R128x128x18.ui16Height);
        GrOffScreen8BPPPaletteSet(&g_sOffScreenDisplay, g_pui32Palette,
                                      0, NUM_PALETTE_ENTRIES);

        //
        // Initialize the OffScreen context and display
        //
        GrContextInit(&g_sOffScreenContext, &g_sOffScreenDisplay);
        GrContextFontSet(&g_sOffScreenContext, g_psFontFixed6x8);
    }

    //
    // Initialize the graphics context
    //
    GrContextInit(&g_sTFTContext, &g_sST7735R128x128x18);
    GrContextFontSet(&g_sTFTContext, g_psFontFixed6x8);

    //
    // Set Battery drawing rectangle
    //

    // Image draw corner plus 1 pixel offset
    g_sBattImageRect.i16YMin = 5 + BATT_ICON_YMIN_OFFSET;

    // Image draw corner plus image height minus 1 pixel offset
    g_sBattImageRect.i16YMax = 5 + g_pui8BatteryIcon[3] - BATT_ICON_YMAX_OFFSET;

    // Image draw corner plus image width minus 1 pixel offset
    g_sBattImageRect.i16XMax = X_MAX - 20 + g_pui8BatteryIcon[1] -
            BATT_ICON_XMAX_OFFSET;

    //
    // Battery text rectangle to clear it
    //
    g_sBattPercentRect.i16XMax = X_MAX - 22;
    g_sBattPercentRect.i16XMin = X_MAX - 45;
    g_sBattPercentRect.i16YMin = 0;
    g_sBattPercentRect.i16YMax = 13;

    //
    // Change foreground for white text.
    //
    GrContextForegroundSet(&g_sTFTContext, ClrWhite);
    GrContextForegroundSet(&g_sOffScreenContext, ClrWhite);
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
    tContext *psContext;

    if(g_bOffscreen)
    {
        psContext = &g_sOffScreenContext;
    }
    else
    {
        psContext = &g_sTFTContext;
    }
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
    GrImageDraw(&g_sTFTContext, g_pui8AxisLogo, 0, 0);
    SysCtlDelay(SysCtlClockGet() * 2);
    CharterClrScreen();
}

//*****************************************************************************
//! Display battery bar.
//!
//! \param ui8Percentage is the integer value of the current ui8Percentage to display
//! [0 - 100].
//!
//! This function will draw the given ui8Percentage with a battery symbol overlay
//! onto the display screen
//!
//! \return None.
//
//*****************************************************************************
void CharterShowBattPercent(uint8_t ui8Percentage, bool isCharging)
{
    char percentStr[6];
    tContext *psContext;

    if(g_bOffscreen)
    {
        psContext = &g_sOffScreenContext;
    }
    else
    {
        psContext = &g_sTFTContext;
    }

    //
    // Put the percent string
    //
    usprintf(percentStr, "%u%%", ui8Percentage);
    GrContextForegroundSet(psContext, ClrBlack);
    GrRectFill(psContext, &g_sBattPercentRect);

    GrContextBackgroundSet(psContext, ClrBlack);
    GrContextForegroundSet(psContext, ClrWhite);
    GrImageDraw(psContext, g_pui8BatteryIcon, X_MAX - 20, 5);

    GrStringDrawCentered(psContext, (const char*)percentStr, -1, X_MAX - 32, 10, false);

    uint32_t ui32Status;

    if(ui8Percentage > 80)
    {
        ui32Status = ClrGreen;
    }
    else if(ui8Percentage > 50)
    {
        ui32Status = ClrOrange;
    }
    else if(ui8Percentage > 25)
    {
        ui32Status = ClrOrangeRed;
    }
    else
    {
        ui32Status = ClrRed;
    }

    GrContextForegroundSet(psContext, ui32Status);
    g_sBattImageRect.i16XMin = g_sBattImageRect.i16XMax -
            (int16_t)((float) ui8Percentage / 100 * (BATT_ICON_DRAW_WIDTH - 1));
    GrRectFill(psContext, &g_sBattImageRect);

    if(isCharging)
    {
        GrContextBackgroundSet(psContext, ClrBlack);
        GrContextForegroundSet(psContext, ClrWhite);
        //
        // GrTransparentImageDraw color takes index of color from color palette to
        // set as transparent
        //
        GrTransparentImageDraw(psContext, g_pui8BatteryChargeIcon, X_MAX - 20 - 1, 5, 2);
    }

    //
    // Reset the foreground color
    //
    GrContextForegroundSet(psContext, ClrWhite);
}

//
// Internal Use
//
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

//
// Internal Use
//
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
//! \param angle is a float value in radians [0 - 2*pi)
//!
//! This function will draw an angle heading on the screen \em relative \em to
//! the X-axis 0° from the display orientation.
//!
//! \return None.
//
//*****************************************************************************
void CharterDrawHeading(float angle)
{
    static tHeadingPosition sPrevHeading;
    tHeadingPosition sHeading;
    tContext *psContext;

    if(g_bOffscreen)
    {
        psContext = &g_sOffScreenContext;
    }
    else
    {
        psContext = &g_sTFTContext;
    }


    angle = -(angle);

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
//! Flush the graphics controller
//!
//! This function will flush any graphics operations waiting from the graphics
//! controller.
//!
//! \return None.
//*****************************************************************************
void CharterFlush(void)
{
    // Nothing needs to be done if raw display
    if(g_bOffscreen)
    {
        OffScreenFlush(&g_sTFTContext);
    }
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

    CharterInit(true);

    //
    // Draw pixels to the screen
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = X_MAX;
    sRect.i16YMax = Y_MAX;

    GrContextForegroundSet(&g_sOffScreenContext, ClrWhite);
    GrRectFill(&g_sOffScreenContext, &sRect);
    OffScreenFlush(&g_sTFTContext);

    MAP_SysCtlDelay(MAP_SysCtlClockGet() / 2 / 3);

    //
    // Draw to main screen anyways since it is an image
    //
    CharterSplashScreen();


    CharterClrScreen();
    OffScreenFlush(&g_sTFTContext);

    float percent = 0;
    uint8_t charging = 0x01;
    while(1)
    {
        CharterDrawScreen(percent * 360 / 100, (uint8_t) percent, charging < 0x0f);
/*        CharterShowBattPercent(&g_sOffScreenContext, (uint8_t) percent, charging < 0x0f);
        CharterDrawHeading(&g_sOffScreenContext, percent * 360 / 100);
        OffScreenFlush(&g_sTFTContext);*/
        MAP_SysCtlDelay(MAP_SysCtlClockGet() * 0.005);
        percent += 0.5;
        percent = percent > 100 ? percent - 100 : percent;
        charging = (charging <<= 1) == 0 ? 1 : charging;
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
    CharterInit(true);

    uint32_t index;
    while (1)
    {
        for (index = 0; g_pui8Neural[index] != 0x0; index++)
        {
            GrImageDraw(&g_sOffScreenContext, g_pui8Neural[index], 0, 0);
            OffScreenFlush(&g_sTFTContext);
            MAP_SysCtlDelay(MAP_SysCtlClockGet() * 0.1 / 10.0);
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
    CharterInit(true);

    //
    // Change foreground for white text.
    //
    GrContextForegroundSet(&g_sOffScreenContext, ClrWhite);

    GrStringDrawCentered(&g_sOffScreenContext, testStr, -1, X_MAX/2, Y_MAX / 2, false);

    OffScreenFlush(&g_sTFTContext);
}
