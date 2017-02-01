/*
 * charter_module.c
 *

 *  Created on: Dec 26, 2016
 *      Author: Anthony
 */

#include "charter_module.h"

// Display defines
#define X_MAX   (GrContextDpyWidthGet(&g_sContext) - 1)
#define Y_MAX   (GrContextDpyHeightGet(&g_sContext) - 1)

tContext g_sContext;

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

}

void CharterTest(void)
{
    tRectangle sRect;

    CharterInit();

    //
    // Draw pixels to the screen
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = X_MAX;
    sRect.i16YMin = Y_MAX;

    GrContextForegroundSet(&g_sContext, ClrGreen);
    GrRectFill(&g_sContext, &sRect);
}
