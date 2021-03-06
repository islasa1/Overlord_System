/*
 * charter_module.h
 *
 *
 *  Created on: Dec 26, 2016
 *      Author: Anthony
 */

#ifndef __DISPLAY_CHARTERMODULE_H__
#define __DISPLAY_CHARTERMODULE_H__

#include "../peripherals/st7735r128x128x18.h"

#define PWM_SYS_PERIPH      SYSCTL_PERIPH_GPIOC
#define PWM_GPIO_CONFIG     GPIO_PC4_M0PWM6
#define PWM_GPIO_PORT       GPIO_PORTC_BASE
#define PWM_GPIO_PIN        GPIO_PIN_4

//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
extern void CharterInit(bool);
extern void CharterClrScreen(void);
extern void CharterSplashScreen(void);
extern void CharterShowBattPercent(uint8_t ui8Percentage, bool isCharging);
extern void CharterDrawHeading(float angle);
extern void CharterDrawPosition(float x, float y);
extern void CharterFlush(void);

extern void CharterTest_1(void);
extern void CharterTest_2(void);
extern void CharterTest_3(void);

#endif /* _DISPLAY_CHARTERMODULE_H__ */
