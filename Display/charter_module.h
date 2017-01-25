/*
 * charter_module.h
 *
 *
 *  Created on: Dec 26, 2016
 *      Author: Anthony
 */

#ifndef __DISPLAY_CHARTERMODULE_H__
#define __DISPLAY_CHARTERMODULE_H__

#include "HAL_ST7735.h"

#define SPI_CLK_RATE        SysCtlClockGet()
#define SPI_BIT_RATE        (400000)
#define SPI_PKT_SIZE        (8)
#define SPI_SSI_SYS_PERIPH  SYSCTL_PERIPH_SSI0
#define SPI_GPIO_SYS_PERIPH SYSCTL_PERIPH_GPIOA
#define SPI_DC_PORT         GPIO_PORTA_BASE
#define SPI_DC_PIN          GPIO_PIN_4
#define SPI_CS_PORT         GPIO_PORTA_BASE
#define SPI_CS_PIN          GPIO_PIN_3
#define SCLK_GPIO_CONFIG    GPIO_PA2_SSI0CLK
#define SPI_SCLK_PORT       GPIO_PORTA_BASE
#define SPI_SCLK_PIN        GPIO_PIN_2
#define MOSI_GPIO_CONFIG    GPIO_PA5_SSI0TX
#define SPI_MOSI_PORT       GPIO_PORTA_BASE
#define SPI_MOSI_PIN        GPIO_PIN_5

#define PWM_SYS_PERIPH      SYSCTL_PERIPH_GPIOC
#define PWM_GPIO_CONFIG     GPIO_PC4_M0PWM6
#define PWM_GPIO_PORT       GPIO_PORTC_BASE
#define PWM_GPIO_PIN        GPIO_PIN_4

extern void CharterInit(void);
extern void CharterTest(void);

#endif /* _DISPLAY_CHARTERMODULE_H__ */
