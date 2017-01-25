/*
 * HAL_ST7735R.h
 *
 *  Created on: Dec 25, 2016
 *      Author: Anthony
 */

#ifndef __DISPLAY_HAL_ST7735_H__
#define __DISPLAY_HAL_ST7735_H__

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

// Project Files
#include "../Peripherals/spi.h"

// Taken from Adafruit ST7735 Library

// some flags for initR() :
#define INITR_GREENTAB 0x0
#define INITR_REDTAB   0x1
#define INITR_BLACKTAB   0x2

#define INITR_18GREENTAB    INITR_GREENTAB
#define INITR_18REDTAB      INITR_REDTAB
#define INITR_18BLACKTAB    INITR_BLACKTAB
#define INITR_144GREENTAB   0x1

#define ST7735_TFTWIDTH  128
// for 1.44" display
#define ST7735_TFTHEIGHT_144 128
// for 1.8" display
#define ST7735_TFTHEIGHT_18  160

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF

typedef struct
{
    //
    // SPI Connection
    //
    tSPIInstance *psSPIInst;

    //
    // Data/Command Pin and Port
    //
    uint8_t ui8DCPin;
    uint32_t ui32DCPort;

    //
    // Tab Color (?)
    //
    uint8_t ui8Tabcolor;

    //
    // Screen Column Start
    //
    uint32_t ui32ColStart;

    //
    // Screen Row Start
    //
    uint32_t ui32RowStart;

    //
    // Screen Height and Width
    //
    uint32_t ui32YMax;
    uint32_t ui32XMax;

    // Reset - must be signed for -1 sentinel
    int8_t i8Rst;

    // Rotation - Taken from Adafruit GFX Library
    uint8_t ui8Rotation;

} tST7735RDisplayInstance;


extern void ST7735RInit(tST7735RDisplayInstance *psDispInst,
                        tSPIInstance *psSPIInst, uint8_t ui8DCPin,
                        uint32_t ui32DCPort, uint8_t ui8Options);
extern void pushColor(tST7735RDisplayInstance *psDispInst, uint16_t color);
extern void pushColorN(tST7735RDisplayInstance *psDispInst, uint16_t color, uint8_t n);
extern void fillScreen(tST7735RDisplayInstance *psDispInst, uint16_t color);
extern void drawPixel(tST7735RDisplayInstance *psDispInst, int16_t x, int16_t y, uint16_t color);
extern void drawFastVLine(tST7735RDisplayInstance *psDispInst, int16_t x, int16_t y, int16_t h, uint16_t color);
extern void drawFastHLine(tST7735RDisplayInstance *psDispInst, int16_t x, int16_t y, int16_t w, uint16_t color);
extern void fillRect(tST7735RDisplayInstance *psDispInst, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
extern void setRotation(tST7735RDisplayInstance *psDispInst, uint8_t r);
extern void invertDisplay(tST7735RDisplayInstance *psDispInst, bool i);
extern uint16_t Color565(uint8_t r, uint8_t g, uint8_t b);

#endif /* __DISPLAY_HAL_ST7735R_H__ */
