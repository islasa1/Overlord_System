/*
 * HAL_ST7735R.c
 *
 *  Created on: Dec 25, 2016
 *      Author: Anthony
 */

#include "HAL_ST7735.h"

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

static uint16_t swapcolor(uint16_t x)
{
    return ((x << 11) | (x & 0x07e0) | (x >> 11));
}

#define DELAY 0x80

// Command Tables
static const uint8_t Rcmd1[] = {
                              // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 };

static const uint8_t Rcmd2green[] = {
                              // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 };      //     XEND = 159

static const uint8_t Rcmd2red[] = {
                              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F };           //     XEND = 159

static const uint8_t Rcmd2green144[] = {
                              // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F };           //     XEND = 127

static const uint8_t Rcmd3[] = {
                              // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay

// Pass RGB Components as Bytes and return 16-bit color
uint16_t Color565(uint8_t r, uint8_t g, uint8_t b)
{
    return (((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
}

static void enableSPIcommand(tST7735RDisplayInstance *psDispInst)
{
    GPIOPinWrite(psDispInst->psSPIInst->ui32CSPort,
              psDispInst->psSPIInst->ui8CSPin,
              0);
    GPIOPinWrite(psDispInst->ui32DCPort, psDispInst->ui8DCPin, psDispInst->ui8DCPin);
}

static void enableSPIdata(tST7735RDisplayInstance *psDispInst)
{
    GPIOPinWrite(psDispInst->psSPIInst->ui32CSPort,
              psDispInst->psSPIInst->ui8CSPin,
              0);
    GPIOPinWrite(psDispInst->ui32DCPort, psDispInst->ui8DCPin, 0);
}

static void disableCS(tST7735RDisplayInstance *psDispInst)
{
    GPIOPinWrite(psDispInst->psSPIInst->ui32CSPort,
              psDispInst->psSPIInst->ui8CSPin,
              psDispInst->psSPIInst->ui8CSPin);
}

static void writecommand(tST7735RDisplayInstance *psDispInst, uint8_t c)
{
    // Begin SPI Transaction
    enableSPIcommand(psDispInst);

    // SPI Write
    SPIWrite(psDispInst->psSPIInst, c);

    // End SPI Transaction
    disableCS(psDispInst);

}

static void writedata(tST7735RDisplayInstance *psDispInst, uint8_t c)
{
    // Begin SPI Transaction
    enableSPIdata(psDispInst);

    // SPI Write
    SPIWrite(psDispInst->psSPIInst, c);

    // End SPI Transaction
    disableCS(psDispInst);

}

static void commandList(tST7735RDisplayInstance *psDispInst, const uint8_t *addr)
{
    uint8_t  numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr;                    // Number of commands to follow
    addr++;

    while(numCommands--)
    {                                       // For each command...
      writecommand(psDispInst, *addr);      //   Read, issue command
      addr++;

      numArgs  = *addr;                     //   Number of args to follow
      addr++;

      ms       = numArgs & DELAY;           //   If hibit set, delay follows args
      numArgs &= ~DELAY;                    //   Mask out delay bit
      while(numArgs--)
      {                                     //   For each argument...
          writedata(psDispInst, *addr);     //   Read, issue argument
          addr++;
      }

      if(ms)
      {
        ms = *addr;                         // Read post-command delay time (ms)
        addr++;

        if(ms == 255)
            ms = 500;                       // If 255, delay for 500 ms
        SysCtlDelay(SysCtlClockGet() / 1000 * ms);
      }
    }
}

static void commonInit(tST7735RDisplayInstance *psDispInst, const uint8_t *cmdList)
{
    psDispInst->ui32ColStart = 0;
    psDispInst->ui32RowStart = 0;

    // Toggle Reset Low to reset, CS low so device will listen

    // Init
    if(cmdList != (void*)0) // Find NULL?
    {
        commandList(psDispInst, cmdList);
    }
}

static void setAddrWindow(tST7735RDisplayInstance *psDispInst, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    writecommand(psDispInst, ST7735_CASET);                 // Column Addr Set
    writedata(psDispInst, 0x00);                            // Start Frame
    writedata(psDispInst, x0 + psDispInst->ui32ColStart);   // X Start
    writedata(psDispInst, 0x00);                            // Start Frame
    writedata(psDispInst, x1 + psDispInst->ui32ColStart);   // X End

    writecommand(psDispInst, ST7735_RASET);                 // Row Addr Set
    writedata(psDispInst, 0x00);                            // Start Frame
    writedata(psDispInst, y0 + psDispInst->ui32RowStart);   // Y Start
    writedata(psDispInst, 0x00);                            // Start Frame
    writedata(psDispInst, y1 + psDispInst->ui32RowStart);   // Y End

    writecommand(psDispInst, ST7735_RAMWR);                 // Write to RAM
}

void ST7735RInit(tST7735RDisplayInstance *psDispInst,
                 tSPIInstance *psSPIInst, uint8_t ui8DCPin,
                 uint32_t ui32DCPort, uint8_t ui8Options)
{
    psDispInst->ui8DCPin = ui8DCPin;
    psDispInst->ui32DCPort = ui32DCPort;

    psDispInst->psSPIInst = psSPIInst;
    psDispInst->ui32XMax = ST7735_TFTWIDTH;
    psDispInst->ui32YMax = ST7735_TFTHEIGHT_18;
    psDispInst->ui8Rotation = 0;

    commonInit(psDispInst, Rcmd1);

    if(ui8Options == INITR_GREENTAB)
    {
        commandList(psDispInst, Rcmd2green);
        psDispInst->ui32ColStart = 2;
        psDispInst->ui32RowStart = 1;
    }
    else if(ui8Options == INITR_144GREENTAB)
    {
        psDispInst->ui32YMax = ST7735_TFTHEIGHT_144;
        commandList(psDispInst, Rcmd2green144);
        psDispInst->ui32ColStart = 2;
        psDispInst->ui32RowStart = 1;
    }
    else
    {
        commandList(psDispInst, Rcmd2red);
    }

    commandList(psDispInst, Rcmd3);

    // If black, change MADCTL color filter
    if(ui8Options == INITR_BLACKTAB)
    {
        writecommand(psDispInst, ST7735_MADCTL);
        writedata(psDispInst, 0xc0); // What is this Adafruit..?
    }

    psDispInst->ui8Tabcolor = ui8Options;
}

void pushColor(tST7735RDisplayInstance *psDispInst, uint16_t color)
{
    // Begin SPI Transaction
    enableSPIdata(psDispInst);

    // SPI Write, could potentially be switched to 16-bit transfer?
    SPIWrite(psDispInst->psSPIInst, color >> 8);
    SPIWrite(psDispInst->psSPIInst, color);

    // End SPI Transaction
    disableCS(psDispInst);
}

// Push a color n times
void pushColorN(tST7735RDisplayInstance *psDispInst, uint16_t color, uint8_t n)
{
    uint8_t hi = color >> 8;
    uint8_t lo = color;

    // Begin SPI Transaction
    enableSPIdata(psDispInst);

    while(n--)
    {
        // SPI Write, could potentially be switched to 16-bit transfer?
        SPIWrite(psDispInst->psSPIInst, hi);
        SPIWrite(psDispInst->psSPIInst, lo);
    }

    // End SPI Transaction
    disableCS(psDispInst);
}

void drawPixel(tST7735RDisplayInstance *psDispInst, int16_t x, int16_t y, uint16_t color)
{
    if((x < 0) || (x >= psDispInst->ui32XMax) || (y < 0) || (y >= psDispInst->ui32YMax))
    {
        return;
    }

    setAddrWindow(psDispInst, x, y, x + 1, y + 1);

    pushColor(psDispInst, color);
}

void drawFastVLine(tST7735RDisplayInstance *psDispInst, int16_t x, int16_t y, int16_t h, uint16_t color)
{
    // Clipping
    if((x >= psDispInst->ui32XMax) || (y >= psDispInst->ui32YMax))
    {
        return;
    }
    if((y + h - 1) >= psDispInst->ui32YMax)
    {
        h = psDispInst->ui32YMax - y;
    }

    setAddrWindow(psDispInst, x, y, x, y + h - 1);

    pushColorN(psDispInst, color, h);

}

void drawFastHLine(tST7735RDisplayInstance *psDispInst, int16_t x, int16_t y, int16_t w, uint16_t color)
{
    // Clipping
    if((x >= psDispInst->ui32XMax) || (y >= psDispInst->ui32YMax))
    {
        return;
    }
    if((x + w - 1) >= psDispInst->ui32XMax)
    {
        w = psDispInst->ui32XMax - x;
    }

    setAddrWindow(psDispInst, x, y, x + w - 1, y);

    pushColorN(psDispInst, color, w);

}

void fillRect(tST7735RDisplayInstance *psDispInst, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    // Clipping
    if((x >= psDispInst->ui32XMax) || (y >= psDispInst->ui32YMax))
    {
        return;
    }
    if((x + w - 1) >= psDispInst->ui32XMax)
    {
        w = psDispInst->ui32XMax - x;
    }
    if((y + h - 1) >= psDispInst->ui32YMax)
    {
        h = psDispInst->ui32YMax - y;
    }

    setAddrWindow(psDispInst, x, y, x + w - 1, y + h - 1);

    pushColorN(psDispInst, color, h * w);
}

void fillScreen(tST7735RDisplayInstance *psDispInst, uint16_t color)
{
    fillRect(psDispInst, 0, 0, psDispInst->ui32XMax, psDispInst->ui32YMax, color);
}

void setRotation(tST7735RDisplayInstance *psDispInst, uint8_t r)
{
    writecommand(psDispInst, ST7735_MADCTL);

    psDispInst->ui8Rotation = r % 4;

    switch(psDispInst->ui8Rotation)
    {
        case 0:
            if(psDispInst->ui8Tabcolor == INITR_BLACKTAB)
            {
                writedata(psDispInst, MADCTL_MX | MADCTL_MY | MADCTL_RGB);
            }
            else
            {
                writedata(psDispInst, MADCTL_MX | MADCTL_MY | MADCTL_BGR);
            }

            psDispInst->ui32XMax = ST7735_TFTWIDTH;

            if(psDispInst->ui8Tabcolor == INITR_144GREENTAB)
            {
                psDispInst->ui32YMax = ST7735_TFTHEIGHT_144;
            }
            else
            {
                psDispInst->ui32YMax = ST7735_TFTHEIGHT_18;
            }

            break;
        case 1:
            if(psDispInst->ui8Tabcolor == INITR_BLACKTAB)
            {
                writedata(psDispInst, MADCTL_MY | MADCTL_MV | MADCTL_RGB);
            }
            else
            {
                writedata(psDispInst, MADCTL_MY | MADCTL_MV | MADCTL_BGR);
            }

            if(psDispInst->ui8Tabcolor == INITR_144GREENTAB)
            {
                psDispInst->ui32XMax = ST7735_TFTHEIGHT_144;
            }
            else
            {
                psDispInst->ui32XMax = ST7735_TFTHEIGHT_18;
            }

            psDispInst->ui32YMax = ST7735_TFTWIDTH;

            break;
        case 2:
            if(psDispInst->ui8Tabcolor == INITR_BLACKTAB)
            {
                writedata(psDispInst, MADCTL_RGB);
            }
            else
            {
                writedata(psDispInst, MADCTL_BGR);
            }

            psDispInst->ui32XMax = ST7735_TFTWIDTH;

            if(psDispInst->ui8Tabcolor == INITR_144GREENTAB)
            {
                psDispInst->ui32YMax = ST7735_TFTHEIGHT_144;
            }
            else
            {
                psDispInst->ui32YMax = ST7735_TFTHEIGHT_18;
            }

            break;
        case 3:
            if(psDispInst->ui8Tabcolor == INITR_BLACKTAB)
            {
                writedata(psDispInst, MADCTL_MX | MADCTL_MV | MADCTL_RGB);
            }
            else
            {
                writedata(psDispInst, MADCTL_MX | MADCTL_MV | MADCTL_BGR);
            }

            if(psDispInst->ui8Tabcolor == INITR_144GREENTAB)
            {
                psDispInst->ui32XMax = ST7735_TFTHEIGHT_144;
            }
            else
            {
                psDispInst->ui32XMax = ST7735_TFTHEIGHT_18;
            }

            psDispInst->ui32YMax = ST7735_TFTWIDTH;

            break;
    } // End Switch
}

void invertDisplay(tST7735RDisplayInstance *psDispInst, bool i)
{
    writecommand(psDispInst, i ? ST7735_INVON : ST7735_INVOFF);
}

