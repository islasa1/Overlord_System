

//*****************************************************************************
//
// st7735rb128x128x18.c - Display driver for the Adafruit ST7735RR TFT LCD
//                        display. This version uses an SSI interface to the
//                        display controller. The 128x128 TFT display only
//                        uses the green tab
//
// Anthony Islas
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup ST7735 Display Driver
//! @{
//
//*****************************************************************************

#include "ST7735R128x128x18.h"

//*****************************************************************************
//
// Device specific defines for 128x128 Green Tab
//
//*****************************************************************************
#define DISPLAY_COL_START 1
#define DISPLAY_ROW_START 2
#define DISPLAY_MAX_X     128
#define DISPLAY_MAX_Y     128

//*****************************************************************************
//
// Device specific defines depending on tab color
//
//*****************************************************************************
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

// Color modes
#define ST7735_12BIT     0x03
#define ST7735_16BIT     0x05
#define ST7735_18BIT     0x06

// Based on color mode - refer to packing
#define ST7735_BYTES_PER_PIXEL  (3)


//*****************************************************************************
//
// Defines the SSI and GPIO peripherals that are used for this display.
//
//*****************************************************************************
#define DISPLAY_SSI_PERIPH          SYSCTL_PERIPH_SSI0
#define DISPLAY_SSI_GPIO_PERIPH     SYSCTL_PERIPH_GPIOA
#define DISPLAY_RST_GPIO_PERIPH     SYSCTL_PERIPH_GPIOD
#define DISPLAY_PWR_GPIO_PERIPH     SYSCTL_PERIPH_GPIOC

//*****************************************************************************
//
// Defines the GPIO pin configuration macros for the pins that are used for
// the SSI function.
//
//*****************************************************************************
#define DISPLAY_PINCFG_SSICLK       GPIO_PA2_SSI0CLK
#define DISPLAY_PINCFG_SSIFSS       GPIO_PA3_SSI0FSS
#define DISPLAY_PINCFG_SSITX        GPIO_PA5_SSI0TX

//*****************************************************************************
//
// Defines the port and pins for the SSI peripheral.
//
//*****************************************************************************
#define DISPLAY_SSI_PORT            GPIO_PORTA_BASE
#define DISPLAY_SSI_PINS            (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5)

//*****************************************************************************
//
// Defines the port and pins for the display voltage enable signal.
//
//*****************************************************************************
#define DISPLAY_ENV_PORT            GPIO_PORTC_BASE
#define DISPLAY_ENV_PIN             GPIO_PIN_4

//*****************************************************************************
//
// Defines the port and pins for the display reset signal.
//
//*****************************************************************************
#define DISPLAY_RST_PORT            GPIO_PORTD_BASE
#define DISPLAY_RST_PIN             GPIO_PIN_7

//*****************************************************************************
//
// Defines the port and pins for the display Data/Command (D/C) signal.
//
//*****************************************************************************
#define DISPLAY_D_C_PORT            GPIO_PORTA_BASE
#define DISPLAY_D_C_PIN             GPIO_PIN_4

//*****************************************************************************
//
// Defines the SSI peripheral used and the data speed.
//
//*****************************************************************************
#define DISPLAY_SSI_BASE            SSI0_BASE // SSI2
#define DISPLAY_SSI_CLOCK           8000000

// Delays for command set
#define DELAY 0x80

//*****************************************************************************
//
// An array that holds a set of commands that are sent to the display when
// it is initialized.
//
//*****************************************************************************
static uint8_t g_ui8DisplayInitCommands0[] =
{
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
      ST7735_18BIT
};

static uint8_t g_ui8DisplayInitCommands1[] =
{
    // ST7753R 128x128 uses only green tab
                              // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F              //     XEND = 127

};
static uint8_t g_ui8DisplayInitCommands2[] =
{
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
      100                    //     100 ms delay
};
#define NUM_INIT_BYTES sizeof(g_ui8DisplayInitCommands)

//*****************************************************************************
//
//! Translates a 24-bit RGB color to a display driver-specific color.
//
//! \param c is the 24-bit RGB color.  The least-significant byte is the blue
//! channel, the next byte is the green channel, and the third byte is the red
//! channel.
//!
//! This macro translates a 24-bit RGB color into a value that can be written
//! into the display's frame buffer in order to reproduce that color, or the
//! closest possible approximation of that color.
//!
//! \return Returns the display-driver specific color.
//!
//! 24-bit format: XXXX XXXX RRRR RRRR GGGG GGGG BBBB BBBB\n
//! 18-bit format: ---- ---- ---- --RR RRRR GGGG GGBB BBBB\n
//! 16-bit format: ---- ---- ---- ---- RRRR RGGG GGGB BBBB\n
//!  8-bit format: ---- ---- ---- ---- ---- ---- RRRG GGBB\n
//
//
//*****************************************************************************
#define DPYCOLORTRANSLATE18(c)  ((((c) & 0x00fc0000) >> 6)  |                 \
                                 (((c) & 0x0000fc00) >> 4)  |                 \
                                 (((c) & 0x000000fc) >> 2))
#define DPYCOLORTRANSLATE16(c)  ((((c) & 0x00f80000) >> 8)  |                 \
                                 (((c) & 0x0000fc00) >> 5)  |                 \
                                 (((c) & 0x000000f8) >> 3))
#define DPYCOLORTRANSLATE8(c)   ((((c) & 0x00e00000) >> 16) |                 \
                                 (((c) & 0x0000e000) >> 11) |                 \
                                 (((c) & 0x000000c0) >> 6))
#define DPYCOLORTRANSLATE DPYCOLORTRANSLATE18

//
// Internal Use
//
#define ST7735R18BitColorPack(c) (((c & 0x3f000) << 14)  | \
                                  ((c & 0x00fc0) << 12)  | \
                                  ((c & 0x0003f) << 10)) & 0xfcfcfc00


//*****************************************************************************
//
//! Write a set of command bytes to the display controller.
//
//! \param pi8Cmd is a pointer to a set of command bytes.
//! \param ui32Count is the count of command bytes.
//!
//! This function provides a way to send multiple command bytes to the display
//! controller.  It can be used for single commands, or multiple commands
//! chained together in a buffer.  It will wait for any previous operation to
//! finish, and then copy all the command bytes to the controller.  It will
//! not return until the last command byte has been written to the SSI FIFO,
//! but data could still be shifting out to the display controller when this
//! function returns.
//!
//! \return None.
//
//*****************************************************************************
static void
ST7735R128x128x18WriteCommand(const uint8_t *pi8Cmd, uint32_t ui32Count)
{
    //
    // Wait for any previous SSI operation to finish.
    //
    while(MAP_SSIBusy(DISPLAY_SSI_BASE));

    //
    // Set the D/C pin low to indicate command
    //
    MAP_GPIOPinWrite(DISPLAY_D_C_PORT, DISPLAY_D_C_PIN, 0);

    //
    // Send all the command bytes to the display
    //
    while(ui32Count--)
    {
        MAP_SSIDataPut(DISPLAY_SSI_BASE, *pi8Cmd);
        pi8Cmd++;
    }
}

//*****************************************************************************
//
//! Write a set of data bytes to the display controller.
//
//! \param pi8Data is a pointer to a set of data bytes, containing pixel data.
//! \param ui32Count is the count of command bytes.
//!
//! This function provides a way to send a set of pixel data to the display.
//! The data will draw pixels according to whatever the most recent col, row
//! settings are for the display.  It will wait for any previous operation to
//! finish, and then copy all the data bytes to the controller.  It will
//! not return until the last data byte has been written to the SSI FIFO,
//! but data could still be shifting out to the display controller when this
//! function returns.
//!
//! \return None.
//
//*****************************************************************************
static void
ST7735R128x128x18WriteData(const uint8_t *pi8Data, uint32_t ui32Count)
{
    //
    // Wait for any previous SSI operation to finish.
    //
    while(MAP_SSIBusy(DISPLAY_SSI_BASE));

    //
    // Set the D/C pin high to indicate data
    //
    MAP_GPIOPinWrite(DISPLAY_D_C_PORT, DISPLAY_D_C_PIN, DISPLAY_D_C_PIN);

    //
    // Send all the data bytes to the display
    //
    while(ui32Count--)
    {
        MAP_SSIDataPut(DISPLAY_SSI_BASE, *pi8Data);
        pi8Data++;
    }
}

static void commandList(const uint8_t *addr)
{
    //
    // Send the initial configuration command bytes to the display
    //
    uint8_t numCommands, numArgs;
    uint16_t ms;

    // Number of commands to follow
    numCommands = *addr;
    addr++;

    while (numCommands--)
    {
        //   Read, issue command
        ST7735R128x128x18WriteCommand(addr, 1);
        addr++;

        //   Number of args to follow
        numArgs = *addr;
        addr++;

        //   If hibit set, delay follows args
        ms = numArgs & DELAY;
        //   Mask out delay bit
        numArgs &= ~DELAY;

        while (numArgs--)
        {
            //   Read, issue argument
            ST7735R128x128x18WriteData(addr, 1);
            addr++;
        }


        if (ms)
        {
            // Read post-command delay time (ms)
            ms = *addr;
            addr++;

            if (ms == 255)
                ms = 500;
            // If 255, delay for 500 ms
            MAP_SysCtlDelay(MAP_SysCtlClockGet() / 100000 * ms);
        }
    }
}

static void
ST7735R128x128x18SetAddrWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    uint8_t ui8Cmd[8];
    uint8_t ui8Data[8];

    //
    // Set column
    //
    xStart += DISPLAY_COL_START;
    xEnd += DISPLAY_COL_START;

    ui8Cmd[0] = ST7735_CASET;
    ui8Data[0] = xStart >> 8; // X Start High
    ui8Data[1] = xStart;      // X Start Low
    ui8Data[2] = xEnd >> 8;   // X End High
    ui8Data[3] = xEnd;        // X End Low

    ST7735R128x128x18WriteCommand(ui8Cmd, 1);
    ST7735R128x128x18WriteData(ui8Data, 4);

    //
    // Set row
    //
    yStart += DISPLAY_ROW_START;
    yEnd += DISPLAY_ROW_START;

    ui8Cmd[0] = ST7735_RASET;
    ui8Data[0] = yStart >> 8; // Y Start High
    ui8Data[1] = yStart;      // Y Start Low
    ui8Data[2] = yEnd >> 8;   // Y End High
    ui8Data[3] = yEnd;        // Y End Low

    ST7735R128x128x18WriteCommand(ui8Cmd, 1);
    ST7735R128x128x18WriteData(ui8Data, 4);

    //
    // Write to RAM
    //
    ui8Cmd[0] = ST7735_RAMWR;
    ST7735R128x128x18WriteCommand(ui8Cmd, 1);
}

//*****************************************************************************
//
//! Draws a pixel on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param i32X is the X coordinate of the pixel.
//! \param i32Y is the Y coordinate of the pixel.
//! \param ui32Value is the color of the pixel.
//!
//! This function sets the given pixel to a particular color.  The coordinates
//! of the pixel are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void
ST7735R128x128x18PixelDraw(void *pvDisplayData, int32_t i32X, int32_t i32Y,
                            uint32_t ulValue)
{
    uint32_t ui32PackedColor;

    ST7735R128x128x18SetAddrWindow(i32X, i32Y, i32X + 1, i32Y + 1);

    ui32PackedColor = ST7735R18BitColorPack(DPYCOLORTRANSLATE(ulValue));
    ST7735R128x128x18WriteData((uint8_t*) &ui32PackedColor, 3);
}

//*****************************************************************************
//
//! Draws a horizontal sequence of pixels on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param i32X is the X coordinate of the first pixel.
//! \param i32Y is the Y coordinate of the first pixel.
//! \param i32X0 is sub-pixel offset within the pixel data, which is valid for 1
//! or 4 bit per pixel formats.
//! \param i32Count is the number of pixels to draw.
//! \param i32BPP is the number of bits per pixel; must be 1, 4, or 8 optionally
//! ORed with various flags unused by this driver.
//! \param pui8Data is a pointer to the pixel data.  For 1 and 4 bit per pixel
//! formats, the most significant bit(s) represent the left-most pixel.
//! \param pui8Palette is a pointer to the palette used to draw the pixels.
//!
//! This function draws a horizontal sequence of pixels on the screen, using
//! the supplied palette.  For 1 bit per pixel format, the palette contains
//! pre-translated colors; for 4 and 8 bit per pixel formats, the palette
//! contains 24-bit RGB values that must be translated before being written to
//! the display.
//!
//! \return None.
//
//*****************************************************************************
static void
ST7735R128x128x18PixelDrawMultiple(void *pvDisplayData, int32_t i32X, int32_t i32Y, int32_t i32X0,
                                    int32_t i32Count, int32_t i32BPP,
                                    const uint8_t *pui8Data,
                                    const uint8_t *pui8Palette)
{
    uint32_t ui32Byte;
    uint32_t ui32Color;

    ST7735R128x128x18SetAddrWindow(i32X, i32Y, DISPLAY_MAX_X + 1, DISPLAY_MAX_Y + 1);

    //
    // Determine how to interpret the pixel data based on the number of bits
    // per pixel.
    //
    switch(i32BPP & 0xFF)
    {
        //
        // The pixel data is in 1 bit per pixel format.
        //
        case 1:
        {
            //
            // Loop while there are more pixels to draw.
            //
            while(i32Count)
            {
                //
                // Get the next byte of image data.
                //
                ui32Byte = *pui8Data++;

                //
                // Loop through the pixels in this byte of image data.
                //
                for(; (i32X0 < 8) && i32Count; i32X0++, i32Count--)
                {
                    //
                    // Draw this pixel in the appropriate color.
                    //
                    ui32Color = ST7735R18BitColorPack(((uint32_t *)pui8Palette)[(ui32Byte >> (7 - i32X0)) & 1]); // retrieve already translated color and pack it
                    ST7735R128x128x18WriteData((uint8_t*) &ui32Color, 3);
                }

                //
                // Start at the beginning of the next byte of image data.
                //
                i32X0 = 0;
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // The pixel data is in 4 bit per pixel format.
        //
        case 4:
        {
            //
            // Loop while there are more pixels to draw.  "Duff's device" is
            // used to jump into the middle of the loop if the first nibble of
            // the pixel data should not be used.  Duff's device makes use of
            // the fact that a case statement is legal anywhere within a
            // sub-block of a switch statement.  See
            // http://en.wikipedia.org/wiki/Duff's_device for detailed
            // information about Duff's device.
            //
            switch(i32X0 & 1)
            {
            case 0:
                while(i32Count)
                {
                    uint32_t ui32Color;

                    //
                    // Get the upper nibble of the next byte of pixel data
                    // and extract the corresponding entry from the
                    // palette.
                    //
                    ui32Byte = (*pui8Data >> 4) * 3;
                    ui32Byte = (*(uint32_t *)(pui8Palette + ui32Byte) & 0x00ffffff);

                    //
                    // Translate this palette entry and write it to the
                    // screen.
                    //
                    ui32Color = ST7735R18BitColorPack(DPYCOLORTRANSLATE(ui32Byte));
                    ST7735R128x128x18WriteData((uint8_t*) &ui32Color, 3);

                    //
                    // Decrement the count of pixels to draw.
                    //
                    i32Count--;

                    //
                    // See if there is another pixel to draw.
                    //
                    if(i32Count)
                    {
            case 1:
                //
                // Get the lower nibble of the next byte of pixel
                // data and extract the corresponding entry from
                // the palette.
                //
                ui32Byte = (*pui8Data++ & 15) * 3;
                ui32Byte = (*(uint32_t *)(pui8Palette + ui32Byte) & 0x00ffffff);

                //
                // Translate this palette entry and write it to the
                // screen.
                //
                ui32Color = ST7735R18BitColorPack(DPYCOLORTRANSLATE(ui32Byte));
                ST7735R128x128x18WriteData((uint8_t*) &ui32Color, 3);

                //
                // Decrement the count of pixels to draw.
                //
                i32Count--;
                    }
                }
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // The pixel data is in 8 bit per pixel format.
        //
        case 8:
        {
            //
            // Loop while there are more pixels to draw.
            //
            while(i32Count--)
            {

                //
                // Get the next byte of pixel data and extract the
                // corresponding entry from the palette.
                //
                ui32Byte = *pui8Data++ * 3;
                ui32Byte = *(uint32_t *)(pui8Palette + ui32Byte) & 0x00ffffff;

                //
                // Translate this palette entry and write it to the screen.
                //
                ui32Color = ST7735R18BitColorPack(DPYCOLORTRANSLATE(ui32Byte));
                ST7735R128x128x18WriteData((uint8_t*) &ui32Color, 3);
            }

            //
            // The image data has been drawn.
            //
            break;
        }
    }
}

//*****************************************************************************
//
//! Draws a horizontal line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param i32X1 is the X coordinate of the start of the line.
//! \param i32X2 is the X coordinate of the end of the line.
//! \param i32Y is the Y coordinate of the line.
//! \param ui32Value is the color of the line.
//!
//! This function draws a horizontal line on the display.  The coordinates of
//! the line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void
ST7735R128x128x18LineDrawH(void *pvDisplayData, int32_t i32X1, int32_t i32X2, int32_t i32Y,
                            uint32_t ui32Value)
{
    uint8_t ui8LineBuf[16 * ST7735_BYTES_PER_PIXEL];
    unsigned int uIdx;
    uint32_t ui32PackedColor =  ST7735R18BitColorPack(ui32Value);

    ST7735R128x128x18SetAddrWindow(i32X1 < i32X2 ? i32X1 : i32X2, i32Y, DISPLAY_MAX_X + 1, i32Y + 1);

    //
    // Use buffer of pixels to draw line, so multiple bytes can be sent at
    // one time.  Fill the buffer with the line color.
    //
    for(uIdx = 0; uIdx < sizeof(ui8LineBuf); uIdx += ST7735_BYTES_PER_PIXEL)
    {
        ui8LineBuf[uIdx] = (ui32PackedColor >> 24) & 0x000000ff;
        ui8LineBuf[uIdx + 1] = (ui32PackedColor >> 16) & 0x000000ff;
        ui8LineBuf[uIdx + 2] = (ui32PackedColor >> 8) & 0x000000ff;
    }

    uIdx = (i32X1 < i32X2) ? (i32X2 - i32X1) : (i32X1 - i32X2);
    uIdx += 1;
    uIdx *= 3;
    // uIdx is now number of bytes instead of number of pixels
    // Send at 12 bytes at a time (3 pixels)
    while(uIdx)
    {
        ST7735R128x128x18WriteData(ui8LineBuf, (uIdx < 12) ? uIdx : 12);
        uIdx -= (uIdx < 12) ? uIdx : 12;
    }
}

//*****************************************************************************
//
//! Draws a vertical line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param i32X is the X coordinate of the line.
//! \param i32Y1 is the Y coordinate of the start of the line.
//! \param i32Y2 is the Y coordinate of the end of the line.
//! \param ui32Value is the color of the line.
//!
//! This function draws a vertical line on the display.  The coordinates of the
//! line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void
ST7735R128x128x18LineDrawV(void *pvDisplayData, int32_t i32X, int32_t i32Y1, int32_t i32Y2,
                            uint32_t ui32Value)
{
    uint8_t ui8LineBuf[16 * ST7735_BYTES_PER_PIXEL];
    unsigned int uIdx;
    uint32_t ui32PackedColor =  ST7735R18BitColorPack(ui32Value);

    ST7735R128x128x18SetAddrWindow(i32X, i32Y1 < i32Y2 ? i32Y1 : i32Y2, i32X + 1, DISPLAY_MAX_Y);

    //
    // Use buffer of pixels to draw line, so multiple bytes can be sent at
    // one time.  Fill the buffer with the line color.
    //
    for(uIdx = 0; uIdx < sizeof(ui8LineBuf); uIdx += ST7735_BYTES_PER_PIXEL)
    {
        ui8LineBuf[uIdx] = (ui32PackedColor >> 24) & 0x000000ff;
        ui8LineBuf[uIdx + 1] = (ui32PackedColor >> 16) & 0x000000ff;
        ui8LineBuf[uIdx + 2] = (ui32PackedColor >> 8) & 0x000000ff;
    }

    uIdx = (i32Y1 < i32Y2) ? (i32Y2 - i32Y1) : (i32Y1 - i32Y2);
    uIdx += 1;
    uIdx *= 3;
    // uIdx is now number of bytes instead of number of pixels
    // Send at 12 bytes at a time (3 pixels)
    while(uIdx)
    {
        ST7735R128x128x18WriteData(ui8LineBuf, (uIdx < 12) ? uIdx : 12);
        uIdx -= (uIdx < 12) ? uIdx : 12;
    }
}

//*****************************************************************************
//
//! Fills a rectangle.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param pRect is a pointer to the structure describing the rectangle.
//! \param ui32Value is the color of the rectangle.
//!
//! This function fills a rectangle on the display.  The coordinates of the
//! rectangle are assumed to be within the extents of the display, and the
//! rectangle specification is fully inclusive (in other words, both i16XMin and
//! i16XMax are drawn, aint32_t with i16YMin and i16YMax).
//!
//! \return None.
//
//*****************************************************************************
static void
ST7735R128x128x18RectFill(void *pvDisplayData, const tRectangle *pRect,
                           uint32_t ui32Value)
{
    unsigned int uY;

    for(uY = pRect->i16YMin; uY <= pRect->i16YMax; uY++)
    {
        ST7735R128x128x18LineDrawH(0, pRect->i16XMin, pRect->i16XMax, uY, ui32Value);
    }
}

//*****************************************************************************
//
//! Translates a 24-bit RGB color to a display driver-specific color.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param ui32Value is the 24-bit RGB color.  The least-significant byte is the
//! blue channel, the next byte is the green channel, and the third byte is the
//! red channel.
//!
//! This function translates a 24-bit RGB color into a value that can be
//! written into the display's frame buffer in order to reproduce that color,
//! or the closest possible approximation of that color.
//!
//! \return Returns the display-driver specific color.
//
//*****************************************************************************
static uint32_t
ST7735R128x128x18ColorTranslate(void *pvDisplayData, uint32_t ui32Value)
{
    //
    // Translate from a 24-bit RGB color to a 6-6-6 RGB color.
    //
    return(DPYCOLORTRANSLATE(ui32Value));
}

//*****************************************************************************
//
//! Flushes any cached drawing operations.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//!
//! This functions flushes any cached drawing operations to the display.  This
//! is useful when a local frame buffer is used for drawing operations, and the
//! flush would copy the local frame buffer to the display.  Since no memory
//! based frame buffer is used for this driver, the flush is a no operation.
//!
//! \return None.
//
//*****************************************************************************
static void
ST7735R128x128x18Flush(void *pvDisplayData)
{
    //
    // There is nothing to be done.
    //
}

//*****************************************************************************
//
//! The display structure that describes the driver for the Crystalfontz
//! CFAL9664-F-B1 OLED panel with SSD 1332 controller.
//
//*****************************************************************************
const tDisplay g_sST7735R128x128x18 =
{
    sizeof(tDisplay),
    0,
    128,
    128,
    ST7735R128x128x18PixelDraw,
    ST7735R128x128x18PixelDrawMultiple,
    ST7735R128x128x18LineDrawH,
    ST7735R128x128x18LineDrawV,
    ST7735R128x128x18RectFill,
    ST7735R128x128x18ColorTranslate,
    ST7735R128x128x18Flush
};

//*****************************************************************************
//
//! Initializes the display driver.
//!
//! This function initializes the ST7735 display, preparing it to display data.
//!
//! \return None.
//
//*****************************************************************************
void
ST7735R128x128x18PeriphInit(void)
{
    //
    // Enable the peripherals used by this driver
    //
    MAP_SysCtlPeripheralEnable(DISPLAY_SSI_PERIPH);
    MAP_SysCtlPeripheralEnable(DISPLAY_SSI_GPIO_PERIPH);
    MAP_SysCtlPeripheralEnable(DISPLAY_RST_GPIO_PERIPH);
    MAP_SysCtlPeripheralEnable(DISPLAY_PWR_GPIO_PERIPH);

    //
    // Select the SSI function for the appropriate pins
    //
    MAP_GPIOPinConfigure(DISPLAY_PINCFG_SSICLK);
    MAP_GPIOPinConfigure(DISPLAY_PINCFG_SSIFSS);
    MAP_GPIOPinConfigure(DISPLAY_PINCFG_SSITX);


    //
    // Configure the pins for the SSI function
    //
    MAP_GPIOPinTypeSSI(DISPLAY_SSI_PORT, DISPLAY_SSI_PINS);

    //
    // Configure display control pins as GPIO output
    //
    MAP_GPIOPinTypeGPIOOutput(DISPLAY_RST_PORT, DISPLAY_RST_PIN);
    MAP_GPIOPinTypeGPIOOutput(DISPLAY_ENV_PORT, DISPLAY_ENV_PIN);
    MAP_GPIOPinTypeGPIOOutput(DISPLAY_D_C_PORT, DISPLAY_D_C_PIN);

    //
    // Reset pin high, power off
    //
    MAP_GPIOPinWrite(DISPLAY_RST_PORT, DISPLAY_RST_PIN, DISPLAY_RST_PIN);
    MAP_GPIOPinWrite(DISPLAY_ENV_PORT, DISPLAY_ENV_PIN, 0);
    MAP_SysCtlDelay(1000);

    //
    // Drive the reset pin low while we do other stuff
    //
    MAP_GPIOPinWrite(DISPLAY_RST_PORT, DISPLAY_RST_PIN, 0);

    //
    // Configure the SSI port
    //
    MAP_SSIDisable(DISPLAY_SSI_BASE);
    MAP_SSIConfigSetExpClk(DISPLAY_SSI_BASE, MAP_SysCtlClockGet(),
                           SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER,
                           DISPLAY_SSI_CLOCK, 8);
    MAP_SSIEnable(DISPLAY_SSI_BASE);

    //
    // Take the display out of reset
    //
    MAP_SysCtlDelay(1000);
    MAP_GPIOPinWrite(DISPLAY_RST_PORT, DISPLAY_RST_PIN, DISPLAY_RST_PIN);
    MAP_SysCtlDelay(1000);

    //
    // Enable display power supply
    //
    MAP_GPIOPinWrite(DISPLAY_ENV_PORT, DISPLAY_ENV_PIN, DISPLAY_ENV_PIN);
    MAP_SysCtlDelay(1000);


    MAP_SysCtlDelay(1000);

}

//*****************************************************************************
//
//! Initializes the software for the display driver.
//!
//! This function initializes the ST7735 display, preparing it to display data.
//!
//! \return None.
//
//*****************************************************************************
void
ST7735R128x128x18SWInit(void)
{
    tRectangle sRect;

    commandList(g_ui8DisplayInitCommands0);
    commandList(g_ui8DisplayInitCommands1);
    commandList(g_ui8DisplayInitCommands2);


    //
    // Fill the entire display with a black rectangle, to clear it.
    //
    sRect.i16XMin = 0;
    sRect.i16XMax = DISPLAY_MAX_X;
    sRect.i16YMin = 0;
    sRect.i16YMax = DISPLAY_MAX_Y;
    ST7735R128x128x18RectFill(0, &sRect, ClrBlack);
}

//*****************************************************************************
//
//! Initializes the entire display driver.
//!
//! This function initializes the ST7735 display, preparing it to display data.
//!
//! \return None.
//
//*****************************************************************************
void
ST7735R128x128x18Init(void)
{
    ST7735R128x128x18PeriphInit();
    ST7735R128x128x18SWInit();
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

