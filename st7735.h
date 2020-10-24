#include <stdbool.h>

#include "stm32f4xx_hal.h"

#include "../Inc/primitive_types.h"


/*---------------------------------------------------------------------------------------------------------*/

// Pins & Ports [Redefine if necessary]


#define ST7735_SPI_PORT hspi1
extern SPI_HandleTypeDef ST7735_SPI_PORT;

#define ST7735_RESET_GPIO_Pin        GPIO_PIN_11
#define ST7735_RESET_GPIO_Port       GPIOD

#define ST7735_CS_GPIO_Pin           GPIO_PIN_4
#define ST7735_CS_GPIO_Port          GPIOA

#define ST7735_A0_GPIO_Pin           GPIO_PIN_10
#define ST7735_A0_GPIO_Port          GPIOD

/*---------------------------------------------------------------------------------------------------------*/

//


#define ST7735_MADCTL_MY             0x80
#define ST7735_MADCTL_MX             0x40
#define ST7735_MADCTL_MV             0x20
#define ST7735_MADCTL_ML             0x10
#define ST7735_MADCTL_RGB            0x00
#define ST7735_MADCTL_BGR            0x08
#define ST7735_MADCTL_MH             0x04

/*---------------------------------------------------------------------------------------------------------*/

// LCD Information [Redefine if necessary]


// 160 x 128 | default orientation
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160
#define ST7735_XSTART 0
#define ST7735_YSTART 0
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY)
*/

// 160 x 128 | rotate right

#define ST7735_IS_160X128 1
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 128
#define ST7735_XSTART 0
#define ST7735_YSTART 0
#define ST7735_ROTATION (ST7735_MADCTL_MY | ST7735_MADCTL_MV)


// 160 x 128 | rotate left
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 128
#define ST7735_XSTART 0
#define ST7735_YSTART 0
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MV)
*/

// 160 x 128 | upside down
/*
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160
#define ST7735_XSTART 0
#define ST7735_YSTART 0
#define ST7735_ROTATION (0)
*/

/*---------------------------------------------------------------------------------------------------------*/

// Commands


#define ST7735_NOP                   0x00
#define ST7735_SWRESET               0x01
#define ST7735_RDDID                 0x04
#define ST7735_RDDST                 0x09

#define ST7735_SLPIN                 0x10
#define ST7735_SLPOUT                0x11
#define ST7735_PTLON                 0x12
#define ST7735_NORON                 0x13

#define ST7735_INVOFF                0x20
#define ST7735_INVON                 0x21
#define ST7735_DISPOFF               0x28
#define ST7735_DISPON                0x29
#define ST7735_CASET                 0x2A
#define ST7735_RASET                 0x2B
#define ST7735_RAMWR                 0x2C
#define ST7735_RAMRD                 0x2E

#define ST7735_PTLAR                 0x30
#define ST7735_COLMOD                0x3A
#define ST7735_MADCTL                0x36

#define ST7735_FRMCTR1               0xB1
#define ST7735_FRMCTR2               0xB2
#define ST7735_FRMCTR3               0xB3
#define ST7735_INVCTR                0xB4
#define ST7735_DISSET5               0xB6

#define ST7735_PWCTR1                0xC0
#define ST7735_PWCTR2                0xC1
#define ST7735_PWCTR3                0xC2
#define ST7735_PWCTR4                0xC3
#define ST7735_PWCTR5                0xC4
#define ST7735_VMCTR1                0xC5

#define ST7735_RDID1                 0xDA
#define ST7735_RDID2                 0xDB
#define ST7735_RDID3                 0xDC
#define ST7735_RDID4                 0xDD

#define ST7735_PWCTR6                0xFC

#define ST7735_GMCTRP1               0xE0
#define ST7735_GMCTRN1               0xE1

/*---------------------------------------------------------------------------------------------------------*/

// Colours [RGB565]


// Default colours
#define ST7735_COLOUR_WHITE          0xFFFF
#define ST7735_COLOUR_BLACK          0x0000
#define ST7735_COLOUR_RED            0xF800
#define ST7735_COLOUR_ORANGE         0xFD20
#define ST7735_COLOUR_YELLOW         0xFFE0
#define ST7735_COLOUR_GREEN          0x07E0
#define ST7735_COLOUR_BLUE           0x001F
#define ST7735_COLOUR_PURPLE         0xF81F

// Custom colour
#define ST7735_COLOUR_RGB(r, g, b)   ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3)

/*---------------------------------------------------------------------------------------------------------*/

// Drawing mode


typedef enum
{
    ST7735_LINE                    = 0x0000,
    ST7735_FILL                    = 0x0001
} ST7735_MODE_TYPE;

/*---------------------------------------------------------------------------------------------------------*/

// Status types


typedef enum
{
  ST7735_OK                        = 0x0000,
  ST7735_ERROR                     = 0x0001
} ST7735_STATUS_TYPE;

/*---------------------------------------------------------------------------------------------------------*/

struct Polygon
{
    ST7735_MODE_TYPE mode;

    u16 boundary_colour;
    u16 fill_colour;
};

/*---------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/

u64 st7735GCD(u64 a, u64 b);
u64 st7735LCM(u64 a, u64 b);
s64 st7735Min(s64 a, s64 b);
s64 st7735Max(s64 a, s64 b);

/*--------------------------------------------------------------------------------*/

/*
 * Initialises the LCD.
 *
 * @parameter : void,                                        No parameters.
 *
 * @return    : void,                                        No return value.
 *
*/
void st7735Initialisation();

/*--------------------------------------------------------------------------------*/

/*
 * Draws pixel according to arguments provided
 *
 * @parameter : uint8_t             x,                       x coordinate which signifies the pixels position.
 * @parameter : uint8_t             y,                       y coordinate which signifies the pixels position.
 * @parameter : uint16_t            colour,                  The colour which the given pixel should be.                              [see ST7735_COLOUR_XXX]
 *
 * @return    : ST7735_STATUS_TYPE,                          Returns status type.                                                     [see ST7735_STATUS_TYPE]
 *
*/
ST7735_STATUS_TYPE st7735DrawPixel(u8 x, u8 y, u16 colour);

/*--------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735DrawHorizontalLine(u8 x0, u8 x1, u8 y, u16 colour);

/*--------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735DrawVerticalLine(u8 y0, u8 y1, u8 x, u16 colour);

/*--------------------------------------------------------------------------------*/

/*
 * Draws a line using Bresenham’s line drawing algorithm according to arguments provided.
 *
 * @parameter uint8_t               x0,                      x coordinate for beginning point of line.
 * @parameter uint8_t               x1,                      x coordinate for end point of line.
 * @parameter uint8_t               y0,                      y coordinate for beginning point of line.
 * @parameter uint8_t               y1,                      y coordinate for end point of line.
 * @parameter uint16_t              colour,                  The colour which the given line should be.                               [see ST7735_COLOUR_XXX]
 *
 * @return    : ST7735_STATUS_TYPE,                          Returns status type.                                                     [see ST7735_STATUS_TYPE]
 *
*/
ST7735_STATUS_TYPE st7735DrawBresenhamLine(u8 x0, u8 y0, u8 x1, u8 y1, u16 colour);

/*--------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735Rasterise(u8 vertices[3][2], u16 colour);

/*--------------------------------------------------------------------------------*/

/*
 * Polygon drawing function which draws a convex polygon according to arguments provided.
 *
 * @parameter : float               x,                       x coordinate for centre point.
 * @parameter : float               y,                       y coordinate for centre point.
 * @parameter : uint16_t            points,                  Number of points the given polygon should have.
 * @parameter : float               x_scale,                 Normalised x scale for the given polygon.
 * @parameter : float               y_scale,                 Normalised y scale for the given polygon.
 * @parameter : float               rotation,                Angle between 0-360 to rotate the given polygon.
 * @parameter : uint16_t            boundary_colour,         The colour which the given polygons boundary should be.                  [see ST7735_COLOUR_XXX]
 * @parameter : uint16_t            fill_colour,             The colour which the given polygon should be filled.                     [see ST7735_COLOUR_XXX]
 * @parameter : ST7735_MODE_TYPE    mode,                    The mode which the given polygon should be drawn in.                     [see ST7735_MODE_TYPE]
 *
 * @return    : ST7735_STATUS_TYPE,                          Returns status type.                                                     [see ST7735_STATUS_TYPE]
 *
*/
ST7735_STATUS_TYPE st7735DrawConvexPolygon(u8 x, u8 y, u16 points, f32 x_scale, f32 y_scale, f32 rotation, u16 boundary_colour, u16 fill_colour, ST7735_MODE_TYPE mode);

/*--------------------------------------------------------------------------------*/

/*
 * Draws a rectangle according to arguments provided
 *
 * @parameter : float               x,                       Normalised x coordinate which signifies top right point of rectangle.
 * @parameter : float               y,                       Normalised y coordinate which signifies top right point of rectangle.
 * @parameter : uint8_t             width,                   The width of the given polygon specified in pixels.
 * @parameter : uint8_t             height,                  The height of the given polygon specified in pixels.
 * @parameter : uint16_t            colour,                  The colour which the given rectangle should be.                          [see ST7735_COLOUR_XXX]
 * @parameter : ST7735_MODE_TYPE    mode,                    The mode which the given rectangle should be drawn in.                   [see ST7735_MODE_TYPE]
 *
 * @return    : ST7735_STATUS_TYPE,                          Returns status type.                                                     [see ST7735_STATUS_TYPE]
 *
*/
ST7735_STATUS_TYPE st7735DrawRectangle(f32 x, f32 y, u8 width, u8 height, u16 colour, ST7735_MODE_TYPE mode);

/*--------------------------------------------------------------------------------*/

/*
 * Fill the LCD's screen according to arguments provided.
 *
 * @parameter : uint16_t            colour,                  The colour which the screen should be filled in.                         [see ST7735_COLOUR_XXX]
 *
 * @return    : ST7735_STATUS_TYPE,                          Returns status type.                                                     [see ST7735_STATUS_TYPE]
 *
*/
ST7735_STATUS_TYPE st7735FillScreen(u16 colour);

/*--------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
