#include "st7735.h"

#include <math.h>
#include <stdlib.h>


/*---------------------------------------------------------------------------------------------------------*/

// Initialisation commands
// Based on [https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7735.cpp#L51]

#define DELAY 0x80

static const u8
  initialisation_commands1[] =                                      // Initialisation part 1
  {
    15,                                                             // 15 commands in list:

    ST7735_SWRESET, DELAY,                                          //  1: Software reset, 0 args, w/delay
      150,                                                          //     150 ms delay

    ST7735_SLPOUT, DELAY,                                           //  2: Out of sleep mode, 0 args, w/delay
      255,                                                          //     500 ms delay

    ST7735_FRMCTR1, 3,                                              //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,                                             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)

    ST7735_FRMCTR2, 3,                                              //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,                                             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)

    ST7735_FRMCTR3, 6,                                              //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,                                             //     Dot inversion mode
      0x01, 0x2C, 0x2D,                                             //     Line inversion mode

    ST7735_INVCTR, 1,                                               //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                                                         //     No inversion

    ST7735_PWCTR1, 3,                                               //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                                                         //     -4.6V
      0x84,                                                         //     AUTO mode

    ST7735_PWCTR2, 1,                                               //  8: Power control, 1 arg, no delay:
      0xC5,                                                         //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD

    ST7735_PWCTR3, 2,                                               //  9: Power control, 2 args, no delay:
      0x0A,                                                         //     Opamp current small

      0x00,                                                         //     Boost frequency
    ST7735_PWCTR4, 2,                                               // 10: Power control, 2 args, no delay:
      0x8A,                                                         //     BCLK/2, Opamp current small & Medium low
      0x2A,

    ST7735_PWCTR5, 2,                                               // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,

    ST7735_VMCTR1, 1,                                               // 12: Power control, 1 arg, no delay:
      0x0E,

    ST7735_INVOFF, 0,                                               // 13: Don't invert display, no args, no delay

    ST7735_MADCTL, 1,                                               // 14: Memory access control (directions), 1 arg:
      ST7735_ROTATION,                                              //     row addr/col addr, bottom to top refresh

    ST7735_COLMOD, 1,                                               // 15: set color mode, 1 arg, no delay:
      0x05                                                          //     16-bit color
  },


  initialisation_commands2[] =                                      // Initialisation part 2
  {
    2,                                                              //  2 commands in list:

    ST7735_CASET, 4,                                                //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                                                   //     XSTART = 0
      0x00, 0x7F,                                                   //     XEND = 127

    ST7735_RASET, 4,                                                //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                                                   //     XSTART = 0
      0x00, 0x9F                                                    //     XEND = 159
  },


  initialisation_commands3[] =                                      // Initialisation part 3
  {
    4,                                                              //  4 commands in list:

    ST7735_GMCTRP1, 16,                                             //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,

    ST7735_GMCTRN1, 16,                                              //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,

    ST7735_NORON, DELAY,                                            //  3: Normal display on, no args, w/delay
      10,                                                           //     10 ms delay

    ST7735_DISPON, DELAY,                                           //  4: Main screen turn on, no args w/delay
      100
  };

/*---------------------------------------------------------------------------------------------------------*/
//''
static void st7735WriteData(u8* buffer, size_t buffer_size)
{
    HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, buffer, buffer_size, HAL_MAX_DELAY);
}

/*---------------------------------------------------------------------------------------------------------*/

static void st7735WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

/*---------------------------------------------------------------------------------------------------------*/

// Based on [https://github.com/afiskon/stm32-st7735/blob/master/st7735/st7735.c#L110]

static void st7735ExecuteCommand(const u8* address)
{
    u8 commands_num, argument_num;
    u16 ms;

    commands_num = *address++;
    while (commands_num--)
    {
        u8 command = *address++;
        st7735WriteCommand(command);

        argument_num = *address++;
        // If high bit set, delay follows args
        ms = argument_num & DELAY;
        argument_num &= ~DELAY;
        if (argument_num)
        {
            st7735WriteData((u8*)address, argument_num);
            address += argument_num;
        }

        if (ms)
        {
            ms = *address++;
            if (ms == 255)
            {
                ms = 500;
            }
            HAL_Delay(ms);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/

static void st7735SetAddressWindow(u8 x0, u8 y0, u8 x1, u8 y1)
{
    // Column address set
    st7735WriteCommand(ST7735_CASET);
    u8 data[] = { 0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART };
    st7735WriteData(data, sizeof(data));

    // Row address set
    st7735WriteCommand(ST7735_RASET);
    data[1] = y0 + ST7735_YSTART;
    data[3] = y1 + ST7735_YSTART;
    st7735WriteData(data, sizeof(data));

    // Write to RAM
    st7735WriteCommand(ST7735_RAMWR);
}

/*---------------------------------------------------------------------------------------------------------*/

u64 st7735GCD(u64 a, u64 b)
{
    if (b)
    {
        while ((a %= b) && (b %= a));
    }

    return (a + b);
}

u64 st7735LCM(u64 a, u64 b)
{
    return (a * b) / st7735GCD(a, b);
}

s64 st7735Max(s64 a, s64 b)
{
    return (a > b ) ? a : b;
}

s64 st7735Min(s64 a, s64 b)
{
    return (a > b ) ? b : a;
}

/*---------------------------------------------------------------------------------------------------------*/

void st7735Initialisation()
{
    //
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_RESET);

    // RESX signal will reset the device and it must be applied to properly initialise the chip.
    HAL_GPIO_WritePin(ST7735_RESET_GPIO_Port, ST7735_RESET_GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(ST7735_RESET_GPIO_Port, ST7735_RESET_GPIO_Pin, GPIO_PIN_SET);

    // Execute commands to initialise LCD
    st7735ExecuteCommand(initialisation_commands1);
    st7735ExecuteCommand(initialisation_commands2);
    st7735ExecuteCommand(initialisation_commands3);

    // The serial interface is initialised when CSX is high
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_SET);
}

/*---------------------------------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735DrawPixel(u8 x, u8 y, u16 colour) {
    // Terminate function if position is out of bounds
    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
    {
        return ST7735_ERROR;
    }

    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_RESET);

    u8 data[] = { colour >> 8, colour & 0xFF };

    // Draw pixel
    st7735SetAddressWindow(x, y, x + 1, y + 1);
    st7735WriteData(data, sizeof(data));

    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_SET);

    return ST7735_OK;
}

/*---------------------------------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735DrawHorizontalLine(u8 x0, u8 x1, u8 y, u16 colour)
{
    // Terminate function if position is out of bounds
    if ((x0 >= ST7735_WIDTH) || (x1 >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
    {
        return ST7735_ERROR;
    }

    if (x0 < x1)
    {
        while (x0 < x1)
        {
            st7735DrawPixel(x0, y, colour);
            x0++;
        }
    }
    else if (x1 < x0)
    {
        while (x1 < x0)
        {
            st7735DrawPixel(x1, y, colour);
            x1++;
        }
    }

    return ST7735_OK;
}

/*---------------------------------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735DrawVerticalLine(u8 y0, u8 y1, u8 x, u16 colour)
{
    // Terminate function if position is out of bounds
    if ((y0 >= ST7735_WIDTH) || (y1 >= ST7735_WIDTH) || (x >= ST7735_HEIGHT))
    {
        return ST7735_ERROR;
    }

    if (y0 < y1)
    {
        while (y0 < y1)
        {
            st7735DrawPixel(x, y0, colour);
            y0++;
        }
    }
    else if (y1 < y0)
    {
        while (y1 < y0)
        {
            st7735DrawPixel(x, y1, colour);
            y1++;
        }
    }

    return ST7735_OK;
}

/*---------------------------------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735DrawBresenhamLine(u8 x0, u8 y0, u8 x1, u8 y1, u16 colour)
{
    // Terminate function if position is out of bounds
    if ((x0 >= ST7735_WIDTH)|| (y0 >= ST7735_HEIGHT) || (x1 >= ST7735_WIDTH)|| (y1 >= ST7735_HEIGHT))
    {
        return ST7735_ERROR;
    }

    // Implementation of Breseham's lien algorithm for all 8 octets

    int dx = x1 - x0;
    // if x0 == x1, then it does not matter what we set here
    int ix = ((dx > 0) - (dx < 0));

    dx = abs(dx) << 1;

    int dy = y1 - y0;
    // if y0 == y1, then it does not matter what we set here
    int iy = ((dy > 0) - (dy < 0));
    dy = abs(dy) << 1;

    // Draw pixel
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_RESET);

    u8 data[] = { colour >> 8, colour & 0xFF };
    st7735SetAddressWindow(x0, y0, x0, y0);
    HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_SET);

    HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);

    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_SET);

    if (dx >= dy)
    {
        // Error may go below zero
        int error = (dy - (dx >> 1));

        while (x0 != x1)
        {
            if ((error >= 0) && (error || (ix > 0)))
            {
                error -= dx;
                y0 += iy;
            }

            error += dy;
            x0 += ix;

            // Draw pixel
            HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_RESET);

            u8 data[] = { colour >> 8, colour & 0xFF };
            st7735SetAddressWindow(x0, y0, x0, y0);
            HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_SET);

            HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);

            HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_SET);
        }
    }
    else
    {
        // Error may go below zero
        int error = (dx - (dy >> 1));

        while (y0 != y1)
        {
            if ((error >= 0) && (error || (iy > 0)))
            {
                error -= dy;
                x0 += ix;
            }

            error += dx;
            y0 += iy;

            // Draw pixel
            HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_RESET);

            u8 data[] = { colour >> 8, colour & 0xFF };
            st7735SetAddressWindow(x0, y0, x0, y0);
            HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_SET);

            HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);

            HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_SET);
        }
    }

    return ST7735_OK;
}

/*---------------------------------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735Rasterise(u8 vertices[3][2], u16 colour)
{
    u8 third_var;

    if (vertices[0][1] > vertices[2][1])
    {
        third_var = vertices[0][1];
        vertices[0][1] = vertices[2][1];
        vertices[2][1] = third_var;
    }

    if (vertices[0][1] > vertices[1][1])
    {
        third_var = vertices[0][1];
        vertices[0][1] = vertices[1][1];
        vertices[1][1] = third_var;
    }
    if (vertices[1][1] > vertices[2][1])
    {
        third_var = vertices[1][1];
        vertices[1][1] = vertices[2][1];
        vertices[2][1] = third_var;
    }

    //
    u8 vertex_one[2] = { vertices[2][0], vertices[2][1] }, vertex_two[2] = { vertices[1][0], vertices[1][1] }, vertex_three[2] = { vertices[0][0], vertices[0][1] };

    //
    f32 line_one[2][2] = { { vertex_two[0], vertex_two[1] }, { vertex_one[0], vertex_one[1] } };

    f32 a_one = (line_one[0][1] - line_one[1][1]) / (line_one[0][0] - line_one[1][0]);
    f32 b_one = line_one[0][1] - a_one * line_one[0][0];

    //
    f32 line_two[2][2] = { { vertex_three[0], vertex_three[1] }, { vertex_one[0], vertex_one[1] } };

    f32 a_two = (line_two[0][1] - line_two[1][1]) / (line_two[0][0] - line_two[1][0]);
    f32 b_two = line_two[0][1] - a_two * line_two[0][0];

    //
    for (u8 y = vertex_one[1]; y > (vertex_two[1] > vertex_three[1] ? vertex_two[1] : vertex_three[1]); y--)
    {
        st7735DrawHorizontalLine((y - b_one) / a_one, (y - b_two) / a_two, y, colour);
    }

    //
    if (vertex_two[1] != vertex_three[1])
    {
        if (vertex_two[1] > vertex_three[1])
        {
            f32 line_three[2][2] = { { vertex_three[0], vertex_three[1] }, { vertex_two[0], vertex_two[1] } };

            f32 a_three = (line_three[0][1] - line_three[1][1]) / (line_three[0][0] - line_three[1][0]);
            f32 b_three = line_three[0][1] - a_three * line_three[0][0];

            for (u8 y = vertex_two[1]; y > vertex_three[1]; y--)
            {
                st7735DrawHorizontalLine((y - b_two) / a_two, (y - b_three) / a_three, y, colour);
            }
        }
        else
        {
            f32 line_three[2][2] = { { vertex_two[0], vertex_two[1] }, { vertex_three[0], vertex_three[1] } };

            f32 a_three = (line_three[0][1] - line_three[1][1]) / (line_three[0][0] - line_three[1][0]);
            f32 b_three = line_three[0][1] - a_three * line_three[0][0];

            for (u8 y = vertex_three[1]; y > vertex_two[1]; y--)
            {
                st7735DrawHorizontalLine((y - b_two) / a_two, (y - b_three) / a_three, y, colour);
            }
        }
    }

    return ST7735_OK;
}

/*---------------------------------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735DrawConvexPolygon(u8 x, u8 y, u16 points, f32 x_scale, f32 y_scale, f32 rotation, u16 boundary_colour, u16 fill_colour, ST7735_MODE_TYPE mode)
{
    // Convert normalised parameters into default values
    x_scale = round(x_scale * ST7735_WIDTH);
    y_scale = round(y_scale * ST7735_HEIGHT);

    // Terminate function if position is out of bounds
    if ((x >= ST7735_WIDTH)|| (y >= ST7735_HEIGHT))
    {
        return ST7735_ERROR;
    }

    // Initialise polygon POD
    struct Polygon polygon = { .mode = mode, .boundary_colour = boundary_colour, .fill_colour = fill_colour };

    // Fill 2d array of point coordinates using [https://www.desmos.com/calculator/intzj6i7y7]
    u8 vertices[points][2];

    // Set vertex coordinates
    for (u8 point = 0; (point < points); point++)
    {
        // x
        u8 x_coordinate = round(x + (x_scale * cos(((2 * M_PI * point) / points) + ((2 * M_PI * rotation) / 360) + (M_PI / 4))));
        if (x_coordinate >= 0 && x_coordinate <= ST7735_WIDTH)
        {
            vertices[point][0] = x_coordinate;
        }
        else
        {
            return ST7735_ERROR;
        }

        // y
        u8 y_coordinate = round(y + (y_scale * sin(((2 * M_PI * point) / points) + ((2 * M_PI * rotation) / 360) + (M_PI / 4))));;
        if (y_coordinate >= 0 && y_coordinate <= ST7735_HEIGHT)
        {
            vertices[point][1] = y_coordinate;
        }
        else
        {
            return ST7735_ERROR;
        }
    }

    // Fill

    st7735Rasterise(vertices, fill_colour);


    // Join points (do after filling so it goes a layer above)
    for (u8 i = 0; i < points; i++)
    {
        if (i != points - 1)
        {
            // Draw line from one point to another
            st7735DrawBresenhamLine(vertices[i][0], vertices[i][1], vertices[i + 1][0], vertices[i + 1][1], boundary_colour);
        }
        else
        {
            // Draw line to first point on last index of array
            st7735DrawBresenhamLine(vertices[i][0], vertices[i][1], vertices[0][0], vertices[0][1], boundary_colour);
        }
    }

    return ST7735_OK;
}

/*---------------------------------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735DrawRectangle(f32 x, f32 y, u8 width, u8 height, u16 colour, ST7735_MODE_TYPE mode)
{
    // Convert normalised coordinates into pixel appropriate coordinates
    x = round(x * ST7735_WIDTH);
    y = round(y * ST7735_HEIGHT);

    // Terminate function if positions/size is out of bounds
    if ((x >= ST7735_WIDTH) || (width > ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (height > ST7735_HEIGHT))
    {
        return ST7735_ERROR;
    }

    // Clipping
    if ((x + width - 1) >= ST7735_WIDTH)
    {
        width = ST7735_WIDTH - x;
    }
    if ((y + height - 1) >= ST7735_HEIGHT)
    {
        height = ST7735_HEIGHT - y;
    }

    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_RESET);

    u8 data[] = { colour >> 8, colour & 0xFF };

    // Draw rectangle according to mode
    if (mode == ST7735_LINE)
    {
        // Top line
        st7735SetAddressWindow(x, y, x + width - 1, y);
        HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_SET);

        for (u16 i = x; i < x + width; i++)
        {
            HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }

        // Bottom line
        st7735SetAddressWindow(x, y + height - 1, x + width - 1, y + height - 1);
        HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_SET);

        for (u16 i = x; i < x + width; i++)
        {
            HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }

        // Left line
        st7735SetAddressWindow(x, y, x, y + height - 1);
        HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_SET);

        for (u16 i = y; i < y + height; i++)
        {
            HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }

        // Right line
        st7735SetAddressWindow(x + width - 1, y, x + width - 1, y + height - 1);
        HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_SET);

        for (u16 i = y; i < y + height; i++)
        {
            HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }
    }
    else if (mode == ST7735_FILL)
    {
        st7735SetAddressWindow(x, y, x + width - 1, y + height - 1);
        HAL_GPIO_WritePin(ST7735_A0_GPIO_Port, ST7735_A0_GPIO_Pin, GPIO_PIN_SET);

        for (u16 i = 0; i < width * height; i++)
        {
            HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }
    }

    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_GPIO_Pin, GPIO_PIN_SET);

    return ST7735_OK;
}

/*---------------------------------------------------------------------------------------------------------*/

ST7735_STATUS_TYPE st7735FillScreen(u16 colour) {
    if (st7735DrawRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, colour, ST7735_FILL) == ST7735_ERROR)
    {
        return ST7735_ERROR;
    }
    return ST7735_OK;
}

/*---------------------------------------------------------------------------------------------------------*/

