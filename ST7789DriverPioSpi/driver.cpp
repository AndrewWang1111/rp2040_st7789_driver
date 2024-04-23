#include "driver.h"
#include <iostream>
using namespace std;

static uint8_t DATA_PORCTRL[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
static uint8_t DATA_PWCTRL1[] = {0xA4, 0xA1};
static uint8_t DATA_PVGAMCTRL[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
static uint8_t DATA_NVGAMCTRL[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};

ST7796::ST7796()
{
    LcdColorDeep = COLOR_DEEP;
}
ST7796::~ST7796() {}

void ST7796::HardwardInit()
{

    LcdPinBkl = PIN_BKL;
    LcdPinDc = PIN_DC;
    LcdPinMiso = PIN_MISO;
    LcdPinMosi = PIN_MOSI;
    LcdPinRst = PIN_RST;
    LcdPinSck = PIN_SCK;
    LcdWidth = LCD_WIDTH;
    lcdHeight = LCD_HEIGHT;

    spi = {.pio = pio0, .sm = 0};
    spi.cs_pin = PIN_CS;
    float clkdiv = 1.0;
    uint cpha0_prog_offs = pio_add_program(spi.pio, &spi_cpha0_program);
    uint cpha1_prog_offs = pio_add_program(spi.pio, &spi_cpha1_program);

    pio_spi_init(spi.pio, spi.sm,
                 cpha1_prog_offs,
                 8, // 8 bits per SPI frame
                 clkdiv,
                 1,
                 1,
                 LcdPinSck,
                 LcdPinMosi,
                 LcdPinMiso);

    gpio_init(LcdPinBkl);
    gpio_init(LcdPinRst);
    gpio_init(LcdPinDc);

    gpio_set_dir(LcdPinBkl, GPIO_OUT);
    gpio_set_dir(LcdPinRst, GPIO_OUT);
    gpio_set_dir(LcdPinDc, GPIO_OUT);

    gpio_put(LcdPinBkl, 0);
    gpio_put(LcdPinDc, 1);
    gpio_put(LcdPinRst, 0);
}

void ST7796::LcdHardReset()
{
    gpio_put(LcdPinRst, 1);
    sleep_ms(150);
    gpio_put(LcdPinRst, 0);
    sleep_ms(150);
    gpio_put(LcdPinRst, 1);
    gpio_put(LcdPinBkl, 1);
}

void ST7796::WriteCommand(uint8_t Comm)
{

    gpio_put(LcdPinDc, 0);
    pio_spi_write8_blocking(&spi, &Comm, 1);
}

void ST7796::WriteData(uint8_t *Data, uint16_t len)
{

    uint8_t buff[2];
    gpio_put(LcdPinDc, 1);
    pio_spi_write8_blocking(&spi, Data, len);
}

void ST7796::LcdInit()
{

    HardwardInit();
    LcdHardReset();
    uint8_t Data[16] = {0};

    Data[0] = 0x35;
    WriteCommand(MADCTL);
    WriteData(Data, 1);

    Data[0] = 0x06;
    WriteCommand(COLMOD);
    WriteData(Data, 1);

    WriteCommand(PORCTRL);
    WriteData(DATA_PORCTRL, sizeof(DATA_PORCTRL));
    WriteCommand(PVGAMCTRL);
    WriteData(DATA_PVGAMCTRL, sizeof(PVGAMCTRL));
    WriteCommand(NVGAMCTRL);
    WriteData(DATA_NVGAMCTRL, sizeof(NVGAMCTRL));

    WriteCommand(INVON);
    WriteCommand(SLPOUT);
    sleep_ms(120);
    WriteCommand(DISPON);

    SetDirection(0);
}

void ST7796::SetWindow(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)
{
    uint8_t Data[8] = {0};
    Data[0] = xs >> 8;
    Data[1] = 0x00ff & xs;
    Data[2] = xe >> 8;
    Data[3] = (0x00FF & xe);
    WriteCommand(SetXCmd);
    WriteData(Data, 4);

    Data[0] = ys >> 8;
    Data[1] = 0x00ff & ys;
    Data[2] = ye >> 8;
    Data[3] = (0x00FF & ye);
    WriteCommand(SetYCmd);
    WriteData(Data, 4);

    WriteCommand(WramCmd);
}

void ST7796::DrawPixel(uint16_t x, uint16_t y, uint32_t Color)
{
    uint8_t Data[3];
    Data[2] = (Color >> 24) & 0xff;
    Data[1] = (Color >> 16) & 0xff;
    Data[0] = (Color >> 8) & 0xff;
    SetWindow(x, y, x, y);
    WriteData(Data, 3);
}

void ST7796::SetDirection(uint8_t dir)
{
    SetXCmd = 0x2A;
    SetYCmd = 0x2B;
    WramCmd = 0x2C;
    RramCmd = 0x2E;
    uint8_t Data[8] = {0};
    switch (dir)
    {
    case 0:
        LcdWidth = LCD_WIDTH;
        lcdHeight = LCD_HEIGHT;
        Data[0] = (1 << 6);
        WriteCommand(0x36);
        WriteData(Data, 1);
        break;
    case 1:
        LcdWidth = LCD_HEIGHT;
        lcdHeight = LCD_WIDTH;
        Data[0] = (1 << 5);
        WriteCommand(0x36);
        WriteData(Data, 1);
        break;
    case 2:
        LcdWidth = LCD_WIDTH;
        lcdHeight = LCD_HEIGHT;
        Data[0] = (1 << 7);
        WriteCommand(0x36);
        WriteData(Data, 1);
        break;
    case 3:
        LcdWidth = LCD_HEIGHT;
        lcdHeight = LCD_WIDTH;
        Data[0] = (1 << 5) | (1 << 6) | (1 << 7);
        WriteCommand(0x36);
        WriteData(Data, 1);
        break;

    default:
        break;
    }
}

void ST7796::DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t Color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    delta_x = x2 - x1; // 计算坐标增量
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)
        incx = 1; // 设置单步方向
    else if (delta_x == 0)
        incx = 0; // 垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; // 水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)
        distance = delta_x; // 选取基本增量坐标轴
    else
        distance = delta_y;
    for (t = 0; t <= distance + 1; t++) // 画线输出
    {
        DrawPixel(uRow, uCol, Color); // 画点
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

void ST7796::DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t Color)
{
    DrawLine(x1, y1, x2, y1, Color);
    DrawLine(x1, y1, x1, y2, Color);
    DrawLine(x1, y2, x2, y2, Color);
    DrawLine(x2, y1, x2, y2, Color);
}

void ST7796::LcdFill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t Color)
{
    uint8_t Data[3] = {0};
    Data[2] = (Color >> 24) & 0xff;
    Data[1] = (Color >> 16) & 0xff;
    Data[0] = (Color >> 8) & 0xff;
    uint16_t i, j;
    uint16_t width = ex - sx + 1;  // 得到填充的宽度
    uint16_t height = ey - sy + 1; // 高度
    SetWindow(sx, sy, ex, ey);     // 设置显示窗口
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
            WriteData(Data, 3);
    }
    SetWindow(0, 0, LcdWidth - 1, lcdHeight - 1); // 恢复窗口设置为全屏
}

void ST7796::DrawBmp(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, const uint8_t *img)
{
    SetWindow(xStart, yStart, xEnd, yEnd);
    uint8_t Data[3] = {0};
    for (int i = 0; i < LcdWidth * lcdHeight * 4; i += 4)
    {
        Data[0] = (uint8_t(img[i] * 0.9) / 4) << 2;
        Data[1] = (uint8_t(img[i + 1] * 0.95) / 4) << 2;
        Data[2] = (img[i + 2] / 4) << 2;
        WriteData(Data, 3);
    }
    SetWindow(0, 0, LcdWidth - 1, lcdHeight - 1);
}

void ST7796::DrawBmpPixel(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, const uint8_t *img)
{
    SetWindow(xStart, yStart, xEnd, yEnd);
    uint8_t Data[3] = {0};
    for (int i = 0; i < LcdWidth * lcdHeight * 4; i += 3)
    {
        Data[0] = (uint8_t(img[i] * 0.9) / 4) << 2;
        Data[1] = (uint8_t(img[i + 1] * 0.95) / 4) << 2;
        Data[2] = (img[i + 2] / 4) << 2;
        WriteData(Data, 3);
    }
    SetWindow(0, 0, LcdWidth - 1, lcdHeight - 1);
}
