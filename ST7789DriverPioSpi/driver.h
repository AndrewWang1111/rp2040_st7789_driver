#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pio_spi.h"

#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_MISO 0
#define PIN_CS 1 // 片选由pio程序初始化，由pio程序控制
#define PIN_BKL 4
#define PIN_RST 5
#define PIN_DC 6

#define LCD_HEIGHT 240
#define LCD_WIDTH 320

#define COLOR_RED 0x0000FF00
#define COLOR_GREEN 0x00FF0000
#define COLOR_BLUE 0xFF000000
#define LCD_DIRECTION 2
#define COLOR_DEEP 18

#define SLPOUT 0x11    /*Sleep out*/
#define INVON 0x21     /*Display inversion on*/
#define DISPON 0x29    /*Display on*/
#define CASET 0x2A     /*Set X address*/
#define RASET 0x2B     /*Set Y address*/
#define RAMWR 0x2C     /*Write data*/
#define MADCTL 0x36    /*Memory data access control*/
#define COLMOD 0x3A    /*Interface pixel format*/
#define PORCTRL 0xB2   /*Porch control*/
#define GCTRL 0xB7     /*Gate control*/
#define VCOMS 0xBB     /*VCOM setting*/
#define LCMCTRL 0xC0   /*LCM control*/
#define VDVVRHEN 0xC2  /*VDV and VRH Command Enable*/
#define VRHS 0xC3      /*VRH Set*/
#define VDVSET 0xC4    /*VDV Setting*/
#define FRCTR2 0xC6    /*FR Control2*/
#define PWCTRL1 0xD0   /*Power Control 1*/
#define PVGAMCTRL 0xE0 /*Positive Voltage Gamma Control*/
#define NVGAMCTRL 0xE1 /*Negative Voltage Gamma Control*/

class ST7796
{
public:
    ST7796();
    ~ST7796();
    void LcdInit();
    void DrawPixel(uint16_t x, uint16_t y, uint32_t Color);
    void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t Color);
    void DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t Color);
    void DrawFilledRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t Color);
    void LcdFill(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint32_t Color);
    void DrawBmp(uint16_t xStart, uint16_t ySart, uint16_t xEnd, uint16_t yEnd, const uint8_t *img);
    void DrawBmpPixel(uint16_t xStart, uint16_t ySart, uint16_t xEnd, uint16_t yEnd, const uint8_t *img);
    void SetDirection(uint8_t dir);

private:
    uint16_t LcdPinSck;
    uint16_t LcdPinMosi;
    uint16_t LcdPinMiso;
    uint16_t LcdPinCs;
    uint16_t LcdPinBkl;
    uint16_t LcdPinRst;
    uint16_t LcdPinDc;
    uint16_t LcdPinTouchCs;
    uint16_t LcdPinSdCs;
    uint16_t LcdWidth;
    uint16_t lcdHeight;
    uint16_t WramCmd;
    uint16_t RramCmd;
    uint16_t SetXCmd;
    uint16_t SetYCmd;
    uint16_t LcdColorDeep;
    pio_spi_inst_t spi;
    uint dmaChannel;
    void LcdHardReset();
    void HardwardInit();
    void WriteCommand(uint8_t);
    void WriteData(uint8_t *, uint16_t len);
    void SetWindow(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd);
    uint8_t *ReadData();
};