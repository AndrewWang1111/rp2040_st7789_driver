#include <ctype.h>
#include <stdbool.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "driver.h"
#include <hardware/vreg.h>

using namespace std;

uint bytesAlreadyRead = 0;
uint fNum = 0;
uint currentIndex = 0;
uint currentFrameIndex = 0;
char fileName[16];
char buffer[512];
char BmpHeader[128];

int main()
{
    set_sys_clock_khz(266 * 1000, true);
    stdio_init_all();
    stdio_uart_init();
    ST7796 disp;
    disp.LcdInit();
    disp.LcdFill(0, 0, 240, 320, COLOR_BLUE);
    int x, y;

    while (1)
    {
        uint32_t color = rand() % 0xffffffff;
        x = rand() % 240;
        y = rand() % 320;
        disp.LcdFill(x, y, x + 30, y + 30, color);
    }
    return 0;
}
