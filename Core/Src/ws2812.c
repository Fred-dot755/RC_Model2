//written by Fred Xiao

#include "ws2812.h"

#define WS2812_LowLevel    0xC0     // 0Ты
#define WS2812_HighLevel   0xF0     // 1Ты

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}

void RGB_Color_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    WS2812_Ctrl(r, g, b);
}



uint8_t WS2812_Buffer[PIXEL_NUM * 24];

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    // 1. 一次性准备好所有灯珠的数据
    for(int j = 0; j < PIXEL_NUM; j++)
    {
        int offset = j * 24; // 计算当前灯珠在数组中的起始位置
        for (int i = 0; i < 8; i++)
        {
            // 依然是你的位移逻辑，按 G-R-B 顺序填入
            WS2812_Buffer[offset + 7 - i]  = ((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel;
            WS2812_Buffer[offset + 15 - i] = ((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel;
            WS2812_Buffer[offset + 23 - i] = ((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel;
        }
    }
    
    // 2. 一次性连续发送所有灯的数据，中间不断流
    // 注意：如果是多个灯，超时时间稍微给大一点(1000)
    HAL_SPI_Transmit(&hspi3, WS2812_Buffer, PIXEL_NUM * 24, 1000);
    
    // 3. 发送复位码，触发所有灯珠一起亮起
    uint8_t res = 0x00;
    for (int i = 0; i < 200; i++)
    {
        HAL_SPI_Transmit(&hspi3, &res, 1, 10);
    }
}

// void WS2812_Test(void)
// {
//     // 1. 每次发送前，先清空整个数组（尤其是把后面的 RESET_LEN 区域全部变成 0x00）
//     memset(WS2812_Buffer, 0, sizeof(WS2812_Buffer));

//     // 2. 准备测试颜色：灯1红，灯2绿，灯3蓝 (亮度统一给 30)
//     uint8_t colors[3][3] = {
//         {30, 0, 0},   // 红 (R, G, B)
//         {0, 30, 0},   // 绿
//         {0, 0, 30}    // 蓝
//     };

//     // 3. 填充颜色数据到数组前段
//     for(int j = 0; j < PIXEL_NUM; j++)
//     {
//         uint8_t r = colors[j][0];
//         uint8_t g = colors[j][1];
//         uint8_t b = colors[j][2];
//         int offset = j * 24;

//         for (int i = 0; i < 8; i++)
//         {
//             // 按 G-R-B 顺序填充
//             WS2812_Buffer[offset + 7 - i]  = ((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel;
//             WS2812_Buffer[offset + 15 - i] = ((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel;
//             WS2812_Buffer[offset + 23 - i] = ((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel;
//         }
//     }

//     // 4. 【关键】一口气连贯发送所有数据（颜色 + 300个字节的复位0x00）
//     // 中间没有任何软件间隙，硬件 SPI 会像机枪一样连续扫射输出
//     HAL_SPI_Transmit(&hspi3, WS2812_Buffer, sizeof(WS2812_Buffer), 1000);
// }