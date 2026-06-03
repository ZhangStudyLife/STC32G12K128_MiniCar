#include "key.h"
#include "zf_driver_gpio.h"

volatile uint8 key_data[4] = {0, 0, 0, 0};

void Key_Init(void)
{
    gpio_init(IO_P70, GPI, 1, GPI_PULL_UP);
    gpio_init(IO_P71, GPI, 1, GPI_PULL_UP);
    gpio_init(IO_P72, GPI, 1, GPI_PULL_UP);
    gpio_init(IO_P73, GPI, 1, GPI_PULL_UP);
}

void Key_Update(void)
{
    static uint8 last = 0;
    uint8 now = (~P7) & 0x0F;
    uint8 same = ~(now ^ last);

    if(same & 0x01) key_data[0] = now & 1;
    if(same & 0x02) key_data[1] = (now >> 1) & 1;
    if(same & 0x04) key_data[2] = (now >> 2) & 1;
    if(same & 0x08) key_data[3] = (now >> 3) & 1;
    last = now;
}
