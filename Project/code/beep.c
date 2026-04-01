#include "zf_common_headfile.h"
#include "beep.h"

#define BEEP_PIN        (IO_P67)
#define BEEP_ON_LEVEL   (GPIO_HIGH)
#define BEEP_OFF_LEVEL  (GPIO_LOW)

void Beep_Init(void)
{
    gpio_init(BEEP_PIN, GPO, BEEP_OFF_LEVEL, GPO_PUSH_PULL);
}

void Beep_On(void)
{
    gpio_set_level(BEEP_PIN, BEEP_ON_LEVEL);
}

void Beep_Off(void)
{
    gpio_set_level(BEEP_PIN, BEEP_OFF_LEVEL);
}
