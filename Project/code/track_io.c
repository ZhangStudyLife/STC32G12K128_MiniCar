#include "track_io.h"
#include "zf_driver_gpio.h"

#define TRACK_IO_LEFT_PIN   (IO_P11)
#define TRACK_IO_RIGHT_PIN  (IO_P10)

uint8 track_io_data[2] = {0, 0};

void Track_IO_Init(void)
{
    gpio_init(TRACK_IO_LEFT_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(TRACK_IO_RIGHT_PIN, GPI, 1, GPI_PULL_UP);
}

void Track_IO_Update(void)
{
    track_io_data[TRACK_IO_LEFT] = gpio_get_level(TRACK_IO_LEFT_PIN);
    track_io_data[TRACK_IO_RIGHT] = gpio_get_level(TRACK_IO_RIGHT_PIN);
}
