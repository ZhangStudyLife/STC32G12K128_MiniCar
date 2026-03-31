#include "track_io.h"
#include "zf_driver_gpio.h"

#define TRACK_IO_L2_PIN     (IO_P77)
#define TRACK_IO_L1_PIN     (IO_P33)
#define TRACK_IO_MID_PIN    (IO_P26)
#define TRACK_IO_R1_PIN     (IO_P11)
#define TRACK_IO_R2_PIN     (IO_P10)

uint8 track_io_data[5] = {0, 0, 0, 0, 0};

void Track_IO_Init(void)
{
    gpio_init(TRACK_IO_L2_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(TRACK_IO_L1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(TRACK_IO_MID_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(TRACK_IO_R1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(TRACK_IO_R2_PIN, GPI, 1, GPI_PULL_UP);
}

void Track_IO_Update(void)
{
    track_io_data[TRACK_IO_L2] = gpio_get_level(TRACK_IO_L2_PIN);
    track_io_data[TRACK_IO_L1] = gpio_get_level(TRACK_IO_L1_PIN);
    track_io_data[TRACK_IO_MID] = gpio_get_level(TRACK_IO_MID_PIN);
    track_io_data[TRACK_IO_R1] = gpio_get_level(TRACK_IO_R1_PIN);
    track_io_data[TRACK_IO_R2] = gpio_get_level(TRACK_IO_R2_PIN);
}
