#include "zf_common_headfile.h"
#include "motor.h"
#include "track_io.h"

#define IMU_TEXT_X (0U)
#define IMU_VALUE_X (56U)
#define IMU_LINE_HEIGHT (16U)

static void imu660rb_display_raw_data(void)
{
    ips114_show_string(IMU_TEXT_X, 0U * IMU_LINE_HEIGHT, "ACC X:");
    ips114_show_int16(IMU_VALUE_X, 0U * IMU_LINE_HEIGHT, imu660rb_acc_x);

    ips114_show_string(IMU_TEXT_X, 1U * IMU_LINE_HEIGHT, "ACC Y:");
    ips114_show_int16(IMU_VALUE_X, 1U * IMU_LINE_HEIGHT, imu660rb_acc_y);

    ips114_show_string(IMU_TEXT_X, 2U * IMU_LINE_HEIGHT, "ACC Z:");
    ips114_show_int16(IMU_VALUE_X, 2U * IMU_LINE_HEIGHT, imu660rb_acc_z);

    ips114_show_string(IMU_TEXT_X, 3U * IMU_LINE_HEIGHT, "GYR X:");
    ips114_show_int16(IMU_VALUE_X, 3U * IMU_LINE_HEIGHT, imu660rb_gyro_x);

    ips114_show_string(IMU_TEXT_X, 4U * IMU_LINE_HEIGHT, "GYR Y:");
    ips114_show_int16(IMU_VALUE_X, 4U * IMU_LINE_HEIGHT, imu660rb_gyro_y);

    ips114_show_string(IMU_TEXT_X, 5U * IMU_LINE_HEIGHT, "GYR Z:");
    ips114_show_int16(IMU_VALUE_X, 5U * IMU_LINE_HEIGHT, imu660rb_gyro_z);
}

void main()
{
    clock_init(SYSTEM_CLOCK_30M);
    debug_init();

    Track_IO_Init();

    ips114_init();
    ips114_clear(IPS114_DEFAULT_BGCOLOR);

    imu660rb_init();
    Motor_Init();
    while (1)
    {
        imu660rb_get_acc();
        imu660rb_get_gyro();
        Track_IO_Update();

        ips114_clear(IPS114_DEFAULT_BGCOLOR);
        imu660rb_display_raw_data();
        Motor_Set_Speed(150, 150);
        Track_IO_Update();
        printf("%d,%d,%d,%d,%d\r\n",
               track_io_data[TRACK_IO_L2],
               track_io_data[TRACK_IO_L1],
               track_io_data[TRACK_IO_MID],
               track_io_data[TRACK_IO_R1],
               track_io_data[TRACK_IO_R2]);
    }
}
