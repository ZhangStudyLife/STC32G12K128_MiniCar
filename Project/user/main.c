#include "zf_common_headfile.h"
#include "beep.h"
#include "motor.h"
#include "track_io.h"

#define IMU_TEXT_X (0U)
#define IMU_VALUE_X (56U)
#define IMU_LINE_HEIGHT (16U)

static void gyro_z_pi_50ms_handler(void)
{
    static float integral = 0.0f;
    const int base_speed = 700;
    const float kp = 3.0f;
    const float ki = 0.6f;
    float target;
    float gyro_z;
    float error;
    int diff;

    target = (float)((int)track_io_data[TRACK_IO_L2] * -40
                   + (int)track_io_data[TRACK_IO_L1] * -20
                   + (int)track_io_data[TRACK_IO_R1] * 20
                   + (int)track_io_data[TRACK_IO_R2] * 40);

    // 先不用红外循迹模块，目标直接给0
    target = 0.0f;
    imu660rb_get_gyro();
    gyro_z = - imu660rb_gyro_transition((imu660rb_gyro_z+1));       // +1 去零漂
    error = target - gyro_z;
    integral += error;

    if(integral > 300.0f) integral = 300.0f;
    if(integral < -300.0f) integral = -300.0f;

    diff = (int)(kp * error + ki * integral);
    Motor_Set_Speed(base_speed + diff, base_speed - diff);
}

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
    Beep_Init();
    Motor_Init();
    pit_ms_init(TIM0_PIT, 50);
    tim0_irq_handler = gyro_z_pi_50ms_handler;
    while (1)
    {
        imu660rb_get_acc();
        Track_IO_Update();

        ips114_clear(IPS114_DEFAULT_BGCOLOR);
        imu660rb_display_raw_data();
        Track_IO_Update();
        printf("%d,%d,%d,%d,%d\r\n",
               track_io_data[TRACK_IO_L2],
               track_io_data[TRACK_IO_L1],
               track_io_data[TRACK_IO_MID],
               track_io_data[TRACK_IO_R1],
               track_io_data[TRACK_IO_R2]);
    }
}
