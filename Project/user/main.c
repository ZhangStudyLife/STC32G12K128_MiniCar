#include "zf_common_headfile.h"
#include "beep.h"
#include "motor.h"
#include "track_io.h"

#define TRACK_VALUE_X       (0U)
#define TRACK_REFRESH_DIV   (50U)
#define BEEP_TOGGLE_TICKS   (20U)
#define YAW_UPDATE_DT       (0.05f)
#define TRACK_TARGET_SCALE  (100)

volatile float yaw = 0.0f;

static float track_get_target(void)
{
    static uint8 last_valid_left = 1U;
    static uint8 last_valid_right = 1U;
    uint8 left = track_io_data[TRACK_IO_LEFT] ? 1U : 0U;
    uint8 right = track_io_data[TRACK_IO_RIGHT] ? 1U : 0U;

    if((left != 0U) || (right != 0U))
    {
        last_valid_left = left;
        last_valid_right = right;
    }
    else
    {
        left = last_valid_left;
        right = last_valid_right;
    }

    return (float)((int)left * TRACK_TARGET_SCALE
                 + (int)right * -TRACK_TARGET_SCALE);
}

static void gyro_z_pi_50ms_handler(void)
{
    static float integral = 0.0f;
    static uint8 tick_50ms = 0;
    static uint8 beep_on = 1;
    const int base_speed = 400;
    const float kp = 3.0f;
    const float ki = 1.2f;
    float target;
    float gyro_z;
    float error;
    int diff;

    target = track_get_target();

    // 先不用红外循迹模块，目标直接给0
    // target = 0.0f;
    imu660rb_get_gyro();
    gyro_z = - imu660rb_gyro_transition((imu660rb_gyro_z+1));       // +1 去零漂
    yaw += gyro_z * YAW_UPDATE_DT;
    if(yaw > 180.0f)
    {
        yaw -= 360.0f;
    }
    else if(yaw < -180.0f)
    {
        yaw += 360.0f;
    }

    error = target - gyro_z;
    integral += error;

    if(integral > 300.0f) integral = 300.0f;
    if(integral < -300.0f) integral = -300.0f;

    diff = (int)(kp * error + ki * integral);
    Motor_Set_Speed(base_speed + diff, base_speed - diff);

    // if(++tick_50ms >= BEEP_TOGGLE_TICKS)
    // {
    //     tick_50ms = 0;
    //     beep_on = !beep_on;
    //     if(beep_on)
    //     {
    //         Beep_On();
    //     }
    //     else
    //     {
    //         Beep_Off();
    //     }
    // }
}

static void track_io_display_2_low_rate(void)
{
    static uint8 refresh_div = 0;
    static uint8 last_pattern = 0xFF;
    uint8 pattern;
    char track_text[3] = "00";

    if(++refresh_div < TRACK_REFRESH_DIV)
    {
        return;
    }
    refresh_div = 0;

    pattern = ((track_io_data[TRACK_IO_LEFT] ? 1U : 0U) << 1)
            | ((track_io_data[TRACK_IO_RIGHT] ? 1U : 0U) << 0);

    if(pattern == last_pattern)
    {
        return;
    }
    last_pattern = pattern;

    track_text[0] = track_io_data[TRACK_IO_RIGHT] ? '1' : '0';
    track_text[1] = track_io_data[TRACK_IO_LEFT] ? '1' : '0';

    ips114_show_string(TRACK_VALUE_X, 0U, track_text);
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
    Beep_On();
    Motor_Init();
    pit_ms_init(TIM0_PIT, 50);
    tim0_irq_handler = gyro_z_pi_50ms_handler;
    Beep_Off();
    while (1)
    {
        Track_IO_Update();
        track_io_display_2_low_rate();
    }
}
