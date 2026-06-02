#include "zf_common_headfile.h"
#include "beep.h"
#include "motor.h"
#include "track_io.h"

#define TRACK_VALUE_X       (0U)
#define TRACK_REFRESH_DIV   (50U)
#define BEEP_TOGGLE_TICKS   (20U)
#define YAW_UPDATE_DT       (0.05f)
#define TRACK_TARGET_SCALE  (80)

volatile float yaw = 0.0f;
static uint8 wireless_uart_ok = 0;
static uint8 wireless_uart_pending = 0;
static float wireless_uart_data[5];
static uint8 xdata wireless_uart_tx_buffer[96];

static void wireless_uart_try_init(void)
{
    uint16 timeout = 5000;

    gpio_init(WIRELESS_UART_RTS_PIN, GPI, 1, GPI_PULL_UP);
    while(timeout-- && gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        system_delay_ms(1);
    }
    if(timeout != 0xFFFF)
    {
        uart_init(WIRELESS_UART_INDEX, WIRELESS_UART_BUAD_RATE, WIRELESS_UART_RX_PIN, WIRELESS_UART_TX_PIN);
        wireless_uart_ok = 1;
    }
}

static void wireless_uart_send_5float(float a, float b, float c, float d, float e)
{
    uint16 len;

    if(!wireless_uart_ok || gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        return;
    }
    if(DMA_UR4T_STA & 0x01)
    {
        DMA_UR4T_STA = 0;
        DMA_UR4T_CR = 0;
    }
    if(DMA_UR4T_CR & 0x80)
    {
        return;
    }
    len = zf_sprintf((int8 *)wireless_uart_tx_buffer, (const int8 *)"%f,%f,%f,%f,%f\r\n", a, b, c, d, e);
    DMA_UR4T_STA = 0;
    DMA_UR4T_AMT = (len - 1) & 0xFF;
    DMA_UR4T_AMTH = (len - 1) >> 8;
    DMA_UR4T_TXAH = (uint8)((uint16)wireless_uart_tx_buffer >> 8);
    DMA_UR4T_TXAL = (uint8)((uint16)wireless_uart_tx_buffer);
    DMA_UR4T_CR = 0xC0;
}

float track_get_target(void)
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

static float target_gyro_z_by_time(uint32 tick)
{
    tick %= 160;
    if(tick < 40)  return (float)tick * 2.0f;
    if(tick < 80)  return (float)(80 - tick) * 2.0f;
    if(tick < 120) return -(float)(tick - 80) * 2.0f;
    return -(float)(160 - tick) * 2.0f;
}

static void gyro_z_pi_50ms_handler(void)
{
    static uint32 control_tick = 0;
    static float last_target = 0.0f;
    static float integral = 0.0f;
    static uint8 tick_50ms = 0;
    static uint8 beep_on = 1;
    const int base_speed = 500;
    const float kp = 2.0f;
    const float ki = 1.0f;
    float target;
    float gyro_z;
    float error;
    int diff;

    target = target_gyro_z_by_time(control_tick++);
    if((target * last_target) < 0.0f)
    {
        integral = 0.0f;
    }
    last_target = target;

    // 目标角速度只按时间规划，不跟随红外循迹。
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
    integral += error * YAW_UPDATE_DT;

    if(integral > 80.0f) integral = 80.0f;
    if(integral < -80.0f) integral = -80.0f;

    diff = (int)(kp * error + ki * integral);
    if(diff > 180) diff = 180;
    if(diff < -180) diff = -180;
    Motor_Set_Speed(base_speed + diff, base_speed - diff);
    wireless_uart_data[0] = target;
    wireless_uart_data[1] = gyro_z;
    wireless_uart_data[2] = yaw;
    wireless_uart_data[3] = error;
    wireless_uart_data[4] = integral;
    wireless_uart_pending = 1;

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

void track_io_display_2_low_rate(void)
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
    float wireless_uart_dat[5];

    clock_init(SYSTEM_CLOCK_30M);
    debug_init();
    wireless_uart_try_init();
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
        if(wireless_uart_pending)
        {
            interrupt_global_disable();
            wireless_uart_pending = 0;
            wireless_uart_dat[0] = wireless_uart_data[0];
            wireless_uart_dat[1] = wireless_uart_data[1];
            wireless_uart_dat[2] = wireless_uart_data[2];
            wireless_uart_dat[3] = wireless_uart_data[3];
            wireless_uart_dat[4] = wireless_uart_data[4];
            interrupt_global_enable();
            wireless_uart_send_5float(wireless_uart_dat[0], wireless_uart_dat[1], wireless_uart_dat[2], wireless_uart_dat[3], wireless_uart_dat[4]);
        }
        // track_io_display_2_low_rate();
    }
}
