#include "zf_common_headfile.h"
#include "beep.h"
#include "key.h"
#include "motor.h"

#define YAW_UPDATE_DT       (0.05f)

volatile float yaw = 0.0f;
volatile uint8 mode_enable[4] = {0, 0, 0, 0};
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

static void wireless_uart_send_5float(float *dat)
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
    len = zf_sprintf((int8 *)wireless_uart_tx_buffer, (const int8 *)"%f,%f,%f,%f,%f\r\n", dat[0], dat[1], dat[2], dat[3], dat[4]);
    DMA_UR4T_STA = 0;
    DMA_UR4T_AMT = (len - 1) & 0xFF;
    DMA_UR4T_AMTH = (len - 1) >> 8;
    DMA_UR4T_TXAH = (uint8)((uint16)wireless_uart_tx_buffer >> 8);
    DMA_UR4T_TXAL = (uint8)((uint16)wireless_uart_tx_buffer);
    DMA_UR4T_CR = 0xC0;
}

static void mode1(void)
{
    static uint8 control_tick = 0;
    static float last_target = 0.0f;
    static float integral = 0.0f;
    const int base_speed = 500;
    const float kp = 2.0f;
    const float ki = 1.0f;
    float target;
    float gyro_z;
    float error;
    int diff;

    if(control_tick < 40)       target = (float)control_tick * 2.0f;
    else if(control_tick < 80)  target = (float)(80 - control_tick) * 2.0f;
    else if(control_tick < 120) target = -(float)(control_tick - 80) * 2.0f;
    else                       target = -(float)(160 - control_tick) * 2.0f;
    if(++control_tick >= 160)
    {
        control_tick = 0;
    }
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
    wireless_uart_data[0] = key_data[0];
    wireless_uart_data[1] = key_data[1];
    wireless_uart_data[2] = key_data[2];
    wireless_uart_data[3] = key_data[3];
    wireless_uart_data[4] = 0;
    wireless_uart_pending = 1;
}

static void mode2(void)
{
}

static void mode3(void)
{
}

static void mode4(void)
{
}

static void mode_disable_all(void)
{
    mode_enable[0] = 0;
    mode_enable[1] = 0;
    mode_enable[2] = 0;
    mode_enable[3] = 0;
    Motor_Set_Speed(0, 0);
}

static void mode_key_toggle(uint8 index)
{
    if(mode_enable[index])
    {
        mode_disable_all();
    }
    else
    {
        mode_disable_all();
        mode_enable[index] = 1;
    }
}

static void tim0_50ms_handler(void)
{
    static uint8 key_last[4] = {0, 0, 0, 0};
    uint8 i;

    Key_Update();
    for(i = 0; i < 4; i++)
    {
        if(key_data[i] && !key_last[i])
        {
            mode_key_toggle(i);
        }
        key_last[i] = key_data[i];
    }

    if(mode_enable[0]) mode1();
    else if(mode_enable[1]) mode2();
    else if(mode_enable[2]) mode3();
    else if(mode_enable[3]) mode4();
}

void main()
{
    float wireless_uart_dat[5];

    clock_init(SYSTEM_CLOCK_30M);
    debug_init();
    wireless_uart_try_init();
    ips114_init();
    ips114_clear(IPS114_DEFAULT_BGCOLOR);

    imu660rb_init();
    Beep_Init();
    Key_Init();
    Beep_On();
    Motor_Init();
    pit_ms_init(TIM0_PIT, 50);
    tim0_irq_handler = tim0_50ms_handler;
    Beep_Off();
    while (1)
    {
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
            wireless_uart_send_5float(wireless_uart_dat);
        }
    }
}
