#include "zf_common_headfile.h"
#include "beep.h"
#include "key.h"
#include "motor.h"
#include "track_io.h"

#define YAW_UPDATE_DT       (0.05f)
#define TRACK_TARGET_SCALE  (50)
#define MODE1_LOST_TICK     (4)
#define MODE1_SEARCH_SPEED  (350)
#define MODE2_RUN_TICK      (40)
#define MODE2_RUN_SPEED     (500)
#define MODE2_TURN_SPEED    (650)
#define MODE2_TURN_ANGLE    (90.0f)

volatile float yaw = 0.0f;
volatile uint8 mode_enable[4] = {0, 0, 0, 0};
static uint8 wireless_uart_ok = 0;
static uint8 wireless_uart_pending = 0;
static uint8 mode2_state = 0;
static uint8 mode2_tick = 0;
static float mode2_yaw = 0.0f;
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

static void mode1(void)
{
    static uint8 lost_tick = 0;
    static int search_speed = MODE1_SEARCH_SPEED;
    static float last_target = 0.0f;
    static float integral = 0.0f;
    int base_speed = 500;
    const float kp = 2.0f;
    const float ki = 1.0f;
    float target;
    float gyro_z;
    float error;
    int diff;

    if((track_io_data[TRACK_IO_LEFT] == 0) && (track_io_data[TRACK_IO_RIGHT] == 0))
    {
        if(lost_tick < MODE1_LOST_TICK)
        {
            lost_tick++;
        }
    }
    else
    {
        if(lost_tick >= MODE1_LOST_TICK)
        {
            integral = 0.0f;
            last_target = 0.0f;
        }
        lost_tick = 0;
    }

    if(lost_tick >= MODE1_LOST_TICK)
    {
        integral = 0.0f;
        last_target = 0.0f;
        Motor_Set_Speed(search_speed, -search_speed);
        return;
    }
    else
    {
        target = track_get_target();
    }
    if((target * last_target) < 0.0f)
    {
        integral = 0.0f;
    }
    last_target = target;

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
    if(diff > 0) search_speed = MODE1_SEARCH_SPEED;
    else if(diff < 0) search_speed = -MODE1_SEARCH_SPEED;
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
    float gyro_z;

    if(mode2_state == 0)
    {
        Motor_Set_Speed(MODE2_RUN_SPEED, MODE2_RUN_SPEED);
        if(++mode2_tick >= MODE2_RUN_TICK)
        {
            mode2_tick = 0;
            mode2_yaw = 0.0f;
            mode2_state = 1;
        }
    }
    else
    {
        imu660rb_get_gyro();
        gyro_z = - imu660rb_gyro_transition((imu660rb_gyro_z+1));
        mode2_yaw += gyro_z * YAW_UPDATE_DT;
        Motor_Set_Speed(MODE2_TURN_SPEED, -MODE2_TURN_SPEED);
        if((mode2_yaw >= MODE2_TURN_ANGLE) || (mode2_yaw <= -MODE2_TURN_ANGLE))
        {
            mode2_tick = 0;
            mode2_yaw = 0.0f;
            mode2_state = 0;
        }
    }
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
    mode2_state = 0;
    mode2_tick = 0;
    mode2_yaw = 0.0f;
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
    Track_IO_Update();
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
    Track_IO_Init();
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
