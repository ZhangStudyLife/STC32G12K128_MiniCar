#include "zf_common_headfile.h"
#include "motor.h"

#define MOTOR_PWM_FREQ           (20000U)
#define MOTOR_SPEED_MAX          (1000)
#define MOTOR_SPEED_LIMIT        (900)
#define MOTOR_THROTTLE_DEADZONE  (125)
#define MOTOR_DUTY_SCALE         (10U)

#define MOTOR_LEFT_PWM_PIN       (PWMA_CH4P_P66)
#define MOTOR_RIGHT_PWM_PIN      (PWMA_CH2P_P62)
#define MOTOR_LEFT_DIR_PIN       (IO_P64)
#define MOTOR_RIGHT_DIR_PIN      (IO_P60)

#define MOTOR_DIR_FORWARD        (GPIO_LOW)
#define MOTOR_DIR_REVERSE        (GPIO_HIGH)

static int motor_limit_speed(int speed)
{
    if((speed >= -MOTOR_THROTTLE_DEADZONE) && (speed <= MOTOR_THROTTLE_DEADZONE))
    {
        speed = 0;
    }

    if(speed > MOTOR_SPEED_LIMIT)
    {
        speed = MOTOR_SPEED_LIMIT;
    }
    else if(speed < -MOTOR_SPEED_LIMIT)
    {
        speed = -MOTOR_SPEED_LIMIT;
    }

    return speed;
}

static void motor_apply_speed(gpio_pin_enum dir_pin, pwm_channel_enum pwm_pin, int speed)
{
    uint16 duty = 0;

    speed = motor_limit_speed(speed);

    if(speed < 0)
    {
        gpio_set_level(dir_pin, MOTOR_DIR_REVERSE);
        duty = (uint16)((-speed) * MOTOR_DUTY_SCALE);
    }
    else
    {
        gpio_set_level(dir_pin, MOTOR_DIR_FORWARD);
        duty = (uint16)(speed * MOTOR_DUTY_SCALE);
    }

    pwm_set_duty(pwm_pin, duty);
}

void Motor_Init(void)
{
    gpio_init(MOTOR_LEFT_DIR_PIN, GPO, MOTOR_DIR_FORWARD, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_DIR_PIN, GPO, MOTOR_DIR_FORWARD, GPO_PUSH_PULL);

    pwm_init(MOTOR_LEFT_PWM_PIN, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_RIGHT_PWM_PIN, MOTOR_PWM_FREQ, 0);
}

void Motor_Set_Speed(int left, int right)
{
    motor_apply_speed(MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN, left);
    motor_apply_speed(MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_PIN, right);
}
