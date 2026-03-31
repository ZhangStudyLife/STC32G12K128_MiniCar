#include "zf_common_headfile.h"
#include "motor.h"

#define MOTOR_TEST_STEP_MS          (10U)
#define MOTOR_TEST_RAMP_UP_MS       (3000U)
#define MOTOR_TEST_RAMP_DOWN_MS     (6000U)
#define MOTOR_TEST_RAMP_RETURN_MS   (3000U)
#define MOTOR_TEST_CYCLE_MS         (MOTOR_TEST_RAMP_UP_MS + MOTOR_TEST_RAMP_DOWN_MS + MOTOR_TEST_RAMP_RETURN_MS)
#define MOTOR_TEST_SPEED_MIN        (-300)
#define MOTOR_TEST_SPEED_MAX        (300)

static int motor_test_get_speed(uint16 cycle_time_ms)
{
    int32 speed = 0;

    if(cycle_time_ms < MOTOR_TEST_RAMP_UP_MS)
    {
        speed = (int32)MOTOR_TEST_SPEED_MAX * cycle_time_ms / MOTOR_TEST_RAMP_UP_MS;
    }
    else if(cycle_time_ms < (MOTOR_TEST_RAMP_UP_MS + MOTOR_TEST_RAMP_DOWN_MS))
    {
        speed = MOTOR_TEST_SPEED_MAX;
        speed -= (int32)(MOTOR_TEST_SPEED_MAX - MOTOR_TEST_SPEED_MIN) *
                 (cycle_time_ms - MOTOR_TEST_RAMP_UP_MS) / MOTOR_TEST_RAMP_DOWN_MS;
    }
    else
    {
        speed = MOTOR_TEST_SPEED_MIN;
        speed += (int32)(0 - MOTOR_TEST_SPEED_MIN) *
                 (cycle_time_ms - MOTOR_TEST_RAMP_UP_MS - MOTOR_TEST_RAMP_DOWN_MS) / MOTOR_TEST_RAMP_RETURN_MS;
    }

    return (int)speed;
}

void main()
{
    uint16 cycle_time_ms = 0;
    int speed = 0;

    clock_init(SYSTEM_CLOCK_30M);
    debug_init();
    Motor_Init();

    while(1)
    {
        speed = motor_test_get_speed(cycle_time_ms);
        Motor_Set_Speed(speed, speed);

        system_delay_ms(MOTOR_TEST_STEP_MS);
        cycle_time_ms += MOTOR_TEST_STEP_MS;
        if(cycle_time_ms >= MOTOR_TEST_CYCLE_MS)
        {
            cycle_time_ms = 0;
        }
    }
}
