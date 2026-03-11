#include "motion/wheel.hpp"

Wheel::Wheel(Motor& motor_ref)
    : motor(motor_ref)
{
}

int Wheel::clamp(int speed)
{
    if(speed < MIN_PWM)
        speed = MIN_PWM;

    if(speed > MAX_PWM)
        speed = MAX_PWM;

    return speed;
}

void Wheel::setSpeed(Direction dir, int speed)
{
    if(speed == 0)
    {
        stop();
        return;
    }

    speed = clamp(speed);

    if(dir == Direction::Backward)
        speed = -speed;

    target_speed = speed;
    current_speed = speed;

    // kickstart
    motor.setSpeed((speed > 0) ? MAX_PWM : -MAX_PWM);

    starting = true;
    kickstart_time = HAL_GetTick();
}

void Wheel::update()
{
    if(starting)
    {
        if(HAL_GetTick() - kickstart_time > KICKSTART_TIME_MS)
        {
            motor.setSpeed(current_speed);
            starting = false;
        }
    }
}

void Wheel::stop()
{
    motor.stop();

    current_speed = 0;
    target_speed = 0;
    starting = false;
}
