#include "motion/wheel.hpp"

Wheel::Wheel(Motor& motor_ref)
    : motor(motor_ref)
{
}

int Wheel::clamp(int speed)
{
    if(speed > 0)
    {
        if(speed < MIN_PWM) return MIN_PWM;
        if(speed > MAX_PWM) return MAX_PWM;
    }

    if(speed < 0)
    {
        if(speed > -MIN_PWM) return -MIN_PWM;
        if(speed < -MAX_PWM) return -MAX_PWM;
    }

    return speed;
}

void Wheel::setSpeed(int speed)
{
    if(speed == 0)
    {
        stop();
        return;
    }

    int new_speed = clamp(speed);

    target_speed = new_speed;
    current_speed = new_speed;

    // kickstart
    motor.setSpeed((new_speed > 0) ? MAX_PWM : -MAX_PWM);

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
