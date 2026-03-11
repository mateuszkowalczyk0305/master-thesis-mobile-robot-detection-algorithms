#pragma once

#include "motor.hpp"
#include "main.h"

class Wheel
{
private:

    Motor& motor;

    static constexpr int MIN_PWM = 1000;
    static constexpr int MAX_PWM = 1500;
    static constexpr uint32_t KICKSTART_TIME_MS = 200;

    int current_speed = 0;
    int target_speed = 0;

    uint32_t kickstart_time = 0;
    bool starting = false;

    int clamp(int speed);

public:

    Wheel(Motor& motor_ref);

    void setSpeed(int speed);

    void update();

    void stop();
};
