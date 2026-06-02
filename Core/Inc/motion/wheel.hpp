#pragma once

#include "motor.hpp"
#include "main.h"

/* Directions */
enum class Direction
{
    Forward,
    Backward
};

class Wheel
{
private:

    Motor& motor;

    static constexpr int MIN_PWM = 200;
    static constexpr int MAX_PWM = 999;
    static constexpr uint32_t KICKSTART_TIME_MS = 120;

    int current_speed = 0;
    int target_speed = 0;

    uint32_t kickstart_time = 0;
    bool starting = false;
    bool direction_inverted = false;

    int clamp(int speed);

public:

    Wheel(Motor& motor_ref, bool invert_direction = false);

    void setSpeed(Direction dir, int speed);

    void update();

    void stop();
};
