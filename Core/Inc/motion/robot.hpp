#pragma once

#include "wheel.hpp"

class Robot
{
private:

    Wheel& w1;
    Wheel& w2;
    Wheel& w3;
    Wheel& w4;

public:

    Robot(Wheel& wheel1,
          Wheel& wheel2,
          Wheel& wheel3,
          Wheel& wheel4);

    void forward(int speed);
    void backward(int speed);

    void turnLeft(int speed);
    void turnRight(int speed);

    void stop();

    void update();
};
