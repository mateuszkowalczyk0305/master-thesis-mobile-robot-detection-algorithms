#include "motion/robot.hpp"

Robot::Robot(Wheel& wheel1,
             Wheel& wheel2,
             Wheel& wheel3,
             Wheel& wheel4)
    : w1(wheel1),
      w2(wheel2),
      w3(wheel3),
      w4(wheel4)
{
}

void Robot::forward(int speed)
{
    w1.setSpeed(speed);
    w2.setSpeed(speed);
    w3.setSpeed(speed);
    w4.setSpeed(speed);
}

void Robot::backward(int speed)
{
    w1.setSpeed(-speed);
    w2.setSpeed(-speed);
    w3.setSpeed(-speed);
    w4.setSpeed(-speed);
}

void Robot::turnLeft(int speed)
{
    w1.setSpeed(-speed);
    w2.setSpeed(speed);
    w3.setSpeed(-speed);
    w4.setSpeed(speed);
}

void Robot::turnRight(int speed)
{
    w1.setSpeed(speed);
    w2.setSpeed(-speed);
    w3.setSpeed(speed);
    w4.setSpeed(-speed);
}

void Robot::stop()
{
    w1.stop();
    w2.stop();
    w3.stop();
    w4.stop();
}

void Robot::update()
{
    w1.update();
    w2.update();
    w3.update();
    w4.update();
}
