#include "commands/MotionCommandHandler.hpp"

namespace
{
constexpr int PROTOCOL_SPEED_MAX = 9;
constexpr int ROBOT_MIN_PWM = 800;
constexpr int ROBOT_MAX_PWM = 999;

int protocolSpeedToPwm(uint8_t protocolSpeed)
{
    if (protocolSpeed > PROTOCOL_SPEED_MAX)
    {
        protocolSpeed = PROTOCOL_SPEED_MAX;
    }

    return ROBOT_MIN_PWM +
        ((ROBOT_MAX_PWM - ROBOT_MIN_PWM) * static_cast<int>(protocolSpeed)) /
        PROTOCOL_SPEED_MAX;
}
}

void MotionCommandHandler::execute(Robot& robot, const Esp32Command& command)
{
    const int speed = protocolSpeedToPwm(command.speed);

    switch (command.motionDirection)
    {
        case Esp32MotionDirection::Front:
            robot.forward(speed);
            break;

        case Esp32MotionDirection::Back:
            robot.backward(speed);
            break;

        case Esp32MotionDirection::Right:
            robot.turnRight(speed);
            break;

        case Esp32MotionDirection::Left:
            robot.turnLeft(speed);
            break;

        case Esp32MotionDirection::Stop:
            robot.stop();
            break;

        default:
            break;
    }
}
