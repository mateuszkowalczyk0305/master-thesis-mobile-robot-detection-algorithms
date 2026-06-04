#pragma once

#include "commands/Esp32CommandReceiver.h"
#include "motion/robot.hpp"

class MotionCommandHandler
{
public:
    static void execute(Robot& robot, const Esp32Command& command);
};
