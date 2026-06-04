#pragma once

#include "commands/Esp32CommandReceiver.h"
#include "detection/DetectionMethodResult.hpp"
#include "detection/IrDetectionMethod.hpp"
#include "main.h"
#include "motion/robot.hpp"

class DetectionMethodExecutor
{
public:
    DetectionMethodExecutor(Robot& robot,
                            IrSensors& irSensors,
                            TIM_HandleTypeDef& timer)
        : irMethod(robot, irSensors, timer)
    {
    }

    DetectionMethodResult execute(Esp32DetectionMode mode)
    {
        switch (mode)
        {
            case Esp32DetectionMode::Ir:
                return irMethod.run();

            default:
                return DetectionMethodResult{};
        }
    }

private:
    IrDetectionMethod irMethod;
};
