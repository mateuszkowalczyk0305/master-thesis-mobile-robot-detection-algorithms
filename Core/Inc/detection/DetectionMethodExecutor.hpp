#pragma once

#include "commands/Esp32CommandReceiver.h"
#include "detection/DetectionMethodResult.hpp"
#include "detection/IrDetectionMethod.hpp"
#include "detection/UltrasonicDetectionMethod.hpp"
#include "main.h"
#include "motion/robot.hpp"
#include "sensors/UltrasonicSensor.h"

class DetectionMethodExecutor
{
public:
    DetectionMethodExecutor(Robot& robot,
                            IrSensors& irSensors,
                            UltrasonicSensor& leftUltrasonic,
                            UltrasonicSensor& rightUltrasonic,
                            TIM_HandleTypeDef& timer)
        : irMethod(robot, irSensors, timer),
          ultrasonicMethod(robot, leftUltrasonic, rightUltrasonic, timer)
    {
    }

    DetectionMethodResult execute(Esp32DetectionMode mode)
    {
        switch (mode)
        {
            case Esp32DetectionMode::Ir:
                return irMethod.run();

            case Esp32DetectionMode::Ultrasonic:
                return ultrasonicMethod.run();

            default:
                return DetectionMethodResult{};
        }
    }

private:
    IrDetectionMethod irMethod;
    UltrasonicDetectionMethod ultrasonicMethod;
};
