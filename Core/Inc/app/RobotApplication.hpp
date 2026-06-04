#pragma once

#include "commands/Esp32CommandReceiver.h"
#include "detection/DetectionMethodExecutor.hpp"
#include "motion/robot.hpp"

class RobotApplication
{
public:
    struct DebugData
    {
        volatile uint8_t stateCode = 0;
        volatile uint8_t activeDetectionModeCode = 0;
        volatile uint32_t lastDetectionTimeUs = 0;
        volatile uint8_t lastDetectionSectorCode = 0;
        volatile uint8_t lastDetectionStateCode = 0;
        volatile uint8_t lastDetectionTargetReached = 0;
    };

    RobotApplication(Robot& robot,
                     Esp32CommandReceiver& commandReceiver,
                     DetectionMethodExecutor& detectionMethods);

    void update();

    DebugData debug;

private:
    enum class State : uint8_t
    {
        Idle = 0,
        HandlingMotionCommand = 1,
        RunningDetectionMethod = 2
    };

    Robot& robot;
    Esp32CommandReceiver& commandReceiver;
    DetectionMethodExecutor& detectionMethods;
    Esp32Command command;
    Esp32DetectionMode activeDetectionMode;

    void setState(State state);
    void handleCommand(const Esp32Command& receivedCommand);
    void handleMotionCommand(const Esp32Command& receivedCommand);
    void handleDetectionCommand(const Esp32Command& receivedCommand);
};
