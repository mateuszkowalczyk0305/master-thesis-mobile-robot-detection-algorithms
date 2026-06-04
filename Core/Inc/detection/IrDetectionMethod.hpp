#pragma once

#include "sensors/IrSensors.h"
#include "detection/DetectionMethodResult.hpp"
#include "main.h"
#include "motion/robot.hpp"

class IrDetectionMethod
{
public:
    IrDetectionMethod(Robot& robot,
                      IrSensors& irSensors,
                      TIM_HandleTypeDef& timer);

    DetectionMethodResult run();

private:
    enum class State : uint8_t
    {
        Search = 0,
        Approach = 1,
        TargetReached = 2,
        Timeout = 3
    };

    Robot& robot;
    IrSensors& irSensors;
    TIM_HandleTypeDef& timer;

    uint32_t timerUs() const;
    void runRobotForControlPeriod();
    bool isTargetReached(IrSensors::Sector sector) const;
    State nextState(State state, IrSensors::Sector sector, bool targetReached) const;
    void driveSearchState();
    void driveApproachState(IrSensors::Sector sector);
    void driveState(State state, IrSensors::Sector sector);
};
