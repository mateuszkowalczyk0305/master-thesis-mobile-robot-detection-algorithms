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
        Timeout = 3,
        ScanComplete = 4,
        AlignToBest = 5
    };

    struct FrontScanObservation
    {
        float centerVoltage = 0.0f;
        uint32_t rotationTimeUs = 0;
        IrSensors::Sector sector = IrSensors::Sector::None;
    };

    Robot& robot;
    IrSensors& irSensors;
    TIM_HandleTypeDef& timer;

    uint32_t timerUs() const;
    void runRobotForDuration(uint32_t durationUs);
    FrontScanObservation scanFullRotation();
    void rotateToBestObservation(const FrontScanObservation& observation);
    bool approachTarget(DetectionMethodResult& result);
    bool isTargetReached() const;
    void driveApproachState(IrSensors::Sector sector);
};
