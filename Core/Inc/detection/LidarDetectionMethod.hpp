#pragma once

#include "commands/Esp32CommandReceiver.h"
#include "detection/DetectionMethodResult.hpp"
#include "main.h"
#include "motion/robot.hpp"
#include "sensors/RPLidar.h"

class LidarDetectionMethod
{
public:
    LidarDetectionMethod(Robot& robot,
                         RPLidar& lidar,
                         Esp32CommandReceiver& commandReceiver,
                         TIM_HandleTypeDef& timer);

    DetectionMethodResult run();

private:
    enum class State : uint8_t
    {
        Search = 0,
        AlignToObject = 1,
        Approach = 2,
        TargetReached = 3,
        Timeout = 4,
        Cancelled = 5
    };

    enum class Sector : uint8_t
    {
        None = 0,
        Left = 1,
        Center = 2,
        Right = 3
    };

    Robot& robot;
    RPLidar& lidar;
    Esp32CommandReceiver& commandReceiver;
    TIM_HandleTypeDef& timer;
    bool cancellationRequested;

    uint32_t timerUs() const;
    bool runRobotForDuration(uint32_t durationUs);

    bool waitForObject(RPLidarDetectedObject& object);
    bool rotateToObject(const RPLidarDetectedObject& object);
    bool approachTarget(DetectionMethodResult& result);
    void driveTowardObject(const RPLidarDetectedObject& object);
    bool shouldCancel();

    static float absoluteValue(float value);
    static float signedAngleToObject(float lidarAngleDeg);
    static uint32_t rotationTimeForAngle(float angleDeg);
    static int clampPwm(int pwm);
    static uint8_t sectorCodeFromAngle(float signedAngleDeg);
};
