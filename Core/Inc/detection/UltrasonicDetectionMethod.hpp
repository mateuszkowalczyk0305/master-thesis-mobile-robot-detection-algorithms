#pragma once

#include "detection/DetectionMethodResult.hpp"
#include "main.h"
#include "motion/robot.hpp"
#include "sensors/UltrasonicSensor.h"

/*
 * Ultrasonic method assumptions:
 * - HC-SR04 distance comes from echo time of flight inside UltrasonicSensor.
 * - Measurements without valid echo are rejected by UltrasonicSensor::measure().
 * - Median of repeated samples limits single bad readings.
 * - Comparing left and right distances estimates the object side.
 */
class UltrasonicDetectionMethod
{
public:
    UltrasonicDetectionMethod(Robot& robot,
                              UltrasonicSensor& leftSensor,
                              UltrasonicSensor& rightSensor,
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

    enum class Sector : uint8_t
    {
        None = 0,
        Left = 1,
        Center = 2,
        Right = 3
    };

    struct StereoMeasurement
    {
        bool leftValid = false;
        bool rightValid = false;
        float leftDistanceCm = 0.0f;
        float rightDistanceCm = 0.0f;
        Sector sector = Sector::None;
    };

    struct ScanObservation
    {
        bool valid = false;
        float distanceCm = 0.0f;
        uint32_t rotationTimeUs = 0;
        Sector sector = Sector::None;
    };

    Robot& robot;
    UltrasonicSensor& leftSensor;
    UltrasonicSensor& rightSensor;
    TIM_HandleTypeDef& timer;

    uint32_t timerUs() const;
    void runRobotForDuration(uint32_t durationUs);
    bool measureMedian(UltrasonicSensor& sensor, float& medianDistanceCm);
    StereoMeasurement measureStereoMedian();
    ScanObservation scanFullRotation();
    void rotateToBestObservation(const ScanObservation& observation);
    bool approachTarget(DetectionMethodResult& result);
    void driveApproachState(const StereoMeasurement& measurement);

    static Sector classifySector(const StereoMeasurement& measurement);
    static bool hasValidDistance(const StereoMeasurement& measurement);
    static float closestDistanceCm(const StereoMeasurement& measurement);
    static uint8_t sectorCode(Sector sector);
};
