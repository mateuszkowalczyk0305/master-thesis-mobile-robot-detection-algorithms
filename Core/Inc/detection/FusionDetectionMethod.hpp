#pragma once

#include "commands/Esp32CommandReceiver.h"
#include "detection/DetectionMethodResult.hpp"
#include "main.h"
#include "motion/robot.hpp"
#include "sensors/IrSensors.h"
#include "sensors/RPLidar.h"
#include "sensors/UltrasonicSensor.h"

class FusionDetectionMethod
{
public:
    FusionDetectionMethod(Robot& robot,
                          IrSensors& irSensors,
                          UltrasonicSensor& leftUltrasonic,
                          UltrasonicSensor& rightUltrasonic,
                          RPLidar& lidar,
                          Esp32CommandReceiver& commandReceiver,
                          TIM_HandleTypeDef& timer);

    DetectionMethodResult run();

private:
    enum class State : uint8_t
    {
        SearchLidar = 0,
        AlignLidar = 1,
        FineAlignIr = 2,
        Approach = 3,
        TargetReached = 4,
        Timeout = 5,
        Cancelled = 6
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

    Robot& robot;
    IrSensors& irSensors;
    UltrasonicSensor& leftUltrasonic;
    UltrasonicSensor& rightUltrasonic;
    RPLidar& lidar;
    Esp32CommandReceiver& commandReceiver;
    TIM_HandleTypeDef& timer;
    bool cancellationRequested;

    uint32_t timerUs() const;
    bool runRobotForDuration(uint32_t durationUs);

    bool waitForLidarObject(RPLidarDetectedObject& object);
    bool rotateToLidarObject(const RPLidarDetectedObject& object);
    bool fineAlignWithIr(DetectionMethodResult& result);
    bool approachTarget(DetectionMethodResult& result);
    void driveFusionApproach(bool hasLidarObject,
                             const RPLidarDetectedObject& lidarObject,
                             IrSensors::Sector irSector);

    bool measureMedian(UltrasonicSensor& sensor, float& medianDistanceCm);
    StereoMeasurement measureStereoMedian();
    bool shouldStopOnUltrasonic(float currentDistanceCm,
                                bool hasPreviousDistance,
                                float previousDistanceCm,
                                uint32_t previousMeasurementUs,
                                uint32_t currentMeasurementUs) const;
    bool shouldCancel();

    static void sortDistances(float* distances, uint8_t count);
    static float absoluteValue(float value);
    static float signedAngleToObject(float lidarAngleDeg);
    static uint32_t rotationTimeForAngle(float angleDeg);
    static int clampPwm(int pwm);
    static Sector classifyUltrasonicSector(const StereoMeasurement& measurement);
    static bool hasValidUltrasonicDistance(const StereoMeasurement& measurement);
    static float closestUltrasonicDistanceCm(const StereoMeasurement& measurement);
    static uint8_t sectorCodeFromIr(IrSensors::Sector sector);
    static uint8_t sectorCodeFromAngle(float signedAngleDeg);
    static uint8_t sectorCodeFromUltrasonic(Sector sector);
};
