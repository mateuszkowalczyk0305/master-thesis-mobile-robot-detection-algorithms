#include "detection/FusionDetectionMethod.hpp"

namespace
{
constexpr int FUSION_ROTATION_PWM = 999;
constexpr int FUSION_FINE_ALIGN_PWM = 900;
constexpr int FUSION_APPROACH_PWM = 910;
constexpr int FUSION_IR_STEERING_PWM_DELTA = 90;
constexpr int FUSION_LIDAR_MAX_STEERING_DELTA = 90;
constexpr int FUSION_MIN_DRIVE_PWM = 800;
constexpr int FUSION_MAX_DRIVE_PWM = 999;

constexpr uint32_t FUSION_FULL_ROTATION_TIME_US = 4000000U;
constexpr uint32_t FUSION_LIDAR_WAIT_TIMEOUT_US = 3000000U;
constexpr uint32_t FUSION_IR_FINE_ALIGN_TIMEOUT_US = 1200000U;
constexpr uint32_t FUSION_IR_NO_OBJECT_TIMEOUT_US = 250000U;
constexpr uint32_t FUSION_APPROACH_TIMEOUT_US = 20000000U;
constexpr uint32_t FUSION_CONTROL_PERIOD_US = 60000U;
constexpr uint32_t FUSION_IR_CONTROL_PERIOD_US = 40000U;
constexpr uint32_t FUSION_US_SENSOR_SETTLE_US = 25000U;
constexpr uint32_t FUSION_US_EARLY_STOP_TIME_US = 300000U;

constexpr uint8_t FUSION_US_MEDIAN_SAMPLE_COUNT = 3;
constexpr uint8_t FUSION_IR_CENTER_CONFIRMATIONS = 3;

constexpr float FUSION_ANGLE_DEADBAND_DEG = 5.0f;
constexpr float FUSION_LIDAR_STEERING_KP = 3.5f;
constexpr float FUSION_US_CENTER_DISTANCE_DELTA_CM = 4.0f;
constexpr float FUSION_US_TARGET_STOP_CM = 5.0f;
}

FusionDetectionMethod::FusionDetectionMethod(Robot& robotRef,
                                             IrSensors& irSensorsRef,
                                             UltrasonicSensor& leftUltrasonicRef,
                                             UltrasonicSensor& rightUltrasonicRef,
                                             RPLidar& lidarRef,
                                             Esp32CommandReceiver& commandReceiverRef,
                                             TIM_HandleTypeDef& timerRef)
    : robot(robotRef),
      irSensors(irSensorsRef),
      leftUltrasonic(leftUltrasonicRef),
      rightUltrasonic(rightUltrasonicRef),
      lidar(lidarRef),
      commandReceiver(commandReceiverRef),
      timer(timerRef),
      cancellationRequested(false)
{
}

DetectionMethodResult FusionDetectionMethod::run()
{
    DetectionMethodResult result;
    const uint32_t startUs = timerUs();
    cancellationRequested = false;

    RPLidarDetectedObject lidarObject;

    result.stateCode = static_cast<uint8_t>(State::SearchLidar);

    if (!waitForLidarObject(lidarObject))
    {
        result.stateCode = cancellationRequested ?
            static_cast<uint8_t>(State::Cancelled) :
            static_cast<uint8_t>(State::Timeout);
        robot.stop();
        robot.update();
        result.elapsedUs = timerUs() - startUs;
        return result;
    }

    result.sectorCode = sectorCodeFromAngle(signedAngleToObject(lidarObject.angleDeg));

    result.stateCode = static_cast<uint8_t>(State::AlignLidar);

    if (!rotateToLidarObject(lidarObject))
    {
        result.stateCode = static_cast<uint8_t>(State::Cancelled);
        robot.stop();
        robot.update();
        result.elapsedUs = timerUs() - startUs;
        return result;
    }

    result.stateCode = static_cast<uint8_t>(State::FineAlignIr);

    if (!fineAlignWithIr(result))
    {
        result.stateCode = static_cast<uint8_t>(State::Cancelled);
        robot.stop();
        robot.update();
        result.elapsedUs = timerUs() - startUs;
        return result;
    }

    result.stateCode = static_cast<uint8_t>(State::Approach);
    result.targetReached = approachTarget(result);

    if (result.targetReached)
    {
        result.stateCode = static_cast<uint8_t>(State::TargetReached);
    }
    else if (cancellationRequested)
    {
        result.stateCode = static_cast<uint8_t>(State::Cancelled);
    }
    else
    {
        result.stateCode = static_cast<uint8_t>(State::Timeout);
    }

    robot.stop();
    robot.update();
    result.elapsedUs = timerUs() - startUs;

    return result;
}

uint32_t FusionDetectionMethod::timerUs() const
{
    return __HAL_TIM_GET_COUNTER(&timer);
}

bool FusionDetectionMethod::runRobotForDuration(uint32_t durationUs)
{
    const uint32_t startUs = timerUs();

    while ((timerUs() - startUs) < durationUs)
    {
        if (shouldCancel())
        {
            return false;
        }

        robot.update();
    }

    return true;
}

bool FusionDetectionMethod::waitForLidarObject(RPLidarDetectedObject& object)
{
    const uint32_t waitStartUs = timerUs();

    while ((timerUs() - waitStartUs) < FUSION_LIDAR_WAIT_TIMEOUT_US)
    {
        if (shouldCancel())
        {
            return false;
        }

        if (lidar.getDetectedObject(object))
        {
            return true;
        }

        robot.update();
    }

    return false;
}

bool FusionDetectionMethod::rotateToLidarObject(const RPLidarDetectedObject& object)
{
    if (shouldCancel())
    {
        return false;
    }

    const float angleDeg = signedAngleToObject(object.angleDeg);
    const uint32_t rotationTimeUs = rotationTimeForAngle(angleDeg);

    if (rotationTimeUs == 0U)
    {
        return true;
    }

    if (angleDeg < 0.0f)
    {
        robot.rotateLeft(FUSION_ROTATION_PWM);
    }
    else
    {
        robot.rotateRight(FUSION_ROTATION_PWM);
    }

    if (!runRobotForDuration(rotationTimeUs))
    {
        return false;
    }

    robot.stop();
    robot.update();

    return true;
}

bool FusionDetectionMethod::fineAlignWithIr(DetectionMethodResult& result)
{
    const uint32_t alignStartUs = timerUs();
    uint8_t centeredSamples = 0;
    bool irObjectSeen = false;

    while ((timerUs() - alignStartUs) < FUSION_IR_FINE_ALIGN_TIMEOUT_US)
    {
        if (shouldCancel())
        {
            return false;
        }

        irSensors.update();
        const IrSensors::Sector sector = irSensors.getSector();
        result.sectorCode = sectorCodeFromIr(sector);

        switch (sector)
        {
            case IrSensors::Sector::Left:
                irObjectSeen = true;
                centeredSamples = 0;
                robot.turnLeft(FUSION_FINE_ALIGN_PWM);
                break;

            case IrSensors::Sector::Right:
                irObjectSeen = true;
                centeredSamples = 0;
                robot.turnRight(FUSION_FINE_ALIGN_PWM);
                break;

            case IrSensors::Sector::Center:
            case IrSensors::Sector::FrontWide:
                irObjectSeen = true;
                centeredSamples++;
                robot.stop();
                robot.update();

                if (centeredSamples >= FUSION_IR_CENTER_CONFIRMATIONS)
                {
                    return true;
                }
                break;

            case IrSensors::Sector::None:
            default:
                robot.stop();
                robot.update();

                if (!irObjectSeen &&
                    (timerUs() - alignStartUs) >= FUSION_IR_NO_OBJECT_TIMEOUT_US)
                {
                    return true;
                }
                break;
        }

        if (!runRobotForDuration(FUSION_IR_CONTROL_PERIOD_US))
        {
            return false;
        }
    }

    robot.stop();
    robot.update();

    return true;
}

bool FusionDetectionMethod::approachTarget(DetectionMethodResult& result)
{
    const uint32_t approachStartUs = timerUs();
    uint32_t previousMeasurementUs = approachStartUs;
    float previousDistanceCm = 0.0f;
    bool hasPreviousDistance = false;

    while ((timerUs() - approachStartUs) < FUSION_APPROACH_TIMEOUT_US)
    {
        if (shouldCancel())
        {
            return false;
        }

        const StereoMeasurement ultrasonicMeasurement = measureStereoMedian();
        const uint32_t measurementUs = timerUs();

        if (cancellationRequested)
        {
            return false;
        }

        irSensors.update();
        const IrSensors::Sector irSector = irSensors.getSector();

        RPLidarDetectedObject lidarObject;
        const bool hasLidarObject = lidar.getDetectedObject(lidarObject);

        if (irSector != IrSensors::Sector::None)
        {
            result.sectorCode = sectorCodeFromIr(irSector);
        }
        else if (hasLidarObject)
        {
            result.sectorCode = sectorCodeFromAngle(
                signedAngleToObject(lidarObject.angleDeg)
            );
        }
        else
        {
            result.sectorCode = sectorCodeFromUltrasonic(ultrasonicMeasurement.sector);
        }

        if (hasValidUltrasonicDistance(ultrasonicMeasurement))
        {
            const float currentDistanceCm =
                closestUltrasonicDistanceCm(ultrasonicMeasurement);

            if (shouldStopOnUltrasonic(currentDistanceCm,
                                       hasPreviousDistance,
                                       previousDistanceCm,
                                       previousMeasurementUs,
                                       measurementUs))
            {
                return true;
            }

            previousDistanceCm = currentDistanceCm;
            previousMeasurementUs = measurementUs;
            hasPreviousDistance = true;
        }

        driveFusionApproach(hasLidarObject, lidarObject, irSector);

        if (!runRobotForDuration(FUSION_CONTROL_PERIOD_US))
        {
            return false;
        }
    }

    return false;
}

void FusionDetectionMethod::driveFusionApproach(bool hasLidarObject,
                                                const RPLidarDetectedObject& lidarObject,
                                                IrSensors::Sector irSector)
{
    int leftSpeed = FUSION_APPROACH_PWM;
    int rightSpeed = FUSION_APPROACH_PWM;

    if (hasLidarObject)
    {
        int lidarCorrection = 0;
        const float signedAngleDeg = signedAngleToObject(lidarObject.angleDeg);

        if (absoluteValue(signedAngleDeg) > FUSION_ANGLE_DEADBAND_DEG)
        {
            lidarCorrection = static_cast<int>(
                signedAngleDeg * FUSION_LIDAR_STEERING_KP
            );

            if (lidarCorrection > FUSION_LIDAR_MAX_STEERING_DELTA)
            {
                lidarCorrection = FUSION_LIDAR_MAX_STEERING_DELTA;
            }
            else if (lidarCorrection < -FUSION_LIDAR_MAX_STEERING_DELTA)
            {
                lidarCorrection = -FUSION_LIDAR_MAX_STEERING_DELTA;
            }
        }

        leftSpeed += lidarCorrection;
        rightSpeed -= lidarCorrection;
    }

    switch (irSector)
    {
        case IrSensors::Sector::Left:
            leftSpeed -= FUSION_IR_STEERING_PWM_DELTA;
            rightSpeed += FUSION_IR_STEERING_PWM_DELTA;
            break;

        case IrSensors::Sector::Right:
            leftSpeed += FUSION_IR_STEERING_PWM_DELTA;
            rightSpeed -= FUSION_IR_STEERING_PWM_DELTA;
            break;

        case IrSensors::Sector::Center:
        case IrSensors::Sector::FrontWide:
        case IrSensors::Sector::None:
        default:
            break;
    }

    robot.forwardDifferential(clampPwm(leftSpeed), clampPwm(rightSpeed));
}

bool FusionDetectionMethod::measureMedian(UltrasonicSensor& sensor,
                                          float& medianDistanceCm)
{
    float distances[FUSION_US_MEDIAN_SAMPLE_COUNT] = {};
    uint8_t validCount = 0;

    for (uint8_t sampleIndex = 0;
         sampleIndex < FUSION_US_MEDIAN_SAMPLE_COUNT;
         sampleIndex++)
    {
        float distanceCm = 0.0f;

        if (sensor.measure(distanceCm))
        {
            distances[validCount] = distanceCm;
            validCount++;
        }

        if (!runRobotForDuration(FUSION_US_SENSOR_SETTLE_US))
        {
            return false;
        }
    }

    if (validCount == 0)
    {
        return false;
    }

    sortDistances(distances, validCount);

    if (validCount == 2)
    {
        medianDistanceCm = (distances[0] + distances[1]) * 0.5f;
    }
    else
    {
        medianDistanceCm = distances[validCount / 2U];
    }

    return true;
}

FusionDetectionMethod::StereoMeasurement FusionDetectionMethod::measureStereoMedian()
{
    StereoMeasurement measurement;

    measurement.leftValid = measureMedian(leftUltrasonic, measurement.leftDistanceCm);
    measurement.rightValid = measureMedian(rightUltrasonic, measurement.rightDistanceCm);
    measurement.sector = classifyUltrasonicSector(measurement);

    return measurement;
}

bool FusionDetectionMethod::shouldStopOnUltrasonic(float currentDistanceCm,
                                                   bool hasPreviousDistance,
                                                   float previousDistanceCm,
                                                   uint32_t previousMeasurementUs,
                                                   uint32_t currentMeasurementUs) const
{
    if (currentDistanceCm <= FUSION_US_TARGET_STOP_CM)
    {
        return true;
    }

    if (!hasPreviousDistance || currentDistanceCm >= previousDistanceCm)
    {
        return false;
    }

    const uint32_t elapsedUs = currentMeasurementUs - previousMeasurementUs;

    if (elapsedUs == 0U)
    {
        return false;
    }

    const float closingSpeedCmPerUs =
        (previousDistanceCm - currentDistanceCm) / static_cast<float>(elapsedUs);
    const float predictedDistanceCm =
        currentDistanceCm -
        closingSpeedCmPerUs * static_cast<float>(FUSION_US_EARLY_STOP_TIME_US);

    return predictedDistanceCm <= FUSION_US_TARGET_STOP_CM;
}

bool FusionDetectionMethod::shouldCancel()
{
    if (cancellationRequested)
    {
        return true;
    }

    if (!commandReceiver.consumeStopMotionCommand())
    {
        return false;
    }

    cancellationRequested = true;
    robot.stop();
    robot.update();
    commandReceiver.sendMotionOk();

    return true;
}

void FusionDetectionMethod::sortDistances(float* distances, uint8_t count)
{
    for (uint8_t firstIndex = 0; firstIndex < count; firstIndex++)
    {
        for (uint8_t secondIndex = static_cast<uint8_t>(firstIndex + 1U);
             secondIndex < count;
             secondIndex++)
        {
            if (distances[secondIndex] < distances[firstIndex])
            {
                const float temp = distances[firstIndex];
                distances[firstIndex] = distances[secondIndex];
                distances[secondIndex] = temp;
            }
        }
    }
}

float FusionDetectionMethod::absoluteValue(float value)
{
    return (value < 0.0f) ? -value : value;
}

float FusionDetectionMethod::signedAngleToObject(float lidarAngleDeg)
{
    while (lidarAngleDeg > 180.0f)
    {
        lidarAngleDeg -= 360.0f;
    }

    while (lidarAngleDeg < -180.0f)
    {
        lidarAngleDeg += 360.0f;
    }

    return lidarAngleDeg;
}

uint32_t FusionDetectionMethod::rotationTimeForAngle(float angleDeg)
{
    const float angleMagnitudeDeg = absoluteValue(angleDeg);

    return static_cast<uint32_t>(
        (angleMagnitudeDeg * static_cast<float>(FUSION_FULL_ROTATION_TIME_US)) /
        360.0f
    );
}

int FusionDetectionMethod::clampPwm(int pwm)
{
    if (pwm < FUSION_MIN_DRIVE_PWM)
    {
        return FUSION_MIN_DRIVE_PWM;
    }

    if (pwm > FUSION_MAX_DRIVE_PWM)
    {
        return FUSION_MAX_DRIVE_PWM;
    }

    return pwm;
}

FusionDetectionMethod::Sector FusionDetectionMethod::classifyUltrasonicSector(
    const StereoMeasurement& measurement)
{
    if (measurement.leftValid && measurement.rightValid)
    {
        const float distanceDifference =
            measurement.leftDistanceCm - measurement.rightDistanceCm;

        if (absoluteValue(distanceDifference) <= FUSION_US_CENTER_DISTANCE_DELTA_CM)
        {
            return Sector::Center;
        }

        return (measurement.leftDistanceCm < measurement.rightDistanceCm) ?
            Sector::Left : Sector::Right;
    }

    if (measurement.leftValid)
    {
        return Sector::Left;
    }

    if (measurement.rightValid)
    {
        return Sector::Right;
    }

    return Sector::None;
}

bool FusionDetectionMethod::hasValidUltrasonicDistance(
    const StereoMeasurement& measurement)
{
    return measurement.leftValid || measurement.rightValid;
}

float FusionDetectionMethod::closestUltrasonicDistanceCm(
    const StereoMeasurement& measurement)
{
    if (measurement.leftValid && measurement.rightValid)
    {
        return (measurement.leftDistanceCm < measurement.rightDistanceCm) ?
            measurement.leftDistanceCm : measurement.rightDistanceCm;
    }

    if (measurement.leftValid)
    {
        return measurement.leftDistanceCm;
    }

    return measurement.rightDistanceCm;
}

uint8_t FusionDetectionMethod::sectorCodeFromIr(IrSensors::Sector sector)
{
    switch (sector)
    {
        case IrSensors::Sector::Left:
            return static_cast<uint8_t>(Sector::Left);

        case IrSensors::Sector::Right:
            return static_cast<uint8_t>(Sector::Right);

        case IrSensors::Sector::Center:
        case IrSensors::Sector::FrontWide:
            return static_cast<uint8_t>(Sector::Center);

        case IrSensors::Sector::None:
        default:
            return static_cast<uint8_t>(Sector::None);
    }
}

uint8_t FusionDetectionMethod::sectorCodeFromAngle(float signedAngleDeg)
{
    if (absoluteValue(signedAngleDeg) <= FUSION_ANGLE_DEADBAND_DEG)
    {
        return static_cast<uint8_t>(Sector::Center);
    }

    if (signedAngleDeg < 0.0f)
    {
        return static_cast<uint8_t>(Sector::Left);
    }

    return static_cast<uint8_t>(Sector::Right);
}

uint8_t FusionDetectionMethod::sectorCodeFromUltrasonic(Sector sector)
{
    return static_cast<uint8_t>(sector);
}
