#include "detection/UltrasonicDetectionMethod.hpp"

namespace
{
constexpr int US_ROTATION_PWM = 999;
constexpr int US_APPROACH_PWM = 910;
constexpr int US_STEERING_PWM_DELTA = 70;
constexpr uint32_t US_FULL_ROTATION_TIME_US = 4000000U;
constexpr uint32_t US_APPROACH_TIMEOUT_US = 20000000U;
constexpr uint32_t US_EARLY_STOP_TIME_US = 300000U;
constexpr uint32_t US_FORWARD_STEP_TIME_US = 60000U;
constexpr uint32_t US_SENSOR_SETTLE_US = 25000U;
constexpr uint8_t US_MEDIAN_SAMPLE_COUNT = 3;
constexpr float US_CENTER_DISTANCE_DELTA_CM = 4.0f;
constexpr float US_TARGET_STOP_CM = 5.0f;

void sortDistances(float* distances, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        for (uint8_t j = static_cast<uint8_t>(i + 1); j < count; j++)
        {
            if (distances[j] < distances[i])
            {
                const float temp = distances[i];
                distances[i] = distances[j];
                distances[j] = temp;
            }
        }
    }
}

float absoluteValue(float value)
{
    return (value < 0.0f) ? -value : value;
}

bool shouldStopApproach(float currentDistanceCm,
                        bool hasPreviousDistance,
                        float previousDistanceCm,
                        uint32_t previousMeasurementUs,
                        uint32_t currentMeasurementUs)
{
    if (currentDistanceCm <= US_TARGET_STOP_CM)
    {
        return true;
    }

    if (!hasPreviousDistance || currentDistanceCm >= previousDistanceCm)
    {
        return false;
    }

    const uint32_t elapsedUs = currentMeasurementUs - previousMeasurementUs;

    if (elapsedUs == 0)
    {
        return false;
    }

    const float closingSpeedCmPerUs =
        (previousDistanceCm - currentDistanceCm) / static_cast<float>(elapsedUs);
    const float predictedDistanceCm =
        currentDistanceCm - closingSpeedCmPerUs * static_cast<float>(US_EARLY_STOP_TIME_US);

    return predictedDistanceCm <= US_TARGET_STOP_CM;
}
}

UltrasonicDetectionMethod::UltrasonicDetectionMethod(Robot& robotRef,
                                                     UltrasonicSensor& leftSensorRef,
                                                     UltrasonicSensor& rightSensorRef,
                                                     TIM_HandleTypeDef& timerRef)
    : robot(robotRef),
      leftSensor(leftSensorRef),
      rightSensor(rightSensorRef),
      timer(timerRef)
{
}

DetectionMethodResult UltrasonicDetectionMethod::run()
{
    DetectionMethodResult result;
    const uint32_t startUs = timerUs();

    result.stateCode = static_cast<uint8_t>(State::Search);
    const ScanObservation bestObservation = scanFullRotation();

    if (!bestObservation.valid)
    {
        result.stateCode = static_cast<uint8_t>(State::Timeout);
        robot.stop();
        robot.update();
        result.elapsedUs = timerUs() - startUs;
        return result;
    }

    result.stateCode = static_cast<uint8_t>(State::ScanComplete);
    result.sectorCode = sectorCode(bestObservation.sector);

    result.stateCode = static_cast<uint8_t>(State::AlignToBest);
    rotateToBestObservation(bestObservation);

    result.stateCode = static_cast<uint8_t>(State::Approach);
    result.targetReached = approachTarget(result);

    if (result.targetReached)
    {
        result.stateCode = static_cast<uint8_t>(State::TargetReached);
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

uint32_t UltrasonicDetectionMethod::timerUs() const
{
    return __HAL_TIM_GET_COUNTER(&timer);
}

void UltrasonicDetectionMethod::runRobotForDuration(uint32_t durationUs)
{
    const uint32_t startUs = timerUs();

    while ((timerUs() - startUs) < durationUs)
    {
        robot.update();
    }
}

bool UltrasonicDetectionMethod::measureMedian(UltrasonicSensor& sensor,
                                              float& medianDistanceCm)
{
    float distances[US_MEDIAN_SAMPLE_COUNT] = {};
    uint8_t validCount = 0;

    for (uint8_t sample = 0; sample < US_MEDIAN_SAMPLE_COUNT; sample++)
    {
        float distanceCm = 0.0f;

        if (sensor.measure(distanceCm))
        {
            distances[validCount] = distanceCm;
            validCount++;
        }

        runRobotForDuration(US_SENSOR_SETTLE_US);
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

UltrasonicDetectionMethod::StereoMeasurement UltrasonicDetectionMethod::measureStereoMedian()
{
    StereoMeasurement measurement;

    measurement.leftValid = measureMedian(leftSensor, measurement.leftDistanceCm);
    measurement.rightValid = measureMedian(rightSensor, measurement.rightDistanceCm);
    measurement.sector = classifySector(measurement);

    return measurement;
}

UltrasonicDetectionMethod::ScanObservation UltrasonicDetectionMethod::scanFullRotation()
{
    ScanObservation bestObservation;
    const uint32_t rotationStartUs = timerUs();

    robot.rotateLeft(US_ROTATION_PWM);

    while ((timerUs() - rotationStartUs) < US_FULL_ROTATION_TIME_US)
    {
        const StereoMeasurement measurement = measureStereoMedian();

        if (hasValidDistance(measurement))
        {
            const float closestDistance = closestDistanceCm(measurement);

            if (!bestObservation.valid || closestDistance < bestObservation.distanceCm)
            {
                bestObservation.valid = true;
                bestObservation.distanceCm = closestDistance;
                bestObservation.rotationTimeUs = timerUs() - rotationStartUs;
                bestObservation.sector = measurement.sector;
            }
        }

        robot.update();
    }

    robot.stop();
    robot.update();

    return bestObservation;
}

void UltrasonicDetectionMethod::rotateToBestObservation(const ScanObservation& observation)
{
    const uint32_t leftTurnTimeUs = observation.rotationTimeUs;
    const uint32_t rightTurnTimeUs = US_FULL_ROTATION_TIME_US - observation.rotationTimeUs;

    if (leftTurnTimeUs <= rightTurnTimeUs)
    {
        robot.rotateLeft(US_ROTATION_PWM);
        runRobotForDuration(leftTurnTimeUs);
    }
    else
    {
        robot.rotateRight(US_ROTATION_PWM);
        runRobotForDuration(rightTurnTimeUs);
    }

    robot.stop();
    robot.update();
}

bool UltrasonicDetectionMethod::approachTarget(DetectionMethodResult& result)
{
    const uint32_t approachStartUs = timerUs();
    uint32_t previousMeasurementUs = approachStartUs;
    float previousDistanceCm = 0.0f;
    bool hasPreviousDistance = false;

    robot.forward(US_APPROACH_PWM);

    while ((timerUs() - approachStartUs) < US_APPROACH_TIMEOUT_US)
    {
        const StereoMeasurement measurement = measureStereoMedian();
        const uint32_t measurementUs = timerUs();
        result.sectorCode = sectorCode(measurement.sector);

        if (!hasValidDistance(measurement))
        {
            robot.forward(US_APPROACH_PWM);
            runRobotForDuration(US_FORWARD_STEP_TIME_US);
            continue;
        }

        const float currentDistanceCm = closestDistanceCm(measurement);

        if (shouldStopApproach(currentDistanceCm,
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

        driveApproachState(measurement);
        runRobotForDuration(US_FORWARD_STEP_TIME_US);
    }

    return false;
}

void UltrasonicDetectionMethod::driveApproachState(const StereoMeasurement& measurement)
{
    int leftSpeed = US_APPROACH_PWM;
    int rightSpeed = US_APPROACH_PWM;

    if (measurement.leftValid && measurement.rightValid)
    {
        const float distanceDifference =
            measurement.leftDistanceCm - measurement.rightDistanceCm;

        if (absoluteValue(distanceDifference) > US_CENTER_DISTANCE_DELTA_CM)
        {
            if (distanceDifference < 0.0f)
            {
                leftSpeed -= US_STEERING_PWM_DELTA;
                rightSpeed += US_STEERING_PWM_DELTA;
            }
            else
            {
                leftSpeed += US_STEERING_PWM_DELTA;
                rightSpeed -= US_STEERING_PWM_DELTA;
            }
        }
    }
    else if (measurement.leftValid)
    {
        leftSpeed -= US_STEERING_PWM_DELTA;
        rightSpeed += US_STEERING_PWM_DELTA;
    }
    else if (measurement.rightValid)
    {
        leftSpeed += US_STEERING_PWM_DELTA;
        rightSpeed -= US_STEERING_PWM_DELTA;
    }

    robot.forwardDifferential(leftSpeed, rightSpeed);
}

UltrasonicDetectionMethod::Sector UltrasonicDetectionMethod::classifySector(
    const StereoMeasurement& measurement)
{
    if (measurement.leftValid && measurement.rightValid)
    {
        const float distanceDifference =
            measurement.leftDistanceCm - measurement.rightDistanceCm;

        if (absoluteValue(distanceDifference) <= US_CENTER_DISTANCE_DELTA_CM)
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

bool UltrasonicDetectionMethod::hasValidDistance(const StereoMeasurement& measurement)
{
    return measurement.leftValid || measurement.rightValid;
}

float UltrasonicDetectionMethod::closestDistanceCm(const StereoMeasurement& measurement)
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

uint8_t UltrasonicDetectionMethod::sectorCode(Sector sector)
{
    return static_cast<uint8_t>(sector);
}
