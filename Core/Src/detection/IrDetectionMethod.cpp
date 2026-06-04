#include "detection/IrDetectionMethod.hpp"

namespace
{
constexpr int IR_ROTATION_PWM = 999;
constexpr int IR_ALIGN_PWM = 999;
constexpr int IR_APPROACH_PWM = 999;
constexpr uint32_t IR_FULL_ROTATION_TIME_US = 4000000U;
constexpr uint32_t IR_APPROACH_TIMEOUT_US = 20000000U;
constexpr uint32_t IR_CONTROL_PERIOD_US = 10000U;
constexpr float IR_TARGET_STOP_CM = 5.0f;
constexpr float IR_CLOSE_OBJECT_VOLTAGE = 2.75f;

bool isSensorAtTargetDistance(const IrSensors::SensorData& sensorData)
{
    return sensorData.distanceCm <= IR_TARGET_STOP_CM ||
        sensorData.filteredCm <= IR_TARGET_STOP_CM ||
        sensorData.voltage >= IR_CLOSE_OBJECT_VOLTAGE;
}
}

IrDetectionMethod::IrDetectionMethod(Robot& robotRef,
                                     IrSensors& irSensorsRef,
                                     TIM_HandleTypeDef& timerRef)
    : robot(robotRef),
      irSensors(irSensorsRef),
      timer(timerRef)
{
}

DetectionMethodResult IrDetectionMethod::run()
{
    DetectionMethodResult result;
    const uint32_t startUs = timerUs();

    result.stateCode = static_cast<uint8_t>(State::Search);
    const FrontScanObservation bestObservation = scanFullRotation();

    result.stateCode = static_cast<uint8_t>(State::ScanComplete);
    result.sectorCode = static_cast<uint8_t>(bestObservation.sector);

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

uint32_t IrDetectionMethod::timerUs() const
{
    return __HAL_TIM_GET_COUNTER(&timer);
}

void IrDetectionMethod::runRobotForDuration(uint32_t durationUs)
{
    const uint32_t startUs = timerUs();

    while ((timerUs() - startUs) < durationUs)
    {
        robot.update();
    }
}

IrDetectionMethod::FrontScanObservation IrDetectionMethod::scanFullRotation()
{
    FrontScanObservation bestObservation;
    const uint32_t rotationStartUs = timerUs();

    robot.rotateLeft(IR_ROTATION_PWM);

    while ((timerUs() - rotationStartUs) < IR_FULL_ROTATION_TIME_US)
    {
        irSensors.update();
        const IrSensors::SensorData centerData =
            irSensors.getSensorData(IrSensors::Sensor::Center);

        if (centerData.voltage > bestObservation.centerVoltage)
        {
            bestObservation.centerVoltage = centerData.voltage;
            bestObservation.rotationTimeUs = timerUs() - rotationStartUs;
            bestObservation.sector = irSensors.getSector();
        }

        robot.update();
    }

    robot.stop();
    robot.update();

    return bestObservation;
}

void IrDetectionMethod::rotateToBestObservation(const FrontScanObservation& observation)
{
    const uint32_t leftTurnTimeUs = observation.rotationTimeUs;
    const uint32_t rightTurnTimeUs = IR_FULL_ROTATION_TIME_US - observation.rotationTimeUs;

    if (leftTurnTimeUs <= rightTurnTimeUs)
    {
        robot.rotateLeft(IR_ROTATION_PWM);
        runRobotForDuration(leftTurnTimeUs);
    }
    else
    {
        robot.rotateRight(IR_ROTATION_PWM);
        runRobotForDuration(rightTurnTimeUs);
    }

    robot.stop();
    robot.update();
}

bool IrDetectionMethod::approachTarget(DetectionMethodResult& result)
{
    const uint32_t approachStartUs = timerUs();

    while ((timerUs() - approachStartUs) < IR_APPROACH_TIMEOUT_US)
    {
        irSensors.update();
        const IrSensors::Sector sector = irSensors.getSector();

        result.sectorCode = static_cast<uint8_t>(sector);

        if (isTargetReached())
        {
            return true;
        }

        driveApproachState(sector);
        runRobotForDuration(IR_CONTROL_PERIOD_US);
    }

    return false;
}

bool IrDetectionMethod::isTargetReached() const
{
    const IrSensors::SensorData leftData =
        irSensors.getSensorData(IrSensors::Sensor::Left);
    const IrSensors::SensorData centerData =
        irSensors.getSensorData(IrSensors::Sensor::Center);
    const IrSensors::SensorData rightData =
        irSensors.getSensorData(IrSensors::Sensor::Right);

    return isSensorAtTargetDistance(leftData) ||
        isSensorAtTargetDistance(centerData) ||
        isSensorAtTargetDistance(rightData);
}

void IrDetectionMethod::driveApproachState(IrSensors::Sector sector)
{
    switch (sector)
    {
        case IrSensors::Sector::Center:
        case IrSensors::Sector::FrontWide:
            robot.forward(IR_APPROACH_PWM);
            break;

        case IrSensors::Sector::Left:
            robot.turnLeft(IR_ALIGN_PWM);
            break;

        case IrSensors::Sector::Right:
            robot.turnRight(IR_ALIGN_PWM);
            break;

        case IrSensors::Sector::None:
        default:
            robot.forward(IR_APPROACH_PWM);
            break;
    }
}
