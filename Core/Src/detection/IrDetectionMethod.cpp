#include "detection/IrDetectionMethod.hpp"

namespace
{
constexpr int IR_DETECTION_PWM = 850;
constexpr float IR_TARGET_STOP_CM = 20.0f;
constexpr uint32_t IR_DETECTION_TIMEOUT_US = 10000000U;
constexpr uint32_t IR_CONTROL_PERIOD_US = 10000U;

float minDistance(float firstDistanceCm, float secondDistanceCm)
{
    return (firstDistanceCm < secondDistanceCm) ? firstDistanceCm : secondDistanceCm;
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
    State state = State::Search;
    const uint32_t startUs = timerUs();

    while ((timerUs() - startUs) < IR_DETECTION_TIMEOUT_US)
    {
        irSensors.update();
        const IrSensors::Sector sector = irSensors.getSector();
        state = nextState(state, sector, isTargetReached(sector));

        result.sectorCode = static_cast<uint8_t>(sector);
        result.stateCode = static_cast<uint8_t>(state);

        if (state == State::TargetReached)
        {
            result.targetReached = true;
            break;
        }

        driveState(state, sector);
        runRobotForControlPeriod();
    }

    if (!result.targetReached)
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

void IrDetectionMethod::runRobotForControlPeriod()
{
    const uint32_t periodStartUs = timerUs();

    while ((timerUs() - periodStartUs) < IR_CONTROL_PERIOD_US)
    {
        robot.update();
    }
}

bool IrDetectionMethod::isTargetReached(IrSensors::Sector sector) const
{
    const IrSensors::SensorData leftData =
        irSensors.getSensorData(IrSensors::Sensor::Left);
    const IrSensors::SensorData centerData =
        irSensors.getSensorData(IrSensors::Sensor::Center);
    const IrSensors::SensorData rightData =
        irSensors.getSensorData(IrSensors::Sensor::Right);

    const float closestFrontDistanceCm =
        minDistance(centerData.filteredCm,
                    minDistance(leftData.filteredCm, rightData.filteredCm));

    if (sector == IrSensors::Sector::Center)
    {
        return centerData.filteredCm <= IR_TARGET_STOP_CM;
    }

    if (sector == IrSensors::Sector::FrontWide)
    {
        return closestFrontDistanceCm <= IR_TARGET_STOP_CM;
    }

    return false;
}

IrDetectionMethod::State IrDetectionMethod::nextState(State state,
                                                      IrSensors::Sector sector,
                                                      bool targetReached) const
{
    if (targetReached)
    {
        return State::TargetReached;
    }

    if (sector == IrSensors::Sector::None)
    {
        return State::Search;
    }

    if (state == State::Search)
    {
        return State::Approach;
    }

    return state;
}

void IrDetectionMethod::driveSearchState()
{
    robot.rotateLeft(IR_DETECTION_PWM);
}

void IrDetectionMethod::driveApproachState(IrSensors::Sector sector)
{
    switch (sector)
    {
        case IrSensors::Sector::Center:
        case IrSensors::Sector::FrontWide:
            robot.forward(IR_DETECTION_PWM);
            break;

        case IrSensors::Sector::Left:
            robot.rotateLeft(IR_DETECTION_PWM);
            break;

        case IrSensors::Sector::Right:
            robot.rotateRight(IR_DETECTION_PWM);
            break;

        case IrSensors::Sector::None:
        default:
            robot.stop();
            break;
    }
}

void IrDetectionMethod::driveState(State state, IrSensors::Sector sector)
{
    switch (state)
    {
        case State::Search:
            driveSearchState();
            break;

        case State::Approach:
            driveApproachState(sector);
            break;

        case State::TargetReached:
        case State::Timeout:
        default:
            robot.stop();
            break;
    }
}
