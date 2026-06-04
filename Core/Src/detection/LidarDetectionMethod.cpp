#include "detection/LidarDetectionMethod.hpp"

namespace
{
constexpr int LIDAR_ROTATION_PWM = 999;
constexpr int LIDAR_APPROACH_PWM = 910;
constexpr int LIDAR_MAX_STEERING_DELTA = 120;
constexpr int LIDAR_MIN_DRIVE_PWM = 800;
constexpr int LIDAR_MAX_DRIVE_PWM = 999;

constexpr uint32_t LIDAR_FULL_ROTATION_TIME_US = 4000000U;
constexpr uint32_t LIDAR_OBJECT_WAIT_TIMEOUT_US = 3000000U;
constexpr uint32_t LIDAR_APPROACH_TIMEOUT_US = 20000000U;
constexpr uint32_t LIDAR_CONTROL_PERIOD_US = 60000U;
constexpr uint32_t LIDAR_FINAL_FORWARD_TIME_US = 300000U;

constexpr float LIDAR_TARGET_STOP_MM = 180.0f;
constexpr float LIDAR_ANGLE_DEADBAND_DEG = 5.0f;
constexpr float LIDAR_STEERING_KP = 4.0f;
}

LidarDetectionMethod::LidarDetectionMethod(Robot& robotRef,
                                           RPLidar& lidarRef,
                             Esp32CommandReceiver& commandReceiverRef,
                                           TIM_HandleTypeDef& timerRef)
    : robot(robotRef),
      lidar(lidarRef),
    commandReceiver(commandReceiverRef),
    timer(timerRef),
    cancellationRequested(false)
{
}

DetectionMethodResult LidarDetectionMethod::run()
{
    DetectionMethodResult result;
    const uint32_t startUs = timerUs();
    cancellationRequested = false;

    RPLidarDetectedObject object;

    result.stateCode = static_cast<uint8_t>(State::Search);

    if (!waitForObject(object))
    {
        result.stateCode = cancellationRequested ?
            static_cast<uint8_t>(State::Cancelled) :
            static_cast<uint8_t>(State::Timeout);
        robot.stop();
        robot.update();
        result.elapsedUs = timerUs() - startUs;
        return result;
    }

    result.sectorCode = sectorCodeFromAngle(signedAngleToObject(object.angleDeg));

    result.stateCode = static_cast<uint8_t>(State::AlignToObject);

    if (!rotateToObject(object))
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

uint32_t LidarDetectionMethod::timerUs() const
{
    return __HAL_TIM_GET_COUNTER(&timer);
}

bool LidarDetectionMethod::runRobotForDuration(uint32_t durationUs)
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

bool LidarDetectionMethod::waitForObject(RPLidarDetectedObject& object)
{
    const uint32_t waitStartUs = timerUs();

    while ((timerUs() - waitStartUs) < LIDAR_OBJECT_WAIT_TIMEOUT_US)
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

bool LidarDetectionMethod::rotateToObject(const RPLidarDetectedObject& object)
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
        robot.rotateLeft(LIDAR_ROTATION_PWM);
    }
    else
    {
        robot.rotateRight(LIDAR_ROTATION_PWM);
    }

    if (!runRobotForDuration(rotationTimeUs))
    {
        return false;
    }

    robot.stop();
    robot.update();

    return true;
}

bool LidarDetectionMethod::approachTarget(DetectionMethodResult& result)
{
    const uint32_t approachStartUs = timerUs();

    while ((timerUs() - approachStartUs) < LIDAR_APPROACH_TIMEOUT_US)
    {
        if (shouldCancel())
        {
            return false;
        }

        RPLidarDetectedObject object;

        if (!lidar.getDetectedObject(object))
        {
            robot.stop();
            robot.update();
            if (!runRobotForDuration(LIDAR_CONTROL_PERIOD_US))
            {
                return false;
            }
            continue;
        }

        const float signedAngleDeg = signedAngleToObject(object.angleDeg);
        result.sectorCode = sectorCodeFromAngle(signedAngleDeg);

        if (object.distanceMm <= LIDAR_TARGET_STOP_MM)
        {
            robot.forward(LIDAR_APPROACH_PWM);
            return runRobotForDuration(LIDAR_FINAL_FORWARD_TIME_US);
        }

        driveTowardObject(object);

        if (!runRobotForDuration(LIDAR_CONTROL_PERIOD_US))
        {
            return false;
        }
    }

    return false;
}

void LidarDetectionMethod::driveTowardObject(const RPLidarDetectedObject& object)
{
    const float signedAngleDeg = signedAngleToObject(object.angleDeg);
    int correction = 0;

    if (absoluteValue(signedAngleDeg) > LIDAR_ANGLE_DEADBAND_DEG)
    {
        correction = static_cast<int>(signedAngleDeg * LIDAR_STEERING_KP);

        if (correction > LIDAR_MAX_STEERING_DELTA)
        {
            correction = LIDAR_MAX_STEERING_DELTA;
        }
        else if (correction < -LIDAR_MAX_STEERING_DELTA)
        {
            correction = -LIDAR_MAX_STEERING_DELTA;
        }
    }

    const int leftSpeed = clampPwm(LIDAR_APPROACH_PWM + correction);
    const int rightSpeed = clampPwm(LIDAR_APPROACH_PWM - correction);

    robot.forwardDifferential(leftSpeed, rightSpeed);
}

bool LidarDetectionMethod::shouldCancel()
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

float LidarDetectionMethod::absoluteValue(float value)
{
    return (value < 0.0f) ? -value : value;
}

float LidarDetectionMethod::signedAngleToObject(float lidarAngleDeg)
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

uint32_t LidarDetectionMethod::rotationTimeForAngle(float angleDeg)
{
    const float angleMagnitudeDeg = absoluteValue(angleDeg);

    return static_cast<uint32_t>(
        (angleMagnitudeDeg * static_cast<float>(LIDAR_FULL_ROTATION_TIME_US)) /
        360.0f
    );
}

int LidarDetectionMethod::clampPwm(int pwm)
{
    if (pwm < LIDAR_MIN_DRIVE_PWM)
    {
        return LIDAR_MIN_DRIVE_PWM;
    }

    if (pwm > LIDAR_MAX_DRIVE_PWM)
    {
        return LIDAR_MAX_DRIVE_PWM;
    }

    return pwm;
}

uint8_t LidarDetectionMethod::sectorCodeFromAngle(float signedAngleDeg)
{
    if (absoluteValue(signedAngleDeg) <= LIDAR_ANGLE_DEADBAND_DEG)
    {
        return static_cast<uint8_t>(Sector::Center);
    }

    if (signedAngleDeg < 0.0f)
    {
        return static_cast<uint8_t>(Sector::Left);
    }

    return static_cast<uint8_t>(Sector::Right);
}
