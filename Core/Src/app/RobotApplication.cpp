#include "app/RobotApplication.hpp"

#include "commands/MotionCommandHandler.hpp"

RobotApplication::RobotApplication(Robot& robotRef,
                                   Esp32CommandReceiver& commandReceiverRef,
                                   DetectionMethodExecutor& detectionMethodsRef)
    : robot(robotRef),
      commandReceiver(commandReceiverRef),
      detectionMethods(detectionMethodsRef),
      activeDetectionMode(Esp32DetectionMode::None)
{
    setState(State::Idle);
}

void RobotApplication::update()
{
    robot.update();

    if (commandReceiver.getCommand(command))
    {
        handleCommand(command);
    }
}

void RobotApplication::setState(State state)
{
    debug.stateCode = static_cast<uint8_t>(state);
}

void RobotApplication::handleCommand(const Esp32Command& receivedCommand)
{
    switch (receivedCommand.type)
    {
        case Esp32CommandType::Motion:
            handleMotionCommand(receivedCommand);
            break;

        case Esp32CommandType::DetectionMode:
            handleDetectionCommand(receivedCommand);
            break;

        default:
            break;
    }

    setState(State::Idle);
}

void RobotApplication::handleMotionCommand(const Esp32Command& receivedCommand)
{
    setState(State::HandlingMotionCommand);
    MotionCommandHandler::execute(robot, receivedCommand);
    commandReceiver.sendMotionOk();
}

void RobotApplication::handleDetectionCommand(const Esp32Command& receivedCommand)
{
    setState(State::RunningDetectionMethod);

    activeDetectionMode = receivedCommand.detectionMode;
    debug.activeDetectionModeCode = static_cast<uint8_t>(activeDetectionMode);

    const DetectionMethodResult result = detectionMethods.execute(activeDetectionMode);

    debug.lastDetectionTimeUs = result.elapsedUs;
    debug.lastDetectionSectorCode = result.sectorCode;
    debug.lastDetectionStateCode = result.stateCode;
    debug.lastDetectionTargetReached = result.targetReached ? 1U : 0U;

    commandReceiver.sendDetectionResponse(activeDetectionMode, result.elapsedUs);
}
