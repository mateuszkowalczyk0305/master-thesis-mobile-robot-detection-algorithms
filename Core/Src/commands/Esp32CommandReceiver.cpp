#include "commands/Esp32CommandReceiver.h"

Esp32CommandReceiver* Esp32CommandReceiver::activeReceiver = nullptr;

Esp32CommandReceiver::Esp32CommandReceiver(UART_HandleTypeDef* uartHandle)
    : receivedBytes(0),
      framesReceived(0),
      validCommands(0),
      invalidCommands(0),
      overflowErrors(0),
      droppedCommands(0),
      queuedCommands(0),
      uart(uartHandle),
      rxByte(0),
      frameIndex(0),
      commandQueueHead(0),
      commandQueueTail(0)
{
    resetFrame();
    activeReceiver = this;
}

void Esp32CommandReceiver::init()
{
    resetFrame();
    queuedCommands = 0;
    commandQueueHead = 0;
    commandQueueTail = 0;

    if (uart != nullptr)
    {
        HAL_UART_Receive_IT(uart, &rxByte, 1);
    }
}

bool Esp32CommandReceiver::getCommand(Esp32Command& command)
{
    if (queuedCommands == 0)
    {
        return false;
    }

    __disable_irq();

    if (queuedCommands == 0)
    {
        __enable_irq();
        return false;
    }

    command = commandQueue[commandQueueHead];
    commandQueueHead = static_cast<uint8_t>((commandQueueHead + 1) % COMMAND_QUEUE_SIZE);
    queuedCommands--;

    __enable_irq();
    return true;
}

void Esp32CommandReceiver::onUartRxComplete(UART_HandleTypeDef* huart)
{
    if (!isOwnerOfUart(huart))
    {
        return;
    }

    receivedBytes++;
    processByte(rxByte);

    HAL_UART_Receive_IT(uart, &rxByte, 1);
}

void Esp32CommandReceiver::onUartError(UART_HandleTypeDef* huart)
{
    if (!isOwnerOfUart(huart))
    {
        return;
    }

    resetFrame();
    HAL_UART_Receive_IT(uart, &rxByte, 1);
}

bool Esp32CommandReceiver::isOwnerOfUart(UART_HandleTypeDef* huart) const
{
    if (huart == nullptr || uart == nullptr)
    {
        return false;
    }

    return huart->Instance == uart->Instance;
}

void Esp32CommandReceiver::sendDetectionResponse(Esp32DetectionMode mode, uint32_t elapsedUs)
{
    switch (mode)
    {
        case Esp32DetectionMode::Ir:
            sendTextWithTime("#M:I,", elapsedUs);
            break;

        case Esp32DetectionMode::Ultrasonic:
            sendTextWithTime("#M:U,", elapsedUs);
            break;

        case Esp32DetectionMode::Lidar:
            sendTextWithTime("#M:L,", elapsedUs);
            break;

        case Esp32DetectionMode::Fusion:
            sendTextWithTime("#M:F,", elapsedUs);
            break;

        default:
            break;
    }
}

void Esp32CommandReceiver::sendMotionOk()
{
    sendText("#D:OK;");
}

void Esp32CommandReceiver::dispatchUartRxComplete(UART_HandleTypeDef* huart)
{
    if (activeReceiver != nullptr)
    {
        activeReceiver->onUartRxComplete(huart);
    }
}

void Esp32CommandReceiver::dispatchUartError(UART_HandleTypeDef* huart)
{
    if (activeReceiver != nullptr)
    {
        activeReceiver->onUartError(huart);
    }
}

void Esp32CommandReceiver::resetFrame()
{
    frameIndex = 0;

    for (uint8_t index = 0; index < FRAME_BUFFER_SIZE; index++)
    {
        frameBuffer[index] = '\0';
    }
}

void Esp32CommandReceiver::processByte(uint8_t byte)
{
    const char character = static_cast<char>(byte);

    if (character == '#')
    {
        resetFrame();
        frameBuffer[frameIndex++] = character;
        return;
    }

    if (frameIndex == 0)
    {
        return;
    }

    if (frameIndex >= (FRAME_BUFFER_SIZE - 1))
    {
        overflowErrors++;
        resetFrame();
        return;
    }

    frameBuffer[frameIndex++] = character;

    if (character != ';')
    {
        return;
    }

    frameBuffer[frameIndex] = '\0';
    framesReceived++;

    Esp32Command parsedCommand;
    if (parseFrame(frameBuffer, frameIndex, parsedCommand))
    {
        enqueueCommand(parsedCommand);
        validCommands++;
    }
    else
    {
        invalidCommands++;
    }

    resetFrame();
}

void Esp32CommandReceiver::enqueueCommand(const Esp32Command& command)
{
    if (queuedCommands >= COMMAND_QUEUE_SIZE)
    {
        droppedCommands++;
        return;
    }

    commandQueue[commandQueueTail] = command;
    commandQueueTail = static_cast<uint8_t>((commandQueueTail + 1) % COMMAND_QUEUE_SIZE);
    queuedCommands++;
}

bool Esp32CommandReceiver::parseFrame(const char* frame,
                                      uint8_t length,
                                      Esp32Command& command) const
{
    if (frame == nullptr || length < 5)
    {
        return false;
    }

    if (frame[0] != '#' || frame[length - 1] != ';')
    {
        return false;
    }

    if (length == 5 && frame[1] == 'M' && frame[2] == ':')
    {
        command.type = Esp32CommandType::DetectionMode;

        switch (frame[3])
        {
            case 'I':
                command.detectionMode = Esp32DetectionMode::Ir;
                return true;

            case 'U':
                command.detectionMode = Esp32DetectionMode::Ultrasonic;
                return true;

            case 'L':
                command.detectionMode = Esp32DetectionMode::Lidar;
                return true;

            case 'F':
                command.detectionMode = Esp32DetectionMode::Fusion;
                return true;

            default:
                return false;
        }
    }

    const bool normalMotionFrame =
        (length == 7 && frame[1] == 'D' && frame[2] == ':' && frame[4] == ',');
    const bool stopFrameWithComma =
        (length == 7 && frame[1] == 'D' && frame[2] == ',' && frame[3] == '5' && frame[4] == ',');

    if (!normalMotionFrame && !stopFrameWithComma)
    {
        return false;
    }

    const char directionCode = frame[3];
    const char speedCode = frame[5];

    if (speedCode < '0' || speedCode > '9')
    {
        return false;
    }

    command.type = Esp32CommandType::Motion;
    command.speed = static_cast<uint8_t>(speedCode - '0');

    switch (directionCode)
    {
        case '1':
            command.motionDirection = Esp32MotionDirection::Front;
            return normalMotionFrame;

        case '2':
            command.motionDirection = Esp32MotionDirection::Back;
            return normalMotionFrame;

        case '3':
            command.motionDirection = Esp32MotionDirection::Right;
            return normalMotionFrame;

        case '4':
            command.motionDirection = Esp32MotionDirection::Left;
            return normalMotionFrame;

        case '5':
            command.motionDirection = Esp32MotionDirection::Stop;
            return true;

        default:
            return false;
    }
}

void Esp32CommandReceiver::sendText(const char* text)
{
    if (uart == nullptr || text == nullptr)
    {
        return;
    }

    uint16_t length = 0;
    while (text[length] != '\0')
    {
        length++;
    }

    HAL_UART_Transmit(uart,
                      reinterpret_cast<const uint8_t*>(text),
                      length,
                      100);
}

void Esp32CommandReceiver::sendTextWithTime(const char* prefix, uint32_t elapsedUs)
{
    if (prefix == nullptr)
    {
        return;
    }

    char response[RESPONSE_BUFFER_SIZE] = {};
    uint8_t index = 0;

    while (prefix[index] != '\0' && index < (RESPONSE_BUFFER_SIZE - 1))
    {
        response[index] = prefix[index];
        index++;
    }

    char digits[10] = {};
    uint8_t digitCount = 0;

    do
    {
        digits[digitCount++] = static_cast<char>('0' + (elapsedUs % 10));
        elapsedUs /= 10;
    } while (elapsedUs > 0 && digitCount < sizeof(digits));

    while (digitCount > 0 && index < (RESPONSE_BUFFER_SIZE - 1))
    {
        response[index++] = digits[--digitCount];
    }

    if (index < (RESPONSE_BUFFER_SIZE - 1))
    {
        response[index++] = ';';
    }

    response[index] = '\0';
    sendText(response);
}

extern "C" void Esp32_OnUartRx(UART_HandleTypeDef* huart)
{
    Esp32CommandReceiver::dispatchUartRxComplete(huart);
}

extern "C" void Esp32_OnUartError(UART_HandleTypeDef* huart)
{
    Esp32CommandReceiver::dispatchUartError(huart);
}
