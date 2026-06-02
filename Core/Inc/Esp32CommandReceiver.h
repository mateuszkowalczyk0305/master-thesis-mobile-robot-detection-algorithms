#pragma once

#include "main.h"
#include <cstdint>

enum class Esp32CommandType : uint8_t
{
    None = 0,
    DetectionMode,
    Motion
};

enum class Esp32DetectionMode : uint8_t
{
    None = 0,
    Ir,
    Ultrasonic,
    Lidar,
    Fusion
};

enum class Esp32MotionDirection : uint8_t
{
    None = 0,
    Front,
    Back,
    Right,
    Left,
    Stop
};

struct Esp32Command
{
    Esp32CommandType type = Esp32CommandType::None;
    Esp32DetectionMode detectionMode = Esp32DetectionMode::None;
    Esp32MotionDirection motionDirection = Esp32MotionDirection::None;
    uint8_t speed = 0;
};

class Esp32CommandReceiver
{
public:
    explicit Esp32CommandReceiver(UART_HandleTypeDef* uartHandle);

    void init();
    bool getCommand(Esp32Command& command);
    void onUartRxComplete(UART_HandleTypeDef* huart);
    void onUartError(UART_HandleTypeDef* huart);
    bool isOwnerOfUart(UART_HandleTypeDef* huart) const;

    void sendDetectionResponse(Esp32DetectionMode mode, uint32_t timeMs = 0);
    void sendMotionOk();

    static void dispatchUartRxComplete(UART_HandleTypeDef* huart);
    static void dispatchUartError(UART_HandleTypeDef* huart);

    volatile uint32_t receivedBytes;
    volatile uint32_t framesReceived;
    volatile uint32_t validCommands;
    volatile uint32_t invalidCommands;
    volatile uint32_t overflowErrors;
    volatile uint32_t droppedCommands;
    volatile uint8_t queuedCommands;

private:
    static constexpr uint8_t FRAME_BUFFER_SIZE = 16;
    static constexpr uint8_t RESPONSE_BUFFER_SIZE = 24;
    static constexpr uint8_t COMMAND_QUEUE_SIZE = 16;

    UART_HandleTypeDef* uart;
    uint8_t rxByte;

    char frameBuffer[FRAME_BUFFER_SIZE];
    uint8_t frameIndex;

    Esp32Command commandQueue[COMMAND_QUEUE_SIZE];
    uint8_t commandQueueHead;
    uint8_t commandQueueTail;

    void resetFrame();
    void processByte(uint8_t byte);
    void enqueueCommand(const Esp32Command& command);
    bool parseFrame(const char* frame, uint8_t length, Esp32Command& command) const;
    void sendText(const char* text);
    void sendTextWithTime(const char* prefix, uint32_t timeMs);

    static Esp32CommandReceiver* activeReceiver;
};

extern "C" void Esp32_OnUartRx(UART_HandleTypeDef* huart);
extern "C" void Esp32_OnUartError(UART_HandleTypeDef* huart);
