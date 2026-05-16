#ifndef INC_DEBUGDATA_HPP_
#define INC_DEBUGDATA_HPP_

#include <cstdint>

struct IrDebugData
{
    uint16_t leftRaw;
    uint16_t centerRaw;
    uint16_t rightRaw;

    float leftVoltage;
    float centerVoltage;
    float rightVoltage;

    float leftDistance;
    float centerDistance;
    float rightDistance;

    float leftFiltered;
    float centerFiltered;
    float rightFiltered;

    uint8_t leftDetected;
    uint8_t centerDetected;
    uint8_t rightDetected;

    uint8_t sector;
};

struct UltrasonicDebugData
{
    float leftDistance;
    float rightDistance;

    uint8_t leftOk;
    uint8_t rightOk;

    uint32_t leftCounter;
    uint32_t rightCounter;

    uint32_t leftErrorCounter;
    uint32_t rightErrorCounter;
};

extern IrDebugData irDebug;
extern UltrasonicDebugData ultrasonicDebug;

#endif /* INC_DEBUGDATA_HPP_ */
