#ifndef INC_DEBUGDATA_HPP_
#define INC_DEBUGDATA_HPP_

#include <cstdint>

struct IrDebugData
{
    volatile uint16_t leftRaw;
    volatile uint16_t centerRaw;
    volatile uint16_t rightRaw;

    volatile float leftVoltage;
    volatile float centerVoltage;
    volatile float rightVoltage;

    volatile float leftDistance;
    volatile float centerDistance;
    volatile float rightDistance;

    volatile float leftFiltered;
    volatile float centerFiltered;
    volatile float rightFiltered;

    volatile uint8_t leftDetected;
    volatile uint8_t centerDetected;
    volatile uint8_t rightDetected;

    volatile uint8_t sector;
};

extern IrDebugData irDebug;

#endif /* INC_DEBUGDATA_HPP_ */
