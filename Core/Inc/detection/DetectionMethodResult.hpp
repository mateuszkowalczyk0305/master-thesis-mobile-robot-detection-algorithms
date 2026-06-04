#pragma once

#include "main.h"

struct DetectionMethodResult
{
    uint32_t elapsedUs = 0;
    uint8_t sectorCode = 0;
    uint8_t stateCode = 0;
    bool targetReached = false;
};
