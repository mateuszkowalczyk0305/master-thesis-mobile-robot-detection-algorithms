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

struct RPLidarDebugData
{
    float angleDeg;
    float distanceMm;

    uint8_t quality;
    uint8_t startFlag;
    uint8_t valid;

    uint8_t inDetectionZone;

    float filteredAngleDeg;
    float filteredDistanceMm;
    uint8_t filteredQuality;

    uint32_t receivedBytes;
    uint32_t validPoints;
    uint32_t invalidNodes;
    uint32_t filteredPoints;

    uint32_t clustersCreated;
    uint32_t detectedObjects;

    uint8_t currentClusterValid;
    uint16_t currentClusterPoints;
    float currentClusterStartAngle;
    float currentClusterEndAngle;
    float currentClusterCenterAngle;
    float currentClusterDistance;
    float currentClusterWidth;

    uint8_t bestClusterValid;
    uint16_t bestClusterPoints;
    float bestClusterCenterAngle;
    float bestClusterDistance;
    float bestClusterWidth;

    uint8_t objectDetected;
    float objectAngle;
    float objectDistance;
    float objectWidth;
    uint16_t objectPoints;
};

extern IrDebugData irDebug;
extern UltrasonicDebugData ultrasonicDebug;
extern RPLidarDebugData lidarDebug;

#endif /* INC_DEBUGDATA_HPP_ */
