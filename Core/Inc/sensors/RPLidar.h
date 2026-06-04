#ifndef RPLIDAR_HPP
#define RPLIDAR_HPP

#include "main.h"
#include <cstdint>

struct RPLidarPoint
{
    float angleDeg;
    float distanceMm;
    uint8_t quality;
    bool startFlag;
    bool valid;
};

struct RPLidarCluster
{
    bool valid;

    float startAngleDeg;
    float endAngleDeg;
    float centerAngleDeg;

    float averageDistanceMm;
    float minDistanceMm;
    float maxDistanceMm;

    float widthDeg;

    uint16_t pointsCount;
};

struct RPLidarDetectedObject
{
    bool detected;

    float angleDeg;
    float distanceMm;

    float clusterWidthDeg;
    uint16_t pointsCount;
};

enum class RPLidarObjectSector : uint8_t
{
    None = 0,
    Left = 1,
    Center = 2,
    Right = 3
};

class RPLidar
{
public:
    RPLidar(UART_HandleTypeDef* uartHandle,
            TIM_HandleTypeDef* motorTimerHandle,
            uint32_t motorTimerChannel);

    void init();

    void startMotor(uint16_t pwmValue);
    void stopMotor();

    void reset();
    void startScan();
    void stopScan();

    void startUartReception();
    void onUartRxComplete(UART_HandleTypeDef* huart);

    bool getPoint(RPLidarPoint& point);

    bool isPointInDetectionZone(const RPLidarPoint& point) const;

    void processPointForDetection(const RPLidarPoint& point);

    bool getDetectedObject(RPLidarDetectedObject& object) const;
    RPLidarObjectSector getObjectSector() const;

    RPLidarCluster getCurrentCluster() const;
    RPLidarCluster getBestCluster() const;

    bool isOwnerOfUart(UART_HandleTypeDef* huart) const;

    volatile uint32_t receivedBytes;
    volatile uint32_t validPoints;
    volatile uint32_t invalidNodes;

    volatile uint32_t filteredPoints;
    volatile uint32_t clustersCreated;
    volatile uint32_t detectedObjects;

private:
    enum class ParserState
    {
        Idle,
        WaitingDescriptor,
        ReadingMeasurement
    };

    UART_HandleTypeDef* uart;
    TIM_HandleTypeDef* motorTimer;
    uint32_t motorChannel;

    static constexpr uint16_t PWM_MAX = 999;

    static constexpr uint8_t SYNC_BYTE = 0xA5;

    static constexpr uint8_t CMD_STOP = 0x25;
    static constexpr uint8_t CMD_SCAN = 0x20;
    static constexpr uint8_t CMD_RESET = 0x40;

    static constexpr uint8_t DESCRIPTOR_SIZE = 7;
    static constexpr uint8_t NODE_SIZE = 5;

    /*
     * Zakres detekcji.
     * Metoda LiDAR wybiera najbliższy klaster z pełnego skanu 360 stopni.
     */
    static constexpr float MIN_DETECTION_DISTANCE_MM = 150.0f;
    static constexpr float MAX_DETECTION_DISTANCE_MM = 1500.0f;

    static constexpr float MIN_DETECTION_ANGLE_DEG = 0.0f;
    static constexpr float MAX_DETECTION_ANGLE_DEG = 360.0f;

    static constexpr uint8_t MIN_QUALITY = 5;

    /*
     * Parametry klastrowania.
     */
    static constexpr float MAX_CLUSTER_ANGLE_GAP_DEG = 3.0f;
    static constexpr float MAX_CLUSTER_DISTANCE_GAP_MM = 150.0f;

    static constexpr uint16_t MIN_CLUSTER_POINTS = 3;
    static constexpr float MAX_CLUSTER_WIDTH_DEG = 35.0f;

    uint8_t rxByte;

    ParserState parserState;

    uint8_t descriptorBuffer[DESCRIPTOR_SIZE];
    uint8_t descriptorIndex;

    uint8_t nodeBuffer[NODE_SIZE];
    uint8_t nodeIndex;

    RPLidarPoint lastPoint;
    volatile bool newPointAvailable;

    RPLidarPoint previousFilteredPoint;
    bool hasPreviousFilteredPoint;

    RPLidarCluster currentCluster;
    RPLidarCluster bestCluster;
    RPLidarDetectedObject detectedObject;

    float currentClusterDistanceSum;

    void sendCommand(uint8_t command);
    void processByte(uint8_t byte);
    void processMeasurementNode();

    bool isValidMeasurementNode() const;

    void clearCluster(RPLidarCluster& cluster);
    void startNewCluster(const RPLidarPoint& point);
    void addPointToCurrentCluster(const RPLidarPoint& point);
    void finishCurrentCluster();

    bool isPointCloseToPrevious(const RPLidarPoint& point) const;
    bool isClusterObjectCandidate(const RPLidarCluster& cluster) const;
    bool isBetterCluster(const RPLidarCluster& candidate,
                         const RPLidarCluster& currentBest) const;

    float angleForwardDiff(float startDeg, float endDeg) const;

    void updateDebugCounters() const;
    void updateDebugRawPoint(const RPLidarPoint& point,
                             bool pointInDetectionZone) const;
    void updateDebugFilteredPoint(const RPLidarPoint& point) const;
    void updateDebugCurrentCluster() const;
    void updateDebugBestCluster() const;
    void updateDebugObject() const;
};

#endif
