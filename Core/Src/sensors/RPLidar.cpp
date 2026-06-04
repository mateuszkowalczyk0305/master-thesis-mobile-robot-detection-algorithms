#include "sensors/RPLidar.h"
#include "commands/Esp32CommandReceiver.h"

/*
 * Zakładamy jeden LiDAR w projekcie.
 * Dzięki temu callback HAL-a może być w tym samym pliku co klasa.
 */
static RPLidar* activeLidarObject = nullptr;

RPLidar::RPLidar(UART_HandleTypeDef* uartHandle,
                 TIM_HandleTypeDef* motorTimerHandle,
                 uint32_t motorTimerChannel)
    : receivedBytes(0),
      validPoints(0),
      invalidNodes(0),
      filteredPoints(0),
      clustersCreated(0),
      detectedObjects(0),
      uart(uartHandle),
      motorTimer(motorTimerHandle),
      motorChannel(motorTimerChannel),
      rxByte(0),
      parserState(ParserState::Idle),
      descriptorIndex(0),
      nodeIndex(0),
      newPointAvailable(false),
      hasPreviousFilteredPoint(false),
      currentClusterDistanceSum(0.0f)
{
    lastPoint.angleDeg = 0.0f;
    lastPoint.distanceMm = 0.0f;
    lastPoint.quality = 0;
    lastPoint.startFlag = false;
    lastPoint.valid = false;

    previousFilteredPoint = lastPoint;

    clearCluster(currentCluster);
    clearCluster(bestCluster);

    detectedObject.detected = false;
    detectedObject.angleDeg = 0.0f;
    detectedObject.distanceMm = 0.0f;
    detectedObject.clusterWidthDeg = 0.0f;
    detectedObject.pointsCount = 0;

    activeLidarObject = this;
}

void RPLidar::init()
{
    startMotor(700);

    HAL_Delay(1000);

    reset();

    HAL_Delay(2000);

    startUartReception();

    startScan();
}

void RPLidar::startMotor(uint16_t pwmValue)
{
    if (pwmValue > PWM_MAX)
    {
        pwmValue = PWM_MAX;
    }

    HAL_TIM_PWM_Start(motorTimer, motorChannel);
    __HAL_TIM_SET_COMPARE(motorTimer, motorChannel, pwmValue);
}

void RPLidar::stopMotor()
{
    __HAL_TIM_SET_COMPARE(motorTimer, motorChannel, 0);
    HAL_TIM_PWM_Stop(motorTimer, motorChannel);
}

void RPLidar::reset()
{
    parserState = ParserState::Idle;
    descriptorIndex = 0;
    nodeIndex = 0;

    sendCommand(CMD_RESET);
}

void RPLidar::startScan()
{
    parserState = ParserState::WaitingDescriptor;
    descriptorIndex = 0;
    nodeIndex = 0;

    sendCommand(CMD_SCAN);
}

void RPLidar::stopScan()
{
    parserState = ParserState::Idle;
    descriptorIndex = 0;
    nodeIndex = 0;

    sendCommand(CMD_STOP);
}

void RPLidar::startUartReception()
{
    HAL_UART_Receive_IT(uart, &rxByte, 1);
}

void RPLidar::onUartRxComplete(UART_HandleTypeDef* huart)
{
    if (!isOwnerOfUart(huart))
    {
        return;
    }

    receivedBytes++;

    processByte(rxByte);

    HAL_UART_Receive_IT(uart, &rxByte, 1);
}

bool RPLidar::getPoint(RPLidarPoint& point)
{
    if (!newPointAvailable)
    {
        return false;
    }

    point = lastPoint;
    newPointAvailable = false;

    return true;
}

bool RPLidar::isPointInDetectionZone(const RPLidarPoint& point) const
{
    if (!point.valid)
    {
        return false;
    }

    if (point.distanceMm < MIN_DETECTION_DISTANCE_MM)
    {
        return false;
    }

    if (point.distanceMm > MAX_DETECTION_DISTANCE_MM)
    {
        return false;
    }

    if (point.quality < MIN_QUALITY)
    {
        return false;
    }

    bool angleInRange = false;

    /*
     * Normalny zakres, np. 60-120 stopni.
     */
    if (MIN_DETECTION_ANGLE_DEG <= MAX_DETECTION_ANGLE_DEG)
    {
        angleInRange = (point.angleDeg >= MIN_DETECTION_ANGLE_DEG &&
                        point.angleDeg <= MAX_DETECTION_ANGLE_DEG);
    }
    /*
     * Zakres przez zero, np. 300-60 stopni.
     */
    else
    {
        angleInRange = (point.angleDeg >= MIN_DETECTION_ANGLE_DEG ||
                        point.angleDeg <= MAX_DETECTION_ANGLE_DEG);
    }

    return angleInRange;
}

void RPLidar::processPointForDetection(const RPLidarPoint& point)
{
    /*
     * Początek nowego obrotu.
     * Kończymy ostatni klaster z poprzedniego obrotu.
     * Najlepszy klaster staje się wynikiem detekcji.
     */
    if (point.startFlag)
    {
        finishCurrentCluster();

        if (bestCluster.valid)
        {
            detectedObject.detected = true;
            detectedObject.angleDeg = bestCluster.centerAngleDeg;
            detectedObject.distanceMm = bestCluster.averageDistanceMm;
            detectedObject.clusterWidthDeg = bestCluster.widthDeg;
            detectedObject.pointsCount = bestCluster.pointsCount;

            detectedObjects++;
        }
        else
        {
            detectedObject.detected = false;
            detectedObject.angleDeg = 0.0f;
            detectedObject.distanceMm = 0.0f;
            detectedObject.clusterWidthDeg = 0.0f;
            detectedObject.pointsCount = 0;
        }

        clearCluster(bestCluster);
        hasPreviousFilteredPoint = false;
    }

    /*
     * Punkt poza strefą detekcji kończy aktualny klaster.
     */
    if (!isPointInDetectionZone(point))
    {
        finishCurrentCluster();
        hasPreviousFilteredPoint = false;
        return;
    }

    filteredPoints++;

    /*
     * Brak aktywnego klastra — zaczynamy nowy.
     */
    if (!currentCluster.valid)
    {
        startNewCluster(point);
        previousFilteredPoint = point;
        hasPreviousFilteredPoint = true;
        return;
    }

    /*
     * Punkt pasuje do klastra albo zaczyna nowy klaster.
     */
    if (hasPreviousFilteredPoint && isPointCloseToPrevious(point))
    {
        addPointToCurrentCluster(point);
    }
    else
    {
        finishCurrentCluster();
        startNewCluster(point);
    }

    previousFilteredPoint = point;
    hasPreviousFilteredPoint = true;
}

bool RPLidar::getDetectedObject(RPLidarDetectedObject& object) const
{
    object = detectedObject;
    return detectedObject.detected;
}

RPLidarObjectSector RPLidar::getObjectSector() const
{
    if (!detectedObject.detected)
    {
        return RPLidarObjectSector::None;
    }

    float angle = detectedObject.angleDeg;

    /*
     * Dla sektora detekcji 300-60 stopni:
     *
     * LEFT:   300-330 deg
     * CENTER: 330-360 deg oraz 0-30 deg
     * RIGHT:  30-60 deg
     */

    if (angle >= 330.0f || angle <= 30.0f)
    {
        return RPLidarObjectSector::Center;
    }

    if (angle > 30.0f && angle <= 60.0f)
    {
        return RPLidarObjectSector::Right;
    }

    if (angle >= 300.0f && angle < 330.0f)
    {
        return RPLidarObjectSector::Left;
    }

    return RPLidarObjectSector::None;
}

RPLidarCluster RPLidar::getCurrentCluster() const
{
    return currentCluster;
}

RPLidarCluster RPLidar::getBestCluster() const
{
    return bestCluster;
}

bool RPLidar::isOwnerOfUart(UART_HandleTypeDef* huart) const
{
    if (huart == nullptr)
    {
        return false;
    }

    if (uart == nullptr)
    {
        return false;
    }

    return huart->Instance == uart->Instance;
}

void RPLidar::sendCommand(uint8_t command)
{
    uint8_t packet[2];

    packet[0] = SYNC_BYTE;
    packet[1] = command;

    HAL_UART_Transmit(uart, packet, sizeof(packet), 100);
}

void RPLidar::processByte(uint8_t byte)
{
    switch (parserState)
    {
        case ParserState::Idle:
        {
            break;
        }

        case ParserState::WaitingDescriptor:
        {
            descriptorBuffer[descriptorIndex] = byte;
            descriptorIndex++;

            if (descriptorIndex >= DESCRIPTOR_SIZE)
            {
                descriptorIndex = 0;
                nodeIndex = 0;
                parserState = ParserState::ReadingMeasurement;
            }

            break;
        }

        case ParserState::ReadingMeasurement:
        {
            nodeBuffer[nodeIndex] = byte;
            nodeIndex++;

            if (nodeIndex >= NODE_SIZE)
            {
                nodeIndex = 0;
                processMeasurementNode();
            }

            break;
        }

        default:
        {
            parserState = ParserState::Idle;
            break;
        }
    }
}

bool RPLidar::isValidMeasurementNode() const
{
    bool startBit = (nodeBuffer[0] & 0x01) != 0;
    bool invertedStartBit = (nodeBuffer[0] & 0x02) != 0;

    bool checkBit = (nodeBuffer[1] & 0x01) != 0;

    /*
     * Start bit i jego negacja muszą być przeciwne.
     */
    if (startBit == invertedStartBit)
    {
        return false;
    }

    /*
     * Bit kontrolny kąta powinien być ustawiony.
     */
    if (!checkBit)
    {
        return false;
    }

    return true;
}

void RPLidar::processMeasurementNode()
{
    if (!isValidMeasurementNode())
    {
        invalidNodes++;
        return;
    }

    bool startBit = (nodeBuffer[0] & 0x01) != 0;

    uint8_t quality = nodeBuffer[0] >> 2;

    uint16_t angleRaw = static_cast<uint16_t>((nodeBuffer[1] >> 1) |
                                             (nodeBuffer[2] << 7));

    uint16_t distanceRaw = static_cast<uint16_t>(nodeBuffer[3] |
                                                (nodeBuffer[4] << 8));

    float angleDeg = static_cast<float>(angleRaw) / 64.0f;
    float distanceMm = static_cast<float>(distanceRaw) / 4.0f;

    lastPoint.angleDeg = angleDeg;
    lastPoint.distanceMm = distanceMm;
    lastPoint.quality = quality;
    lastPoint.startFlag = startBit;
    lastPoint.valid = true;

    validPoints++;
    newPointAvailable = true;
}

void RPLidar::clearCluster(RPLidarCluster& cluster)
{
    cluster.valid = false;

    cluster.startAngleDeg = 0.0f;
    cluster.endAngleDeg = 0.0f;
    cluster.centerAngleDeg = 0.0f;

    cluster.averageDistanceMm = 0.0f;
    cluster.minDistanceMm = 0.0f;
    cluster.maxDistanceMm = 0.0f;

    cluster.widthDeg = 0.0f;

    cluster.pointsCount = 0;
}

void RPLidar::startNewCluster(const RPLidarPoint& point)
{
    currentCluster.valid = true;

    currentCluster.startAngleDeg = point.angleDeg;
    currentCluster.endAngleDeg = point.angleDeg;
    currentCluster.centerAngleDeg = point.angleDeg;

    currentCluster.averageDistanceMm = point.distanceMm;
    currentCluster.minDistanceMm = point.distanceMm;
    currentCluster.maxDistanceMm = point.distanceMm;

    currentCluster.widthDeg = 0.0f;

    currentCluster.pointsCount = 1;

    currentClusterDistanceSum = point.distanceMm;
}

void RPLidar::addPointToCurrentCluster(const RPLidarPoint& point)
{
    if (!currentCluster.valid)
    {
        startNewCluster(point);
        return;
    }

    currentCluster.endAngleDeg = point.angleDeg;
    currentCluster.pointsCount++;

    currentClusterDistanceSum += point.distanceMm;

    currentCluster.averageDistanceMm =
        currentClusterDistanceSum / static_cast<float>(currentCluster.pointsCount);

    if (point.distanceMm < currentCluster.minDistanceMm)
    {
        currentCluster.minDistanceMm = point.distanceMm;
    }

    if (point.distanceMm > currentCluster.maxDistanceMm)
    {
        currentCluster.maxDistanceMm = point.distanceMm;
    }

    currentCluster.widthDeg =
        angleForwardDiff(currentCluster.startAngleDeg, currentCluster.endAngleDeg);

    currentCluster.centerAngleDeg =
        currentCluster.startAngleDeg + (currentCluster.widthDeg * 0.5f);

    if (currentCluster.centerAngleDeg >= 360.0f)
    {
        currentCluster.centerAngleDeg -= 360.0f;
    }
}

void RPLidar::finishCurrentCluster()
{
    if (!currentCluster.valid)
    {
        return;
    }

    if (isClusterObjectCandidate(currentCluster))
    {
        clustersCreated++;

        if (isBetterCluster(currentCluster, bestCluster))
        {
            bestCluster = currentCluster;
        }
    }

    clearCluster(currentCluster);
    currentClusterDistanceSum = 0.0f;
}

bool RPLidar::isPointCloseToPrevious(const RPLidarPoint& point) const
{
    float angleGap = angleForwardDiff(previousFilteredPoint.angleDeg, point.angleDeg);

    float distanceGap = point.distanceMm - previousFilteredPoint.distanceMm;

    if (distanceGap < 0.0f)
    {
        distanceGap = -distanceGap;
    }

    if (angleGap > MAX_CLUSTER_ANGLE_GAP_DEG)
    {
        return false;
    }

    if (distanceGap > MAX_CLUSTER_DISTANCE_GAP_MM)
    {
        return false;
    }

    return true;
}

bool RPLidar::isClusterObjectCandidate(const RPLidarCluster& cluster) const
{
    if (!cluster.valid)
    {
        return false;
    }

    if (cluster.pointsCount < MIN_CLUSTER_POINTS)
    {
        return false;
    }

    if (cluster.widthDeg > MAX_CLUSTER_WIDTH_DEG)
    {
        return false;
    }

    return true;
}

bool RPLidar::isBetterCluster(const RPLidarCluster& candidate,
                              const RPLidarCluster& currentBest) const
{
    if (!candidate.valid)
    {
        return false;
    }

    if (!currentBest.valid)
    {
        return true;
    }

    /*
     * Dla robota sumo wybieramy najbliższy obiekt.
     */
    return candidate.averageDistanceMm < currentBest.averageDistanceMm;
}

float RPLidar::angleForwardDiff(float startDeg, float endDeg) const
{
    float diff = endDeg - startDeg;

    if (diff < 0.0f)
    {
        diff += 360.0f;
    }

    return diff;
}

/*
 * Callback HAL-a zostaje w pliku klasy.
 * Nie dodawaj drugiego HAL_UART_RxCpltCallback w main.cpp.
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (activeLidarObject != nullptr)
    {
        activeLidarObject->onUartRxComplete(huart);
    }

    Esp32_OnUartRx(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    Esp32_OnUartError(huart);
}
