#ifndef INC_IRSENSORS_HPP_
#define INC_IRSENSORS_HPP_

#include "main.h"
#include <cstdint>

class IrSensors
{
public:
    static constexpr uint8_t SENSOR_COUNT = 3;

    enum class Sensor : uint8_t
    {
        Left = 0,
        Center = 1,
        Right = 2
    };

    enum class Sector : uint8_t
    {
        None = 0,
        Left,
        Center,
        Right,
        FrontWide
    };

    struct SensorData
    {
        uint16_t adcRaw = 0;
        float voltage = 0.0f;
        float distanceCm = 80.0f;
        float filteredCm = 80.0f;
        bool detected = false;
    };

    explicit IrSensors(ADC_HandleTypeDef* hadc);

    void init();
    void startDma();
    void update();

    SensorData getSensorData(Sensor sensor) const;
    Sector getSector() const;

private:
    ADC_HandleTypeDef* hadc_;

    uint16_t adcBuffer_[SENSOR_COUNT];

    SensorData sensors_[SENSOR_COUNT];

    static constexpr float ADC_MAX_VALUE = 4095.0f;
    static constexpr float ADC_REF_VOLTAGE = 3.3f;

    static constexpr float DETECT_ON_CM = 45.0f;
    static constexpr float DETECT_OFF_CM = 55.0f;

    static constexpr float IIR_ALPHA = 0.25f;

    float voltageToDistance(float voltage) const;
    bool hysteresis(float distanceCm, bool previousState) const;
};

#endif /* INC_IRSENSORS_HPP_ */
