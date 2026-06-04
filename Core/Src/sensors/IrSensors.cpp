#include "sensors/IrSensors.h"

IrSensors::IrSensors(ADC_HandleTypeDef* hadc)
    : hadc_(hadc)
{
    init();
}

void IrSensors::init()
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        adcBuffer_[i] = 0;

        sensors_[i].adcRaw = 0;
        sensors_[i].voltage = 0.0f;
        sensors_[i].distanceCm = 80.0f;
        sensors_[i].filteredCm = 80.0f;
        sensors_[i].detected = false;
    }
}

void IrSensors::startDma()
{
    if (hadc_ == nullptr)
    {
        return;
    }

    HAL_ADC_Start_DMA(
        hadc_,
        reinterpret_cast<uint32_t*>(adcBuffer_),
        SENSOR_COUNT
    );
}

void IrSensors::update()
{
    /*
     * DMA cały czas aktualizuje adcBuffer_.
     *
     * Lewy i prawy czujnik są fizycznie zamienione miejscami.
     * Logiczne LEFT czyta ADC1_IN4, a logiczne RIGHT czyta ADC1_IN0.
     */

    constexpr uint8_t adcIndexForSensor[SENSOR_COUNT] =
    {
        2,
        1,
        0
    };

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        sensors_[i].adcRaw = adcBuffer_[adcIndexForSensor[i]];

        sensors_[i].voltage =
            (static_cast<float>(sensors_[i].adcRaw) * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;

        sensors_[i].distanceCm = voltageToDistance(sensors_[i].voltage);

        sensors_[i].filteredCm =
            IIR_ALPHA * sensors_[i].distanceCm +
            (1.0f - IIR_ALPHA) * sensors_[i].filteredCm;

        sensors_[i].detected =
            hysteresis(sensors_[i].filteredCm, sensors_[i].detected);
    }
}

IrSensors::SensorData IrSensors::getSensorData(Sensor sensor) const
{
    uint8_t index = static_cast<uint8_t>(sensor);

    if (index >= SENSOR_COUNT)
    {
        return sensors_[0];
    }

    return sensors_[index];
}

IrSensors::Sector IrSensors::getSector() const
{
    bool left = sensors_[static_cast<uint8_t>(Sensor::Left)].detected;
    bool center = sensors_[static_cast<uint8_t>(Sensor::Center)].detected;
    bool right = sensors_[static_cast<uint8_t>(Sensor::Right)].detected;

    if (left && center && right)
    {
        return Sector::FrontWide;
    }

    if (center)
    {
        return Sector::Center;
    }

    if (left && !right)
    {
        return Sector::Left;
    }

    if (right && !left)
    {
        return Sector::Right;
    }

    if (left && right)
    {
        return Sector::FrontWide;
    }

    return Sector::None;
}

bool IrSensors::hysteresis(float distanceCm, bool previousState) const
{
    if (!previousState)
    {
        if (distanceCm < DETECT_ON_CM)
        {
            return true;
        }

        return false;
    }

    if (distanceCm > DETECT_OFF_CM)
    {
        return false;
    }

    return true;
}

float IrSensors::voltageToDistance(float voltage) const
{
    /*
     * Przybliżona tabela dla Sharp GP2Y0A21YK0F.
     * Zakres pracy czujnika to 10-80 cm, wyjście jest analogowe i nieliniowe.
     * Później najlepiej zrobić własną kalibrację pod Twojego robota.
     */

    constexpr float voltageTable[] =
    {
        2.75f, 2.30f, 1.65f, 1.30f, 1.05f,
        0.85f, 0.70f, 0.60f, 0.50f, 0.42f
    };

    constexpr float distanceTable[] =
    {
        10.0f, 15.0f, 20.0f, 25.0f, 30.0f,
        40.0f, 50.0f, 60.0f, 70.0f, 80.0f
    };

    constexpr uint8_t tableSize =
        sizeof(voltageTable) / sizeof(voltageTable[0]);

    if (voltage >= voltageTable[0])
    {
        return 10.0f;
    }

    if (voltage <= voltageTable[tableSize - 1])
    {
        return 80.0f;
    }

    for (uint8_t i = 0; i < tableSize - 1; i++)
    {
        if ((voltage <= voltageTable[i]) && (voltage >= voltageTable[i + 1]))
        {
            float v1 = voltageTable[i];
            float v2 = voltageTable[i + 1];

            float d1 = distanceTable[i];
            float d2 = distanceTable[i + 1];

            return d1 + ((voltage - v1) * (d2 - d1)) / (v2 - v1);
        }
    }

    return 80.0f;
}
