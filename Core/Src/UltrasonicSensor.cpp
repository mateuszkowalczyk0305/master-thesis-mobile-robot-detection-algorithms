#include "UltrasonicSensor.h"

static constexpr uint32_t TRIGGER_TIME_US = 10;

// 12000 us daje około 207 cm zakresu.
// Do robota sumo to spokojnie wystarczy.
static constexpr uint32_t TIMEOUT_US = 12000;

// Dla HC-SR04 praktycznie używa się wzoru:
// distance_cm = echo_time_us / 58
static constexpr float US_TO_CM = 58.0f;

static constexpr float MIN_DISTANCE_CM = 2.0f;
static constexpr float MAX_DISTANCE_CM = 100.0f;

UltrasonicSensor* UltrasonicSensor::s_sensors[UltrasonicSensor::MAX_REGISTERED_SENSORS] = {
    nullptr,
    nullptr,
    nullptr,
    nullptr
};

UltrasonicSensor::UltrasonicSensor(
    TIM_HandleTypeDef* timer,
    GPIO_TypeDef* trigPort,
    uint16_t trigPin,
    GPIO_TypeDef* echoPort,
    uint16_t echoPin
)
    : m_timer(timer),
      m_trigPort(trigPort),
      m_trigPin(trigPin),
      m_echoPort(echoPort),
      m_echoPin(echoPin)
{
}

void UltrasonicSensor::init()
{
    HAL_GPIO_WritePin(m_trigPort, m_trigPin, GPIO_PIN_RESET);

    // Timer działa jako licznik mikrosekundowy.
    HAL_TIM_Base_Start(m_timer);
}

void UltrasonicSensor::registerForExti()
{
    // Jeżeli ten czujnik już jest zarejestrowany, nie dodawaj go drugi raz.
    for (uint8_t i = 0; i < MAX_REGISTERED_SENSORS; i++)
    {
        if (s_sensors[i] == this)
        {
            return;
        }
    }

    // Znajdź wolne miejsce.
    for (uint8_t i = 0; i < MAX_REGISTERED_SENSORS; i++)
    {
        if (s_sensors[i] == nullptr)
        {
            s_sensors[i] = this;
            return;
        }
    }
}

uint32_t UltrasonicSensor::micros() const
{
    return __HAL_TIM_GET_COUNTER(m_timer);
}

void UltrasonicSensor::delayUs(uint32_t us)
{
    const uint32_t start = micros();

    while ((micros() - start) < us)
    {
        // aktywne czekanie
    }
}

void UltrasonicSensor::sendTrigger()
{
    HAL_GPIO_WritePin(m_trigPort, m_trigPin, GPIO_PIN_RESET);
    delayUs(2);

    HAL_GPIO_WritePin(m_trigPort, m_trigPin, GPIO_PIN_SET);
    delayUs(TRIGGER_TIME_US);

    HAL_GPIO_WritePin(m_trigPort, m_trigPin, GPIO_PIN_RESET);
}

bool UltrasonicSensor::measure(float& distanceCm)
{
    m_measuring = true;
    m_echoStarted = false;
    m_echoDone = false;
    m_startUs = 0;
    m_endUs = 0;

    sendTrigger();

    const uint32_t timeoutStart = micros();

    while (!m_echoDone)
    {
        if ((micros() - timeoutStart) > TIMEOUT_US)
        {
            m_measuring = false;
            return false;
        }
    }

    m_measuring = false;

    const uint32_t pulseWidthUs = m_endUs - m_startUs;

    distanceCm = static_cast<float>(pulseWidthUs) / US_TO_CM;

    if (distanceCm < MIN_DISTANCE_CM)
    {
        return false;
    }

    if (distanceCm > MAX_DISTANCE_CM)
    {
        return false;
    }

    return true;
}

void UltrasonicSensor::handleExti(uint16_t gpioPin)
{
    if (!m_measuring)
    {
        return;
    }

    if (gpioPin != m_echoPin)
    {
        return;
    }

    const GPIO_PinState echoState = HAL_GPIO_ReadPin(m_echoPort, m_echoPin);

    if (echoState == GPIO_PIN_SET)
    {
        m_startUs = micros();
        m_echoStarted = true;
    }
    else
    {
        if (m_echoStarted)
        {
            m_endUs = micros();
            m_echoDone = true;
        }
    }
}

void UltrasonicSensor::dispatchExti(uint16_t gpioPin)
{
    for (uint8_t i = 0; i < MAX_REGISTERED_SENSORS; i++)
    {
        if (s_sensors[i] != nullptr)
        {
            s_sensors[i]->handleExti(gpioPin);
        }
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    UltrasonicSensor::dispatchExti(GPIO_Pin);
}
