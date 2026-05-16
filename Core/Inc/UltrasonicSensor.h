#pragma once

#include "stm32f4xx_hal.h"
#include <cstdint>

class UltrasonicSensor
{
public:
    UltrasonicSensor(
        TIM_HandleTypeDef* timer,
        GPIO_TypeDef* trigPort,
        uint16_t trigPin,
        GPIO_TypeDef* echoPort,
        uint16_t echoPin
    );

    void init();

    void registerForExti();

    bool measure(float& distanceCm);

    void handleExti(uint16_t gpioPin);

    static void dispatchExti(uint16_t gpioPin);

private:
    TIM_HandleTypeDef* m_timer;

    GPIO_TypeDef* m_trigPort;
    uint16_t m_trigPin;

    GPIO_TypeDef* m_echoPort;
    uint16_t m_echoPin;

    volatile bool m_measuring = false;
    volatile bool m_echoStarted = false;
    volatile bool m_echoDone = false;

    volatile uint32_t m_startUs = 0;
    volatile uint32_t m_endUs = 0;

private:
    uint32_t micros() const;
    void delayUs(uint32_t us);
    void sendTrigger();

private:
    static constexpr uint8_t MAX_REGISTERED_SENSORS = 4;
    static UltrasonicSensor* s_sensors[MAX_REGISTERED_SENSORS];
};
