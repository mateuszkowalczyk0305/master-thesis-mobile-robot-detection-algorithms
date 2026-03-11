/*
 * motor.cpp
 *
 *  Created on: Mar 9, 2026
 *      Author: m_kowalczyk
 */

#include <motion/motor.hpp>

Motor::Motor(TIM_HandleTypeDef* timerA,
             uint32_t pwm_channelA,
             TIM_HandleTypeDef* timerB,
             uint32_t pwm_channelB)
{
    htimA = timerA;
    channelA = pwm_channelA;

    htimB = timerB;
    channelB = pwm_channelB;

    HAL_TIM_PWM_Start(htimA, channelA);
    HAL_TIM_PWM_Start(htimB, channelB);
}

void Motor::setSpeed(int speed)
{
    if(speed > 0)
    {
        __HAL_TIM_SET_COMPARE(htimA, channelA, speed);
        __HAL_TIM_SET_COMPARE(htimB, channelB, 0);
    }
    else if(speed < 0)
    {
        __HAL_TIM_SET_COMPARE(htimA, channelA, 0);
        __HAL_TIM_SET_COMPARE(htimB, channelB, -speed);
    }
    else
    {
        stop();
    }
}

void Motor::stop()
{
    __HAL_TIM_SET_COMPARE(htimA, channelA, 0);
    __HAL_TIM_SET_COMPARE(htimB, channelB, 0);
}
