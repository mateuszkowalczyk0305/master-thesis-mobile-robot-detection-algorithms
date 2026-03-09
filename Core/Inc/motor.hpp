/*
 * motor.hpp
 *
 *  Created on: Mar 9, 2026
 *      Author: m_kowalczyk
 */

#pragma once

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include "main.h"

class Motor
{
private:

    TIM_HandleTypeDef* htimA;
    uint32_t channelA;

    TIM_HandleTypeDef* htimB;
    uint32_t channelB;

public:

    Motor(TIM_HandleTypeDef* timerA,
          uint32_t pwm_channelA,
          TIM_HandleTypeDef* timerB,
          uint32_t pwm_channelB);

    void setSpeed(int speed);

    void stop();
};
#endif /* INC_MOTOR_HPP_ */
