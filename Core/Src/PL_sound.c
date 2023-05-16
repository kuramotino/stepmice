/*
 * PL_sound.c
 *
 *  Created on: 2022/11/23
 *      Author: Ryu
 */
#include "PL_sound.h"
#include "tim.h"

void sound(float c, int delay)
{
	int count=(uint16_t)(1000000.0/c-1.0);
	__HAL_TIM_SET_AUTORELOAD(&htim15, count);
	__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,count/2);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_Delay(delay);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
}

