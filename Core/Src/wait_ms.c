/*
 * wait_ms.c
 *
 *  Created on: Sep 21, 2022
 *      Author: Ryu
 */
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "stdio.h"
#include "wait_ms.h"

volatile uint32_t g_timCount;


void pl_timer_init(void){
	HAL_TIM_Base_Start_IT(&htim6);
}

void pl_timer_count(void){
	g_timCount++;
}

void wait_ms(uint32_t wait_time) {
	g_timCount= 0;
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(g_timCount< wait_time) {
	}
}

void TimReset(void)
{
	g_timCount=0;
}

float Counter(void)
{
	return 0.001*(float)g_timCount;
}

