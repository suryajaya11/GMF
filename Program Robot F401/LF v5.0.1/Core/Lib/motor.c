/*
 * motor.c
 *
 *  Created on: Oct 11, 2022
 *      Author: Surya
 */
#include "motor.h"
#include "main.h"
#include "stdio.h"
#include "stdlib.h"

int16_t enc_speed[2];
int16_t target_speed_motor[2];
int16_t pwm_motor[2];

//control
int16_t error_motor[2];
int16_t prev_error_motor[2];
float sum_error_motor[2];

//constants
float Kp_motor[2] = {40, 40};
float Ki_motor[2] = {10, 10};
float Ki_lim_motor[2] = {60, 60};

void Motor(){
	enc_speed[0] = (int16_t) TIM2->CNT;
	enc_speed[1] = (int16_t) TIM5->CNT;

	TIM2->CNT = 0;
	TIM5->CNT = 0;

	for(uint8_t mtr = 0; mtr < 2; mtr++){
		prev_error_motor[mtr] = error_motor[mtr];
		error_motor[mtr] = target_speed_motor[mtr] - enc_speed[mtr];

		sum_error_motor[mtr] += error_motor[mtr] * Ki_motor[mtr];

		float lim_integral = abs(target_speed_motor[mtr]) * Ki_lim_motor[mtr];
		if(sum_error_motor[mtr] > lim_integral) sum_error_motor[mtr] = lim_integral;
		if(sum_error_motor[mtr] < -lim_integral) sum_error_motor[mtr] = -lim_integral;

		pwm_motor[mtr] = error_motor[mtr] * Kp_motor[mtr] + sum_error_motor[mtr];

		if(pwm_motor[mtr] > 499) pwm_motor[mtr] = 499;
		if(pwm_motor[mtr] < -499) pwm_motor[mtr] = -499;
	}

	TIM4->CCR1 = pwm_motor[0] < 0 ? (499 + pwm_motor[0]) : (pwm_motor[0]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, pwm_motor[0] < 0);

	TIM4->CCR2 = pwm_motor[1] < 0 ? (499 + pwm_motor[1]) : (pwm_motor[1]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, pwm_motor[1] < 0);
}
