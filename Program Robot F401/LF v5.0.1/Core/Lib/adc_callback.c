/*
 * adc_callback.c
 *
 *  Created on: Oct 16, 2022
 *      Author: Surya
 */
#include "adc_callback.h"

uint32_t buf_adc[3];
uint16_t left_sens, right_sens;
uint16_t left_sens_ar[6], right_sens_ar[6];
uint8_t adc_ptr = 0;

extern void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	left_sens = buf_adc[2];
	right_sens = buf_adc[1];

	left_sens_ar[adc_ptr] = left_sens;
	right_sens_ar[adc_ptr] = right_sens;

	adc_ptr++;
	if(adc_ptr == 6) adc_ptr = 0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (adc_ptr >> 2) & 1);  //c
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (adc_ptr >> 1) & 1);  //b
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (adc_ptr >> 0) & 1); //a
}

