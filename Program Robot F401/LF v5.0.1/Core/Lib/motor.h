/*
 * motor.h
 *
 *  Created on: Oct 11, 2022
 *      Author: Surya
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

extern int16_t pwm_motor[2];
extern int16_t target_speed_motor[2];

void Motor();

#endif /* INC_MOTOR_H_ */
