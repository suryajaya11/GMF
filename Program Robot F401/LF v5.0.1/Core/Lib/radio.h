/*
 * radio.h
 *
 *  Created on: Oct 12, 2022
 *      Author: Surya
 */

#ifndef INC_RADIO_H_
#define INC_RADIO_H_

#include "main.h"

extern uint8_t serial_byte;
extern uint16_t channels[16];

void parse_frame();
int32_t map_val(float val, float in_min, float in_max, float out_min, float out_max);

#endif /* INC_RADIO_H_ */
