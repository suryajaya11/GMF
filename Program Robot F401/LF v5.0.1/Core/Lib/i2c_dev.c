/*
 * i2c_dev.c
 *
 *  Created on: Oct 17, 2022
 *      Author: Surya
 */

#include "i2c_dev.h"
#include "i2c.h"

void EEPROM_write(){
	uint8_t data[2] = {0x01, 0x02};
	HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x0000, 2, data, 2, 1000);
}
