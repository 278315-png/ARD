#include "main.h"
#ifndef INC_C22_H_
#define INC_C22_H_
#define AUDIO_I2C_ADDR	0x94

void cs43l22_write(I2C_HandleTypeDef *hi2c,uint8_t reg, uint8_t value);
void cs43l22_init(I2C_HandleTypeDef *hi2c);
void cs43l22_power_down(I2C_HandleTypeDef *hi2c);
#endif
