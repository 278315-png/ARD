#include"c22.h"
void cs43l22_write(I2C_HandleTypeDef *hi2c,uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(hi2c, AUDIO_I2C_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}
void cs43l22_init(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_WritePin(AUDIO_RST_GPIO_Port, AUDIO_RST_Pin, GPIO_PIN_SET);

	cs43l22_write(hi2c,0x04, 0xaf);
	cs43l22_write(hi2c,0x06, 0x07);
	cs43l22_write(hi2c,0x02, 0x9e);
}
void cs43l22_power_down(I2C_HandleTypeDef *hi2c)
{
    cs43l22_write(hi2c, 0x02, 0x01);
    HAL_GPIO_WritePin(AUDIO_RST_GPIO_Port, AUDIO_RST_Pin, GPIO_PIN_RESET);
}



