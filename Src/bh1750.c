/*
 * bh1750.c
 *
 *	The MIT License.
 *  Created on: 02.12.2018
 *      Author: Mateusz Salamon
 *		www.msalamon.pl
 *		mateusz@msalamon.pl
 *
 *	https://msalamon.pl/pomiar-natezenia-swiatla-z-wykorzystaniem-stm32/
 *	https://github.com/lamik/Light_Sensors_STM32
 */

#include "main.h"
#include "i2c.h"

#include "bh1750.h"

I2C_HandleTypeDef 	*bh1750_i2c;	// Handler to I2C interface
bh1750_mode 		Bh1750_Mode;	// Current sensor mode
uint8_t 			Bh1750_Mtreg;	// Current MT register value

//
//	Initialization.
//
BH1750_STATUS BH1750_Init(I2C_HandleTypeDef *hi2c)
{
	bh1750_i2c = hi2c;
	if(BH1750_OK == BH1750_Reset())
	{
		if(BH1750_OK == BH1750_SetMtreg(BH1750_DEFAULT_MTREG)) // Set default value;
			return BH1750_OK;
	}
	return BH1750_ERROR;
}

//
//	Reset all registers to default value.
//
BH1750_STATUS BH1750_Reset(void)
{
	uint8_t tmp = 0x07;
	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &tmp, 1, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}

//
//	Set the power state.
//	0 - sleep, low power.
//	1 - running.
//
BH1750_STATUS BH1750_PowerState(uint8_t PowerOn)
{
	PowerOn = (PowerOn? 1:0);
	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &PowerOn, 1, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}

//
//	Set the mode of converting. Look into bh1750_mode enum.
//
BH1750_STATUS BH1750_SetMode(bh1750_mode Mode)
{
	if(!((Mode >> 4) || (Mode >> 5))) return BH1750_ERROR;
	if((Mode & 0x0F) > 3) return BH1750_ERROR;

	Bh1750_Mode = Mode;
	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &Mode, 1, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}

//
//	Set the Measurement Time register. It allows to increase or decrease the sensitivity.
//
BH1750_STATUS BH1750_SetMtreg(uint8_t Mtreg)
{
	HAL_StatusTypeDef retCode;
	if (Mtreg < 31 || Mtreg > 254) {
		return BH1750_ERROR;
	}

	Bh1750_Mtreg = Mtreg;

	uint8_t tmp[2];

	tmp[0] = (0x40 | (Mtreg >> 5));
	tmp[1] = (0x60 | (Mtreg & 0x1F));

	retCode = HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &tmp[0], 1, 10);
	if (HAL_OK != retCode) {
		return BH1750_ERROR;
	}

	retCode = HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &tmp[1], 1, 10);
	if (HAL_OK == retCode) {
		return BH1750_OK;
	}

	return BH1750_ERROR;
}

//
//	Trigger the conversion in manual modes.
//	For low resolution conversion time is typical 16 ms,
//	for high resolution 120 ms. You need to wait until read the measurement value.
//	There is no need to exit low power mode for manual conversion. It makes automatically.
//
BH1750_STATUS BH1750_TriggerManualConversion(void)
{
	if(BH1750_OK == BH1750_SetMode(Bh1750_Mode))
	{
		return BH1750_OK;
	}
	return BH1750_ERROR;
}

//
//	Read the converted value and calculate the result.
//
BH1750_STATUS BH1750_ReadLight(float *Result)
{
	float result;
	uint8_t tmp[2];

	if(HAL_OK == HAL_I2C_Master_Receive(bh1750_i2c, BH1750_ADDRESS, tmp, 2, 10))
	{
		result = (tmp[0] << 8) | (tmp[1]);

		if(Bh1750_Mtreg != BH1750_DEFAULT_MTREG)
		{
			result *= (float)((uint8_t)BH1750_DEFAULT_MTREG/(float)Bh1750_Mtreg);
		}

		if(Bh1750_Mode == ONETIME_HIGH_RES_MODE_2 || Bh1750_Mode == CONTINUOUS_HIGH_RES_MODE_2)
		{
			result /= 2.0;
		}

		*Result = result / (float)BH1750_CONVERSION_FACTOR;
		return BH1750_OK;
	}
	return BH1750_ERROR;
}
