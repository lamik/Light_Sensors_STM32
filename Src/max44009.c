/*
 * max44009.c
 *
 *	The MIT License.
 *  Created on: 06.12.2018
 *      Author: Mateusz Salamon
 *		www.msalamon.pl
 *		mateusz@msalamon.pl
 *
 *	https://msalamon.pl/pomiar-natezenia-swiatla-z-wykorzystaniem-stm32/
 *	https://github.com/lamik/Light_Sensors_STM32
 */


#include "main.h"
#include "i2c.h"

#include <math.h>
#include "max44009.h"

I2C_HandleTypeDef 	*max44009_i2c;	// Handler to I2C interface

//
//	Initialization.
//
MAX44009_STATUS MAX44009_Init(I2C_HandleTypeDef *hi2c)
{
	max44009_i2c = hi2c;

	return MAX44009_OK;
}

//
//	Read from Interrupt Status register.
//	0 - No interrupt occurred.
//	1 - Interrupt occurred.
//
MAX44009_STATUS MAX44009_ReadInterruptStatus(uint8_t *Status)
{
	if(HAL_OK == HAL_I2C_Mem_Read(max44009_i2c, MAX44009_ADDRESS, MAX44009_INTERRUPT_STATUS_REGISTER, 1, Status, 1, 10))
	{
		return MAX44009_OK;
	}

	return MAX44009_ERROR;
}

//
//	Write to Interrupt Enable register.
//	0 - Interrupt disable.
//	1 - Interrupt Enable.
//
MAX44009_STATUS MAX44009_WriteInterruptEnable(uint8_t Enable)
{
	Enable = (Enable? 1:0);
	if(HAL_OK == HAL_I2C_Mem_Write(max44009_i2c, MAX44009_ADDRESS, MAX44009_INTERRPUT_ENABLE_REGISTER, 1, &Enable, 1, 10))
	{
		return MAX44009_OK;
	}

	return MAX44009_ERROR;
}

//
//	Read from Configuration register.
//
MAX44009_STATUS MAX44009_ReadConfigurationRegister(uint8_t *Config)
{
	if(HAL_OK == HAL_I2C_Mem_Read(max44009_i2c, MAX44009_ADDRESS, MAX44009_CONFIGURATION_REGISTER, 1, Config, 1, 10))
	{
		return MAX44009_OK;
	}

	return MAX44009_ERROR;
}

//
//	Write to Configuration register.
//
MAX44009_STATUS MAX44009_WriteConfigurationRegister(uint8_t Config)
{
	if(HAL_OK == HAL_I2C_Mem_Write(max44009_i2c, MAX44009_ADDRESS, MAX44009_CONFIGURATION_REGISTER, 1, &Config, 1, 10))
	{
		return MAX44009_OK;
	}

	return MAX44009_ERROR;
}

//
//	Set Continuous Mode bit in Configuration register.
//	0 - Continuous mode disable.
//	1 - Continuous mode enable.
//
MAX44009_STATUS MAX44009_ContinuousMode(uint8_t Enable)
{
	uint8_t Config;

	Enable = (Enable? 1:0);

	if(MAX44009_OK == MAX44009_ReadConfigurationRegister(&Config))
	{
		Config &= 0x7F; // Clear BIT7
		Config |= (Enable<<7);
		if(MAX44009_OK == MAX44009_WriteConfigurationRegister(Config))
		{
			return MAX44009_OK;
		}
	}
	return MAX44009_ERROR;
}

//
//	Set Manual Convertion bit in Configuration register.
//	0 - Manual Convertion mode disable.
//	1 - Manual Convertion mode enable.
//
MAX44009_STATUS MAX44009_ManualConfiguration(uint8_t Enable)
{
	uint8_t Config;

	Enable = (Enable? 1:0);

	if(MAX44009_OK == MAX44009_ReadConfigurationRegister(&Config))
	{
		Config &= 0xBF; // Clear BIT6
		Config |= (Enable<<6);
		if(MAX44009_OK == MAX44009_WriteConfigurationRegister(Config))
		{
			return MAX44009_OK;
		}
	}
	return MAX44009_ERROR;
}

//
//	Set Current Division Ratio bit in Configuration register.
//	0 - Convertion Ratio disable.
//	1 - Convertion Ratio enable. ADC Current is divided by 8.
//
MAX44009_STATUS MAX44009_CurrentDivisionRatio(uint8_t Enable)
{
	uint8_t Config;

	Enable = (Enable? 1:0);

	if(MAX44009_OK == MAX44009_ReadConfigurationRegister(&Config))
	{
		Config &= 0xF7; // Clear BIT3
		Config |= (Enable<<3);
		if(MAX44009_OK == MAX44009_WriteConfigurationRegister(Config))
		{
			return MAX44009_OK;
		}
	}
	return MAX44009_ERROR;
}

//
//	Set signal integration time.
//	In automatic mode (MANUAL = 0), integration time is automatically selected by the on-chip algorithm to be either
//	100ms/200ms/400ms/800ms. In manual mode, integration time can be varied by the user all the way from 6.25ms to
//	800ms. See max44009_timer enum.
//
MAX44009_STATUS MAX44009_IntegrationTime(max44009_timer Timer)
{
	uint8_t Config;

	if(Timer > 7) Timer = 7;

	if(MAX44009_OK == MAX44009_ReadConfigurationRegister(&Config))
	{
		Config &= 0xF8; // Clear BIT[0:2]
		Config |= Timer;
		if(MAX44009_OK == MAX44009_WriteConfigurationRegister(Config))
		{
			return MAX44009_OK;
		}
	}
	return MAX44009_ERROR;
}

//
//	Read converted value.
//	There is read only higher register which only allows to use lower resolution.
//
MAX44009_STATUS MAX44009_ReadLightLowResolution(float *Result)
{
	uint8_t tmp;

	if(HAL_OK == HAL_I2C_Mem_Read(max44009_i2c, MAX44009_ADDRESS, MAX44009_LUX_HIGH_BYTE_REGISTER, 1, &tmp, 1, 10))
	{
		uint8_t exponent = tmp>>4;
		uint32_t mantisa = tmp & 0x0F;
		mantisa <<= exponent;

		*Result = ((float)mantisa * 0.72);
		return MAX44009_OK;
	}
	return MAX44009_ERROR;
}

//
//	Read converted value.
//	There is read both value registers. Full resolution.
//
MAX44009_STATUS MAX44009_ReadLightHighResolution(float *Result)
{
	uint8_t tmp[2];

	if(HAL_OK == HAL_I2C_Mem_Read(max44009_i2c, MAX44009_ADDRESS, MAX44009_LUX_HIGH_BYTE_REGISTER, 1, tmp, 1, 10))
	{
		if(HAL_OK == HAL_I2C_Mem_Read(max44009_i2c, MAX44009_ADDRESS, MAX44009_LUX_LOW_BYTE_REGISTER, 1, tmp+1, 1, 10))
		{
			uint8_t exponent = tmp[0]>>4;
			uint32_t mantisa = ((tmp[0] & 0x0F)<<4) + (tmp[1] & 0x0F);
			mantisa <<= exponent;

			*Result = ((float)(mantisa) * 0.045);
			return MAX44009_OK;
		}
	}
	return MAX44009_ERROR;
}

//
//	Helper function.
//	Set Threshold register.
//
MAX44009_STATUS MAX44009_SetThreshold(uint8_t Register, float Threshold)
{
	uint8_t result;
	uint32_t mantisa = (uint32_t)(round(Threshold * 22.222222));
	uint8_t exponent = 0;

	while(mantisa > 255)
	{
		mantisa >>= 1;
		exponent++;
	}

	mantisa = (mantisa>>4) & 0x0F;
	exponent = (exponent<<4);
	result = exponent | mantisa;

	if(HAL_OK == HAL_I2C_Mem_Write(max44009_i2c, MAX44009_ADDRESS, Register, 1, &result, 1, 10))
	{
		return MAX44009_OK;
	}
	return MAX44009_ERROR;
}

//
//	Set Upper Threshold register value.
//	If measurement is above this value, interrupt is generated.
//
MAX44009_STATUS MAX44009_SetUpperThreshold(float Threshold)
{
	if(MAX44009_OK == MAX44009_SetThreshold(MAX44009_UPPER_THRESHOLD_REGISTER, Threshold))
	{
		return MAX44009_OK;
	}
	return MAX44009_ERROR;
}

//
//	Set Lower Threshold register value.
//	If measurement is below this value, interrupt is generated.
//
MAX44009_STATUS MAX44009_SetLowerThreshold(float Threshold)
{
	if(MAX44009_OK == MAX44009_SetThreshold(MAX44009_LOWER_THRESHOLD_REGISTER, Threshold))
	{
		return MAX44009_OK;
	}
	return MAX44009_ERROR;
}

//
//	Set Threshold Timer register value.
//	Time in light value must be higher/lower than set Threshold registers to generate the interrupt.
//
MAX44009_STATUS MAX44009_SetThresholdTimer(uint8_t Timer)
{
	if(HAL_OK == HAL_I2C_Mem_Write(max44009_i2c, MAX44009_ADDRESS, MAX44009_THRESHOLD_TIMER_REGISTER, 1, &Timer, 1, 10))
	{
		return MAX44009_OK;
	}
	return MAX44009_ERROR;
}



