/*
 * temt6000.c
 *
 *	The MIT License.
 *  Created on: 19.01.2019
 *      Author: Mateusz Salamon
 *		www.msalamon.pl
 *		mateusz@msalamon.pl
 *
 *	https://msalamon.pl/pomiar-natezenia-swiatla-z-wykorzystaniem-stm32/
 *	https://github.com/lamik/Light_Sensors_STM32
 */

#include "main.h"
#include "adc.h"

#include "temt6000.h"

ADC_HandleTypeDef *temt6000_adc;

volatile uint16_t AdcValue[TEMT6000_ADC_SAMPLES];

TEMT6000_STATUS TEMT6000_Init(ADC_HandleTypeDef *hadc)
{
	temt6000_adc = hadc;

	if(HAL_OK == HAL_ADC_Start_DMA(temt6000_adc, (uint32_t*)AdcValue, TEMT6000_ADC_SAMPLES))
		return TEMT6000_OK;

	return TEMT6000_ERROR;
}

TEMT6000_STATUS TEMT6000_ReadLight(float *Result)
{
	uint32_t AdcAverage;
	uint8_t i;

	AdcAverage = 0;

	for(i = 0; i < TEMT6000_ADC_SAMPLES; i++)
	{
		AdcAverage += AdcValue[i];
	}

	AdcAverage /= TEMT6000_ADC_SAMPLES;

	*Result = ((((float)AdcAverage / (float)TEMT6000_ADC_MAX_VALUE) * (float)TEMT6000_POWER_SUPPLY) / TEMT6000_RESISTOR_OHMS) * 2000000.0;

	return TEMT6000_OK;
}
