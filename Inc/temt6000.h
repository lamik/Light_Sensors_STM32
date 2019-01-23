/*
 * temt6000.h
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

#ifndef TEMT6000_H_
#define TEMT6000_H_

#define TEMT6000_ADC_MAX_VALUE	4096
#define TEMT6000_POWER_SUPPLY	3.3
#define TEMT6000_RESISTOR_OHMS	1000
#define TEMT6000_ADC_SAMPLES 8

typedef enum {
	TEMT6000_OK		= 0,
	TEMT6000_ERROR	= 1
} TEMT6000_STATUS;

TEMT6000_STATUS TEMT6000_Init(ADC_HandleTypeDef *hadc);

TEMT6000_STATUS TEMT6000_ReadLight(float *Result);
#endif /* TEMT6000_H_ */
