/*
 * max44009.h
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

#ifndef MAX44009_H_
#define MAX44009_H_

#define MAX44009_ADDRESS			(0x4A<<1)

//
//	Registers
//
#define MAX44009_INTERRUPT_STATUS_REGISTER	0x00
#define MAX44009_INTERRPUT_ENABLE_REGISTER	0x01
#define MAX44009_CONFIGURATION_REGISTER		0x02
#define MAX44009_LUX_HIGH_BYTE_REGISTER		0x03
#define MAX44009_LUX_LOW_BYTE_REGISTER		0x04
#define MAX44009_UPPER_THRESHOLD_REGISTER	0x05
#define MAX44009_LOWER_THRESHOLD_REGISTER	0x06
#define MAX44009_THRESHOLD_TIMER_REGISTER	0x07


typedef enum {
	MAX44009_OK		= 0,
	MAX44009_ERROR	= 1
} MAX44009_STATUS;

typedef enum
{
	INTEGRATION_TIME_6_25_MS = 0x07,	// Manual mode only
	INTEGRATION_TIME_12_5_MS = 0x06,	// Manual mode only
	INTEGRATION_TIME_25_MS 	 = 0x05,	// Manual mode only
	INTEGRATION_TIME_50_MS 	 = 0x04,	// Manual mode only
	INTEGRATION_TIME_100_MS  = 0x03,	// This is a preferred mode for high-brightness applications
	INTEGRATION_TIME_200_MS  = 0x02,
	INTEGRATION_TIME_400_MS  = 0x01,
	INTEGRATION_TIME_800_MS  = 0x00		// This is a preferred mode for boosting low-light sensitivity
} max44009_timer;


MAX44009_STATUS MAX44009_Init(I2C_HandleTypeDef *hi2c);

MAX44009_STATUS MAX44009_ReadConfigurationRegister(uint8_t *Config);
MAX44009_STATUS MAX44009_WriteConfigurationRegister(uint8_t Config);

MAX44009_STATUS MAX44009_ContinuousMode(uint8_t Enable);
MAX44009_STATUS MAX44009_ManualConfiguration(uint8_t Enable);
MAX44009_STATUS MAX44009_CurrentDivisionRatio(uint8_t Enable);
MAX44009_STATUS MAX44009_IntegrationTime(max44009_timer Timer);

MAX44009_STATUS MAX44009_ReadLightLowResolution(float *Result);
MAX44009_STATUS MAX44009_ReadLightHighResolution(float *Result);

MAX44009_STATUS MAX44009_ReadInterruptStatus(uint8_t *Status);
MAX44009_STATUS MAX44009_WriteInterruptEnable(uint8_t Enable);

MAX44009_STATUS MAX44009_SetUpperThreshold(float Threshold);
MAX44009_STATUS MAX44009_SetLowerThreshold(float Threshold);
MAX44009_STATUS MAX44009_SetThresholdTimer(uint8_t Timer);

#endif /* MAX44009_H_ */
