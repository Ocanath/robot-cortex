/*
 * init.h
 *
 *  Created on: Jun 7, 2019
 *      Author: Ocanath
 */

#ifndef INIT_H_
#define INIT_H_
#include "main.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;


void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_I2C1_Init(void);
void MX_SPI1_Init(void);
void MX_TIM1_Init(void);
void MX_USART2_UART_Init(void);

#endif /* INIT_H_ */
