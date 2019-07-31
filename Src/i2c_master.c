/*
 * i2c_master.c
 *
 *  Created on: Mar 7, 2018
 *      Author: Ocanath
 */
#include "i2c_master.h"

i2c_state i2c_master_state = I2C_TRANSMIT_READY;
uint8_t send_i2c_packet = 0;

void i2c_packet_start()
{
	send_i2c_packet = 1;
}

uint8_t i2c_packet_sent()
{
	return (!send_i2c_packet)&1;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_master_state = I2C_RECIEVE_READY;
	send_i2c_packet = 0;
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_master_state = I2C_TRANSMIT_READY;
}


void I2C1_Reset()
{
	I2C1->CR1 |= I2C_CR1_START;
	I2C1->CR1 |= I2C_CR1_STOP;

	//Maybe send a stop condition first?
	I2C1->CR1 = I2C1->CR1 | (0x8000);	//set the swrst bit to reset the bus
	while( (I2C1->CR1 & 0x8000)== 0);	//spin until the bit be high

	I2C1->CR1 = I2C1->CR1 & (~0x8000);	//zero out that bit to bring the peripheral back out of reset.
	while( (I2C1->CR1 & 0x8000) != 0);	//spin until the bit goes low

	hi2c1.State = HAL_I2C_STATE_RESET;
	MX_I2C1_Init();
}
/*
 * for legacy support, but buggy (data alignment wise)
 */
uint32_t not_busy_ts = 0;
int handle_i2c_master(I2C_HandleTypeDef * hi2c, uint16_t slave_address, uint8_t * rx_data, int rx_size, uint8_t * tx_data, int tx_size )
{
	int ret = 0;
	if(i2c_master_state == I2C_RECIEVE_READY)	//first state. only progress if you are allowed by send_i2c_packet
	{
		if(HAL_I2C_Master_Receive_IT(hi2c, slave_address, rx_data, rx_size) != HAL_OK)
			ret = -1;
		else
			not_busy_ts = HAL_GetTick();

		i2c_master_state = I2C_RECIEVE_BUSY;
	}
	else if(i2c_master_state == I2C_TRANSMIT_READY)
	{
		if(HAL_I2C_Master_Transmit_IT(hi2c, slave_address, tx_data, tx_size) != HAL_OK)
			ret = -1;
		else
			not_busy_ts = HAL_GetTick();

		i2c_master_state = I2C_TRANSMIT_BUSY;
	}

	if(HAL_GetTick() - not_busy_ts > 10)
		ret = -1;

	return ret;
}




uint32_t i2c_busy_time()
{
	return HAL_GetTick()- i2c_ok_ts;
}
