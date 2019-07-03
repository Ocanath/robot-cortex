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


void reset_i2c()
{
	I2C1->CR1 = I2C1->CR1 & (~0x1);	//clear bit zero for bus reset
	while((I2C1->CR1 & 1) != 0);	//spin until it actually resets
	I2C1->CR1 = I2C1->CR1 | 1;		//re-enable it
	MX_I2C1_Init();
}

/*
 * for legacy support, but buggy (data alignment wise)
 */
static int addr_offset = 0;
int handle_i2c_master(I2C_HandleTypeDef * hi2c, uint16_t base_addr, uint16_t num_slaves, uint8_t * rx_data, int rx_size, uint8_t * tx_data, int tx_size)
{
	int ret = 0;
	if(i2c_master_state == I2C_RECIEVE_READY)	//first state. only progress if you are allowed by send_i2c_packet
	{
		addr_offset++;
		if(addr_offset >= num_slaves)
			addr_offset = 0;
		if(HAL_I2C_Master_Receive_IT(hi2c, (base_addr+addr_offset)<<1, rx_data, rx_size) != HAL_OK)
			ret = -1;
		i2c_master_state = I2C_RECIEVE_BUSY;
	}
	else if(i2c_master_state == I2C_TRANSMIT_READY)
	{
		if(HAL_I2C_Master_Transmit_IT(hi2c, (base_addr+addr_offset)<<1, tx_data, tx_size) != HAL_OK)
			ret = -1;
		i2c_master_state = I2C_TRANSMIT_BUSY;
	}
	return ret;
}

floatsend_t i2c_float_rx_buf;
floatsend_t i2c_float_tx_buf;
uint16_t motor_offset = 0;
uint32_t i2c_ok_ts = 0;
uint16_t cur_address = 0;
void handle_i2c_motor_chain(I2C_HandleTypeDef * hi2c, uint16_t base_address, uint16_t num_motors, float * q, float * tau, int * sign)
{

	if(i2c_master_state == I2C_RECIEVE_READY)	//first state. only progress if you are allowed by send_i2c_packet
	{
		motor_offset++;
		if(motor_offset >= num_motors)
			motor_offset = 0;
		cur_address =(base_address+motor_offset)<<1;
		if(HAL_I2C_Master_Receive_IT(hi2c, cur_address, i2c_float_rx_buf.d, RX_SIZE) == HAL_OK)	//HAL_I2C_Master_Receive_IT(hi2c, cur_address, (uint8_t*)&(q[motor_offset]), sizeof(float));
			i2c_ok_ts = HAL_GetTick();
		i2c_master_state = I2C_RECIEVE_BUSY;
	}
	else if (i2c_master_state == I2C_TRANSMIT_READY)
	{
		q[motor_offset] = i2c_float_rx_buf.v;
		if(sign[motor_offset] < 0)
			q[motor_offset] = -q[motor_offset];
		cur_address =(base_address+motor_offset)<<1;
		if(HAL_I2C_Master_Transmit_IT(hi2c, cur_address, i2c_float_tx_buf.d,TX_SIZE) == HAL_OK)
			i2c_ok_ts = HAL_GetTick();
		i2c_master_state = I2C_TRANSMIT_BUSY;
	}
	i2c_float_tx_buf.v = tau[motor_offset];

	if(HAL_GetTick() - i2c_ok_ts > 5)
	{
		HAL_GPIO_WritePin(STAT_GPIO_Port,STAT_Pin,1);

		i2c_master_state = I2C_TRANSMIT_READY;

		I2C1->CR1 = I2C1->CR1 & (~0x1);	//clear bit zero for bus reset
		while((I2C1->CR1 & 1) != 0);	//spin until it actually resets
		I2C1->CR1 = I2C1->CR1 | 1;		//re-enable it
		MX_I2C1_Init();
//		hi2c->State = HAL_I2C_STATE_RESET;

	}
	else
		HAL_GPIO_WritePin(STAT_GPIO_Port,STAT_Pin,0);
}


uint32_t i2c_busy_time()
{
	return HAL_GetTick()- i2c_ok_ts;
}
