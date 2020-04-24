/*
 * i2c_master.c
 *
 *  Created on: Mar 7, 2018
 *      Author: Ocanath
 */
#include "i2c_master.h"

i2c_state i2c_master_state = I2C_TRANSMIT_READY;
uint8_t send_i2c_packet = 0;

uint8_t i2c_tx_cplt = 0;
uint8_t i2c_rx_cplt = 0;

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
	i2c_tx_cplt = 1;
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_master_state = I2C_TRANSMIT_READY;
	i2c_rx_cplt = 1;
}


void I2C1_Reset()
{
	//I2C1->CR1 |= I2C_CR1_START;
	//I2C1->CR1 |= I2C_CR1_STOP;
	I2C1->CR2 |= I2C_CR2_START;
	I2C1->CR2 |= I2C_CR2_STOP;
	//Maybe send a stop condition first?

	I2C1->CR1 &= ~(I2C_CR1_PE);	//zero out that bit to bring the peripheral back out of reset.
	while( (I2C1->CR1 & I2C_CR1_PE) != 0);	//spin until the bit goes low

	I2C1->CR1 |= I2C_CR1_PE;	//set the peripheral enable bit to re-enable i2c
	while( (I2C1->CR1 & I2C_CR1_PE)== 0);	//spin until the bit be high

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
		if(HAL_I2C_Master_Seq_Receive_IT(hi2c, slave_address, rx_data, rx_size, I2C_LAST_FRAME) != HAL_OK)
			ret = -1;
		else
			not_busy_ts = HAL_GetTick();

		i2c_master_state = I2C_RECIEVE_BUSY;
	}
	else if(i2c_master_state == I2C_TRANSMIT_READY)
	{
		if(HAL_I2C_Master_Seq_Transmit_IT(hi2c, slave_address, tx_data, tx_size, I2C_FIRST_FRAME) != HAL_OK)
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


static uint16_t i2c_addr_offset = 0;
static uint32_t i2c_frame_offset = 1;
void i2c_robot_master(uint8_t * addr_map, int num_frames,
		floatsend_t * q_i2c,	float * i2c_rx_previous,
		floatsend_t * tau, float * q)
{
	int rc = handle_i2c_master(&hi2c1, (addr_map[i2c_addr_offset] << 1), q_i2c[i2c_frame_offset].d, 4, tau[i2c_frame_offset].d, 4);	//This works!!!
	if(rc == -1 || hi2c1.ErrorCode != 0)
	{
		HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);				//and maybe doing this are critical for i2c_IT error recovery
		while(HAL_NVIC_GetPendingIRQ(I2C1_EV_IRQn)==1);
		I2C1_Reset();
		i2c_master_state = I2C_TRANSMIT_READY;

		i2c_addr_offset++;
		if(i2c_addr_offset >= num_frames-1)
			i2c_addr_offset = 0;
		i2c_frame_offset = i2c_addr_offset + 1;


		i2c_tx_cplt = 0;
		i2c_rx_cplt = 0;
	}
	if(i2c_tx_cplt == 1 && i2c_rx_cplt == 1)
	{
		uint32_t * fptr_chk = (uint32_t*)(&(q_i2c[i2c_frame_offset].v));	//mask pointer of the most recently read q value
		if( (*fptr_chk & 0x7f800000) == 0x7f800000)		//if the most recent q value is NaN
			q[i2c_frame_offset] = i2c_rx_previous[i2c_frame_offset];
		else	//if it is a number,
		{
			q[i2c_frame_offset] = q_i2c[i2c_frame_offset].v;	//load it into q so we can use it!
			i2c_rx_previous[i2c_frame_offset] = q[i2c_frame_offset];//only do this here... only works here anyway...
		}

		i2c_addr_offset++;
		if(i2c_addr_offset >= num_frames-1)
			i2c_addr_offset = 0;
		i2c_frame_offset = i2c_addr_offset + 1;

		i2c_tx_cplt = 0;
		i2c_rx_cplt = 0;
	}
}
