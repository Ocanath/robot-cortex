/*
 * i2c_master.h
 *
 *  Created on: Mar 7, 2018
 *      Author: Ocanath
 */


#ifndef I2C_MASTER_H_
#define I2C_MASTER_H_
#include "init.h"

//#define I2C_RECIEVE_READY 	0
//#define I2C_RECIEVE_BUSY 	1
//#define I2C_TRANSMIT_READY 	2
//#define I2C_TRANSMIT_BUSY 	3

typedef union
{
	float v;
	uint8_t d[4];
}floatsend_t;

typedef enum {I2C_RECIEVE_READY = 0,I2C_RECIEVE_BUSY = 1,I2C_TRANSMIT_READY = 2, I2C_TRANSMIT_BUSY = 3  }i2c_state;

#define RX_SIZE 4
#define TX_SIZE 4


//#define I2C_BASE 0x50
//#define I2C_BASE 0x40

//extern uint8_t i2c_float_rx_buf[RX_SIZE];
//extern uint8_t i2c_float_tx_buf[TX_SIZE];
extern floatsend_t i2c_float_rx_buf;
extern floatsend_t i2c_float_tx_buf;

extern i2c_state i2c_master_state;
uint8_t send_i2c_packet;
uint8_t rx_ready ;
uint8_t tx_ready;
extern uint32_t i2c_ok_ts;
extern uint16_t cur_address;

void reset_i2c();
void i2c_packet_start();
uint8_t i2c_packet_sent();
int handle_i2c_master(I2C_HandleTypeDef * hi2c, uint16_t base_addr, uint16_t num_slaves, uint8_t * rx_data, int rx_size, uint8_t * tx_data, int tx_size);
void handle_i2c_motor_chain(I2C_HandleTypeDef * hi2c, uint16_t base_address, uint16_t num_motors, float * q, float * tau, int * sign);

void handle_i2c(I2C_HandleTypeDef * hi2c, uint16_t base_address, uint8_t * i2c_float_rx_buf, uint8_t * i2c_float_tx_buf, int num_bytes);
uint32_t i2c_busy_time();

#endif /* I2C_MASTER_H_ */
