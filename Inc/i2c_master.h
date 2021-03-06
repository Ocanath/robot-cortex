/*
 * i2c_master.h
 *
 *  Created on: Mar 7, 2018
 *      Author: Ocanath
 */


#ifndef I2C_MASTER_H_
#define I2C_MASTER_H_
#include "init.h"
#include "uart-disp-tools.h"

//#define I2C_RECIEVE_READY 	0
//#define I2C_RECIEVE_BUSY 	1
//#define I2C_TRANSMIT_READY 	2
//#define I2C_TRANSMIT_BUSY 	3

typedef enum {I2C_RECIEVE_READY = 0,I2C_RECIEVE_BUSY = 1,I2C_TRANSMIT_READY = 2, I2C_TRANSMIT_BUSY = 3  }i2c_state;

#define RX_SIZE 4
#define TX_SIZE 4

extern uint8_t i2c_tx_cplt;
extern uint8_t i2c_rx_cplt;

//#define I2C_BASE 0x50
//#define I2C_BASE 0x40

//extern uint8_t i2c_float_rx_buf[RX_SIZE];
//extern uint8_t i2c_float_tx_buf[TX_SIZE];

typedef union
{
	float v;
	uint8_t d[4];
}floatsend_t;

extern i2c_state i2c_master_state;
uint8_t send_i2c_packet;
uint8_t rx_ready ;
uint8_t tx_ready;
extern uint32_t i2c_ok_ts;
extern uint16_t cur_address;
extern uint32_t not_busy_ts;
extern uint8_t gl_addr_used[128];

void I2C1_Reset();
uint8_t is_nan(float v);
void i2c_packet_start();
uint8_t i2c_packet_sent();
int handle_i2c_master(I2C_HandleTypeDef * hi2c, uint16_t slave_address, uint8_t * rx_data, int rx_size, uint8_t * tx_data, int tx_size );
void handle_i2c(I2C_HandleTypeDef * hi2c, uint16_t base_address, uint8_t * i2c_float_rx_buf, uint8_t * i2c_float_tx_buf, int num_bytes);
uint32_t i2c_busy_time();
int check_i2c_devices(void);
void disp_i2c_map(void);

void i2c_robot_master(uint8_t * addr_map, int num_frames,
		floatsend_t * q_i2c,	float * i2c_rx_previous,
		floatsend_t * tau, float * q);

#endif /* I2C_MASTER_H_ */
