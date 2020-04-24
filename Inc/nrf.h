/*
 * nrf.h
 *
 *  Created on: Apr 19, 2020
 *      Author: Ocanath
 */

#ifndef NRF_H_
#define NRF_H_
#define NUM_BYTES_PAYLOAD 3
#include "init.h"
#include "nrf24l01.h"

extern uint8_t rx_buf[NUM_BYTES_PAYLOAD];
extern uint8_t tx_buf[NUM_BYTES_PAYLOAD];
extern uint32_t comm_down_ts;

void init_nrf(void);
void change_nrf_tx_address(const uint8_t * tx_address);
void change_nrf_payload_length(uint8_t length);


#endif /* NRF_H_ */
