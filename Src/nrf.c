/*
 * nrf.c
 *
 *  Created on: Apr 19, 2020
 *      Author: Ocanath
 */
#include "nrf.h"

uint8_t tx_address[5] = {0x32,  0x77, 0x62, 0xFA, 0x00};
uint8_t rx_address[5] = {'h', 'e', 'X', 'r', 'o'};

nrf24l01 nrf;
nrf24l01_config config;
uint8_t rx_buf[NUM_BYTES_PAYLOAD];
uint8_t tx_buf[NUM_BYTES_PAYLOAD];
uint32_t comm_down_ts = 0;
static uint32_t rx_ts = 0;
static uint8_t nrf_new_packet = 0;
void nrf_packet_received_callback(nrf24l01 * dev, uint8_t * data)
{
	//	nrf_send_packet(&nrf, (uint8_t*)&tx_data_nrf);
	rx_ts = HAL_GetTick();
	dev->rx_busy = 0;
	nrf_new_packet = 1;
	//    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin,GPIO_PIN_RESET);
}

/*
 * callback function that handles the gpio interrupt count
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_7)
		nrf_irq_handler(&nrf);
}

void change_nrf_payload_length(uint8_t length)
{
    HAL_GPIO_WritePin(nrf.config.ce_port, nrf.config.ce_pin, GPIO_PIN_RESET);

	config.payload_length   = length;
	nrf.config.payload_length = length;
    nrf_set_rx_payload_width_p0(&nrf, nrf.config.payload_length);
    nrf_set_rx_payload_width_p1(&nrf, nrf.config.payload_length);

    HAL_GPIO_WritePin(nrf.config.ce_port, nrf.config.ce_pin, GPIO_PIN_SET);
}

void change_nrf_tx_address(const uint8_t * tx_address)
{
    HAL_GPIO_WritePin(nrf.config.ce_port, nrf.config.ce_pin, GPIO_PIN_RESET);
    nrf_set_rx_address_p0(&nrf, tx_address	);
    nrf_set_tx_address(&nrf, tx_address 	);
    HAL_GPIO_WritePin(nrf.config.ce_port, nrf.config.ce_pin, GPIO_PIN_SET);
}

void init_nrf(void)
{
	config.data_rate = NRF_DATA_RATE_1MBPS;		//hoping lower baud rate will be more reliable
	config.tx_power = NRF_TX_PWR_0dBm;
	config.crc_width = NRF_ADDR_WIDTH_5;
	config.payload_length   = NUM_BYTES_PAYLOAD;    // payload bytes. maximum is 32 bytes.
	config.retransmit_count = 15;   // maximum is 15 times
	config.retransmit_delay = 0x0F; // 4000us, LSB:250us
	config.rf_channel       = 0;
	config.rx_address       = rx_address;
	config.tx_address       = tx_address;
	config.rx_buffer        = rx_buf;

	config.spi = &hspi1;
	config.spi_timeout = 100;

	config.ce_port = NRF_CE_GPIO_Port;		//GPIOD
	config.ce_pin = NRF_CE_Pin;				//GPIO_PIN_2
	config.irq_port = NRF_INT_GPIO_Port;	//board ok
	config.irq_pin = NRF_INT_Pin;			//board ok
	config.csn_port = NRF_SS_GPIO_Port;		//
	config.csn_pin = NRF_SS_Pin;			//

	nrf_init(&nrf, &config);		//TODO: figure out why nrf init spin locks (probably bad cube init)

	change_nrf_payload_length(4);//unnecessary
	uint8_t tx_addr[5] = {0x01, 0x07, 0x33, 0xA0, 0};
	for(int attempt = 0; attempt < 5; attempt++)
	{
		for(int strip = 0; strip < 3; strip++)
		{
			tx_addr[4]=strip;
			change_nrf_tx_address((const uint8_t * )tx_addr);
			uint32_t r32 = 1023;
			uint32_t g32 = 0;
			uint32_t b32 = 1023;
			uint32_t tmp = ((r32 & 0x03FF) << 22) | ((g32 & 0x03FF) << 12) | ((b32 & 0x03FF) << 2);
			nrf_send_packet_noack(&nrf, (uint8_t *)(&tmp) );
			HAL_Delay(5);
		}
	}
}

