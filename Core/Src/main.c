#include "init.h"
#include "CAN.h"
#include "rgb.h"

typedef struct mat4
{
	float m[4][4];
}mat4;

typedef struct joint
{
	uint16_t id;
	int frame;
	mat4 h0_i;
	mat4 him1_i;
	floatsend_t q;
	floatsend_t tau;
	uint8_t led_cmd;
}joint;

#define NUM_JOINTS 2

joint chain[NUM_JOINTS] = {
		{
			.id = 0x7FF-23,
			.frame = 1,
			.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
			.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
			.q = {.v = 0.f},
			.tau = {.v = 0.f},
			.led_cmd = LED_OFF
		},
		{
			.id = 0x7FF-24,
			.frame = 2,
			.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
			.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
			.q = {.v = 0.f},
			.tau = {.v = 0.f},
			.led_cmd = LED_OFF
		}
};

int main(void)
{

	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	HAL_Delay(100);
	CAN_Init();
//	MX_CAN1_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();

	rgb_play((rgb_t){255,255,255});

	uint32_t can_tx_ts = 0;
	int joint_idx = 0;
	uint32_fmt_t can_tx_data = {0};
	uint32_fmt_t can_rx_data = {0};
	while (1)
	{
		if(HAL_GetTick()>can_tx_ts)
		{
//			can_tx_header.StdId = chain[joint_idx].id;
//			can_tx_data.d[3] = chain[joint_idx].led_cmd;
			can_tx_header.StdId = 23;
//			can_tx_data.d[3] = LED_OFF;
			can_tx_data.v = 0x55555555;
			HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data.d, &can_tx_mailbox);
			//joint_idx = (joint_idx + 1) % NUM_JOINTS;
			can_tx_ts = HAL_GetTick() + 10;
		}
		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
			{}	//NOTE the rx header will contain the message ID
		}
	}

}
