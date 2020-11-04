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

#define NUM_JOINTS 3

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
		},
		{
			.id = 0x7FF-25,
			.frame = 3,
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

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	rgb_play((rgb_t){0,255,0});
	HAL_Delay(50);

	int led_state = 0;
	uint32_t can_tx_ts = 0;
	uint32_fmt_t can_tx_data = {0};
	uint32_fmt_t can_rx_data = {0};

	while(1)
	{
		if(HAL_GetTick()>can_tx_ts)
		{
			led_state = (led_state + 1) % (NUM_JOINTS + 1);
			if(led_state == NUM_JOINTS)
			{
				rgb_play((rgb_t){0,255,0});
				for(int i = 0; i < NUM_JOINTS; i++)
					chain[i].led_cmd = LED_OFF;
			}
			else
			{
					rgb_play((rgb_t){0,0,0});
					for(int i = 0; i < NUM_JOINTS; i++)
						chain[i].led_cmd = LED_OFF;
					chain[led_state].led_cmd = LED_ON;
			}

			for(int i = 0; i < NUM_JOINTS; i++)
			{
				can_tx_header.StdId = chain[i].id;
				can_tx_data.d[3]=chain[i].led_cmd;
				HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data.d, &can_tx_mailbox);
				while(HAL_CAN_IsTxMessagePending(&hcan1, can_tx_mailbox));
				while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) < 1);
				if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) != HAL_OK)
				{}
			}

			can_tx_ts = HAL_GetTick()+150;
		}

		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) != HAL_OK)
			{}
		}
		//HAL_Delay(10);
	}
}
