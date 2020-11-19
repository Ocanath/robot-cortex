#include "init.h"
#include "CAN.h"
#include "rgb.h"
#include "sin-math.h"

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
	float q;
	floatsend_t tau;
	float qd;
	uint8_t misc_cmd;
}joint;

#define NUM_JOINTS 3

joint chain[NUM_JOINTS] = {
		{
				.id = 23,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.tau = {.v = 0.f},
				.qd = 0,
				.misc_cmd = LED_OFF
		},
		{
				.id = 24,
				.frame = 2,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.tau = {.v = 0.f},
				.qd = 0,
				.misc_cmd = LED_OFF
		},
		{
				.id = 25,
				.frame = 3,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.tau = {.v = 0.f},
				.qd = 0,
				.misc_cmd = LED_OFF
		}
};

void can_comm_misc(joint * chain, int num_joints)
{
	for(int i = 0; i < num_joints; i++)
	{
		can_tx_header.StdId = 0x7FF - chain[i].id;
		can_tx_data.d[3]=chain[i].misc_cmd;
		HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data.d, &can_tx_mailbox);

		for(uint32_t exp_ts = HAL_GetTick()+1; HAL_GetTick() < exp_ts;)
		{
			if(HAL_CAN_IsTxMessagePending(&hcan1,can_tx_mailbox) == 0)
				break;
		}
		for(uint32_t exp_ts = HAL_GetTick()+10;  HAL_GetTick() < exp_ts;)
		{
			if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
			{
				if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) != HAL_OK)
				{}
				break;
			}
		}
	}
}

void can_comm_motor(joint * chain, int num_joints)
{
	for(int i = 0; i < num_joints; i++)
	{
		can_tx_header.StdId = chain[i].id;
		HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, chain[i].tau.d, &can_tx_mailbox);

		for(uint32_t exp_ts = HAL_GetTick()+1; HAL_GetTick() < exp_ts;)
		{
			if(HAL_CAN_IsTxMessagePending(&hcan1,can_tx_mailbox) == 0)
				break;
		}

		for(uint32_t exp_ts = HAL_GetTick()+10;  HAL_GetTick() < exp_ts;)
		{
			if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
			{
				if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
				{
					if(can_rx_header.StdId == chain[i].id)
						chain[i].q = can_rx_data.v;
					else
					{
						for(int sb = 0; sb < num_joints; sb++)	//sb = search base
						{
							int sidx = (sb + i) % num_joints;
							if(can_rx_header.StdId == chain[sidx].id)
								chain[sidx].q = can_rx_data.v;
						}
					}
				}
				break;
			}
		}
	}
}

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

	can_comm_misc(chain,NUM_JOINTS);
	HAL_Delay(100);
	chain[0].misc_cmd = DIS_UART_ENC;
	can_comm_misc(chain,NUM_JOINTS);

	int led_state = NUM_JOINTS;
	uint32_t can_tx_ts = 0;
	chain[0].tau.v = 15.f;
	chain[1].tau.v = 0;
	can_comm_motor(chain, NUM_JOINTS);

	rgb_play((rgb_t){0,255,0});

	while(1)
	{
		if(HAL_GetTick()>can_tx_ts)
		{
			if(led_state == NUM_JOINTS)
			{
				rgb_play((rgb_t){0,255,0});
				for(int i = 0; i < NUM_JOINTS; i++)
					chain[i].misc_cmd = LED_OFF;
			}
			else
			{
				rgb_play((rgb_t){0,0,0});
				for(int i = 0; i < NUM_JOINTS; i++)
					chain[i].misc_cmd = LED_OFF;
				chain[led_state].misc_cmd = LED_ON;
			}

			led_state = (led_state + 1) % (NUM_JOINTS + 1);
			can_tx_ts = HAL_GetTick()+1000;
			can_comm_misc(chain,NUM_JOINTS);
		}
		float t = ((float)HAL_GetTick())*.001f;
		chain[0].qd = 3.f*sin_fast(t);
		chain[0].tau.v = 5.f*(chain[0].qd - chain[0].q);
		chain[1].qd = -chain[0].qd;
		chain[1].tau.v = 5.f*(chain[1].qd-chain[1].q);
		can_comm_motor(chain, NUM_JOINTS);


	}
}
