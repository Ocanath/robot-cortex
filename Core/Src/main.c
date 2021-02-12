#include "init.h"
#include "CAN.h"
#include "rgb.h"
#include "sin-math.h"
#include "uart-disp-tools.h"


#define NUM_JOINTS 6

joint chain[NUM_JOINTS] = {
		{
				.id = 26,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0,
				.tau = {.v = 0.f},
				.qd = -.77f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 27,
				.frame = 2,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0,
				.tau = {.v = 0.f},
				.qd = -2.45f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 28,
				.frame = 3,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0,
				.tau = {.v = 0.f},
				.qd = -2.90f,
				.misc_cmd = LED_OFF
		},
		{
				.id = 24,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0,
				.tau = {.v = 0.f},
				.qd = 0,
				.misc_cmd = LED_OFF
		},
		{
				.id = 23,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0,
				.tau = {.v = 0.f},
				.qd = 0,
				.misc_cmd = LED_OFF
		},
		{
				.id = 25,
				.frame = 1,
				.h0_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.him1_i = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
				.q = 0,
				.q_offset = 0,
				.tau = {.v = 0.f},
				.qd = 0,
				.misc_cmd = LED_OFF
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

	CAN_comm_misc(chain);
	HAL_Delay(100);
	for(int i = 0; i < NUM_JOINTS; i++)
		chain[i].misc_cmd = EN_UART_ENC;
	CAN_comm_misc(chain);



	int led_idx = NUM_JOINTS;
	int prev_led_idx = NUM_JOINTS-1;
	uint32_t can_tx_ts = 0;
	chain[0].tau.v = 15.f;
	chain[1].tau.v = 0;
	CAN_comm_motor(chain, NUM_JOINTS);

	rgb_play((rgb_t){0,255,0});



	uint32_t disp_ts = HAL_GetTick()+15;

	while(1)
	{
		if(HAL_GetTick()>can_tx_ts)
		{
			if(led_idx == NUM_JOINTS)
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

			}

			led_idx = (led_idx + 1) % (NUM_JOINTS+1);
			if(led_idx == NUM_JOINTS)
				can_tx_ts = HAL_GetTick()+1000;
			else
				can_tx_ts = HAL_GetTick()+50;

			if(led_idx < NUM_JOINTS)
			{
				chain[led_idx].misc_cmd = LED_ON;
				CAN_comm_misc(&(chain[led_idx]));
			}
			if(prev_led_idx < NUM_JOINTS)
			{
				chain[prev_led_idx].misc_cmd = LED_OFF;
				CAN_comm_misc(&(chain[prev_led_idx]));
			}

			prev_led_idx = led_idx;
		}

		float t = ((float)HAL_GetTick())*.001f;
		//		chain[0].qd =
		float f = 4.f;
		chain[0].qd = -0.77f + .6f*sin_fast(t*f);
		chain[1].qd = -2.45f + .6f*sin_fast(t*f-1.57f);
		chain[2].qd = -1.90f + 1.4f*sin_fast(t*f-3.14f);

		//chain[0].qd = 3.f*sin_fast(t);
		for(int joint = 0; joint < NUM_JOINTS; joint++)
		{
			float sign = -1.f;	//sign inversion because of the gearbox
			float tau = sign*75.f*(chain[joint].qd - chain[joint].q);
			if(tau > 35.f)
				tau = 35.f;
			if(tau < -35.f)
				tau = -35.f;
			chain[joint].tau.v = tau;
		}
		CAN_comm_motor(chain, NUM_JOINTS);

		if(HAL_GetTick()>disp_ts)
		{
			for(int i = 0; i < NUM_JOINTS; i++)
			{
				sprintf(gl_print_str, "q[%d] = %d, ", i, (int)(chain[i].q*1000));
				print_string(gl_print_str);
			}
			print_string("\r\n");

			disp_ts = HAL_GetTick()+15;
		}

	}
}
