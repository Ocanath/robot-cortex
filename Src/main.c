#include "init.h"
#include "i2c_master.h"
#include "sin-math.h"

const int num_frames = 2;	//number of frames on the robot, including the zeroeth frame. If a robot has 6dof, it has 7 frames.

#define I2C_BASE 0x20


uint8_t uart_rx_cplt_flag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_rx_cplt_flag = 1;
}

typedef union
{
	uint32_t v;
	uint8_t d[4];
}uint_cast_t;

typedef union
{
	uint8_t d[24];
	float v[6];
}hand_format_i2c;

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	float q[num_frames];
	float tau[num_frames];

	float uart_rx_buf[num_frames];
	int start_idx = 0;
	int num_bytes_frame_buf = sizeof(float)*num_frames;
	floatsend_t align_word;
	align_word.d[3] = 0xDE;
	align_word.d[2] = 0xAD;
	align_word.d[1] = 0xBE;
	align_word.d[0] = 0xEF;
	uint32_t uart_tx_ts = 0;

	uint32_t led_ts = 0;

	for(int bs = 0; bs < 50; bs++)
	{
		HAL_Delay(50);
		HAL_GPIO_TogglePin(NRF_CE_GPIO_Port, NRF_CE_Pin);
	}

	int sign[num_frames];
	sign[0] = 1;
	sign[1] = 1;
	uint32_t i2c_ready_ts = 0;

	uint32_t speed_update_ts = 0;
	float theta_set = 0;
	const float tau_lim = 50.0f;
	while (1)
	{

		float t = ((float)HAL_GetTick())*.001f;
		float theta_set = 17.3f*(sin_fast(t*2.0f - HALF_PI)+1.0f);
//		float rpm_des = 100.0f;
//		if(HAL_GetTick() > speed_update_ts)
//		{
//			theta_set += rpm_des*0.000104719755f;
//			speed_update_ts = HAL_GetTick();
//		}

//		int rc = handle_i2c_master(&hi2c1, (0x50 << 1), rx_fmt.d, 24, i2c_tx_buf, 25);

//		handle_i2c_motor_chain(&hi2c1, I2C_BASE, num_frames-1, q, tau, sign);
//		tau[1] = 5.0f;
		tau[1] = 10.0f*(theta_set - q[1]);

		if(tau[1] > tau_lim)
			tau[1] = tau_lim;
		if(tau[1] < -tau_lim)
			tau[1] = -tau_lim;

		handle_i2c_frame_buf(&hi2c1, 0x20, num_frames, q, tau);



		uint8_t * fmt_ptr_rx_buf = (uint8_t * )uart_rx_buf;	//is the same memory
		HAL_UART_Receive_IT(&huart1, fmt_ptr_rx_buf, num_bytes_frame_buf);
		if(uart_rx_cplt_flag==1)
		{
			int bidx;
			for(bidx = 0; bidx < num_bytes_frame_buf; bidx++)
			{
				uint_cast_t fmt;

				for(int i = 0; i < 4; i++)
				{
					int ld_idx = (bidx + i) % num_bytes_frame_buf;
					fmt.d[i] = fmt_ptr_rx_buf[ld_idx];
				}
				if(fmt.v == 0xFEEDBEEF)
					start_idx = bidx;
			}

			uint8_t * tau_ref = (uint8_t *)tau;
			for(bidx = 0; bidx < num_bytes_frame_buf; bidx++)
			{
				tau_ref[bidx] = fmt_ptr_rx_buf[ ( (bidx+start_idx)%num_bytes_frame_buf )];
			}
			uart_rx_cplt_flag = 0;
		}

//		for(int frame = 1; frame < num_frames; frame++)
//			q[frame] = tau[frame];


		if(HAL_GetTick() > uart_tx_ts)
		{
			q[0] = align_word.v;
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)q, num_bytes_frame_buf);
			uart_tx_ts = HAL_GetTick() + 10;
		}



		if(HAL_GetTick() > led_ts)
		{
//			HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
			HAL_GPIO_TogglePin(NRF_CE_GPIO_Port, NRF_CE_Pin);
			led_ts = HAL_GetTick()+250;
		}
	}
}



