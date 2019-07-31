#include "init.h"
#include "i2c_master.h"
#include "sin-math.h"

const int num_frames = 7;	//number of frames on the robot, including the zeroeth frame. If a robot has 6dof, it has 7 frames.

typedef enum {MODE_POS_SAFE = 0xAD, MODE_POS_UNSAFE = 0xAB, MODE_TORQUE = 0xAC, MODE_SPEED = 0xAE}api_control_mode;
enum {RETMODE_POS = 0xAF, RETMODE_PRES = 0xB0};

#define ALIGN_MODE_0 0xFEEDBEEF
#define ALIGN_MODE_1 0xFEEDBEF0
#define ALIGN_MODE_2 0xFEEDBEF1
#define ALIGN_MODE_3 0xFEEDBEF2

uint8_t uart_rx_cplt_flag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_rx_cplt_flag = 1;
}

typedef union
{
	float v;
	uint8_t d[4];
}floatsend_t;
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

void start_pwm();

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	HAL_GPIO_WritePin(EN_HP_GPIO_Port, EN_HP_Pin, 1);
	HAL_Delay(500);
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	float q[num_frames];
	float tau[num_frames];

	start_pwm();
	TIMER_UPDATE_DUTY(500,900,400);


//	uint32_t enhp_ts = 0;
//	enum {POWER_ON, POWER_OFF, OPEN, CLOSE, OPEN_2, CLOSE_2};
//	uint8_t state = POWER_ON;
//	uint32_t grip_time = 1000;
//	uint8_t grip_speed = 230;
//	while(1)
//	{
//		float t = ((float)HAL_GetTick())*.001f;
//		float tau = t*.2;
//		float r = 1000.0f*(.5f*sin_fast(tau+TWO_PI*sin_fast(tau))+.5f);
//		float g = 1000.0f*(.5f*sin_fast(tau-TWO_PI*sin_fast(tau)+ONE_BY_THREE_PI)+.5f);
//		float b = 1000.0f*(.5f*sin_fast(tau+TWO_PI*sin_fast(tau-ONE_BY_THREE_PI))+ .5f);
//		TIMER_UPDATE_DUTY(r,g,b);
//
//		if(HAL_GetTick() > enhp_ts)
//		{
//			switch(state)
//			{
//			case POWER_ON:
//			{
//				HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, 1);
//				HAL_GPIO_WritePin(EN_HP_GPIO_Port, EN_HP_Pin, 1);
//				HAL_Delay(10);
//				I2C1_Reset();
//
//				enhp_ts = HAL_GetTick()+1000;
//				state = CLOSE;
//				break;
//			}
//			case CLOSE:
//			{
//				uint8_t i2c_tx_buf[3] = {0x1D, 0x01, grip_speed};
//				while(HAL_I2C_Master_Transmit(&hi2c1, 0x50<<1, i2c_tx_buf, 3, 10) != HAL_OK)
//					I2C1_Reset();
//				enhp_ts = HAL_GetTick()+grip_time;
//				state = OPEN;
//				break;
//			}
//			case OPEN:
//			{
//				uint8_t i2c_tx_buf[3] = {0x1D, 0x00, grip_speed};
//				while(HAL_I2C_Master_Transmit(&hi2c1, 0x50<<1, i2c_tx_buf, 3, 10) != HAL_OK)
//					I2C1_Reset();
//				enhp_ts = HAL_GetTick()+grip_time;
//				state = CLOSE_2;
//				break;
//			}
//			case CLOSE_2:
//			{
//				uint8_t i2c_tx_buf[3] = {0x1D, 0x01, grip_speed};
//
//				while(HAL_I2C_Master_Transmit(&hi2c1, 0x50<<1, i2c_tx_buf, 3, 10) != HAL_OK)
//					I2C1_Reset();
//
//				enhp_ts = HAL_GetTick()+grip_time;
//				state = OPEN_2;
//				break;
//			}
//			case OPEN_2:
//			{
//				uint8_t i2c_tx_buf[3] = {0x1D, 0x00, grip_speed};
//
//				while(HAL_I2C_Master_Transmit(&hi2c1, 0x50<<1, i2c_tx_buf, 3, 10) != HAL_OK)
//					I2C1_Reset();
//
//				enhp_ts = HAL_GetTick()+grip_time;
//				state = POWER_OFF;
//				break;
//			}
//			case POWER_OFF:
//			{
//				HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, 0);
//				HAL_GPIO_WritePin(EN_HP_GPIO_Port, EN_HP_Pin, 0);
////				HAL_Delay(500);		//TODO:
////				NVIC_SystemReset();	//make it so that this and the above line are not necessary anymore.
//				enhp_ts = HAL_GetTick()+500;
//				state = POWER_ON;
//				break;
//			}
//			default:
//				break;
//			};
//		}
//		if(hi2c1.ErrorCode != 0)
//			I2C1_Reset();
//	}

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

	hand_format_i2c tx_fmt;
	hand_format_i2c rx_fmt;
	uint8_t i2c_tx_buf[25];
//	HAL_Delay(5000);

	api_control_mode control_mode = MODE_POS_SAFE;
	control_mode = MODE_POS_SAFE;
	uint32_t time_start_ts = HAL_GetTick();
	float qd[num_frames];
	for(int frame = 1; frame < num_frames; frame++)
		qd[frame]=10.0f;
	while (1)
	{
		float t = ((float)(HAL_GetTick()-time_start_ts))*.001f;
		for(int frame = 1; frame < num_frames; frame++)
			qd[frame]=70.0f*(.5f*sin_fast(t)+.5f)+10.0f;
		qd[5] = 40.0f*(.5f*sin_fast(t)+.5f)+10.0f;
		qd[6] = 35.0f*(.5f*sin_fast(t+HALF_PI)+.5f)+10.0f;

		for(int frame = 1; frame < num_frames; frame++)
		{
			tx_fmt.v[frame-1] = qd[frame];
		}

		for(int i = 1; i < 25; i++)
			i2c_tx_buf[i]  = tx_fmt.d[i-1];
		i2c_tx_buf[0] = control_mode;	//protected position control mode


//		HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, (0x50<<1), i2c_tx_buf,25, I2C_FIRST_FRAME);		//doesn't work
//		HAL_I2C_Master_Seq_Receive_IT(&hi2c1, (0x50<<1), rx_fmt.d, 24, I2C_LAST_FRAME);

//		int rc = 0;
//		HAL_I2C_Master_Transmit(&hi2c1, (0x50 << 1), i2c_tx_buf, 25, 10);		//works
//		HAL_I2C_Master_Receive(&hi2c1, (0x50 << 1), rx_fmt.d, 24, 10);
//		if(hi2c1.ErrorCode != 0)	//if you encounter an error
//			I2C1_Reset();			//this only appears to play well with blocking transmission

		/***********************************************************************************************/
		int rc = handle_i2c_master(&hi2c1, (0x50 << 1), rx_fmt.d, 24, i2c_tx_buf, 25);	//This works!!!
		if(rc == -1 || hi2c1.ErrorCode != 0)
		{
			HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);				//and maybe doing this are critical for i2c_IT error recovery
			while(HAL_NVIC_GetPendingIRQ(I2C1_EV_IRQn)==1);
			I2C1_Reset();
			i2c_master_state = I2C_TRANSMIT_READY;
		}
		/**********************************************************************************************/

		if(rc == 0)
		{
			for(int frame = 1; frame < num_frames; frame++)
				q[frame] = rx_fmt.v[frame-1];	//deg->rad
		}
		else
		{
			for(int frame = 1; frame < num_frames; frame++)
				q[frame] = 100.0f*(0.5f*sin_fast(t*10.0f)+.5f);
		}

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
				uint32_t align_mode_word = (uint32_t)fmt.v;
				if(align_mode_word >= ALIGN_MODE_0 && align_mode_word <= ALIGN_MODE_3)
				{
					start_idx = bidx;
					control_mode = (uint8_t)(align_mode_word-ALIGN_MODE_0)+0xAB;
				}
			}
			uint8_t * tau_ref = (uint8_t *)tau;
			for(bidx = 0; bidx < num_bytes_frame_buf; bidx++)
			{
				tau_ref[bidx] = fmt_ptr_rx_buf[ ( (bidx+start_idx)%num_bytes_frame_buf )];
			}
			uart_rx_cplt_flag = 0;
		}

		if(HAL_GetTick() > uart_tx_ts)
		{
			q[0] = align_word.v;	//q[0] is undefined. we're using it to align data in the tx packet.
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)q, num_bytes_frame_buf);
			uart_tx_ts = HAL_GetTick() + 20;
		}



		if(HAL_GetTick() > led_ts)
		{
//			HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
//			HAL_GPIO_TogglePin(NRF_CE_GPIO_Port, NRF_CE_Pin);
			led_ts = HAL_GetTick()+250;
		}
	}
}


/*
 * same as writing to CCER register. this is done in one line elsewhere in the code
 */
void start_pwm()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	//	TIM1->CCER = (TIM1->CCER & DIS_ALL) | ENABLE_ALL;
}



