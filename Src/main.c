#include "init.h"
#include "i2c_master.h"
#include "sin-math.h"

const int num_frames = 2;	//number of frames on the robot, including the zeroeth frame. If a robot has 6dof, it has 7 frames.

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
	MX_USART2_UART_Init();

	start_pwm();
	TIMER_UPDATE_DUTY(0,1000,0);	//B,G,R
	HAL_Delay(1000);

	float uart_rx_buf[num_frames];
	int start_idx = 0;
	int num_bytes_frame_buf = sizeof(float)*num_frames;
	floatsend_t align_word;
	align_word.d[3] = 0xDE;
	align_word.d[2] = 0xAD;
	align_word.d[1] = 0xBE;
	align_word.d[0] = 0xEF;
	uint32_t uart_tx_ts = 0;


	float qd[num_frames];


	//tau -> input, q_i2c and i2c_rx_previous are state, q -> output
	floatsend_t tau[num_frames];
	floatsend_t q_i2c[num_frames];	//receptions are weird. since the interrupt could complete itself ANYWHERE, we need to debuffer this in an interrupt handler or flag handler
	float i2c_rx_previous[num_frames];	//this is used ONLY FOR NaN handling!!!!!
	float q[num_frames];	//this is used for pcontrol


	uint32_t led_ts = 0;
	uint32_t uart_ts = 0;

	uint32_t i2c_addr_offset = 0;
	uint32_t i2c_frame_offset = i2c_addr_offset + 1;

	while (1)
	{
		int rc = handle_i2c_master(&hi2c1, ((0x20+i2c_addr_offset) << 1), q_i2c[i2c_frame_offset].d, 4, tau[i2c_frame_offset].d, 4);	//This works!!!
		if(rc == -1 || hi2c1.ErrorCode != 0)
		{
			HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);				//and maybe doing this are critical for i2c_IT error recovery
			while(HAL_NVIC_GetPendingIRQ(I2C1_EV_IRQn)==1);
			I2C1_Reset();
			i2c_master_state = I2C_TRANSMIT_READY;

			i2c_tx_cplt = 0;
			i2c_rx_cplt = 0;
		}
		if(i2c_tx_cplt == 1 && i2c_rx_cplt == 1)
		{
			uint32_t * fptr_chk = (uint32_t*)(&(q_i2c[i2c_frame_offset].v));	//mask pointer of the most recently read q value
			if( (*fptr_chk & 0x7f800000) == 0x7f800000)		//if the most recent q value is NaN
				q[i2c_frame_offset] = i2c_rx_previous[i2c_frame_offset];
			else	//if it is a number,
			{
				q[i2c_frame_offset] = q_i2c[i2c_frame_offset].v;	//load it into q so we can use it!
				i2c_rx_previous[i2c_frame_offset] = q[i2c_frame_offset];//only do this here... only works here anyway...
			}

			i2c_addr_offset++;
			if(i2c_addr_offset >= num_frames-1)
				i2c_addr_offset = 0;
			i2c_frame_offset = i2c_addr_offset + 1;

			i2c_tx_cplt = 0;
			i2c_rx_cplt = 0;
		}


		/**********************************Inside here, q and q_previous are legitimate*************************************************************/

		float t = ((float)(HAL_GetTick()))*.001f;
		for(int frame = 1; frame < num_frames; frame++)
			qd[frame] = 3.0f*(sin_fast(t*4.0f));

		for(int frame = 1; frame < num_frames; frame++)
			tau[frame].v = 10.0f*(qd[frame]-q[frame]);
		//tau[frame].v = qd[frame];

		if(HAL_GetTick()>uart_ts)
		{
			HAL_UART_Transmit(&huart2, tau[1].d, 4, 10);
			HAL_UART_Transmit(&huart2, (uint8_t *)(&q[1]), 4, 10);
			uart_ts = HAL_GetTick()+15;
		}

		if(rc == 0)
		{
			TIMER_UPDATE_DUTY(100,900,0);
		}
		else
		{
			TIMER_UPDATE_DUTY(900,00,100);
		}

		if(HAL_GetTick() > led_ts)
		{
			//			HAL_GPIO_TogglePin(STAT_GPIO_Port, STAT_Pin);
			//			HAL_GPIO_TogglePin(NRF_CE_GPIO_Port, NRF_CE_Pin);
			led_ts = HAL_GetTick()+250;
		}
		/**********************************End Legit q*************************************************************/
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
