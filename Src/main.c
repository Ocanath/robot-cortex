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
	MX_USART1_UART_Init();




	start_pwm();
	TIMER_UPDATE_DUTY(500,900,400);

	float uart_rx_buf[num_frames];
	int start_idx = 0;
	int num_bytes_frame_buf = sizeof(float)*num_frames;
	floatsend_t align_word;
	align_word.d[3] = 0xDE;
	align_word.d[2] = 0xAD;
	align_word.d[1] = 0xBE;
	align_word.d[0] = 0xEF;
	uint32_t uart_tx_ts = 0;


	floatsend_t qd[num_frames];

	floatsend_t q[num_frames];
	float tau[num_frames];

	uint32_t led_ts = 0;
	while (1)
	{
		float t = ((float)(HAL_GetTick()))*.001f;
		for(int frame = 1; frame < num_frames; frame++)
			qd[frame].v = 70.0f*(.5f*sin_fast(t)+.5f)+10.0f;

		/***********************************************************************************************/
		int rc = handle_i2c_master(&hi2c1, (0x20 << 1), q[1].d, 4, qd[1].d, 4);	//This works!!!
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



