#include "init.h"
#include "i2c_master.h"
#include "sin-math.h"
#include "nrf24l01.h"
#include "nrf.h"
#include <string.h>


#define DEG_TO_RAD 0.0174532925f

const int num_frames = 10;	//number of frames on the robot, including the zeroeth frame. If a robot has 6dof, it has 7 frames.

void color_wheel(uint32_t hue, float saturation, uint8_t * r, uint8_t * g, uint8_t * b);

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

void start_pwm();

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

uint8_t is_nan(float v)
{
	uint32_t * fptr_chk = (uint32_t*)(&v);	//mask pointer of the most recently read q value
	if( (*fptr_chk & 0x7f800000) == 0x7f800000)		//if the most recent q value is NaN
		return 1;
	else
		return 0;
}

/*
 * constrained fast sin for zero to one, with phase and frequency modulation
 */
float sin_z1(float freq, float phase)
{
	return sin_fast(freq+phase)*.5f+.5f;
}
void rgb_disp(uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t r16 = (1000*((uint16_t)r))/255;
	uint16_t g16 = (1000*((uint16_t)g))/255;
	uint16_t b16 = (1000*((uint16_t)b))/255;
	TIMER_UPDATE_DUTY(b16,g16,r16);	//B,G,R
}

/*
 * x is state variable for integral delay
 */
void controller_PI(float i_q_ref, float i_q, float Kp, float Ki, float * x, float * u)
{
	float err = i_q_ref-i_q;
	*u = *x + Kp*err;
	*x = *x + Ki*err;
}

char str[64];
void print_string(const char * str)
{
	int strlen;
	for(strlen = 0; str[strlen] != 0; strlen++);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen, 10);
}
void print_float(float f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)(&f), 4, 10);
}
void disp_i2c_map(uint8_t addr_used[128])
{
	print_string("    0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	for(int r = 0; r < 8; r++)
	{
		sprintf(str, "%d0: ", r);
		print_string(str);
		for(int c = 0; c < 16; c++)
		{
			uint8_t addr = addr_used[c+r*16];
			if(addr != 0)
			{
				sprintf(str, "%.2x ", addr);
				print_string(str);
			}
			else
				print_string("-- ");
		}
		print_string("\r\n");
	}

}
static uint8_t addr_list[128];
int check_i2c_devices(uint8_t addr_used[128])
{

	for(int i = 0; i < 128; i++)
	{
		addr_used[i] = 0;
		addr_list[i]=i;
	}
	int i2c_addr_offset = 0;
	floatsend_t qtmp;
	floatsend_t tautmp;
	uint32_t check_ts = HAL_GetTick()+500;
	while(HAL_GetTick()<check_ts)
	{
		int rc = handle_i2c_master(&hi2c1, (addr_list[i2c_addr_offset] << 1), qtmp.d, 4, tautmp.d, 4);	//This works!!!
		if(rc == -1 || hi2c1.ErrorCode != 0)
		{
			HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);				//and maybe doing this are critical for i2c_IT error recovery
			while(HAL_NVIC_GetPendingIRQ(I2C1_EV_IRQn)==1);
			I2C1_Reset();
			i2c_master_state = I2C_TRANSMIT_READY;

			i2c_addr_offset++;
			if(i2c_addr_offset >= 128)
				i2c_addr_offset = 0;

			i2c_tx_cplt = 0;
			i2c_rx_cplt = 0;
		}
		if(i2c_tx_cplt == 1 && i2c_rx_cplt == 1)
		{

			addr_used[i2c_addr_offset] = i2c_addr_offset;

			i2c_addr_offset++;
			if(i2c_addr_offset >= 128)
				i2c_addr_offset = 0;

			i2c_tx_cplt = 0;
			i2c_rx_cplt = 0;
		}
	}
	int num_connected_devices = 0;
	for(int i = 0; i < 128; i++)
	{
		if(addr_used[i] != 0)
			num_connected_devices++;
	}
	return num_connected_devices;
}


int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	HAL_GPIO_WritePin(MPU_SS_GPIO_Port, MPU_SS_Pin, 1);
	HAL_GPIO_WritePin(EN_HP_GPIO_Port, EN_HP_Pin, 1);
	HAL_Delay(500);
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();

	start_pwm();
	TIMER_UPDATE_DUTY(0,100,10);	//B,G,R
	HAL_Delay(100);

//	init_nrf();
//	change_nrf_payload_length(4);//unnecessary
//	uint8_t tx_addr[5] = {0x01, 0x07, 0x33, 0xA0, 0};
//	for(int attempt = 0; attempt < 5; attempt++)
//	{
//		for(int strip = 0; strip < 3; strip++)
//		{
//			tx_addr[4]=strip;
//			change_nrf_tx_address((const uint8_t * )tx_addr);
//			uint32_t r32 = 1023;
//			uint32_t g32 = 980;
//			uint32_t b32 = 970;
//			uint32_t tmp = ((r32 & 0x03FF) << 22) | ((g32 & 0x03FF) << 12) | ((b32 & 0x03FF) << 2);
//			nrf_send_packet_noack(&nrf, (uint8_t *)(&tmp) );
//			HAL_Delay(5);
//		}
//	}

	uint8_t addr_used[128];
	if(check_i2c_devices(addr_used) != (num_frames-1))
	{
		while(1)
		{
			TIMER_UPDATE_DUTY(0,0,1000);	//B,G,R
			HAL_Delay(100);
			TIMER_UPDATE_DUTY(0,0,0);	//B,G,R
			HAL_Delay(100);
		}
	}

//	while(1)
//	{
//		sprintf(str, "num_connected = %d\r\n", check_i2c_devices(addr_used));
//		print_string("\033c\r\n");
//		print_string(str);
//		disp_i2c_map(addr_used);
//		HAL_Delay(2000);
//	}


	float qd[num_frames];

	//tau -> input, q_i2c and i2c_rx_previous are state, q -> output
	floatsend_t tau[num_frames];
	floatsend_t q_i2c[num_frames];	//receptions are weird. since the interrupt could complete itself ANYWHERE, we need to debuffer this in an interrupt handler or flag handler
	float i2c_rx_previous[num_frames];	//this is used ONLY FOR NaN handling!!!!!
	float q_direct[num_frames];	//this is used for pcontrol
	float q_offset[num_frames];
	float q[num_frames];
	float ierr[num_frames];
	float x_i[num_frames];
	for(int frame = 1; frame < num_frames; frame++)
	{
		x_i[frame]=0;
		q_direct[frame]=0;
		qd[frame]=0;
		q_offset[frame] = 0;
		ierr[frame]=0;
		tau[frame].v = 0;
	}

	float gbx_conv_ratio[num_frames];
	gbx_conv_ratio[0] = 0;
	gbx_conv_ratio[1] = -1.f;
	gbx_conv_ratio[2] = -1.f;
	gbx_conv_ratio[3] = -1.f;
	gbx_conv_ratio[4] = 0.15789473684f;
	gbx_conv_ratio[5] = 0.15789473684f;
	gbx_conv_ratio[6] = 0.15789473684f;
	gbx_conv_ratio[7] = 0.15789473684f;
	gbx_conv_ratio[8] = 0.15789473684f;
	gbx_conv_ratio[9] = 0.15789473684f;


	float Kp[num_frames];
	Kp[0]= 0;
	Kp[1]= 2.f;
	Kp[2]= 2.f;
	Kp[3]= 2.f;
	Kp[4] = 60.f;
	Kp[5] = 60.f;
	Kp[6] = 60.f;
	Kp[7] = 60.f;
	Kp[8] = 60.f;
	Kp[9] = 60.f;

	float tau_thresh[num_frames];
	tau_thresh[0] = 0;
	tau_thresh[1] = 0.5f;
	tau_thresh[2] = 0.5f;
	tau_thresh[3] = 0.5f;
	tau_thresh[4] = 80.f;
	tau_thresh[5] = 80.f;
	tau_thresh[6] = 80.f;
	tau_thresh[7] = 80.f;
	tau_thresh[8] = 80.f;
	tau_thresh[9] = 80.f;

	uint32_t uart_ts = 0;

	uint8_t addr_map[num_frames-1];

	addr_map[0] = 0x20;
	addr_map[1] = 0x21;
	addr_map[2] = 0x22;

	addr_map[3] = 0x24;
	addr_map[4] = 0x2B;

	addr_map[5] = 0x25;
	addr_map[6] = 0x28;

	addr_map[7] = 0x27;
	addr_map[8] = 0x2A;

	float base_calibration_torque = -0.0f;
	float leg_calibration_torque = -15.0f;
	for(int i = 1; i < num_frames; i++)
		tau[i].v = 0;
	uint32_t init_pos_ts = HAL_GetTick();
	const uint32_t cal_duration = 4000;
	const uint32_t lower_lim_cal_duration = (uint32_t)((float)cal_duration * .20f);
	while(HAL_GetTick() < init_pos_ts + cal_duration)
	{
		uint32_t time = HAL_GetTick()-init_pos_ts;
		if(time > 10)
		{
			if(time < lower_lim_cal_duration)
			{
				tau[1].v = 0.f;
				tau[2].v = 0.f;
				tau[3].v = 0.f;
				tau[4].v = leg_calibration_torque;
				tau[5].v = leg_calibration_torque;
				tau[6].v = leg_calibration_torque;
				tau[7].v = leg_calibration_torque;
				tau[8].v = leg_calibration_torque;
				tau[9].v = leg_calibration_torque;
			}
			else if (time >= lower_lim_cal_duration)
			{
				tau[1].v = base_calibration_torque;
				tau[2].v = base_calibration_torque;
				tau[3].v = base_calibration_torque;
			}
		}
		i2c_robot_master(addr_map, num_frames,
				q_i2c,	i2c_rx_previous,
				tau, q_offset);
	}

	uint32_t ierr_ts = 0;
	uint32_t time_start = HAL_GetTick();

	uint8_t r,g,b;
	while (1)
	{
//		float t= ((float)HAL_GetTick())*.001f;
		//color_wheel(fmod_2pi(q[1])*244.461993f, 1.0, &r, &g, &b);
		color_wheel(HAL_GetTick(), 1.0, &r, &g, &b);

		/*****************This block of code manages I2C robust communications to a chain of i2c devices.****************************************/
		i2c_robot_master(addr_map, num_frames,
				q_i2c,	i2c_rx_previous,
				tau, q_direct);
		/***********************************************************End***************************************************************************/
		for(int frame = 0; frame < num_frames; frame++)
			q[frame] = (q_direct[frame]-q_offset[frame])*gbx_conv_ratio[frame];

			//		/**********************************Inside here, q and q_previous are legitimate*************************************************************/
		float t = ((float)(HAL_GetTick()-time_start))*.001f;
		//		for(int frame = 1; frame < num_frames; frame++)
		//			qd[frame] = 3.0f*(sin_fast(t*4.0f));
		//qd[1] = 1.0f * (.5f * sin_fast(8*t-HALF_PI) + 0.5f);

//		qd[1] = 36.76f*DEG_TO_RAD;
//		qd[2] = 36.76f*DEG_TO_RAD;
//		qd[3] = 36.76f*DEG_TO_RAD;
		qd[1] = 0;
		qd[2] = 0;
		qd[3] = 0;

		qd[4] = 99.40f*DEG_TO_RAD;
		qd[6] = 99.40f*DEG_TO_RAD;
		qd[8] = 99.40f*DEG_TO_RAD;

		qd[5] = 100.32f*DEG_TO_RAD;
		qd[7] = 100.32f*DEG_TO_RAD;
		qd[9] = 100.32f*DEG_TO_RAD;

		//		qd[2] = 1.5f * (.5f * cos_fast(8*t-HALF_PI) + 0.5f);


		if(HAL_GetTick() > ierr_ts)
		{
			for(int frame = 1; frame < num_frames; frame++)
				ierr[frame] += (qd[frame]-q[frame])*.001f;
			ierr_ts = HAL_GetTick();
		}
		for(int frame = 1; frame < num_frames; frame++)
		{
			//float tau_tmp = 20.0f*(qd[frame]-q[frame]);// + 10*ierr[frame];
			float tau_tmp = 0;
			controller_PI(qd[frame], q[frame], Kp[frame], 0.000f, &x_i[frame], &tau_tmp);
			if(tau_tmp > tau_thresh[frame])
				tau_tmp = tau_thresh[frame];
			if(tau_tmp < -tau_thresh[frame])
				tau_tmp = -tau_thresh[frame];
			tau[frame].v = tau_tmp; 
		}

		if(HAL_GetTick()>uart_ts)
		{
			//HAL_UART_Transmit(&huart2, (uint8_t *)(&tau[1].v), 4, 10);
			//HAL_UART_Transmit(&huart2, (uint8_t *)(&tau[2].v), 4, 10);
			print_float(q[3]);
			print_float(q[2]);
			print_float(q[1]);
			uart_ts = HAL_GetTick()+15;
		}

		rgb_disp(r,g,b);

		//		/**********************************End Legit q*************************************************************/
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



/*
 *
 */
void color_wheel(uint32_t hue, float saturation, uint8_t * r, uint8_t * g, uint8_t * b)
{
	hue %=  1536;
	int region = hue/256;
	switch(region)
	{
	case 0:
		*r = 255;
		*g = hue % 256;
		*b = 0;
		break;
	case 1:
		*r = (255 - (hue % 256));
		*g = 255;
		*b = 0;
		break;
	case 2:
		*r = 0;
		*g = 255;
		*b = hue % 256;
		break;
	case 3:
		*r = 0;
		*g = 255-(hue%256);
		*b = 255;
		break;
	case 4:
		*r = hue % 256;
		*g = 0;
		*b = 255;
		break;
	case 5:
		*r = 255;
		*g = 0;
		*b = 255-(hue%256);
		break;
	default:
		break;
	};
	//TODO: add saturation

}

