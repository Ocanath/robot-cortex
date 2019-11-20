#include "init.h"
#include "i2c_master.h"
#include "sin-math.h"
#include "nrf24l01.h"

const int num_frames = 2;	//number of frames on the robot, including the zeroeth frame. If a robot has 6dof, it has 7 frames.

#define NUM_BYTES_PAYLOAD 3
uint8_t tx_address[5] = {0x32,  0x77, 0x62, 0xFA, 0x00};
uint8_t rx_address[5] = {'h', 'e', 'X', 'r', 'o'};

nrf24l01 nrf;
nrf24l01_config config;
static uint8_t rx_buf[NUM_BYTES_PAYLOAD];
static uint8_t tx_buf[NUM_BYTES_PAYLOAD];
static uint32_t rx_ts = 0;
static uint32_t comm_down_ts = 0;
static uint8_t nrf_new_packet = 0;
void nrf_packet_received_callback(nrf24l01 * dev, uint8_t * data)
{
	//	nrf_send_packet(&nrf, (uint8_t*)&tx_data_nrf);
	rx_ts = HAL_GetTick();
	dev->rx_busy = 0;
	nrf_new_packet = 1;
	//    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin,GPIO_PIN_RESET);
}

void color_wheel(uint32_t hue, float saturation, uint8_t * r, uint8_t * g, uint8_t * b);

/*
 * callback function that handles the gpio interrupt count
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_7)
		nrf_irq_handler(&nrf);
}

uint8_t uart_rx_cplt_flag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_rx_cplt_flag = 1;
}

void change_nrf_payload_length(uint8_t length);
void change_nrf_tx_address(const uint8_t * tx_address);

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
	TIMER_UPDATE_DUTY(0,1000,0);	//B,G,R
	HAL_Delay(100);

	float uart_rx_buf[num_frames];
	int start_idx = 0;
	int num_bytes_frame_buf = sizeof(float)*num_frames;
	uint32_t uart_tx_ts = 0;

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
	uint8_t comm_down_led_toggle_flag = 0;



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

	uint32_t led_ts = 0;
	uint32_t uart_ts = 0;

	uint16_t i2c_base_addr = 0x24;

	uint32_t init_pos_ts = HAL_GetTick()+2000;
	while(HAL_GetTick()<init_pos_ts)
	{
		for(int frame = 1; frame < num_frames;frame++)
		{
			if(init_pos_ts - HAL_GetTick() < 10)
				tau[frame].v = -0.0f;
			else
				tau[frame].v = -5.0f;
		}

		i2c_robot_master(i2c_base_addr, num_frames,
				q_i2c,	i2c_rx_previous,
				tau, q_offset);

	}

	uint32_t ierr_ts = 0;
	uint8_t r,g,b;
	int strip = 0;
	uint32_t strip_update_ts = 0;
	uint32_t time_start = HAL_GetTick();
	float q_prev[num_frames];
	float t_prev[num_frames];

	while (1)
	{
//		float t= ((float)HAL_GetTick())*.001f;
		color_wheel(fmod_2pi(q[1])*244.461993f, 1.0, &r, &g, &b);

		/*****************This block of code manages I2C robust communications to a chain of i2c devices.****************************************/
		i2c_robot_master(i2c_base_addr, num_frames,
				q_i2c,	i2c_rx_previous,
				tau, q_direct);
		/***********************************************************End***************************************************************************/
		for(int frame = 0; frame < num_frames; frame++)
			q[frame] = (q_direct[frame]-q_offset[frame])*0.15789473684f;

//		/**********************************Inside here, q and q_previous are legitimate*************************************************************/
		float t = ((float)(HAL_GetTick()-time_start))*.001f;
//		for(int frame = 1; frame < num_frames; frame++)
//			qd[frame] = 3.0f*(sin_fast(t*4.0f));
		qd[1] = 0;

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
			controller_PI(qd[frame], q[frame], 50.0f, .0001f, &x_i[frame], &tau_tmp);
			if(tau_tmp > 40.0f)
				tau_tmp = 40.0f;
			if(tau_tmp < -40.0f)
				tau_tmp = -40.0f;
			tau[frame].v = tau_tmp; 
			//tau[frame].v = 10.0f*sin_fast(t*4.0f);
//			tau[frame].v = 10.0f;
		}

		if(HAL_GetTick()>uart_ts)
		{
			HAL_UART_Transmit(&huart2, (uint8_t *)(&t), 4, 10);
			HAL_UART_Transmit(&huart2, (uint8_t *)(&q[1]), 4, 10);
			float dqdt = 9.54929659f*((q[1] - q_prev[1])/(t-t_prev[1]));
			q_prev[1] = q[1];
			t_prev[1] = t;
			HAL_UART_Transmit(&huart2, (uint8_t *)(&dqdt), 4, 10);
			//HAL_UART_Transmit(&huart2, (uint8_t *)(&tau[1].v), 4, 10);
//			HAL_UART_Transmit(&huart2, (uint8_t *)(&q[2]), 4, 10);
			uart_ts = HAL_GetTick()+15;
		}

//		if(HAL_GetTick()-rx_ts > 1000 && HAL_GetTick() > comm_down_ts)
//		{
//			nrf_init(&nrf, &config);
//			comm_down_ts = HAL_GetTick()+1000;
//		}
//		else if(HAL_GetTick()-rx_ts > 1000 && HAL_GetTick() > comm_down_ts - 900)
//		{
//			TIMER_UPDATE_DUTY(0,0,0);
//		}

		if(HAL_GetTick() > led_ts)
		{
			led_ts = HAL_GetTick()+250;
		}
		rgb_disp(r,g,b);
//		if(HAL_GetTick()>=strip_update_ts)
//		{
//			tx_addr[4]=strip;
//			change_nrf_tx_address((const uint8_t * )tx_addr);
//			uint32_t r32 = r*4;
//			uint32_t g32 = g*4;
//			uint32_t b32 = b*4;
//			uint32_t tmp = ((r32 & 0x03FF) << 22) | ((g32 & 0x03FF) << 12) | ((b32 & 0x03FF) << 2);
//			nrf_send_packet_noack(&nrf, (uint8_t *)(&tmp) );
//
//			strip++;
//			if(strip >= 3)
//				strip=0;
//			strip_update_ts = HAL_GetTick()+5;
//		}

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
