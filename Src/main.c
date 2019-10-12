#include "init.h"
#include "i2c_master.h"
#include "sin-math.h"
#include "nrf24l01.h"

const int num_frames = 2;	//number of frames on the robot, including the zeroeth frame. If a robot has 6dof, it has 7 frames.

#define NUM_BYTES_PAYLOAD 3
uint8_t tx_address[5] = {0x32,  0x77, 0x62, 0xFA, 0x00};
uint8_t rx_address[5] = {'g', 'c', 'a', 'r', '2'};

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
			uint32_t r32 = 200;
			uint32_t g32 = 1023;
			uint32_t b32 = 0;
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
	float q[num_frames];	//this is used for pcontrol
	float q_offset[num_frames];

	uint32_t led_ts = 0;
	uint32_t uart_ts = 0;

	uint16_t i2c_base_addr = 0x20;

	uint32_t init_pos_ts = HAL_GetTick()+500;
	while(HAL_GetTick()<init_pos_ts)
	{
		i2c_robot_master(i2c_base_addr, num_frames,
				q_i2c,	i2c_rx_previous,
				tau, q_offset);
	}

	while (1)
	{
		/*****************This block of code manages I2C robust communications to a chain of i2c devices.****************************************/
		i2c_robot_master(i2c_base_addr, num_frames,
				q_i2c,	i2c_rx_previous,
				tau, q);
		/***********************************************************End***************************************************************************/
		for(int frame = 1; frame < num_frames; frame++)
			q[frame] = q[frame]-q_offset[frame];

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

		if(nrf_new_packet == 1)
		{
			TIMER_UPDATE_DUTY(
					(rx_buf[0]*1000)/255,
					(rx_buf[1]*1000)/255,
					(rx_buf[2]*1000)/255
			);

			nrf_new_packet = 0;
		}


		if(HAL_GetTick()-rx_ts > 1000 && HAL_GetTick() > comm_down_ts)
		{
			nrf_init(&nrf, &config);
			TIMER_UPDATE_DUTY(0,0,1000);
			comm_down_ts = HAL_GetTick()+1000;
		}
		else if(HAL_GetTick()-rx_ts > 1000 && HAL_GetTick() > comm_down_ts - 900)
		{
			TIMER_UPDATE_DUTY(0,0,0);
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
