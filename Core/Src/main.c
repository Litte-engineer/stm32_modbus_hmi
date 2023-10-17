
#include "main.h"
#include "modbus_crc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define Function_code dataRx[1]
#define ADDRESS_FLASH (uint32_t)0x0800FC00
#define STATUS dataRx[4]

#define V1_GAS GPIO_PIN_14// OUT RLY7
#define V2_GAS GPIO_PIN_13 // OUT RLY8
#define V3_GAS GPIO_PIN_12 // OUT RLY9
#define V4_GAS GPIO_PIN_11 // OUT RLY10
#define V5_GAS GPIO_PIN_10 // OUT RLY11
#define V6_GAS GPIO_PIN_1  // OUT RLY12

#define V_WATER GPIO_PIN_9// OUT RLY1
#define V1_TEST GPIO_PIN_8 // OUT RLY2
#define V2_TEST GPIO_PIN_7 // OUT RLY3
#define V3_TEST GPIO_PIN_6 // OUT RLY4
#define V4_TEST GPIO_PIN_5 // OUT RLY5
#define V5_TEST GPIO_PIN_15 // OUT RLY6


#define STOP 0
#define TESTING1 1
#define TESTING2 2
#define FAILED 3
#define PASS 4

#define START 1
#define TEST 2
#define END 3
#define RESET 0

#define HMI_ON 1
#define HMI_OFF 0

#define SAVE_DATA_FLASH 0x01

#define UP_SETUP_PRESSURE_V1_ON 0x0f
#define DOWN_SETUP_PRESSURE_V1_ON 0x1b
#define UP_SETUP_PRESSURE_V2_ON 0x10
#define DOWN_SETUP_PRESSURE_V2_ON 0x1c
#define UP_SETUP_PRESSURE_V3_ON 0x11
#define DOWN_SETUP_PRESSURE_V3_ON 0x1d
#define UP_SETUP_PRESSURE_V4_ON 0x12
#define DOWN_SETUP_PRESSURE_V4_ON 0x1e
#define UP_SETUP_PRESSURE_V5_ON 0x13
#define DOWN_SETUP_PRESSURE_V5_ON 0x1f

#define UP_SETUP_PRESSURE_V1_OFF 0x14
#define DOWN_SETUP_PRESSURE_V1_OFF 0x20
#define UP_SETUP_PRESSURE_V2_OFF 0x15
#define DOWN_SETUP_PRESSURE_V2_OFF 0x21
#define UP_SETUP_PRESSURE_V3_OFF 0x16
#define DOWN_SETUP_PRESSURE_V3_OFF 0x22
#define UP_SETUP_PRESSURE_V4_OFF 0x17
#define DOWN_SETUP_PRESSURE_V4_OFF 0x23
#define UP_SETUP_PRESSURE_V5_OFF 0x18
#define DOWN_SETUP_PRESSURE_V5_OFF 0x24

#define UP_SETUP_PRECISION_FLOW 0x19
#define DOWN_SETUP_PRECISION_FLOW 0x25
#define UP_SETUP_PRECISION_TDS 0x1a
#define DOWN_SETUP_PRECISION_TDS 0x26

#define UP_TIME_TEST_V1 0x27
#define UP_TIME_TEST_V2 0x28
#define UP_TIME_TEST_V3 0x29
#define UP_TIME_TEST_V4 0x2A
#define UP_TIME_TEST_V5 0x2B
#define UP_TIME_TEST_SENSOR 0x2C
#define UP_TIME_TEST_TDS 0x2D

#define DOWN_TIME_TEST_V1 0x2E
#define DOWN_TIME_TEST_V2 0x2F
#define DOWN_TIME_TEST_V3 0x30
#define DOWN_TIME_TEST_V4 0x31
#define DOWN_TIME_TEST_V5 0x32
#define DOWN_TIME_TEST_SENSOR 0x33
#define DOWN_TIME_TEST_TDS 0x34

#define MANUAL_V1_START 0x80
#define MANUAL_V2_START 0x81
#define MANUAL_V3_START 0x82
#define MANUAL_V4_START 0x83
#define MANUAL_V5_START 0x84
#define MANUAL_SENSOR_START 0x85
#define AUTO_START 0x86

#define MANUAL_V1_STOP 0x88
#define MANUAL_V2_STOP 0x89
#define MANUAL_V3_STOP 0x8a
#define MANUAL_V4_STOP 0x8b
#define MANUAL_V5_STOP 0x8c
#define MANUAL_SENSOR_STOP 0x8d
#define AUTO_STOP 0x8e

#define ON 0xff
#define OFF 0x00

#define CHANGE_MODE 0x00
#define START_SAVE 1
#define END_SAVE 0

uint8_t trigger_save_flash;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t dataRx[8];
uint8_t dataTx[20];
uint16_t data[10];
uint16_t Data_Input_Register[10];
int16_t data_flash[19];
uint16_t readAdc[7];
uint8_t button_status_hmi;
uint8_t manual_status_hmi;
uint8_t holding_status_hmi;
uint8_t input_register_status_hmi;

const uint8_t time_const = 10;

struct test
{
	uint8_t v1;
	uint8_t v2;
	uint8_t v3;
	uint8_t v4;
	uint8_t v5;
	uint8_t sensor;
	uint8_t Auto;
} test;

struct time
{
	uint32_t v1;
	uint32_t v2;
	uint32_t v3;
	uint32_t v4;
	uint32_t v5;
	uint32_t Auto;
	uint32_t sensor;
} time;

struct pressure
{
	float v1;
	float v2;
	float v3;
	float v4;
	float v5;
	float v1_on;
	float v2_on;
	float v3_on;
	float v4_on;
	float v5_on;
	float v1_off;
	float v2_off;
	float v3_off;
	float v4_off;
	float v5_off;
} pressure;

struct pressure_decimal
{
	uint16_t v1;
	uint16_t v2;
	uint16_t v3;
	uint16_t v4;
	uint16_t v5;
} pressure_decimal;

struct sensor
{
	uint16_t flow;
	uint16_t tds;
	uint16_t flow_sample;
	uint16_t tds_sample;
} sensor;

struct setup
{
	float pressure_v1_on;
	int16_t pressure_v1_on_decimal;
	float pressure_v2_on;
	int16_t pressure_v2_on_decimal;
	float pressure_v3_on;
	int16_t pressure_v3_on_decimal;
	float pressure_v4_on;
	int16_t pressure_v4_on_decimal;
	float pressure_v5_on;
	int16_t pressure_v5_on_decimal;
	float pressure_v1_off;
	int16_t pressure_v1_off_decimal;
	float pressure_v2_off;
	int16_t pressure_v2_off_decimal;
	float pressure_v3_off;
	int16_t pressure_v3_off_decimal;
	float pressure_v4_off;
	int16_t pressure_v4_off_decimal;
	float pressure_v5_off;
	int16_t pressure_v5_off_decimal;

	int16_t prescision_tds;
	int16_t prescision_flow;

	int16_t time_test_v1;
	int16_t time_test_v2;
	int16_t time_test_v3;
	int16_t time_test_v4;
	int16_t time_test_v5;
	int16_t time_test_auto;
	int16_t time_test_sensor;
	
  int16_t time_test_auto_start ;
	int16_t time_test1_auto;
	int16_t time_test2_auto;
	int16_t time_test3_auto;
	int16_t time_test4_auto;
	int16_t time_test5_auto;
	int16_t time_test_sensor_auto;
	
} setup;

struct status
{
	uint8_t v1;
	uint8_t v2;
	uint8_t v3;
	uint8_t v4;
	uint8_t v5;
	uint8_t tds;
	uint8_t flow;
	uint8_t all;
} status;

struct count
{
	uint8_t v1;
	uint8_t v2;
	uint8_t v3;
	uint8_t v4;
	uint8_t v5;
	uint8_t sensor;
	uint8_t led;
	uint8_t Auto;
} count;

struct manual
{
	uint8_t start_v1;
	uint8_t start_v2;
	uint8_t start_v3;
	uint8_t start_v4;
	uint8_t start_v5;
	uint8_t start_sensor;

	uint8_t stop_v1;
	uint8_t stop_v2;
	uint8_t stop_v3;
	uint8_t stop_v4;
	uint8_t stop_v5;
	uint8_t stop_sensor;

	uint8_t start;
	uint8_t stop;
} manual;

struct Auto{
	uint8_t start;
	uint8_t stop;
}Auto;

struct adc{
	uint16_t channel_0;
	uint16_t channel_1;
	uint16_t channel_2;
	uint16_t channel_3;
	uint16_t channel_4;
	uint16_t channel_5;
	uint16_t channel_6;
}adc;

struct voltage{
	float channel_0;
	float channel_1;
	float channel_2;
	float channel_3;
	float channel_4;
	float channel_5;
	float channel_6;
}voltage;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart1;

void test_sensor_manual();
void test_sensor_auto();
void Flash_Erase(uint32_t address);
void Flash_Write_Int(uint32_t address, int16_t value);
int16_t Flash_Read_Init(uint32_t address);
void Flash_Write_multi_Data(uint32_t address, uint8_t len, int16_t *data);
void Flash_Read_multi_Data(uint32_t address, uint8_t len, int16_t *data);
void Read_Data_Flash();
void Test_Valve_Nc(uint8_t* test ,uint32_t V_GAS, uint32_t V_TEST,uint8_t* count, uint8_t* status, uint32_t* time, 
	              int16_t* setuptime,float* pressure, float* pressure_off,float* pressure_on, float* voltage,
								float* pressure_setup_off,float* pressure_setup_on );
void Test_Valve_No(uint8_t* test ,uint32_t V_GAS, uint32_t V_TEST,uint8_t* count, uint8_t* status, uint32_t* time, 
	              int16_t* setuptime,float* pressure, float* pressure_off,float* pressure_on, float* voltage,
								float* pressure_setup_off,float* pressure_setup_on );
void auto_test_vnc(int16_t* setuptime1 ,int16_t* setuptime2, int16_t* setuptime3, uint8_t* status, uint32_t V_GAS, uint32_t V_TEST, float* pressure_off, float* pressure,
                 float* voltage, float* pressure_on, float* setuppressure_off, float* setuppressure_on );
void auto_test_vno(int16_t* setuptime1 ,int16_t* setuptime2 , int16_t* setuptime3, uint8_t* status, 
	               uint32_t V_GAS, uint32_t V_TEST, float* pressure_off, float* pressure, 
								float* voltage, float* pressure_on, float* setuppressure_off, float* setuppressure_on );
void auto_mode();
								 
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
								 
////// kiem tra ket qua test////
uint8_t check_status(float* setuppressure_off, float* setuppressure_on, float*pressure_on, float* pressure_off )
{
		uint8_t status;
		if(*pressure_off <= *setuppressure_off && *pressure_on >= *setuppressure_on)
		{
			status = PASS;
		}
		else if(*pressure_off > *setuppressure_off || *pressure_on < *setuppressure_on)
		{
			status = FAILED;
		}
		return status;	
}	

///////// doc gia tri cb ap suat/////
float pressure_value(float* voltage)
{
	float pressure_value;
	pressure_value = (12.0*(*voltage) - 3.6)/2.7;
	return pressure_value;
}
///////////////////////////////

//////////// dat gioi han ap suat ////////////////////////
void limit_setup_pressure(float *setup_on, float *setup_off, int16_t *setup_on_decimal, int16_t *setup_off_decimal)
{
	if (*setup_on_decimal > 120)
	{
		*setup_on_decimal = 0;
	}
	else if (*setup_on_decimal < 0)
	{
		*setup_on_decimal = 120;
	}
	if (*setup_off_decimal > 120)
	{
		*setup_off_decimal = 0;
	}
	else if (*setup_off_decimal < 0)
	{
		*setup_off_decimal = 120;
	}
	*setup_on = (float)((*setup_on_decimal) / 10.0);
	*setup_off = (float)((*setup_off_decimal) / 10.0);
}
////////////////////////////////////////////////////////
///////// dat gioi han cam bien  test ///////////////////
void limit_setup_sensor(int16_t *setup_precision_sensor)
{
	if (*setup_precision_sensor > 20)
	{
		*setup_precision_sensor = 0;
	}
	else if (*setup_precision_sensor < 0)
	{
		*setup_precision_sensor = 20;
	}
}
/////////////////////////////////////////////////////////////
//////////////// dat gioi han thoi gian test ///////////////
void limit_setup_time_test(int16_t *time)
{
	if (*time > 90)
	{
		*time = 10;
	}
	else if (*time < 10)
	{
		*time = 90;
	}
}
////////////////////////////////////////////
void manual_status(uint8_t *status1, uint8_t *status2)
{
	if (STATUS == ON)
	{
		*status1 = 1;
		*status2 = 0;
	}
}
/////////////////////////////////////
void van_on(uint32_t van)
{
	HAL_GPIO_WritePin(GPIOB, van, GPIO_PIN_SET);
}
////////////////////////////////////////
void van_off(uint32_t van)
{
	HAL_GPIO_WritePin(GPIOB, van, GPIO_PIN_RESET);
}
//////////////////////////////////////
///////// lay gia tri trung binh ///////
uint16_t medium_pressure(uint16_t* pressure_decimal, float* pressure, uint16_t len)
{
	uint32_t sum;
	for(int i = 0; i < len; i++)
	{
		sum += *pressure *100 ;
	}
	*pressure_decimal = (uint16_t)sum/len;
	return *pressure_decimal;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
	HAL_UART_Receive_IT(&huart1, dataRx, 8);
	Flash_Read_multi_Data(ADDRESS_FLASH, 19, data_flash);
	Auto.stop = 1;
	Auto.start = 0;
	setup.pressure_v1_on_decimal = data_flash[0];
	setup.pressure_v2_on_decimal = data_flash[1];
	setup.pressure_v3_on_decimal = data_flash[2];
	setup.pressure_v4_on_decimal = data_flash[3];
	setup.pressure_v5_on_decimal = data_flash[4];
	setup.pressure_v1_off_decimal = data_flash[5];
	setup.pressure_v2_off_decimal = data_flash[6];
	setup.pressure_v3_off_decimal = data_flash[7];
	setup.pressure_v4_off_decimal = data_flash[8];
	setup.pressure_v5_off_decimal = data_flash[9];
	setup.prescision_flow =  data_flash[10];
	setup.prescision_tds = data_flash[11];
	setup.time_test_v1 = data_flash[12];
	setup.time_test_v2 = data_flash[13];
	setup.time_test_v3 = data_flash[14];
	setup.time_test_v4 = data_flash[15];
	setup.time_test_v5 = data_flash[16];
	setup.time_test_sensor = data_flash[17];
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)readAdc, 7);	
		setup.time_test_auto = setup.time_test_v1 + setup.time_test_v2 + setup.time_test_v3 +
		                       setup.time_test_v4 + setup.time_test_v5 + setup.time_test_sensor;	
    setup.time_test1_auto = setup.time_test_v1;
		setup.time_test2_auto = setup.time_test_v1 + setup.time_test_v2;
		setup.time_test3_auto = setup.time_test_v1 + setup.time_test_v2 + setup.time_test_v3;
		setup.time_test4_auto = setup.time_test_v1 + setup.time_test_v2 + setup.time_test_v3 + setup.time_test_v4;
		setup.time_test5_auto = setup.time_test_v1 + setup.time_test_v2 + setup.time_test_v3 + 
		                        setup.time_test_v4 + setup.time_test_v5;
		setup.time_test_sensor_auto = setup.time_test_v1 + setup.time_test_v2 + setup.time_test_v3 + 
		                              setup.time_test_v4 + setup.time_test_v5 + setup.time_test_sensor;
		adc.channel_0 = readAdc[0];
		adc.channel_1 = readAdc[1];
	  adc.channel_2 = readAdc[2]; 
	  adc.channel_3 = readAdc[3]; 
	  adc.channel_4 = readAdc[4];	
		adc.channel_5 = readAdc[5];	
		adc.channel_6 = readAdc[6];	
		/* doc gia tri dien ap */
		voltage.channel_0 = ((3.3*adc.channel_0)/4095.0);	
	  voltage.channel_1 = ((3.3*adc.channel_1)/4095.0);	
		voltage.channel_2 = ((3.3*adc.channel_2)/4095.0);	
		voltage.channel_3 = ((3.3*adc.channel_3)/4095.0);	
		voltage.channel_4 = ((3.3*adc.channel_4)/4095.0);	
		voltage.channel_5 = ((3.3*adc.channel_5)/4095.0);	
		voltage.channel_6 = ((3.3*adc.channel_6)/4095.0);	
		//////////////////////////////	
		/* doc cam bien ap suat 1 */
		pressure.v1 = pressure_value(&voltage.channel_0);
		pressure_decimal.v1 = (uint16_t)(pressure.v1*100);
		/*doc cam bien ap suat 2*/
	  pressure.v2 = pressure_value(&voltage.channel_1);
		pressure_decimal.v2 = (uint16_t)(pressure.v2*100);
		/*doc cam bien ap suat 3*/
		pressure.v3 = pressure_value(&voltage.channel_2);
		pressure_decimal.v3 = (uint16_t)(pressure.v3*100);
		/*doc cam bien ap suat 4*/
		pressure.v4 = pressure_value(&voltage.channel_3);
		pressure_decimal.v4 = (uint16_t)(pressure.v4*100);
		/*doc cam bien ap suat 5*/	
		pressure.v5 = pressure_value(&voltage.channel_4);
		pressure_decimal.v5 = (uint16_t)(pressure.v5*100);
			
		////////////////////////////////			
		/* doc gia tri cam bien TDS */
		sensor.tds_sample  = (voltage.channel_5*1000)/1.533;
		sensor.tds = (voltage.channel_6*1000)/1.533;
    /////////////////////////////
	  manual.start = ((0xff & (manual.start_v1 << 0)) | (0xff & (manual.start_v2 << 1)) | (0xff & (manual.start_v3 << 2)) |
						(0xff & (manual.start_v4 << 3)) | (0xff & (manual.start_v5 << 4)) | (0xff & (manual.start_sensor << 5)) | (0xff & (Auto.start << 6)));
		manual.stop = ((0xff & (manual.stop_v1 << 0)) | (0xff & (manual.stop_v2 << 1)) | (0xff & (manual.stop_v3 << 2)) |
					   (0xff & (manual.stop_v4 << 3)) | (0xff & (manual.stop_v5 << 4)) | (0xff & (manual.stop_sensor << 5)) | (0xff & (Auto.stop << 6)));
		limit_setup_pressure(&setup.pressure_v1_on, &setup.pressure_v1_off,
							 &setup.pressure_v1_on_decimal, &setup.pressure_v1_off_decimal);
		limit_setup_pressure(&setup.pressure_v2_on, &setup.pressure_v2_off,
							 &setup.pressure_v2_on_decimal, &setup.pressure_v2_off_decimal);
		limit_setup_pressure(&setup.pressure_v3_on, &setup.pressure_v3_off,
							 &setup.pressure_v3_on_decimal, &setup.pressure_v3_off_decimal);
		limit_setup_pressure(&setup.pressure_v4_on, &setup.pressure_v4_off,
							 &setup.pressure_v4_on_decimal, &setup.pressure_v4_off_decimal);
		limit_setup_pressure(&setup.pressure_v4_on, &setup.pressure_v4_off,
							 &setup.pressure_v4_on_decimal, &setup.pressure_v4_off_decimal);
		limit_setup_pressure(&setup.pressure_v5_on, &setup.pressure_v5_off,
							 &setup.pressure_v5_on_decimal, &setup.pressure_v5_off_decimal);
		limit_setup_sensor(&setup.prescision_flow);
		limit_setup_sensor(&setup.prescision_tds);
		limit_setup_time_test(&setup.time_test_v1);
		limit_setup_time_test(&setup.time_test_v2);
		limit_setup_time_test(&setup.time_test_v3);
		limit_setup_time_test(&setup.time_test_v4);
		limit_setup_time_test(&setup.time_test_v5);
		limit_setup_time_test(&setup.time_test_sensor);
		if(trigger_save_flash == START_SAVE)
		{
			Read_Data_Flash();
			Flash_Erase(ADDRESS_FLASH);	
      Flash_Write_multi_Data(ADDRESS_FLASH, 19, data_flash);
      trigger_save_flash = END_SAVE	;		
		}
    if(button_status_hmi == HMI_ON)
		{
			switch (dataRx[3])
			{
       case CHANGE_MODE:
				manual.stop_v1 = 1;
				manual.stop_v2 = 1;
				manual.stop_v3 = 1;
				manual.stop_v4 = 1;
				manual.stop_v5 = 1;
				manual.stop_sensor = 1;
			  Auto.stop = 1;
				test.v1 = RESET;
				test.v2 = RESET;
				test.v3 = RESET;
				test.v4 = RESET;
				test.v5 = RESET;
				test.sensor = RESET;
        test.Auto = RESET;			 
        manual.start_v1 = 0;
				manual.start_v2 = 0;
				manual.start_v3 = 0;
				manual.start_v4 = 0;
				manual.start_v5 = 0;
				manual.start_sensor = 0;
        Auto.start = 0;			 
			break;
			// luu cai dat cac gia tri
			case SAVE_DATA_FLASH:
				trigger_save_flash = START_SAVE;
				break;
				// cai dat ap suat van 1 dinh muc	khi mo
			case UP_SETUP_PRESSURE_V1_ON:
				setup.pressure_v1_on_decimal = setup.pressure_v1_on_decimal + 1;
				break;
			case DOWN_SETUP_PRESSURE_V1_ON:
				setup.pressure_v1_on_decimal = setup.pressure_v1_on_decimal - 1;
				break;
			// cai dat ap suat van 1 dinh muc	khi dong
			case UP_SETUP_PRESSURE_V1_OFF:
				setup.pressure_v1_off_decimal = setup.pressure_v1_off_decimal  + 1;
				break;
			case DOWN_SETUP_PRESSURE_V1_OFF:
				setup.pressure_v1_off_decimal  = setup.pressure_v1_off_decimal  - 1;
				break;
			// cai dat ap suat van 2 dinh muc khi mo
			case UP_SETUP_PRESSURE_V2_ON:
				setup.pressure_v2_on_decimal  = setup.pressure_v2_on_decimal + 1;
				break;
			case DOWN_SETUP_PRESSURE_V2_ON:
				setup.pressure_v2_on_decimal = setup.pressure_v2_on_decimal - 1;
				break;
			// cai dat ap suat van 2 dinh muc khi dong
			case UP_SETUP_PRESSURE_V2_OFF:
				setup.pressure_v2_off_decimal = setup.pressure_v2_off_decimal + 1;
				break;
			case DOWN_SETUP_PRESSURE_V2_OFF:
				setup.pressure_v2_off_decimal = setup.pressure_v2_off_decimal - 1;
				break;
			// cai dat ap suat van 3 dinh muc khi mo
			case UP_SETUP_PRESSURE_V3_ON:
				setup.pressure_v3_on_decimal = setup.pressure_v3_on_decimal + 1;
				break;
			case DOWN_SETUP_PRESSURE_V3_ON:
				setup.pressure_v3_on_decimal = setup.pressure_v3_on_decimal - 1;
				break;
			// cai dat ap suat van 3 dinh muc khi dong
			case UP_SETUP_PRESSURE_V3_OFF:
				setup.pressure_v3_off_decimal = setup.pressure_v3_off_decimal + 1;
				break;
			case DOWN_SETUP_PRESSURE_V3_OFF:
				setup.pressure_v3_off_decimal = setup.pressure_v3_off_decimal - 1;
				break;
			// cai dat ap suat van 4 dinh muc khi mo
			case UP_SETUP_PRESSURE_V4_ON:
				setup.pressure_v4_on_decimal = setup.pressure_v4_on_decimal + 1;
				break;
			case DOWN_SETUP_PRESSURE_V4_ON:
				setup.pressure_v4_on_decimal = setup.pressure_v4_on_decimal - 1;
				break;
			// cai dat ap suat van 4 dinh muc khi dong
			case UP_SETUP_PRESSURE_V4_OFF:
				setup.pressure_v4_off_decimal = setup.pressure_v4_off_decimal + 1;
				break;
			case DOWN_SETUP_PRESSURE_V4_OFF:
				setup.pressure_v4_off_decimal = setup.pressure_v4_off_decimal - 1;
				break;
			// cai dat ap suat van 5 dinh muc khi mo
			case UP_SETUP_PRESSURE_V5_ON:
				setup.pressure_v5_on_decimal = setup.pressure_v5_on_decimal + 1;
				break;
			case DOWN_SETUP_PRESSURE_V5_ON:
				setup.pressure_v5_on_decimal = setup.pressure_v5_on_decimal - 1;
				break;
			// cai dat ap suat van 5 dinh muc khi dong
			case UP_SETUP_PRESSURE_V5_OFF:
				setup.pressure_v5_off_decimal = setup.pressure_v5_off_decimal + 1;
				break;
			case DOWN_SETUP_PRESSURE_V5_OFF:
				setup.pressure_v5_off_decimal = setup.pressure_v5_off_decimal - 1;
				break;
			// cai dat dung sai cam bien flow
			case UP_SETUP_PRECISION_FLOW:
				setup.prescision_flow = setup.prescision_flow + 1;
				break;
			case DOWN_SETUP_PRECISION_FLOW:
				setup.prescision_flow = setup.prescision_flow - 1;
				break;
			// cai dat dung sai cam bien tds
			case UP_SETUP_PRECISION_TDS:
				setup.prescision_tds = setup.prescision_tds + 1;
				break;
			case DOWN_SETUP_PRECISION_TDS:
				setup.prescision_tds = setup.prescision_tds - 1;
				break;
			// cai dat thoi gian test van 1
			case UP_TIME_TEST_V1:
				setup.time_test_v1 = setup.time_test_v1 + 10;
				break;
			case DOWN_TIME_TEST_V1:
				setup.time_test_v1 = setup.time_test_v1 - 10;
				break;
			// cai dat thoi gian test van 2
			case UP_TIME_TEST_V2:
				setup.time_test_v2 = setup.time_test_v2 + 10;
				break;
			case DOWN_TIME_TEST_V2:
				setup.time_test_v2 = setup.time_test_v2 - 10;
				break;
			// cai dat thoi gian test van 3
			case UP_TIME_TEST_V3:
				setup.time_test_v3 = setup.time_test_v3 + 10;
				break;
			case DOWN_TIME_TEST_V3:
				setup.time_test_v3 = setup.time_test_v3 - 10;
				break;
			// cai dat thoi gian test van 4
			case UP_TIME_TEST_V4:
				setup.time_test_v4 = setup.time_test_v4 + 10;
				break;
			case DOWN_TIME_TEST_V4:
				setup.time_test_v4 = setup.time_test_v4 - 10;
				break;
			// cai dat thoi gian test van 5
			case UP_TIME_TEST_V5:
				setup.time_test_v5 = setup.time_test_v5 + 10;
				break;
			case DOWN_TIME_TEST_V5:
				setup.time_test_v5 = setup.time_test_v5 - 10;
				break;
			// cai dat thoi gian test cambien
			case UP_TIME_TEST_SENSOR:
				setup.time_test_sensor = setup.time_test_sensor + 10;
				break;
			case DOWN_TIME_TEST_SENSOR:
				setup.time_test_sensor = setup.time_test_sensor - 10;
				break;
				// bat dau test van 1
			case MANUAL_V1_START:
				manual_status(&manual.start_v1, &manual.stop_v1);
			  test.v1 = START;
				break;
			case MANUAL_V1_STOP:
				manual_status(&manual.stop_v1, &manual.start_v1);
			  test.v1 = RESET;
				break;
				// bat dau test van 2
			case MANUAL_V2_START:
				manual_status(&manual.start_v2, &manual.stop_v2);
			  test.v2 = START;
				break;
			case MANUAL_V2_STOP:
				manual_status(&manual.stop_v2, &manual.start_v2);
			  test.v2 = RESET;
				break;
				// bat dau test van 3
			case MANUAL_V3_START:
				manual_status(&manual.start_v3, &manual.stop_v3);
			  test.v3 = START;
				break;
			case MANUAL_V3_STOP:
				manual_status(&manual.stop_v3, &manual.start_v3);
			  test.v3 = RESET;
				break;
				// bat dau test van 4
			case MANUAL_V4_START:
				manual_status(&manual.start_v4, &manual.stop_v4);
			  test.v4 = START;
				break;
			case MANUAL_V4_STOP:
				manual_status(&manual.stop_v4, &manual.start_v4);
			  test.v4 = RESET;
				break;
				// bat dau test van 5
			case MANUAL_V5_START:
				manual_status(&manual.start_v5, &manual.stop_v5);
			  test.v5 = START;
				break;
			case MANUAL_V5_STOP:
				manual_status(&manual.stop_v5, &manual.start_v5);
			  test.v5 = RESET;
				break;
			// bat dau test luu luong
			case MANUAL_SENSOR_START:
				manual_status(&manual.start_sensor, &manual.stop_sensor);
			  test.sensor = START;
				break;
			case MANUAL_SENSOR_STOP:
				manual_status(&manual.stop_sensor, &manual.start_sensor);
		    test.sensor = RESET;
				break;
			// trang thai auto
			case AUTO_START:
				manual_status(&Auto.start, &Auto.stop);
			  test.Auto = START;
				break;
			case AUTO_STOP:
				manual_status(&Auto.stop, &Auto.start);
			  test.Auto = RESET;
				break;
			}  
			button_status_hmi = HMI_OFF;
		}		
		///////////// doc trang thai manual ///////////////////
		if(manual_status_hmi == HMI_ON)
		{
			dataTx[1] = dataRx[1];
			// uint16_t addr = (dataRx[2] << 8) | (dataRx[3]);
			dataTx[2] = 0x02;
			dataTx[3] = manual.start;
			dataTx[4] = manual.stop;
			uint16_t crc2 = 0;
			crc2 = crc16(dataTx, 5);
			dataTx[5] = (crc2)&0xff;
			dataTx[6] = (crc2 >> 8) & 0xff;
			HAL_UART_Transmit(&huart1, dataTx, 7, 10);
			manual_status_hmi = HMI_OFF;
		}
		////////////////////////////////////////////////////
		////////////////////// DOC THANH GHI HOLDING//////////
		if(holding_status_hmi == HMI_ON)
		{
			dataTx[1] = dataRx[1];
			uint16_t addr = (dataRx[2] << 8) | (dataRx[3]);
			switch (dataRx[3])
			{
			case 0x00:
				//data[0] = pressure_decimal.v1;
			  data[0] = pressure_decimal.v1;
				data[1] = pressure_decimal.v2;
				data[2] = pressure_decimal.v3;
				data[3] = pressure_decimal.v4;
				data[4] = pressure_decimal.v5;
				data[5] = sensor.flow;
				data[6] = sensor.tds;
				break;
			case 0x07:
				data[0] = setup.pressure_v1_on_decimal;
				data[1] = setup.pressure_v2_on_decimal;
				data[2] = setup.pressure_v3_on_decimal;
				data[3] = setup.pressure_v4_on_decimal;
				data[4] = setup.pressure_v5_on_decimal;
				data[5] = setup.pressure_v1_off_decimal;
				data[6] = setup.pressure_v2_off_decimal;
				data[7] = setup.pressure_v3_off_decimal;
				data[8] = setup.pressure_v4_off_decimal;
				data[9] = setup.pressure_v5_off_decimal;
				break;

			case 0x11:
				data[0] = setup.prescision_flow;
				data[1] = setup.prescision_tds;
				break;

			case 0x13:
				data[0] = setup.time_test_v1;
				data[1] = setup.time_test_v2;
				data[2] = setup.time_test_v3;
				data[3] = setup.time_test_v4;
				data[4] = setup.time_test_v5;
				data[5] = setup.time_test_sensor;
				break;

			case 0x1a:
				data[0] = 1997;
				break;
			} 
			uint16_t len = (dataRx[4] << 8) | (dataRx[5]);
			dataTx[2] = len * 2;
			for (uint16_t i = 0; i < len; i++)
			{
				dataTx[3 + i * 2] = (data[i] >> 8) & 0xff;
				dataTx[4 + i * 2] = (data[i]) & 0xff;
			}
			uint16_t crc = 0;
			crc = crc16(dataTx, len * 2 + 3);
			dataTx[len * 2 + 3] = (crc)&0xff;
			dataTx[len * 2 + 4] = (crc >> 8) & 0xff;
			HAL_UART_Transmit(&huart1, dataTx, len * 2 + 5, 20);
			holding_status_hmi = HMI_OFF;		
		}
		/////////////////////////////////////////
		///////////////////////////// DOC THANH GHI INPUT REGISTER/////////////////
		if(input_register_status_hmi == HMI_ON)
		{
			dataTx[1] = dataRx[1];
			uint16_t addr1 = (dataRx[2] << 8) | (dataRx[3]);
			uint16_t len1 = (dataRx[4] << 8) | (dataRx[5]);
			dataTx[2] = len1 * 2;
			Data_Input_Register[0] = status.v1;
			Data_Input_Register[1] = status.v2;
			Data_Input_Register[2] = status.v3;
			Data_Input_Register[3] = status.v4;
			Data_Input_Register[4] = status.v5;
			Data_Input_Register[5] = status.tds;
			Data_Input_Register[6] = status.flow;
			for (uint16_t i = 0; i < len1; i++)
			{
				dataTx[3 + i * 2] = (Data_Input_Register[i] >> 8) & 0xff;
				dataTx[4 + i * 2] = (Data_Input_Register[i]) & 0xff;
			}
			uint16_t crc1 = 0;
			crc1 = crc16(dataTx, len1 * 2 + 3);
			dataTx[len1 * 2 + 3] = (crc1)&0xff;
			dataTx[len1 * 2 + 4] = (crc1 >> 8) & 0xff;
			HAL_UART_Transmit(&huart1, dataTx, len1 * 2 + 5, 10);		
			input_register_status_hmi = HMI_OFF;
		}
		//////////////////////////////////////////	
		Test_Valve_Nc(&test.v2 ,V2_GAS, V2_TEST, &count.v2, &status.v2, &time.v2, 
	              &setup.time_test_v2, &pressure.v2, &pressure.v2_off, &pressure.v2_on, &voltage.channel_1,
								&setup.pressure_v2_off, &setup.pressure_v2_on);	
    Test_Valve_Nc(&test.v3 ,V3_GAS, V3_TEST, &count.v3, &status.v3, &time.v3, 
	              &setup.time_test_v3, &pressure.v3, &pressure.v3_off, &pressure.v3_on, &voltage.channel_2,
								&setup.pressure_v3_off, &setup.pressure_v3_on);	
    Test_Valve_Nc(&test.v1 ,V1_GAS, V1_TEST, &count.v1, &status.v1, &time.v1, 
	              &setup.time_test_v1, &pressure.v1, &pressure.v1_off, &pressure.v1_on, &voltage.channel_0,
								&setup.pressure_v1_off, &setup.pressure_v1_on);	
		Test_Valve_No(&test.v4 ,V4_GAS, V4_TEST, &count.v4, &status.v4, &time.v4, 
	              &setup.time_test_v4, &pressure.v4, &pressure.v4_off, &pressure.v4_on, &voltage.channel_3,
								&setup.pressure_v4_off, &setup.pressure_v4_on);	
		Test_Valve_No(&test.v5 ,V5_GAS, V5_TEST, &count.v5, &status.v5, &time.v5, 
	              &setup.time_test_v5, &pressure.v5, &pressure.v5_off, &pressure.v5_on, &voltage.channel_4,
								&setup.pressure_v5_off, &setup.pressure_v5_on);	
		test_sensor_manual();
    auto_mode();
  }
}  /* USER CODE END 3 */
//////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
	{
		HAL_UART_Receive_IT(&huart1, dataRx, 8);
		dataTx[0] = dataRx[0];
		switch (Function_code)
		{
		case 0x03: 
			holding_status_hmi = HMI_ON;
		break;
		case 0x04: 
			input_register_status_hmi = HMI_ON;
		break;
		case 0x01:
			manual_status_hmi = HMI_ON;
		break;
		case 0x05:
			button_status_hmi = HMI_ON;
		break;
		}
	}
}
/////////////////////////////////////////////
void Test_Valve_Nc(uint8_t* test ,uint32_t V_GAS, uint32_t V_TEST,uint8_t* count, uint8_t* status, uint32_t* time, 
	              int16_t* setuptime,float* pressure, float* pressure_off,float* pressure_on, float* voltage,
								float* pressure_setup_off,float* pressure_setup_on  )
{
	if(*test == START)
	{
		van_off(V_GAS);
		van_off(V_TEST);
		*test = TEST;	
	}
	if(*test == RESET)
	{
		*count = 0;
		*status = STOP;
		van_off(V_GAS);
		van_off(V_TEST);
	}
	if(*test == TEST)
	{
		if(HAL_GetTick() - *time > 1000)
		{
			 *count = *count + 1;
			*status = TESTING2;
			*time = HAL_GetTick() ;
		}
		if(*count > *setuptime)
		{
			van_off(V_GAS);
		  van_off(V_TEST);
			*test = END;		
		}
		/************* day thu 1 bat dau bat van cap khi 1********/
		if(*count >= *setuptime - 9*(*setuptime/time_const))
		{
			van_on(V_GAS);
			van_off(V_TEST);
			*status = TESTING2;
		}
		/************ tu giay thu 4 den day thu 6 doc cam bien ap suat ***********/
	  if(*count >= *setuptime - 6*(*setuptime/time_const)
			  && *count < *setuptime - 4*(*setuptime/time_const))
		{
			 *status = TESTING1;
			 *pressure_off = *pressure ;	
		}
		/********** tu giay thu 6 bat van test 1 ******/
		if(*count >= *setuptime - 4*(*setuptime/time_const))
		{
			*status = TESTING2;
			van_on(V_TEST);
		}
		/********** tu giay thu 7 doc cam bien ap suat ******/
		if(*count >= *setuptime - 3*(*setuptime/time_const)
			&& *count <= *setuptime - 1*(*setuptime/time_const))
		{
			  *status = TESTING1;
			  van_on(V_TEST);
			 *pressure_on = *pressure;
		}	
	}
	if(*test == END)
	{
		van_off(V_GAS);
		van_off(V_TEST);
		if(*pressure_off <= *pressure_setup_off && *pressure_on >= *pressure_setup_on)
		{
			*status = PASS;
		}
		else if(*pressure_off > *pressure_setup_off || *pressure_on < *pressure_setup_on)
		{
			*status = FAILED;
		}
	}																
}
//////////////////////////////////////////////
/////////////////////////////////////////////
void Test_Valve_No(uint8_t* test ,uint32_t V_GAS, uint32_t V_TEST,uint8_t* count, uint8_t* status, uint32_t* time, 
	              int16_t* setuptime,float* pressure, float* pressure_off,float* pressure_on, float* voltage,
								float* pressure_setup_off,float* pressure_setup_on  )
{
	if(*test == START)
	{
		van_off(V_GAS);
		van_on(V_TEST);
		*test = TEST;	
	}
	if(*test == RESET)
	{
		*count = 0;
		*status = STOP;
		van_off(V_GAS);
		van_off(V_TEST);
	}
	if(*test == TEST)
	{
		if(HAL_GetTick() - *time > 1000)
		{
			 *count = *count + 1;
       *status = TESTING2;
			*time = HAL_GetTick() ;
		}
		if(*count > *setuptime)
		{
			van_off(V_GAS);
		  van_off(V_TEST);
			*test = END;		
		}
		/************* day thu 1 bat dau bat van cap khi 1********/
		if(*count >= *setuptime - 9*(*setuptime/time_const))
		{
			*status = TESTING2;
			van_on(V_GAS);
			van_on(V_TEST);
		}
		/************ tu giay thu 4 den day thu 5 doc cam bien ap suat ***********/
	  if(*count >= *setuptime - 6*(*setuptime/time_const)
			  && *count < *setuptime - 4*(*setuptime/time_const))
		{
			*status = TESTING1;
			 van_on(V_TEST);
			 *pressure_off = *pressure = pressure_value(voltage);		
		}
		/********** tu giay thu 5 den giay thu 6 bat van test 1 ******/
		if(*count >= *setuptime - 4*(*setuptime/time_const))
		{
			*status = TESTING2;
			van_off(V_TEST);
		}
		/********** tu giay thu 7 doc cam bien ap suat ******/
		if(*count >= *setuptime - 3*(*setuptime/time_const)
			 && *count >= *setuptime - 1*(*setuptime/time_const))
		{
			*status = TESTING1;
			 van_off(V_TEST);
			 *pressure_on = *pressure ;//= pressure_value(voltage);
		}	
	}
	if(*test == END)
	{
		van_off(V_GAS);
		van_off(V_TEST);
		if(*pressure_off <= *pressure_setup_off && *pressure_on >= *pressure_setup_on)
		{
			*status = PASS;
		}
		else if(*pressure_off > *pressure_setup_off || *pressure_on < *pressure_setup_on)
		{
			*status = FAILED;
		}
	}																
}
/////////////////////////////////////
//////////// che do auto /////////////
void auto_mode()
{
	if(test.Auto == START)
	{
		test.Auto = TEST;
	}	
	if(test.Auto == RESET)
	{
		count.Auto = 0;	
		status.all = ON;
		//status.v1 = status.v2 = status.v3 = status.v4 = status.v5 = status.flow = status.tds = STOP;
	}
	if(test.Auto == END)
	{
		van_off(V1_GAS);
		van_off(V2_GAS);
		van_off(V3_GAS);
		van_off(V4_GAS);
		van_off(V5_GAS);
		van_off(V6_GAS);
		van_off(V1_TEST);
		van_off(V2_TEST);
		van_off(V3_TEST);
		van_off(V4_TEST);
		van_off(V5_TEST);
		van_off(V_WATER);
    status.v1 = check_status(&setup.pressure_v1_off,&setup.pressure_v1_on, 
		                         &pressure.v1_on, &pressure.v1_off);		
		status.v2 = check_status(&setup.pressure_v2_off,&setup.pressure_v2_on, 
		                         &pressure.v2_on, &pressure.v2_off);
		status.v3 = check_status(&setup.pressure_v3_off,&setup.pressure_v3_on, 
		                         &pressure.v3_on, &pressure.v3_off);
		status.v4 = check_status(&setup.pressure_v4_off,&setup.pressure_v4_on, 
		                         &pressure.v4_on, &pressure.v4_off);
		status.v5 = check_status(&setup.pressure_v5_off,&setup.pressure_v5_on, 
		                         &pressure.v5_on, &pressure.v5_off);
		if(sensor.flow >= sensor.flow_sample)
			{
					if(100 - (sensor.flow_sample/sensor.flow)*100 <= setup.prescision_flow)
					{
							status.flow = PASS;
					}
					else 
					{
							status.flow = FAILED;
					}
			}
			else 
			{
					if(100 - (sensor.flow/sensor.flow_sample)*100 <= setup.prescision_flow)
					{
							status.flow = PASS;
					}
					else 
					{
							status.flow = FAILED;
					}
			}
			if(sensor.tds >= sensor.tds_sample)
			{
					if(100 - (sensor.tds_sample/sensor.tds)*100 <= setup.prescision_tds)
					{
							status.tds = PASS;
					}
					else 
					{
							status.tds = FAILED;
					}
			}
			else 
			{
					if(100 - (sensor.tds/sensor.tds_sample)*100 <= setup.prescision_tds)
					{
							status.tds = PASS;
					}
					else 
					{
							status.tds = FAILED;
					}
			}
	}
	if(test.Auto == TEST)
	{
		if(HAL_GetTick() - time.Auto > 1000)
		{
			count.Auto++;
			time.Auto = HAL_GetTick();
		}
		if(count.Auto > setup.time_test_auto + time_const) 
		{
			van_off(V1_GAS);
			van_off(V2_GAS);
			van_off(V3_GAS);
			van_off(V4_GAS);
			van_off(V5_GAS);
			van_off(V6_GAS);
			van_off(V1_TEST);
			van_off(V2_TEST);
			van_off(V3_TEST);
			van_off(V4_TEST);
			van_off(V5_TEST);
			van_off(V_WATER);	
      test.Auto =  END;			
		}	
		auto_test_vnc(&setup.time_test1_auto, &setup.time_test_v1, &setup.time_test_auto_start, 
		            &status.v1, V1_GAS, V1_TEST, &pressure.v1_off, &pressure.v1,
                &voltage.channel_0, &pressure.v1_on, &setup.pressure_v1_off, &setup.pressure_v1_on );
		auto_test_vnc(&setup.time_test2_auto, &setup.time_test_v2, &setup.time_test1_auto, 
		            &status.v2, V2_GAS, V2_TEST, &pressure.v2_off, &pressure.v2,
                &voltage.channel_1, &pressure.v2_on, &setup.pressure_v2_off, &setup.pressure_v2_on );
		auto_test_vnc(&setup.time_test3_auto, &setup.time_test_v3, &setup.time_test2_auto, 
		            &status.v3, V3_GAS, V3_TEST, &pressure.v3_off, &pressure.v3,
                &voltage.channel_2, &pressure.v3_on, &setup.pressure_v3_off, &setup.pressure_v3_on );
		auto_test_vno(&setup.time_test4_auto, &setup.time_test_v4 , &setup.time_test3_auto, &status.v4, 
	                V4_GAS, V4_TEST, &pressure.v4_off, &pressure.v4, 
								  &voltage.channel_3, &pressure.v4_on, &setup.pressure_v4_off, &setup.pressure_v4_on );
		auto_test_vno(&setup.time_test5_auto, &setup.time_test_v5 , &setup.time_test4_auto, &status.v5, 
	                V5_GAS, V5_TEST, &pressure.v5_off, &pressure.v5, 
								  &voltage.channel_4, &pressure.v5_on, &setup.pressure_v5_off, &setup.pressure_v5_on );
		test_sensor_auto();
	}
}/////////////////

void auto_test_vnc(int16_t* setuptime1 ,int16_t* setuptime2 , int16_t* setuptime3, uint8_t* status, 
	                 uint32_t V_GAS, uint32_t V_TEST, float* pressure_off, float* pressure, 
								   float* voltage, float* pressure_on, float* setuppressure_off, float* setuppressure_on )
{
	 if(count.Auto < *setuptime1 && count.Auto > * setuptime3 )
		{

			/************* day thu 1 bat dau bat van cap khi 1********/
			if(count.Auto >=  *setuptime1 - 9*(*setuptime2/time_const) )
			{
				*status = TESTING2;
				van_on(V_GAS);
				van_off(V_TEST);
			}
			/************ tu giay thu 4 den day thu 6 doc cam bien ap suat ***********/
			if(count.Auto >= *setuptime1 - 6*(*setuptime2/time_const)
					&& count.Auto < *setuptime1 - 4*(*setuptime2/time_const))
			{
				*status = TESTING1;
				 *pressure_off = *pressure = pressure_value(voltage);		
			}
			/********** tu giay thu 6 bat van test 1 ******/
			if(count.Auto >= *setuptime1 - 4*(*setuptime2/time_const))
			{
				*status = TESTING2;
				van_on(V_TEST);
			}
			/********** tu giay thu 7 doc cam bien ap suat ******/
			if(count.Auto > *setuptime1 - 3*(*setuptime2/time_const)
				&& count.Auto <= *setuptime1 - 1*(*setuptime2/time_const))
			{
				*status = TESTING1;
				 van_on(V_TEST);
				 *pressure_on = *pressure = pressure_value(voltage);
			}
     }	
		 if(count.Auto >= *setuptime1)
		 {
				van_off(V_GAS);
				van_off(V_TEST);
				if(*pressure_off <= *setuppressure_off && *pressure_on >= *setuppressure_on)
				{
					*status = PASS;
				}
				else if(*pressure_off > *setuppressure_off || *pressure_on < *setuppressure_on)
				{
					*status = FAILED;
	      }
     }				
}
///////////////////////// auto test van no ////////////////////
void auto_test_vno(int16_t* setuptime1 ,int16_t* setuptime2 , int16_t* setuptime3, uint8_t* status, 
	               uint32_t V_GAS, uint32_t V_TEST, float* pressure_off, float* pressure, 
								float* voltage, float* pressure_on, float* setuppressure_off, float* setuppressure_on )
{
	 if(count.Auto < *setuptime1 && count.Auto > * setuptime3 )
		{
			/************* day thu 1 bat dau bat van cap khi 1********/
			if(count.Auto >=  *setuptime1 - 9*(*setuptime2/time_const) )
			{
				*status = TESTING2;
				van_on(V_GAS);
				van_on(V_TEST);
			}
			/************ tu giay thu 4 den day thu 6 doc cam bien ap suat ***********/
			if(count.Auto >= *setuptime1 - 6*(*setuptime2/time_const)
					&& count.Auto < *setuptime1 - 4*(*setuptime2/time_const))
			{
				*status = TESTING1;
				  van_on(V_TEST);
				 *pressure_off = *pressure = pressure_value(voltage);		
			}
			/********** tu giay thu 5 den giay thu 6 bat van test 1 ******/
			if(count.Auto >= *setuptime1 - 4*(*setuptime2/time_const))
			{
				*status = TESTING2;
				van_off(V_TEST);
			}
			/********** tu giay thu 7 doc cam bien ap suat ******/
			if(count.Auto > *setuptime1 - 3*(*setuptime2/time_const)
				&& count.Auto >= *setuptime1 - 1*(*setuptime2/time_const))
			{
				*status = TESTING1;
				  van_off(V_TEST);
				 *pressure_on = *pressure = pressure_value(voltage);
			}
     }	
		 if(count.Auto >= *setuptime1)
		 {
				van_off(V_GAS);
				van_off(V_TEST);
				if(*pressure_off <= *setuppressure_off && *pressure_on >= *setuppressure_on)
				{
					*status = PASS;
				}
				else if(*pressure_off > *setuppressure_off || *pressure_on < *setuppressure_on)
				{
					*status = FAILED;
	      }
			}				
}
////////////////////////////////////
void test_sensor_auto()
{
	if(count.Auto < setup.time_test_sensor_auto && count.Auto > setup.time_test5_auto)
  {
		/************* day thu 1 bat dau tat van cap khi 1********/
		if(count.Auto >= setup.time_test_sensor_auto - 9*(setup.time_test_sensor))
		{
			status.flow = TESTING2;
			status.tds = TESTING2;
			van_off(V6_GAS);
		}
		/************ tu giay thu 2 den day thu 5 doc cam bien ap suat ***********/
		if(count.Auto >= setup.time_test_sensor_auto - 8*(setup.time_test_sensor/time_const)
					&& count.Auto < setup.time_test_sensor_auto - 5*(setup.time_test_sensor/time_const))
		{
			status.flow = TESTING2;
			status.tds = TESTING2;
			van_on(V_WATER);
		}
		/************ tu giay thu 5 den het ***********/
		if(count.Auto >= setup.time_test_sensor_auto - 5*(setup.time_test_sensor/time_const))
		{
			status.flow = TESTING1;
			status.tds = TESTING1;
			sensor.flow = 0;
			sensor.flow_sample = 0;
			sensor.tds = 0;
			sensor.tds_sample = 0;
		}
		if(count.Auto >= setup.time_test_sensor_auto)
		{
			van_off(V_WATER);
			van_on(V6_GAS);			
		}
  }
}
////////////////////////////////////
void test_sensor_manual()
{
	if(test.sensor == START)
	{
		van_off(V6_GAS);
		van_on(V_WATER);
		test.sensor = TEST;
	}
	if(test.sensor == RESET)
	{
		status.tds = STOP;
		status.flow = STOP;
		count.sensor = 0;		
		van_off(V6_GAS);
		van_off(V_WATER);
	}
	if(test.sensor == END)
	{
		van_off(V6_GAS);
		van_off(V_WATER);
		if(sensor.flow >= sensor.flow_sample)
			{
					if(100 - (sensor.flow_sample/sensor.flow)*100 <= setup.prescision_flow)
					{
							status.flow = PASS;
					}
					else 
					{
							status.flow = FAILED;
					}
			}
			else 
			{
					if(100 - (sensor.flow/sensor.flow_sample)*100 <= setup.prescision_flow)
					{
							status.flow = PASS;
					}
					else 
					{
							status.flow = FAILED;
					}
			}
			if(sensor.tds >= sensor.tds_sample)
			{
					if(100 - (sensor.tds_sample/sensor.tds)*100 <= setup.prescision_tds)
					{
							status.tds = PASS;
					}
					else 
					{
							status.tds = FAILED;
					}
			}
			else 
			{
					if(100 - (sensor.tds/sensor.tds_sample)*100 <= setup.prescision_tds)
					{
							status.tds = PASS;
					}
					else 
					{
							status.tds = FAILED;
					}
			}	
	}
	if(test.sensor == TEST)
	{
		if(HAL_GetTick() - time.sensor > 1000)
		{
			count.sensor ++;
			status.flow = TESTING2;
			status.tds = TESTING2;
      time.sensor = HAL_GetTick();			
		}
		if(count.sensor > setup.time_test_sensor + time_const)
		{
			van_off(V6_GAS);
		  van_off(V_WATER);
			test.sensor = END;
		}
		if(count.sensor >= setup.time_test_sensor - 9*(setup.time_test_sensor))
		{
			status.flow = TESTING2;
			status.tds = TESTING2;
			van_off(V6_GAS);
			van_on(V_WATER);
		}
		/************ tu giay thu 2 den giay t5 cam bien ap suat ***********/
		if(count.sensor >= setup.time_test_sensor - 8*(setup.time_test_sensor/time_const)
					&& count.Auto < setup.time_test_sensor - 5*(setup.time_test_sensor/time_const))
		{
			status.flow = TESTING2;
			status.tds = TESTING2;
			van_on(V_WATER);
			van_off(V6_GAS);
		}
		/************ tu giay thu 5 den het ***********/
		if(count.sensor >= setup.time_test_sensor - 5*(setup.time_test_sensor/time_const))
		{
			status.flow = TESTING1;
			status.tds = TESTING1;
			sensor.flow = 0;
			sensor.flow_sample = 0;
			sensor.tds = 0;
			sensor.tds_sample = 0;
		}
		if(count.sensor >= setup.time_test_sensor)
		{
			status.flow = TESTING2;
			status.tds = TESTING2;
			van_off(V_WATER);
			van_on(V6_GAS);			
		}
	
  }
	
	
}
///////////////////////////////////
////// xoa bo nho flash 
void Flash_Erase(uint32_t address)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = 1;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.PageAddress = address;
	uint32_t pageerr = 0;
	HAL_FLASHEx_Erase (&EraseInitStruct, &pageerr);
	HAL_FLASH_Lock();
}
//////////////////////////////////
////////////// ghi 2 byte vao bo nho flash ///////////////
void Flash_Write_Int(uint32_t address, int16_t value)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, value);
    HAL_FLASH_Lock();
}
//////////////////////////////////////////////
////////////// ghi nhieu du lieu vao bo nho flash /////////////////
void Flash_Write_multi_Data(uint32_t address, uint8_t len, int16_t *data)
{
    HAL_FLASH_Unlock();
    for(int i = 0; i < len; i ++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i*2,data[i]);             
    }
    HAL_FLASH_Lock();
}
////////////////////////////////////////
/////// doc 2 byte tu bo nho flash///////////////////////
int16_t Flash_Read_Init(uint32_t address)
{
  return *(__IO int16_t *)(address);
}
///////////////////////////////////
///////// doc nhieu byte tu bo nho flash///////////////////
void Flash_Read_multi_Data(uint32_t address, uint8_t len, int16_t* data)
{
    for(int i = 0; i < len ; i++)
    {
       data[i] =  Flash_Read_Init(address + i*2);
    }
}
////////////////////////////////////////////////////
void Read_Data_Flash()
{
	data_flash[0] = setup.pressure_v1_on_decimal;
	data_flash[1] = setup.pressure_v2_on_decimal;
	data_flash[2] = setup.pressure_v3_on_decimal;
	data_flash[3] = setup.pressure_v4_on_decimal;
	data_flash[4] = setup.pressure_v5_on_decimal;
	data_flash[5] = setup.pressure_v1_off_decimal;
	data_flash[6] = setup.pressure_v2_off_decimal;
	data_flash[7] = setup.pressure_v3_off_decimal;
	data_flash[8] = setup.pressure_v4_off_decimal;
	data_flash[9] = setup.pressure_v5_off_decimal;
	data_flash[10] = setup.prescision_flow;
	data_flash[11] = setup.prescision_tds;
	data_flash[12] = setup.time_test_v1;
	data_flash[13] = setup.time_test_v2;
	data_flash[14] = setup.time_test_v3;
	data_flash[15] = setup.time_test_v4;
	data_flash[16] = setup.time_test_v5;
	data_flash[17] = setup.time_test_sensor;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 28800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 PB11 PB12
                           PB13 PB14 PB15 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
