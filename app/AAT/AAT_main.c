/*
  * AAT_main.c
 *
 *  Created on: 2017. 8. 7. . 2022. 05. 20
 *      Author: Kwon, Yu Jin Park
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hw_config.h"
#include "sw_config.h"

//nrf_Driver inclde
#include "nrf_drv_gpiote.h"

//nrf_library include
#include "app_uart.h"
#include "app_timer.h"

//user library include
#include "AAT_main.h"

#include "AAT_device_EXT_FLASH.h"
#include "AAT_driver_i2c.h"
#include "AAT_driver_spi.h"
#include "AAT_device_LED.h"
#include "AAT_device_RTC.h"

#include "AAT_device_BMI160.h"
#include "AAT_device_OPT3001.h"

//#include "LAP_api.h"
#include "ble_stack.h"

#include "AAT_BLE_Protocol.h"

static int16_t att_on_check_min, att_on_check_max, att_off_check_min, att_off_check_max;

static int16_t gyro_x_val_window[DEFAULT_GYRO_DATA_WINDOW_SIZE] = {0, };
static int16_t gyro_y_val_window[DEFAULT_GYRO_DATA_WINDOW_SIZE] = {0, };
static int16_t gyro_z_val_window[DEFAULT_GYRO_DATA_WINDOW_SIZE] = {0, };

static uint16_t gyro_window_index = 0;
static uint8_t acc_sensor_axis = ATT_ACC_SENSOR_AXIS_DEFAULT;

static uint16_t sleep_count = 0;
static uint8_t ATT_sw_mode = AAT_SW_MODE_DEFAULT;

static msgq_pt ATT_Main_Msgq;


void set_ATT_acc_check_value(int16_t on_check_min, int16_t on_check_max, int16_t off_check_min, int16_t off_check_max)
{
	att_on_check_min = on_check_min;
	att_on_check_max = on_check_max;
	att_off_check_min = off_check_min;
	att_off_check_max = off_check_max;
}

void update_sleep_count()
{
	sleep_count = UPDATE_VAL_SLEEP_COUNT;
}

void ACC_init(nrf_drv_gpiote_evt_handler_t gpiote_evt_handler) {
	//Interrupt Mode Check
	nrf_drv_gpiote_in_config_t pin_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	pin_config.pull = NRF_GPIO_PIN_PULLUP;
	nrf_drv_gpiote_in_init(AAT_BOARD_PIN_ACEEL_INT_1, &pin_config, gpiote_evt_handler);
	nrf_drv_gpiote_in_event_disable(AAT_BOARD_PIN_ACEEL_INT_1);

	AAT_Bmi160_init();

	task_sleep(300);

	nrf_drv_gpiote_in_event_enable(AAT_BOARD_PIN_ACEEL_INT_1, true);
}

void ALS_init(nrf_drv_gpiote_evt_handler_t gpiote_evt_handler) {
	//Interrupt Mode Check
	nrf_drv_gpiote_in_config_t pin_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	pin_config.pull = NRF_GPIO_PIN_PULLUP;
	nrf_drv_gpiote_in_init(AAT_BOARD_PIN_ACEEL_INT_1, &pin_config, gpiote_evt_handler);
	nrf_drv_gpiote_in_event_enable(AAT_BOARD_PIN_ACEEL_INT_1, false);
}

void RTC_init(nrf_drv_gpiote_evt_handler_t gpiote_evt_handler) {
	//nrf_drv_gpiote_in_config_t pin_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	nrf_drv_gpiote_in_config_t pin_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

	nrf_drv_gpiote_in_init(AAT_BOARD_ALS_INT, &pin_config, gpiote_evt_handler);
	nrf_drv_gpiote_in_event_enable(AAT_BOARD_ALS_INT, true);

	AAT_RTC_device_init();
}

bool FLASH_init(void) {
	return AAT_falsh_data_all_erase();
}

void ACC_PIN_Interrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {

	//Task check INT Check
	nrf_drv_gpiote_in_event_disable(AAT_BOARD_PIN_ACEEL_INT_1);

	update_sleep_count();
}

void ALS_PIN_Interrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	nrf_drv_gpiote_in_event_disable(AAT_BOARD_ALS_INT);

	AAT_Main_event_send(AAT_ALS_EVT, 0x00, NULL);

	nrf_drv_gpiote_in_event_enable(AAT_BOARD_ALS_INT, true);
}

void RTC_Interrupt_handler(nrf_drv_gpiote_pin_t pin,
		nrf_gpiote_polarity_t action) {
	nrf_drv_gpiote_in_event_disable(AAT_BOARD_RTC_INT);

	nrf_drv_gpiote_in_event_enable(AAT_BOARD_RTC_INT, true);
}

int send_ble_streaming_data(int16_t ACC_X, int16_t ACC_Y, int16_t ACC_Z
, int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z)
{
	uint8_t tx_temp_packet[PAAR_SETUP_PACKET_LEN_SENSOR_STREAM] = {0, };

	tx_temp_packet[PAAR_PACKET_INDEX_PACKET_TYPE] = 0x88;
	tx_temp_packet[PAAR_PACKET_INDEX_SERVICE_ID] = 0xFE;
	tx_temp_packet[PAAR_PACKET_INDEX_SEQUENCE_NUM] = 0x11;
	tx_temp_packet[PAAR_PACKET_INDEX_DATA_LEN] = 13;
	tx_temp_packet[PAAR_PACKET_INDEX_BODY_DATA_CMD] = 0x07;

	memcpy(&tx_temp_packet[PAAR_SETUP_INDEX_SENSOR_STREAM_ACC_X], &ACC_X, 2);
	memcpy(&tx_temp_packet[PAAR_SETUP_INDEX_SENSOR_STREAM_ACC_Y], &ACC_Y, 2);
	memcpy(&tx_temp_packet[PAAR_SETUP_INDEX_SENSOR_STREAM_ACC_Z], &ACC_Z, 2);
	memcpy(&tx_temp_packet[PAAR_SETUP_INDEX_SENSOR_STREAM_GYRO_X], &GYRO_X, 2);
	memcpy(&tx_temp_packet[PAAR_SETUP_INDEX_SENSOR_STREAM_GYRO_Y], &GYRO_Y, 2);
	memcpy(&tx_temp_packet[PAAR_SETUP_INDEX_SENSOR_STREAM_GYRO_Z], &GYRO_Z, 2);

	return LAP_send_ble_msg_peripheral(tx_temp_packet, PAAR_SETUP_PACKET_LEN_SENSOR_STREAM);
}

int send_ble_streaming_gyro_data(uint16_t gyro_variance)
{
	uint8_t tx_temp_packet[PAAR_SETUP_PACKET_LEN_GYRO_STREAM] = {0, };

	tx_temp_packet[PAAR_PACKET_INDEX_PACKET_TYPE] = 0x88;
	tx_temp_packet[PAAR_PACKET_INDEX_SERVICE_ID] = 0xFE;
	tx_temp_packet[PAAR_PACKET_INDEX_SEQUENCE_NUM] = 0x11;
	tx_temp_packet[PAAR_PACKET_INDEX_DATA_LEN] = 3;
	tx_temp_packet[PAAR_PACKET_INDEX_BODY_DATA_CMD] = 0x09;

	memcpy(&tx_temp_packet[PAAR_PACKET_INDEX_BODY_DATA_BODY], &gyro_variance, 2);

	return LAP_send_ble_msg_peripheral(tx_temp_packet, PAAR_SETUP_PACKET_LEN_GYRO_STREAM);
}

void set_AAT_SW_mode(uint8_t sw_mode)
{
	ATT_sw_mode = sw_mode;
	switch(ATT_sw_mode)
	{
		case AAT_SW_MODE_ACC :
			sleep_count = ATT_SLEEP_CNT_ACC_MODE;
			break;
		case AAT_SW_MODE_GYRO :
			sleep_count = ATT_SLEEP_CNT_GYRO_MODE;
			break;
		case ATT_SW_MODE_SETUP_STREAM_ACC_GYRO_DATA :
			sleep_count = ATT_SLEEP_CNT_SETUP_MODE;
			break;
		case ATT_SW_MODE_SETUP_STREAM_GYRO_WINDOW :
			sleep_count = ATT_SLEEP_CNT_SETUP_MODE;
			break;
		default :
			break;
	}
}

void set_AAT_default_SW_mode()
{
	set_AAT_SW_mode(AAT_SW_MODE_DEFAULT);
}

void ATT_HW_init()
{
	nrf_drv_gpiote_init();

	nrf_gpio_cfg_output(AAT_BOARD_ACCEL_PWR_EN);
	nrf_gpio_pin_set(AAT_BOARD_ACCEL_PWR_EN);

	nrf_gpio_cfg_output(AAT_BOARD_ALS_PWR_EN);
	nrf_gpio_pin_set(AAT_BOARD_ALS_PWR_EN);

	AAT_spi_init();
	AAT_i2c_init();

	OPT3001_init();

	ACC_init(ACC_PIN_Interrupt_handler);
}

void ATT_start_signal_led()
{

	AAT_LED_port_init();
	AAT_LED_ON(RED);
	task_sleep(500);
	AAT_LED_OFF(RED);
	task_sleep(500);
	AAT_LED_ON(RED);
	task_sleep(500);
	AAT_LED_OFF(RED);
	task_sleep(500);
	AAT_LED_ON(RED);
	task_sleep(500);
	AAT_LED_OFF(RED);
	task_sleep(500);
}

void ATT_set_enter_sleep()
{
	AAT_LED_OFF(WHITE);
	AAT_LED_ON(BLUE);
	task_sleep(100);
	AAT_LED_OFF(WHITE);

	sleep_count = 0;
	nrf_drv_gpiote_in_event_disable(AAT_BOARD_PIN_ACEEL_INT_1);
	task_sleep(300);
	AAT_enter_sleep();
}

void Gyro_Data_STREAM_init()
{
	uint8_t i;

	gyro_window_index = 0;

	for(i=0; i<DEFAULT_ACC_DATA_WINDOW_SIZE; i++)
	{
		gyro_x_val_window[i] = 0;
		gyro_y_val_window[i] = 0;
		gyro_z_val_window[i] = 0;
		gyro_window_index++;
		if(gyro_window_index >= DEFAULT_ACC_DATA_WINDOW_SIZE)
			gyro_window_index = 0;
	}
}

void Read_Gyro_Data_For_STREAM()
{
	gyro_x_val_window[gyro_window_index] = BMI160_GYRO_DATA_READ_X_AXIS();
	gyro_y_val_window[gyro_window_index] = BMI160_GYRO_DATA_READ_Y_AXIS();
	gyro_z_val_window[gyro_window_index] = BMI160_GYRO_DATA_READ_Z_AXIS();

	gyro_window_index++;
	if(gyro_window_index >= DEFAULT_ACC_DATA_WINDOW_SIZE)
			gyro_window_index = 0;
}

uint16_t Cal_gyro_variance_all()
{
	uint16_t var_x, var_y, var_z;
	uint8_t i, process_index_1, process_index_2;

	var_x = 0;
	var_y = 0;
	var_z = 0;

	process_index_1 = gyro_window_index+1;
	if(process_index_1 >= DEFAULT_ACC_DATA_WINDOW_SIZE)
		process_index_1 = 0;

	process_index_2 = process_index_1++;
	if(process_index_2 >= DEFAULT_ACC_DATA_WINDOW_SIZE)
		process_index_2 = 0;

	for(i= 0; i<DEFAULT_ACC_DATA_WINDOW_SIZE; i++)
	{
		var_x += abs(gyro_x_val_window[process_index_2] - gyro_x_val_window[process_index_1]);
		var_y += abs(gyro_y_val_window[process_index_2] - gyro_y_val_window[process_index_1]);
		var_z += abs(gyro_z_val_window[process_index_2] - gyro_z_val_window[process_index_1]);

		process_index_1++;
		if(process_index_1 >= DEFAULT_ACC_DATA_WINDOW_SIZE)
			process_index_1 = 0;

		process_index_2++;
		if(process_index_2 >= DEFAULT_ACC_DATA_WINDOW_SIZE)
			process_index_2 = 0;

	}

	return (var_x + var_y + var_z)/DEFAULT_ACC_DATA_WINDOW_SIZE/NUM_ACC_GYRO_AXIS;
}

int16_t Read_ACC_sensor_val()
{
	switch(acc_sensor_axis)
	{
		case AAT_ACC_SENSOR_AXIS_X :
		{
			return BMI160_ACC_DATA_READ_X_AXIS();
		}
			break;
		case AAT_ACC_SENSOR_AXIS_Y :
		{
			return BMI160_ACC_DATA_READ_Y_AXIS();
		}
			break;
		case AAT_ACC_SENSOR_AXIS_Z :
		{
			return BMI160_ACC_DATA_READ_Z_AXIS();
		}
			break;
	}

	
	return 0;
}

void AAT_main_task(void * arg) {

	//AATMainEvt_t Main_msgRxBuffer;

	int16_t temp_sensor_val = 0;
	unsigned int temp_gyro_val = 0;

	//uint8_t gyro_on_count = 5;
	uint8_t gyro_off_count = GYRO_OFF_COUNT_REFRESH_VAL;

	bool is_AAT_ON = false;
	int16_t temp_sensor_acc_x, temp_sensor_acc_y, temp_sensor_acc_z;
	int16_t temp_sensor_gyro_x, temp_sensor_gyro_y, temp_sensor_gyro_z;

	//test code
	#if 1
	task_sleep(10000);

	while(1)
	{
		task_sleep(5000);
		send_AAT_result_packet(ANALOG_ADL_ON);
		task_sleep(5000);
		send_AAT_result_packet(ANALOG_ADL_OFF);
	}
	#endif

	ATT_HW_init();

	task_sleep(1000);

	ATT_start_signal_led();

	//test_code : ACC Streaming
	set_AAT_default_SW_mode();

	if(ATT_sw_mode == ATT_SW_MODE_SETUP_STREAM_GYRO_WINDOW || ATT_sw_mode == AAT_SW_MODE_GYRO)
		Gyro_Data_STREAM_init();

	set_ATT_acc_check_value(ATT_CHECK_VAL_ON_MIN, ATT_CHECK_VAL_ON_MAX, ATT_CHECK_VAL_OFF_MIN, ATT_CHECK_VAL_OFF_MAX);


	while(1)
	{
		while(sleep_count == 0)
		{
			task_sleep(30);
		}

		AAT_wake_up();
	
		switch(ATT_sw_mode)
		{
			case AAT_SW_MODE_ACC :
			{
				temp_sensor_val = Read_ACC_sensor_val();
				if(temp_sensor_val > att_on_check_min && temp_sensor_val < att_on_check_max && is_AAT_ON == false)
				{
					send_AAT_result_packet(ANALOG_ADL_ON);
					update_sleep_count();
					is_AAT_ON = true;
					task_sleep(500);
				}
				else if(temp_sensor_val <= att_off_check_max && temp_sensor_val >= att_off_check_min && is_AAT_ON == true)
				{
					send_AAT_result_packet(ANALOG_ADL_OFF);
					update_sleep_count();
					is_AAT_ON = false;
					task_sleep(500);
				}
				task_sleep(100);
			}
			break;
			case AAT_SW_MODE_GYRO :
			{
				Read_Gyro_Data_For_STREAM();
				temp_gyro_val = Cal_gyro_variance_all();
				
				if(temp_gyro_val > GYRO_VARIANCE_THRESHOLD)
				{
					if(is_AAT_ON == false)
					{
						printf("ACC On\r\n");
						send_AAT_result_packet(ANALOG_ADL_ON);
						is_AAT_ON = true;	
					}

					gyro_off_count = GYRO_OFF_COUNT_REFRESH_VAL;

					update_sleep_count();
					task_sleep(TEST_STEAMING_PERIOD);	
				}
				else if(temp_gyro_val <= GYRO_VARIANCE_THRESHOLD)
				{
					if(is_AAT_ON == true && gyro_off_count == 0)
					{
						printf("ACC Off\r\n");
						send_AAT_result_packet(ANALOG_ADL_OFF);
						is_AAT_ON = false;
					}
					
					if(gyro_off_count > 0)
					{
						gyro_off_count--;
					}

					update_sleep_count();
					task_sleep(TEST_STEAMING_PERIOD);
				}
				task_sleep(100);
			}
			break;
			case ATT_SW_MODE_SETUP_STREAM_ACC_GYRO_DATA :
			{
				update_sleep_count();

				temp_sensor_acc_x = BMI160_ACC_DATA_READ_X_AXIS();
				task_sleep(10);
				temp_sensor_acc_y = BMI160_ACC_DATA_READ_Y_AXIS();
				task_sleep(10);
				temp_sensor_acc_z = BMI160_ACC_DATA_READ_Z_AXIS();
				task_sleep(10);
				temp_sensor_gyro_x = BMI160_GYRO_DATA_READ_X_AXIS();
				task_sleep(10);
				temp_sensor_gyro_y = BMI160_GYRO_DATA_READ_Y_AXIS();
				task_sleep(10);
				temp_sensor_gyro_z = BMI160_GYRO_DATA_READ_Z_AXIS();

				send_ble_streaming_data(temp_sensor_acc_x, temp_sensor_acc_y, temp_sensor_acc_z,
				 						 temp_sensor_gyro_x, temp_sensor_gyro_y, temp_sensor_gyro_z);

				task_sleep(TEST_STEAMING_PERIOD);
			}
			break;
			case ATT_SW_MODE_SETUP_STREAM_GYRO_WINDOW :
			{
				update_sleep_count();

				Read_Gyro_Data_For_STREAM();

				send_ble_streaming_gyro_data(Cal_gyro_variance_all());

				task_sleep(TEST_STEAMING_PERIOD);
			}
			break;
		}

		sleep_count--;
		if(sleep_count <= 0)
		{
			ATT_set_enter_sleep();
		}
	}
}


void AAT_main_task_init(void) {
	int r;

	r = msgq_create(&ATT_Main_Msgq, sizeof(AATMainEvt_t), MAIN_MAX_MSGQ_COUNT);
	if (0 != r) {
		logme("fail at msgq_create\r\n");
	}

	// Task create
	r = task_create(NULL, AAT_main_task, NULL, task_gethighestpriority() - 2, 512, "Main");
	if (r != 0) {
		printf("task_create(AAT_main_task) failed\n\r");
	} else {
		printf("task_create(AAT_main_task) created\n\r");
	}
}

uint8_t AAT_Main_event_send(uint8_t Main_evt, uint16_t Main_evt_state,
		uint8_t* msg) {
	AATMainEvt_t temp_msg;

	temp_msg.event = Main_evt;
	temp_msg.status = Main_evt_state;
	temp_msg.msg = msg;

	msgq_send(ATT_Main_Msgq, (unsigned char*) &temp_msg);

	return 0;
}