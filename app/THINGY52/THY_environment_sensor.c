/*
 * THY_environment_sensor.c
 *
 *  Created on: 2022. 05. 24.
 *      Author: YJPark
 */

#include <ubinos.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "app_timer.h"

#include "THY_environment_sensor.h"
#include "LAP_api.h"

#include "twi_sensor_module.h"
#include "wiz360_uart_wifi_module.h"

#include "hw_config.h"
#include "sw_config.h"

APP_TIMER_DEF(THY_environment_sensor_report_timer);

static msgq_pt THY_env_report_task_msgq;

void THY_env_sensor_check()
{
    THY_env_event_send(THY_ENV_EVENT_REPORT, 0, NULL);
}

#define THY_ENVIRONMENT_REPORT_PERIOD          APP_TIMER_TICKS(5000)

int report_THY_environment_data()
{
  	twi433_sensor_data_t env_data;

	env_data.pressure 		= sh_pressure_get();
	env_data.humidity 		= sh_humidity_get();
	
	env_data.gas_adc_raw	= sh_gas_raw_get();
	env_data.gas_alg_data	= sh_ccs811_alg_result_data_get();
	env_data.color 			= sh_color_get();

	env_data.temperature    = sh_temperature_get();

	uint8_t* ble_msg = NULL;

	uint8_t body_len = PAAR_PACKET_HEADER_LEN + THY_ENV_REPORT_PACKET_SIZE_DATA_BODY;
	uint8_t location_len = strlen(SAAL_LOCATION);
    uint8_t* temp_char_location = NULL;
    temp_char_location = malloc(location_len + 1);
    if(temp_char_location == NULL)
    {
        printf("malloc error: temp_char_location\r\n");
        return -1;
    }

	memset(temp_char_location, 0, location_len + 1);

    strcpy(temp_char_location, SAAL_LOCATION);

	ble_msg = malloc(body_len + THY_ENV_REPORT_SIZE_LOCATION_LEN + location_len);
    if(ble_msg == NULL)
    {
        printf("malloc error: ble_msg\r\n");
        return -1;
    }

	memset(ble_msg, 0, sizeof(ble_msg));

	ble_msg[PAAR_PACKET_INDEX_PACKET_TYPE] = 0x88;
	ble_msg[PAAR_PACKET_INDEX_SERVICE_ID] = 0x18;
	ble_msg[PAAR_PACKET_INDEX_SEQUENCE_NUM] = 0x11;
	ble_msg[PAAR_PACKET_INDEX_DATA_LEN] = THY_ENV_REPORT_PACKET_SIZE_DATA_BODY;
	ble_msg[PAAR_PACKET_INDEX_BODY_DATA_CMD] = 0x01;

	uint16_t press_natural = (uint16_t)env_data.pressure;
	uint16_t press_decimals = (uint16_t)((env_data.pressure * 1000) - ((int)env_data.pressure * 1000));
	uint16_t tempe_natrual = (uint16_t)env_data.temperature;
	uint16_t tempe_decimals = (uint16_t)((env_data.temperature * 1000) - ((int)env_data.temperature * 1000));

	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_PRESS_0] = (char)(press_natural >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_PRESS_1] = (char)(press_natural & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_PRESS_DECIMAL_0] = (char)(press_decimals >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_PRESS_DECIMAL_1] = (char)(press_decimals & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_TEMP_0] = (char)(tempe_natrual >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_TEMP_1] = (char)(tempe_natrual & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_TEMP_DECIMAL_0] = (char)(tempe_decimals >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_TEMP_DECIMAL_1] = (char)(tempe_decimals & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_HUMIDITY_0] = (char)(env_data.humidity >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_HUMIDITY_1] = (char)(env_data.humidity & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_GAS_ADC_0] = (char)(env_data.gas_adc_raw >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_GAS_ADC_1] = (char)(env_data.gas_adc_raw & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_GAS_EC02_0] = (char)(env_data.gas_alg_data.ec02_ppm >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_GAS_EC02_1] = (char)(env_data.gas_alg_data.ec02_ppm & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_GAS_TVOC_0] = (char)(env_data.gas_alg_data.tvoc_ppb >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_GAS_TVOC_1] = (char)(env_data.gas_alg_data.tvoc_ppb & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_COLOR_RED_0] = (char)(env_data.color.red >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_COLOR_RED_1] = (char)(env_data.color.red & 0xff8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_COLOR_GREEN_0] = (char)(env_data.color.green >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_COLOR_GREEN_1] = (char)(env_data.color.green & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_COLOR_BLUE_0] = (char)(env_data.color.blue >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_COLOR_BLUE_1] = (char)(env_data.color.blue & 0xff);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_COLOR_CLEAR_0] = (char)(env_data.color.clear >> 8);
	ble_msg[THY_ENV_REPORT_INDEX_ENV_DATA_COLOR_CLEAR_1] = (char)(env_data.color.clear & 0xff);

	ble_msg[PAAR_PACKET_INDEX_BODY_DATA_CMD+body_len] = location_len;

	memcpy(&ble_msg[PAAR_PACKET_INDEX_BODY_DATA_CMD+body_len+PAAR_MQTT_SIZE_LOCATION_LENGTH], temp_char_location, location_len);

    free(temp_char_location);

	LAP_send_ble_msg_peripheral(ble_msg, body_len + THY_ENV_REPORT_SIZE_LOCATION_LEN + location_len);

	return 0;
}

void THY_environment_report_task(void* arg)
{
    ret_code_t err;

    task_sleep(1000);

    evironment_sensor_module_init();

    task_sleep(THY_ENV_TASK_START_DELAY);

    app_timer_create(&THY_environment_sensor_report_timer, APP_TIMER_MODE_REPEATED, THY_env_sensor_check);
    app_timer_start(THY_environment_sensor_report_timer, THY_ENVIRONMENT_REPORT_PERIOD, NULL);
    THY_env_msgq_event_t THY_env_evt_msg;

    while(1)
    {
        err = msgq_receive(THY_env_report_task_msgq, (unsigned char * ) &THY_env_evt_msg);
		if( err != 0 ) {
			printf("fail at msgq receive\r\n");
		} else {
			switch( THY_env_evt_msg.event ) {

			case THY_ENV_EVENT_REPORT:
            {
                report_THY_environment_data();
				task_sleep(100);
            }
				break;
			default:
				break;
			}
            app_sched_execute();

            if(THY_env_evt_msg.msg != NULL)
		    {
			    free(THY_env_evt_msg.msg);
		    }
		}
    }

}

void THY_envrionment_report_task_init(void)
{
    
    int r;
    	
	r = msgq_create(&THY_env_report_task_msgq, sizeof(THY_env_msgq_event_t), 20);
	if( 0 != r ) {
		printf("fail at task_create\r\n");
	}

    r = task_create(NULL, THY_environment_report_task, NULL, task_gethighestpriority()-2, 512, NULL);
	if (r != 0) {
		printf("== THY_environment_report_task failed \n\r");
	} else {
		printf("== THY_environment_report_task created \n\r");
	}
}

int THY_env_event_send(uint8_t evt, uint8_t state, uint8_t * msg) {
	THY_env_msgq_event_t tmp_THY_env_msgq_event;

	tmp_THY_env_msgq_event.event = evt;
	tmp_THY_env_msgq_event.status = state;
	tmp_THY_env_msgq_event.msg = msg;

	return msgq_send(THY_env_report_task_msgq, (unsigned char * ) &tmp_THY_env_msgq_event);
}