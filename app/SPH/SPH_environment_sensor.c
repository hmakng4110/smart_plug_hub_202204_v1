/*
 * SPH_environment_sensor.c
 *
 *  Created on: 2022. 05. 24.
 *      Author: YJPark
 */

#include <ubinos.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "app_timer.h"

#include "SPH_environment_sensor.h"
#include "LAP_api.h"

#include "twi_sensor_module.h"
#include "wiz360_uart_wifi_module.h"

#include "SAAL_packet_process.h"

#include "hw_config.h"
#include "sw_config.h"

APP_TIMER_DEF(SPH_environment_sensor_report_timer);

static msgq_pt sph_env_report_task_msgq;

void SPH_env_sensor_check()
{
    SPH_env_event_send(SPH_ENV_EVENT_REPORT_SERVER, 0, NULL);
}

#define SPH_ENVIRONMENT_REPORT_PERIOD          APP_TIMER_TICKS(5000)

int report_SPH_environment_data_to_wifi()
{
    twi433_sensor_data_t tmp_env_data;

	memset(&tmp_env_data, 0, sizeof(twi433_sensor_data_t));

	tmp_env_data.pressure = sh_pressure_get();
	tmp_env_data.humidity = sh_humidity_get();
	tmp_env_data.temperature = sh_temperature_get();
	tmp_env_data.gas_adc_raw = sh_gas_raw_get();
	tmp_env_data.gas_alg_data = sh_ccs811_alg_result_data_get();
	tmp_env_data.color = sh_color_get();

	printf("\r\n");
	printf("Pressure 	: %.3fhpa\r\n", tmp_env_data.pressure);
	printf("Humidity	: %d%%\r\n", tmp_env_data.humidity+15);
	printf("Temperature	: %.3fC\r\n", tmp_env_data.temperature);
	printf("Gas Alg		: ec02:%dppm TVOC:%dppb\r\n", tmp_env_data.gas_alg_data.ec02_ppm, tmp_env_data.gas_alg_data.tvoc_ppb);
	printf("Red			: [%d]\r\n", tmp_env_data.color.red);
	printf("Green		: [%d]\r\n", tmp_env_data.color.green);
	printf("Blue		: [%d]\r\n", tmp_env_data.color.blue);
	printf("Clear		: [%d]\r\n", tmp_env_data.color.clear);
	printf("\r\n");

	uint8_t* mqtt_msg = NULL;

    uint8_t body_len = PAAR_MQTT_SIZE_ENV_PACKET;
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

	mqtt_msg = malloc(PAAR_ID_SIZE + PAAR_SERVICE_ID_SIZE + body_len + PAAR_MQTT_SIZE_BODY_LENGTH + PAAR_MQTT_SIZE_LOCATION_LENGTH);

	memset(mqtt_msg, 0, sizeof(mqtt_msg));

	uint8_t report_device_id[4];
	uint8_t service_id;

	report_device_id[0] = PAAR_ID_0;
	report_device_id[1] = PAAR_ID_1;
	report_device_id[2] = PAAR_ID_2;
	report_device_id[3] = PAAR_ID_3;

	service_id = SAAL_SERVICE_ID_SPH;

	memcpy(&mqtt_msg[PAAR_MQTT_INDEX_PAAR_ID], report_device_id, PAAR_ID_SIZE);

	memcpy(&mqtt_msg[PAAR_MQTT_INDEX_SERVCIE_ID], &service_id, PAAR_SERVICE_ID_SIZE);

	mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA_LEN] = body_len;

	uint16_t press_natural = (uint16_t)tmp_env_data.pressure;
	uint16_t press_decimals = (uint16_t)((tmp_env_data.pressure * 1000) - ((int)tmp_env_data.pressure * 1000));
	uint16_t tempe_natrual = (uint16_t)tmp_env_data.temperature;
	uint16_t tempe_decimals = (uint16_t)((tmp_env_data.temperature * 1000) - ((int)tmp_env_data.temperature * 1000));

	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_PRESS_0] = (char)(press_natural >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_PRESS_1] = (char)(press_natural & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_PRESS_DECIMAL_0] = (char)(press_decimals >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_PRESS_DECIMAL_1] = (char)(press_decimals & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_TEMP_0] = (char)(tempe_natrual >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_TEMP_1] = (char)(tempe_natrual & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_TEMP_DECIMAL_0] = (char)(tempe_decimals >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_TEMP_DECIMAL_1] = (char)(tempe_decimals & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_HUMIDITY_0] = (char)(tmp_env_data.humidity >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_HUMIDITY_1] = (char)(tmp_env_data.humidity & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_GAS_ADC_0] = (char)(tmp_env_data.gas_adc_raw >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_GAS_ADC_1] = (char)(tmp_env_data.gas_adc_raw & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_GAS_EC02_0] = (char)(tmp_env_data.gas_alg_data.ec02_ppm >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_GAS_EC02_1] = (char)(tmp_env_data.gas_alg_data.ec02_ppm & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_GAS_TVOC_0] = (char)(tmp_env_data.gas_alg_data.tvoc_ppb >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_GAS_TVOC_1] = (char)(tmp_env_data.gas_alg_data.tvoc_ppb & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_COLOR_RED_0] = (char)(tmp_env_data.color.red >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_COLOR_RED_1] = (char)(tmp_env_data.color.red & 0xff8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_COLOR_GREEN_0] = (char)(tmp_env_data.color.green >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_COLOR_GREEN_1] = (char)(tmp_env_data.color.green & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_COLOR_BLUE_0] = (char)(tmp_env_data.color.blue >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_COLOR_BLUE_1] = (char)(tmp_env_data.color.blue & 0xff);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_COLOR_CLEAR_0] = (char)(tmp_env_data.color.clear >> 8);
	mqtt_msg[PAAR_MQTT_INDEX_ENV_DATA_COLOR_CLEAR_1] = (char)(tmp_env_data.color.clear & 0xff);

    mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA+body_len] = location_len;
    
    memcpy(&mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA+body_len+PAAR_MQTT_SIZE_LOCATION_LENGTH], temp_char_location, location_len);

    free(temp_char_location);

	wifi_processing_event_send(WIFI_PROCESSING_EVENT_SEND_MQTT_ENV, 0, mqtt_msg);

	return 0;
}

void SPH_environment_report_task(void* arg)
{
    ret_code_t err;

    task_sleep(1000);

    evironment_sensor_module_init();

    #if(SAAL_HW_DEVICE_TYPE == SAAL_HW_DEVICE_SPH)
	while(get_wifi_ready() == false)
	{
		task_sleep(100);
	}
    #endif    

    err = sh_environment_enable();
	if(err != NRF_SUCCESS) {
		printf("sh_environment_enable error : %ld\r\n", err);
		//SH_SystemReset(err);
        while(1)
        {
            task_sleep(1000);
        }
	}

    task_sleep(SPH_ENV_TASK_START_DELAY);

    app_timer_create(&SPH_environment_sensor_report_timer, APP_TIMER_MODE_REPEATED, SPH_env_sensor_check);
    app_timer_start(SPH_environment_sensor_report_timer, SPH_ENVIRONMENT_REPORT_PERIOD, NULL);
    sph_env_msgq_event_t SPH_env_evt_msg;

    while(1)
    {
        err = msgq_receive(sph_env_report_task_msgq, (unsigned char * ) &SPH_env_evt_msg);
		if( err != 0 ) {
			printf("fail at msgq receive\r\n");
		} else {
			switch( SPH_env_evt_msg.event ) {

			case SPH_ENV_EVENT_REPORT_SERVER:
            {
                ret_code_t err = NRF_SUCCESS;
                report_SPH_environment_data_to_wifi();
            }
				break;
			default:
				break;
			}
            app_sched_execute();

            if(SPH_env_evt_msg.msg != NULL)
		    {
			    free(SPH_env_evt_msg.msg);
		    }
		}
    }

}

void SPH_envrionment_report_task_init(void)
{
    
    int r;
    	
	r = msgq_create(&sph_env_report_task_msgq, sizeof(sph_env_msgq_event_t), 20);
	if( 0 != r ) {
		printf("fail at task_create\r\n");
	}

    r = task_create(NULL, SPH_environment_report_task, NULL, task_gethighestpriority()-2, 512, NULL);
	if (r != 0) {
		printf("== SPH_environment_report_task failed \n\r");
	} else {
		printf("== SPH_environment_report_task created \n\r");
	}
}

int SPH_env_event_send(uint8_t evt, uint8_t state, uint8_t * msg) {
	sph_env_msgq_event_t tmp_sph_env_msgq_event;

	tmp_sph_env_msgq_event.event = evt;
	tmp_sph_env_msgq_event.status = state;
	tmp_sph_env_msgq_event.msg = msg;

	return msgq_send(sph_env_report_task_msgq, (unsigned char * ) &tmp_sph_env_msgq_event);
}