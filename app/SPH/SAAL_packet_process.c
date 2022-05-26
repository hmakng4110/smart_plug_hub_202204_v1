#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <malloc.h>

#include "sw_config.h"
#include "hw_config.h"

#include "LAP_api.h"
#include "LAP_main.h"

#include "ble_cell_management.h"

#include "../lib_wifi_wizfi360/wiz360_uart_wifi_module.h"

#include "SAAL_packet_process.h"

void process_SAAL_packet_AAT(LAPEvt_msgt LAP_evt_msg)
{
	uint8_t report_device_id[4];

	uint8_t* temp_paar_id = BLE_cell_management_search_paar_id_by_connhandle(LAP_evt_msg.conn_handle);
	if(temp_paar_id != NULL)
	{
		//swap ID 4Bytes
		report_device_id[3] = temp_paar_id[0];
		report_device_id[2] = temp_paar_id[1];
		report_device_id[1] = temp_paar_id[2];
		report_device_id[0] = temp_paar_id[3];
	}
	else
	{
		memset(report_device_id, 0xFF, PAAR_ID_SIZE);
	}

	uint8_t* mqtt_msg = NULL;

	uint8_t service_id = LAP_evt_msg.msg[PAAR_PACKET_INDEX_SERVICE_ID];
	uint8_t body_data_len = LAP_evt_msg.msg[PAAR_PACKET_INDEX_DATA_LEN];
	uint8_t location_data_len = LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_CMD+body_data_len];

	mqtt_msg = malloc(PAAR_ID_SIZE + PAAR_SERVICE_ID_SIZE + body_data_len + location_data_len +3);
	if(mqtt_msg != NULL)
	{
		memset(mqtt_msg, 0, sizeof(mqtt_msg));

		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_PAAR_ID], report_device_id, PAAR_ID_SIZE);

		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_SERVCIE_ID], &service_id, PAAR_SERVICE_ID_SIZE);

		mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA_LEN] = body_data_len;

		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA], &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_CMD], body_data_len);

		mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA + body_data_len] = location_data_len;
		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA + body_data_len+PAAR_MQTT_SIZE_LOCATION_LENGTH], &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY + body_data_len], location_data_len);

		//start_mqtt_send_timeout();
		wifi_processing_event_send(WIFI_PROCESSING_EVENT_SEND_MQTT, 0, mqtt_msg);
	}
	else
	{
		printf("malloc_fail!!\r\n");
		bsp_abortsystem();
	}
}

void process_SAAL_packet_HAT(LAPEvt_msgt LAP_evt_msg)
{
	printf("process_SAAL_packet\r\n");

	uint8_t report_device_id[4];

	uint8_t* temp_paar_id = BLE_cell_management_search_paar_id_by_connhandle(LAP_evt_msg.conn_handle);
	if(temp_paar_id != NULL)
	{
		//swap ID 4Bytes
		report_device_id[3] = temp_paar_id[0];
		report_device_id[2] = temp_paar_id[1];
		report_device_id[1] = temp_paar_id[2];
		report_device_id[0] = temp_paar_id[3];
	}
	else
	{
		memset(report_device_id, 0xFF, PAAR_ID_SIZE);
	}

	uint8_t* mqtt_msg = NULL;

	uint8_t service_id = LAP_evt_msg.msg[PAAR_PACKET_INDEX_SERVICE_ID];
	uint8_t body_data_len = LAP_evt_msg.msg[PAAR_PACKET_INDEX_DATA_LEN];
	uint8_t location_data_len = LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_CMD+body_data_len];

	mqtt_msg = malloc(PAAR_ID_SIZE + PAAR_SERVICE_ID_SIZE + body_data_len + location_data_len +3);
	if(mqtt_msg != NULL)
	{
		memset(mqtt_msg, 0, sizeof(mqtt_msg));

		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_PAAR_ID], report_device_id, PAAR_ID_SIZE);

		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_SERVCIE_ID], &service_id, PAAR_SERVICE_ID_SIZE);

		mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA_LEN] = body_data_len;

		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA], &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_CMD], body_data_len);

		mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA + body_data_len] = location_data_len;
		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA + body_data_len+PAAR_MQTT_SIZE_LOCATION_LENGTH], &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY + body_data_len], location_data_len);

		printf("send mqtt packet to task \r\n");
		int r;
		//start_mqtt_send_timeout();
		r = wifi_processing_event_send(WIFI_PROCESSING_EVENT_SEND_MQTT, 0, mqtt_msg);
		if(r != 0)
		{
			printf("error : wifi_processing_event_send \r\n");
		}
	}
	else
	{
		printf("malloc_fail!!\r\n");
		bsp_abortsystem();
	}
}

void process_SAAL_packet_SPH(LAPEvt_msgt LAP_evt_msg)
{
	printf("process_SAAL_ENV_packet\r\n");

	uint8_t report_device_id[4];

	uint8_t* temp_paar_id = BLE_cell_management_search_paar_id_by_connhandle(LAP_evt_msg.conn_handle);
	if(temp_paar_id != NULL)
	{
		//swap ID 4Bytes
		report_device_id[3] = temp_paar_id[0];
		report_device_id[2] = temp_paar_id[1];
		report_device_id[1] = temp_paar_id[2];
		report_device_id[0] = temp_paar_id[3];
	}
	else
	{
		memset(report_device_id, 0xFF, PAAR_ID_SIZE);
	}

	uint8_t* mqtt_msg = NULL;

	uint8_t service_id = LAP_evt_msg.msg[PAAR_PACKET_INDEX_SERVICE_ID];
	uint8_t body_data_len = LAP_evt_msg.msg[PAAR_PACKET_INDEX_DATA_LEN];
	uint8_t location_data_len = LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_CMD+body_data_len];

	mqtt_msg = malloc(PAAR_ID_SIZE + PAAR_SERVICE_ID_SIZE + body_data_len + location_data_len +3);
	if(mqtt_msg != NULL)
	{
		memset(mqtt_msg, 0, sizeof(mqtt_msg));

		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_PAAR_ID], report_device_id, PAAR_ID_SIZE);

		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_SERVCIE_ID], &service_id, PAAR_SERVICE_ID_SIZE);

		mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA_LEN] = body_data_len;

		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA], &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY], body_data_len);

		mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA + body_data_len] = location_data_len;
		memcpy(&mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA + body_data_len+1], &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY + body_data_len], location_data_len);

		//start_mqtt_send_timeout();
		printf("send mqtt packet to task \r\n");
		wifi_processing_event_send(WIFI_PROCESSING_EVENT_SEND_MQTT_ENV, 0, mqtt_msg);
	}
	else
	{
		printf("malloc_error\r\n");
		bsp_abortsystem();
	}
}

void process_SAAL_packet_SmartFarm(LAPEvt_msgt LAP_evt_msg)
{

}

void process_SAAL_packet_ENV(LAPEvt_msgt LAP_evt_msg)
{

}

void process_SAAL_packet_Footpad(LAPEvt_msgt LAP_evt_msg)
{
	//do nothing...
}

void process_SAAL_packet_Device_Setup(LAPEvt_msgt LAP_evt_msg)
{

}