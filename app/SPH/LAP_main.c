/*
 * LAP_main.c
 *
 *  Created on: 2020. 06. 11.
 *      Author: YJPark
 */


#include "ble_cell_management.h"
#include "ble_process.h"
#include "ble_profile.h"
#include "ble_stack.h"
#include "LAP_api.h"
#include "LAP_main.h"
#include <ubinos.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <malloc.h>

#include "ble_gap.h"
#include "sw_config.h"
#include "hw_config.h"

#include "SAAL_packet_process.h"

#include "wiz360_uart_wifi_module.h"
//#include "../lib_wifi_wizfi360/wiz360_uart_wifi_module.h"

uint16_t cccd_target_connhandle = BLE_CONN_HANDLE_INVALID;

static msgq_pt LAP_msgq;

#define SCAN_FAIL_TIMEOUT_DELAY		120000

APP_TIMER_DEF(scan_fail_timeout_timer);
APP_TIMER_DEF(scan_timeout_timer);
APP_TIMER_DEF(mqtt_send_timeout_timer);

#define MQTT_RESPONSE_TIMEOUT		600000
bool MQTT_res_timer_flag = false;

#define DEFAULT_BLE_SCAN_DELAY_MAX_COUNT	10

uint8_t ble_scan_delay_count = 1;

void mqtt_res_timeout_handler()
{
	bsp_abortsystem();
}

void start_mqtt_send_timeout()
{
	if(MQTT_res_timer_flag == false)
	{
		app_timer_start(mqtt_send_timeout_timer, MQTT_RESPONSE_TIMEOUT, NULL);
		MQTT_res_timer_flag = true;
	}
}

void stop_mqtt_send_timeout()
{
	app_timer_stop(mqtt_send_timeout_timer);
	MQTT_res_timer_flag = false;
}

static scan_target_paar_id_st ble_test_target_paar_id[TEST_MAX_CONNECTION_DEVICE];

#if(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_PERIPHERAL || SP_SW_MODE_SETUP == SP_SW_MODE_TEST_CENTRAL )
uint32_t target_paarid[TEST_MAX_CONNECTION_DEVICE] =
{
		0x020000E0,
};
#elif(SP_SW_MODE_SETUP == SP_SW_MODE_SPH)

#if(SP_TEST_LOCATION == SP_TEST_LOCATION_LIVINGROOM)
uint32_t target_paarid[TEST_MAX_CONNECTION_DEVICE] =
{
		0xFF0101E0,
		0xFF0102E0,
		0xFF0103E0,
		0xFF0101EC,
		0xFF0102EC,
		0xFF0103EC
};

#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_TOILET)
uint32_t target_paarid[TEST_MAX_CONNECTION_DEVICE] =
{
		0xFF0201E0,
		0xFF0202E0,
		0xFF0201EC,
		0xFF0202EC,
		0xFF0203EC
};

#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_ROOM)
uint32_t target_paarid[TEST_MAX_CONNECTION_DEVICE] =
{
		0xFF0301E0,
		0xFF0302E0,
};

#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_GP1309)
uint32_t target_paarid[TEST_MAX_CONNECTION_DEVICE] =
{
		0xFF0401E0,
		0xFF0402E0,
		0xFF0403E0,
		0xFF0401EC,
		0xFF0402EC,
		0xFF0403EC,
};

#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_GP1313)
uint32_t target_paarid[TEST_MAX_CONNECTION_DEVICE] =
{
		0xFF0501E0,
		0xFF0502E0,
		0xFF0501EC,
		0xFF0502EC,
		0xFF0503EC,
};
#endif
/*
uint32_t target_paarid[TEST_MAX_CONNECTION_DEVICE] =
{
		0x090001E0,
		0x030001D8,
};
*/
#elif(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_MONITORING)
uint32_t target_paarid[TEST_MAX_CONNECTION_DEVICE] =
{
		0xFFFF01E0,
		0xFFFF02E0,
		0xFFFF03E0,
		0xFFFF04E0,
		0xFFFF05E0,
		0xFFFF06E0,
		0xFFFF07E0,
		0xFFFF08E0
};
#endif

void set_scan_target_paar_id_test()
{
	ble_test_target_paar_id[0].scan_target_paar_id_enable = true;

	uint8_t i;
	for(i = 0; i<TEST_MAX_CONNECTION_DEVICE; i++)
	{
		ble_test_target_paar_id[i].scan_target_paar_id[0] = target_paarid[i]>>24;
		ble_test_target_paar_id[i].scan_target_paar_id[1] = target_paarid[i]>>16;
		ble_test_target_paar_id[i].scan_target_paar_id[2] = target_paarid[i]>>8;
		ble_test_target_paar_id[i].scan_target_paar_id[3] = target_paarid[i];
	}
}

bool is_test_target_device(LAP_ble_adv_report* pPkt)
{
	if(pPkt->data_len < LAP_ADV_DATA_LEN)
		return false;
#if(ENABLE_BLE_TARGET_BASE_PAAR_ID == 1)
	
	//PAAR ID 0 / PAAR ID 1 기반으로 필터링... 관련 Location Device를 연결함...
	if(pPkt->data[LAP_ADV_IDX_PAAR_DEVICE_ID0] == PAAR_ID_0 && pPkt->data[LAP_ADV_IDX_PAAR_DEVICE_ID1] == PAAR_ID_1)
		return true;

#else

	uint8_t i;
	for(i = 0; i<TEST_MAX_CONNECTION_DEVICE; i++)
	{
		if( memcmp(&ble_test_target_paar_id[i].scan_target_paar_id[0],&pPkt->data[LAP_ADV_IDX_PAAR_DEVICE_ID0], 4) == 0)
		{
			return true;
		}
	}
	
#endif

	return false;
}

void LAP_process_ble_adv_report(LAP_ble_adv_report* pPkt)
{

}

static void process_LAP_scan_timeout()
{
	uint8_t r;
	r = processing_BLE_cell_managemnet();
	printf("BLE Cell Management\r\n");

	app_timer_stop(scan_fail_timeout_timer);
	
	if(r == 0xFF)
	{
		ble_scan_delay_count = 1;
		printf("BLE event : Connecting...\r\n");
	}
	else
	{
		if(get_BLE_cell_management_data_count() < TEST_MAX_CONNECTION_DEVICE)
		{
			//task_sleep(1000);
			//LAP_start_ble_scan(NULL);
			ble_scan_delay_count++;
			if(ble_scan_delay_count >= DEFAULT_BLE_SCAN_DELAY_MAX_COUNT)
				ble_scan_delay_count = DEFAULT_BLE_SCAN_DELAY_MAX_COUNT;
			app_timer_start(scan_timeout_timer, APP_TIMER_TICKS(ble_scan_delay_count*1000), NULL);
		}	
	}

	//app_timer_stop(scan_fail_timeout_timer);
	//app_timer_start(scan_fail_timeout_timer, APP_TIMER_TICKS(5000), NULL);
}

static void processing_LAP_Central_Conn_timeout(LAPEvt_msgt LAP_evt_msg)
{

	printf("LAP Connect timeout \r\n");
	BLE_cell_management_connect_timeout();

	task_sleep(1000);
	
	//LAP_start_ble_scan(NULL);

	app_timer_start(scan_timeout_timer, APP_TIMER_TICKS(1000), NULL);

	//app_timer_stop(scan_fail_timeout_timer);
	//app_timer_start(scan_fail_timeout_timer, APP_TIMER_TICKS(5000), NULL);
}

static void processing_LAP_Central_Scan_timeout(LAPEvt_msgt LAP_evt_msg)
{
	while(get_adv_buffer_count() != 0)
	{
		task_sleep(10);
	}
	process_LAP_scan_timeout();
}

static void processing_LAP_Central_Scan_result(LAPEvt_msgt LAP_evt_msg)
{
	while(get_adv_buffer_count() != 0)
	{
		task_sleep(10);
	}
	process_LAP_scan_timeout();
}

#if(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_CENTRAL)
uint8_t test_send_count = 0;

static void send_test_msg_central(uint16_t conn_handle, uint16_t handle)
{
	uint8_t temp_packet[PAAR_MAXIMUM_PACKET_SIZE] = {0, };

	memset(temp_packet, 0, PAAR_MAXIMUM_PACKET_SIZE);

	temp_packet[0] = test_send_count++;
	if(test_send_count >= 4)
		test_send_count = 1;


	printf("BLE send msg : test_msg %d\r\n", temp_packet[0]);

	LAP_send_ble_msg_central(conn_handle, handle, temp_packet, PAAR_MAXIMUM_PACKET_SIZE);
}
#endif

#if(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_PERIPHERAL)
uint8_t test_send_count = 0;

static void send_test_msg_peripheral()
{
	uint8_t temp_packet[PAAR_MAXIMUM_PACKET_SIZE] = {0, };

	memset(temp_packet, 0, PAAR_MAXIMUM_PACKET_SIZE);
	temp_packet[0] = test_send_count++;
	if(test_send_count >= 4)
		test_send_count = 1;

	printf("BLE send msg : test_msg %d\r\n",temp_packet[0]);

	LAP_send_ble_msg_peripheral(temp_packet, PAAR_MAXIMUM_PACKET_SIZE);
}
#endif

static void send_cccd_handle_enable(uint16_t conn_handle, uint16_t cccd_handle)
{
	uint8_t temp_packet[2];

	temp_packet[0] = (uint8_t)NRF_NOTI_INDI_ENABLE;		// ble notification msg 데이터
	temp_packet[1] = 0x00;

	printf("BLE send msg : CCCD enable\r\n");

	LAP_send_ble_msg_central(conn_handle, cccd_handle, (uint8_t*)temp_packet, 2);

}

static void processing_LAP_Central_Connected(LAPEvt_msgt LAP_evt_msg)
{

	printf("BLE Central connect\r\n");

	uuidhandle temp_uuid_handle;

	int r;
	r = BLE_cell_management_current_cccd_handle(&temp_uuid_handle);
	if(r == -1)
	{
		return;
	}

	//save test_connection_handle
	cccd_target_connhandle = LAP_evt_msg.conn_handle;

	task_sleep(500);

	//send cccd enable
	send_cccd_handle_enable(cccd_target_connhandle, temp_uuid_handle.cccd_handle);

	task_sleep(500);

	send_cccd_handle_enable(cccd_target_connhandle, temp_uuid_handle.cccd_handle);

	task_sleep(200);

	BLE_cell_management_check_connection(LAP_evt_msg.conn_handle);

	increase_conn_count();

	task_sleep(200);

	//LAP_start_ble_scan(NULL);

	app_timer_start(scan_timeout_timer, APP_TIMER_TICKS(1000), NULL);

}

static void processing_LAP_Central_Disconnected(LAPEvt_msgt LAP_evt_msg)
{
	uint8_t index;
	index = BLE_cell_management_search_data_index_by_connhandle(LAP_evt_msg.conn_handle);
	if(index != 0xFF)
	{
		BLE_cell_management_data_delete(index);

		decrease_conn_count();
	}

	printf("BLE Disconnected.\r\n");

	task_sleep(200);

	//LAP_start_ble_scan(NULL);
	ble_scan_delay_count = 1;
	app_timer_start(scan_timeout_timer, APP_TIMER_TICKS(ble_scan_delay_count*1000), NULL);
}

static void processing_LAP_Central_Data_Received(LAPEvt_msgt LAP_evt_msg)
{
	//heap_printheapinfo(NULL);

//	task_sleep(100);
#if(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_CENTRAL)
	printf("BLE receive msg : test_msg  : %d\r\n", LAP_evt_msg.msg[0]);

	uuidhandle profile_data;

	int result;

	result = get_BLE_cell_management_profile_data_by_connhandle(LAP_evt_msg.conn_handle, &profile_data);

	if(result != 0)
		return;

	task_sleep(1000);

	//test_code

	send_test_msg_central(LAP_evt_msg.conn_handle, profile_data.rx_handle);
#elif(SP_SW_MODE_SETUP == SP_SW_MODE_SPH||SP_SW_MODE_SETUP == SP_SW_MODE_TEST_MONITORING )
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

	uint8_t service_id = LAP_evt_msg.msg[PAAR_PACKET_INDEX_SERVICE_ID];
	uint8_t body_data_len = LAP_evt_msg.msg[PAAR_PACKET_INDEX_DATA_LEN];

	printf("0x%02X %02X %02X %02X ADL packet_received!!\r\n", report_device_id[3], report_device_id[2], report_device_id[1], report_device_id[0]);
	
	//SAAL Packet Process
	switch(LAP_evt_msg.msg[PAAR_PACKET_INDEX_SERVICE_ID])
	{
		case SAAL_SERVICE_ID_AAT :
			process_SAAL_packet_AAT(LAP_evt_msg);
			break;
		case SAAL_SERVICE_ID_HAT :
			process_SAAL_packet_HAT(LAP_evt_msg);
			break;
		case SAAL_SERVICE_ID_SPH :
			process_SAAL_packet_SPH(LAP_evt_msg);
			break;
		case SAAL_SERVICE_ID_SMARTFARM :
			process_SAAL_packet_SmartFarm(LAP_evt_msg);
			break;
		case SAAL_SERVICE_ID_FOOTPAD :
			process_SAAL_packet_Footpad(LAP_evt_msg);
			break;
		case SAAL_SERVICE_ID_DEVICE_SETUP :
			process_SAAL_packet_Device_Setup(LAP_evt_msg);
			break;
		default :
			printf("Service ID error!!?\r\n");
			heap_printheapinfo(NULL);
			break;
	}


/*

	if(LAP_evt_msg.msg[PAAR_PACKET_INDEX_SERVICE_ID] == 0x18)
	{
#if(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_MONITORING)
		printf("0x%02X %02X %02X %02X ADL packet_received!!\r\n", report_device_id[3], report_device_id[2], report_device_id[1], report_device_id[0]);
		uint8_t temp_char[30];
		memset(temp_char, 0, 30);
		memcpy(temp_char, &LAP_evt_msg.msg[9],LAP_evt_msg.msg[8]);

		printf("%s", temp_char);
#else

		printf("0x%02X %02X %02X %02X ADL packet_received!!\r\n", report_device_id[3], report_device_id[2], report_device_id[1], report_device_id[0]);
		uint8_t* mqtt_msg = NULL;

		mqtt_msg = malloc(PAAR_ID_SIZE + PAAR_SERVICE_ID_SIZE + body_data_len+2);
		if(mqtt_msg != NULL)
		{
			memset(mqtt_msg, 0, sizeof(mqtt_msg));

			memcpy(&mqtt_msg[PAAR_MQTT_INDEX_PAAR_ID], report_device_id, PAAR_ID_SIZE);

			memcpy(&mqtt_msg[PAAR_MQTT_INDEX_SERVCIE_ID], &service_id, PAAR_SERVICE_ID_SIZE);

			mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA_LEN] = body_data_len;

			memcpy(&mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA], &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY], body_data_len);

			start_mqtt_send_timeout();
			wifi_processing_event_send(WIFI_PROCESSING_EVENT_SEND_MQTT_ENV, 0, mqtt_msg);
		}
		else
		{
			printf("malloc_error\r\n");
			bsp_abortsystem();
		}
#endif
	}
	else if(LAP_evt_msg.msg[PAAR_PACKET_INDEX_SERVICE_ID] == 0xFE)
	{
		if(LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_CMD] == 0x07)
		{
		int16_t pt_acc_x, pt_acc_y, pt_acc_z, pt_gyro_x, pt_gyro_y, pt_gyro_z;

		memcpy(&pt_acc_x, &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY], 2);
		memcpy(&pt_acc_y, &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY+2], 2);
		memcpy(&pt_acc_z, &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY+4], 2);
		memcpy(&pt_gyro_x, &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY+6], 2);
		memcpy(&pt_gyro_y, &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY+8], 2);
		memcpy(&pt_gyro_z, &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY+10], 2);

		//printf("TEST ACC Data Streaming : \r\n");
		printf("TEST ACC X/Y/Z & GYRO X/Y/Z= %d %d %d  /  %d %d %d", pt_acc_x, pt_acc_y, pt_acc_z, pt_gyro_x, pt_gyro_y, pt_gyro_z);
		printf("\r\n");
		}
		else if(LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_CMD] == 0x08)
		{
		int16_t pt_saadc;

		memcpy(&pt_saadc, &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY], 2);

		//printf("TEST ACC Data Streaming : \r\n");
		printf("TEST SAADC= %d", pt_saadc);
		printf("\r\n");
		}
		else if(LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_CMD] == 0x09)
		{
			uint8_t test_gyro;

			memcpy(&test_gyro, &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_BODY], 2);

			printf("TEST GYRO VARIANCE = %d", test_gyro);
			printf("\r\n");
		}
	}
	else
	{
#if(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_MONITORING)
		printf("0x%02X %02X %02X %02X ADL packet_received!!\r\n", report_device_id[3], report_device_id[2], report_device_id[1], report_device_id[0]);

		uint8_t temp_char[30];
		memset(temp_char, 0, 30);
		memcpy(temp_char, &LAP_evt_msg.msg[9],LAP_evt_msg.msg[8]);

		printf("%s\r\n", temp_char);
#else
		printf("0x%02X %02X %02X %02X ADL packet_received!!\r\n", report_device_id[3], report_device_id[2], report_device_id[1], report_device_id[0]);
		uint8_t* mqtt_msg = NULL;
		
		mqtt_msg = malloc(PAAR_ID_SIZE + PAAR_SERVICE_ID_SIZE + body_data_len+2);
		if(mqtt_msg != NULL)
		{
			memset(mqtt_msg, 0, sizeof(mqtt_msg));

			memcpy(&mqtt_msg[PAAR_MQTT_INDEX_PAAR_ID], report_device_id, PAAR_ID_SIZE);

			memcpy(&mqtt_msg[PAAR_MQTT_INDEX_SERVCIE_ID], &service_id, PAAR_SERVICE_ID_SIZE);

			mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA_LEN] = body_data_len;
			memcpy(&mqtt_msg[PAAR_MQTT_INDEX_BODY_DATA], &LAP_evt_msg.msg[PAAR_PACKET_INDEX_BODY_DATA_CMD], body_data_len);

			start_mqtt_send_timeout();
			wifi_processing_event_send(WIFI_PROCESSING_EVENT_SEND_MQTT, 0, mqtt_msg);
		}
		else
		{
			printf("malloc_fail!!\r\n");
			bsp_abortsystem();
		}
#endif
	}

	*/

#endif
}

static void processing_LAP_Peripheral_Connected(LAPEvt_msgt LAP_evt_msg)
{
	printf("BLE Peripheral connect\r\n");
}

static void processing_LAP_Peripheral_Disconnected(LAPEvt_msgt LAP_evt_msg)
{
	task_sleep(TEST_ADV_START_DELAY);
	printf("BLE Peripheral disconnect\r\n");

	printf("BLE ADV start\r\n");
	LAP_start_ble_adv_LIDx();
}

static void processing_LAP_Peripheral_Data_Received(LAPEvt_msgt LAP_evt_msg)
{
#if(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_PERIPHERAL)
	printf("BLE receive msg : test_msg  : %d\r\n", LAP_evt_msg.msg[0]);
	task_sleep(1000);
	send_test_msg_peripheral();
#endif
}

static void processing_LAP_Peripheral_CCCD_Enabled(LAPEvt_msgt LAP_evt_msg)
{
	printf("BLE CCCD is enabled. \r\n");

#if(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_PERIPHERAL)
	task_sleep(2000);
	send_test_msg_peripheral();
#endif
}

static void processing_LAP_Central_event(LAPEvt_msgt LAP_evt_msg)
{
	switch(LAP_evt_msg.status)
	{
	case LAP_CENTRAL_ST_SCAN_TIMEOUT :
		processing_LAP_Central_Scan_timeout(LAP_evt_msg);
		break;	
	case LAP_CENTRAL_ST_CONN_TIMEOUT :
		processing_LAP_Central_Conn_timeout(LAP_evt_msg);
		break;
	case LAP_CENTRAL_ST_SCAN_RESULT :
		processing_LAP_Central_Scan_result(LAP_evt_msg);
		break;
	case LAP_CENTRAL_ST_CONNECTED :
		processing_LAP_Central_Connected(LAP_evt_msg);
		break;
	case LAP_CENTRAL_ST_DISCONNECTED :
		processing_LAP_Central_Disconnected(LAP_evt_msg);
		break;
	case LAP_CENTRAL_ST_DATA_RECEIVED :
		processing_LAP_Central_Data_Received(LAP_evt_msg);
		break;
	}
}

static void processing_LAP_Peripheral_event(LAPEvt_msgt LAP_evt_msg)
{
	switch(LAP_evt_msg.status)
	{
	case LAP_PERIPHERAL_ST_CONNECTED :
		processing_LAP_Peripheral_Connected(LAP_evt_msg);
		break;
	case LAP_PERIPHERAL_ST_DISCONNECTED :
		processing_LAP_Peripheral_Disconnected(LAP_evt_msg);
		break;
	case LAP_PERIPHERAL_ST_DATA_RECEIVED :
		processing_LAP_Peripheral_Data_Received(LAP_evt_msg);
		break;
	case LAP_PERIPHERAL_ST_CCCD_ENABLED :
		processing_LAP_Peripheral_CCCD_Enabled(LAP_evt_msg);
		break;
	}
}

void processing_LAP_LIDx_event(LAPEvt_msgt LAP_evt_msg)
{

}

void processing_LAP_PNIP_event(LAPEvt_msgt LAP_evt_msg)
{

}

void processing_LAP_AMD_event(LAPEvt_msgt LAP_evt_msg)
{

}

//Process Other BLE Events
//**interrupt에 의해 호출되는 handler이므로 malloc을 사용할수 없음
void process_ADV_Report(LAP_ble_adv_report* pPkt)
{
	//Scan 전 연결 대상을 지정했을 경우, 해당 대상(paar id) 확인 시, Scan 종료 후 LAP Task로 정보 전달(LAP_CENTRAL_ST_SCAN_RESULT)
	/*
	if(ble_target_paar_id.scan_target_paar_id_enable == true)
	{
		if(is_Target_adv_packet(pPkt) == false)
			return;

		memcpy(&scan_target_result, pPkt, sizeof(LAP_ble_adv_report));

		LAP_event_send(LAP_CENTRAL_EVT, LAP_CENTRAL_ST_SCAN_RESULT, BLE_CONN_HANDLE_INVALID,
												LAP_EVENT_HANDLE_NULL, LAP_EVENT_MSG_LEN_NULL, NULL);

		PAAR_scan_stop();
	}
	else
	{
	}
		*/

	//test_code : 연결할 Test 타겟 검사
	if(is_test_target_device(pPkt) == false)
		return;

	//연결대상 없이 Advertising Packet 처리
	//checking PAAR ADV data...
	if(is_LAP_adv_packet(pPkt) == true)
	{
		// Do nothing...
		//모든 LAP ADV data는 Slim Hub로 전송
//			FE_send_adv_report(pPkt);

		//process LAP status byte
		switch(pPkt->data[LAP_ADV_IDX_STATUS])
		{
		case LAP_ADV_STATUS_BYTE_LIDX :
			break;
		case LAP_ADV_STATUS_BYTE_PNIP :
			break;
		case LAP_ADV_STATUS_BYTE_AMD :
			break;
		case LAP_ADV_STATUS_BYTE_EC :
			break;
		case LAP_ADV_STATUS_BYTE_INOUT :
			break;
			//Location Request는 저장 후, Scan Result(timeout evt)이후 처리
		case LAP_ADV_STATUS_BYTE_LOCATION_REQ :
				process_LAP_location_request_packet(pPkt);
			break;
		case LAP_ADV_STATUS_BYTE_SLIMHUB_INFO :
			break;
		case LAP_ADV_STATUS_BYTE_CONN_SOSP_WO_MAC :
			break;
		}
	}
}

void LAP_Protocol_start_operation()
{
	set_adv_callback_func(process_ADV_Report);

#if(SP_SW_MODE_SETUP == SP_SW_MODE_SPH)
	set_scan_target_paar_id_test();

	task_sleep(1000);
	/*while(get_wifi_ready() == false)
	{
		task_sleep(100);
	}*/

	task_sleep(100);

	//LAP_start_ble_scan(NULL);
	ble_scan_delay_count = 1;
	app_timer_start(scan_timeout_timer, APP_TIMER_TICKS(ble_scan_delay_count*1000), NULL);
#elif(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_MONITORING)
	set_scan_target_paar_id_test();

	task_sleep(100);

	//LAP_start_ble_scan(NULL);
	app_timer_start(scan_timeout_timer, APP_TIMER_TICKS(1000), NULL);

#elif(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_CENTRAL)
	set_scan_target_paar_id_test();

//	while(get_wifi_ready() == false)
//	{
//		task_sleep(100);
//	}

	task_sleep(100);

	//LAP_start_ble_scan(NULL);
	app_timer_start(scan_timeout_timer, APP_TIMER_TICKS(1000), NULL);

#elif(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_PERIPHERAL)
	printf("BLE ADV start\r\n");
	task_sleep(TEST_ADV_START_DELAY);
	LAP_start_ble_adv_LIDx();
#endif
}

void scan_fail_timer_handler()
{
	LAP_start_ble_scan(NULL);

//	app_timer_start(scan_fail_timeout_timer, APP_TIMER_TICKS(5000), NULL);
}



void scan_timeout_process()
{
	app_timer_stop(scan_fail_timeout_timer);
	app_timer_start(scan_fail_timeout_timer, APP_TIMER_TICKS(SCAN_FAIL_TIMEOUT_DELAY), NULL);
	LAP_start_ble_scan(NULL);
}

void scan_fail_timeout_process()
{
	printf("scan_timeout!! bsp reset.\r\n");
	bsp_abortsystem();
}

void LAP_main_task(void* arg){
	int r;
	LAPEvt_msgt LAP_evt_msg;

	ble_stack_init_wait();

	BLE_cell_management_data_init();

	//set_adv_callback_func(LAP_process_ble_adv_report);

	app_timer_create(&scan_timeout_timer, APP_TIMER_MODE_SINGLE_SHOT, scan_timeout_process);
	app_timer_create(&scan_fail_timeout_timer, APP_TIMER_MODE_SINGLE_SHOT, scan_fail_timeout_process);
	app_timer_create(&mqtt_send_timeout_timer, APP_TIMER_MODE_SINGLE_SHOT, mqtt_res_timeout_handler);

	LAP_Protocol_start_operation();

	for (;;) {
		r = msgq_receive(LAP_msgq, (unsigned char*) &LAP_evt_msg);
		if (0 != r) {
			logme("fail at msgq_receive\r\n");
		} else {
			switch( LAP_evt_msg.event ){
			case LAP_CENTRAL_EVT :
				processing_LAP_Central_event(LAP_evt_msg);
				break;
			case LAP_PERIPHERAL_EVT :
				processing_LAP_Peripheral_event(LAP_evt_msg);
				break;
				
			case LAP_LIDx_EVT :
				processing_LAP_LIDx_event(LAP_evt_msg);
				break;
			case LAP_PNIP_EVT :
				processing_LAP_PNIP_event(LAP_evt_msg);
				break;
			case LAP_AMD_EVT :
				processing_LAP_AMD_event(LAP_evt_msg);
				break;
			}

			if( LAP_evt_msg.msg != NULL ){
				free(LAP_evt_msg.msg);
			}
		}
	}
}

void LAP_main_task_init(void){
	int r;

	r = msgq_create(&LAP_msgq, sizeof(LAPEvt_msgt), 20);
	if (0 != r) {
		printf("fail at msgq create\r\n");
	}

	r = task_create(NULL, LAP_main_task, NULL, task_gethighestpriority()-2, 1024, NULL);
	if (r != 0) {
		printf("== LAP_main_task failed \n\r");
	} else {
		printf("== LAP_main_task created \n\r");
	}
}


int LAP_event_send(uint8_t evt, uint8_t state, uint16_t conn_handle, uint16_t handle,
															uint32_t msg_len, uint8_t* msg)
{
	LAPEvt_msgt lap_msg;

	lap_msg.event = evt;
	lap_msg.status = state;
	lap_msg.handle = handle;
	lap_msg.conn_handle = conn_handle;
	lap_msg.msg_len = msg_len;
	lap_msg.msg = msg;

	int r;
	r = msgq_send(LAP_msgq, (unsigned char*) &lap_msg);
	if(r != 0)
	{
		printf("LAP event send error : %d\r\n", r);
	}

	return r;
}
