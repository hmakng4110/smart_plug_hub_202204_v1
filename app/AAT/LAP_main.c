/*
 * LAP_main.c
 *
 *  Created on: 2020. 06. 11.
 *      Author: YJPark
 */


#include <ubinos.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <malloc.h>


#include "ble_process.h"
#include "ble_profile.h"
#include "ble_stack.h"
#include "LAP_api.h"
#include "LAP_main.h"
#include "ble_cell_management.h"

#include "ble_gap.h"
#include "sw_config.h"
#include "hw_config.h"

static msgq_pt LAP_msgq;

static bool is_ble_peripheral_connected = false;

void set_ble_peripheral_connected_flag(bool val)
{
	is_ble_peripheral_connected = val;

	#if(BLE_DEBUGGING_LED_ENABLE == 1)
		if(val == true)
		{
			AAT_LED_ON(BLUE);
		}
		else
		{
			AAT_LED_OFF(WHITE);
		}
	#endif

}

static void send_cccd_handle_enable(uint16_t conn_handle, uint16_t cccd_handle)
{
	uint8_t temp_packet[2];

	temp_packet[0] = (uint8_t)NRF_NOTI_INDI_ENABLE;		// ble notification msg 데이터
	temp_packet[1] = 0x00;

	printf("BLE send msg : CCCD enable\r\n");

	LAP_send_ble_msg_central(conn_handle, cccd_handle, (uint8_t*)temp_packet, 2);

}

bool get_ble_peripheral_connected_flag()
{
	return is_ble_peripheral_connected;
}

void LAP_process_ble_adv_report(LAP_ble_adv_report* pPkt)
{
	//Do nothing...
}

static void processing_LAP_Central_Conn_timeout(LAPEvt_msgt LAP_evt_msg)
{
	//Do nothing...
}

static void processing_LAP_Central_Scan_timeout(LAPEvt_msgt LAP_evt_msg)
{
	//Do nothing...
}

static void processing_LAP_Central_Scan_result(LAPEvt_msgt LAP_evt_msg)
{
	//Do nothing...
}

static void processing_LAP_Central_Connected(LAPEvt_msgt LAP_evt_msg)
{
	//Do nothing...	
}

static void processing_LAP_Central_Disconnected(LAPEvt_msgt LAP_evt_msg)
{
	//Do nothing...	
}

static void processing_LAP_Central_Data_Received(LAPEvt_msgt LAP_evt_msg)
{
	//Do nothing...	
}

static void processing_LAP_Peripheral_Connected(LAPEvt_msgt LAP_evt_msg)
{
	printf("BLE Peripheral connect\r\n");
	set_ble_peripheral_connected_flag(true);
}

static void processing_LAP_Peripheral_Disconnected(LAPEvt_msgt LAP_evt_msg)
{
	task_sleep(TEST_ADV_START_DELAY);
	printf("BLE Peripheral disconnect\r\n");

	set_ble_peripheral_connected_flag(false);

	task_sleep(1000);

	printf("BLE ADV start\r\n");
	LAP_start_ble_adv_LIDx();
}

static void processing_LAP_Peripheral_Data_Received(LAPEvt_msgt LAP_evt_msg)
{
	//Do nothing...
	printf("BLE Data is Received. \r\n");
}

static void processing_LAP_Peripheral_CCCD_Enabled(LAPEvt_msgt LAP_evt_msg)
{
	printf("BLE CCCD is enabled. \r\n");
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

void LAP_Protocol_start_operation()
{
	printf("BLE ADV start\r\n");
	task_sleep(TEST_ADV_START_DELAY);
	LAP_start_ble_adv_LIDx();
}

void LAP_main_task(void* arg){
	int r;
	LAPEvt_msgt LAP_evt_msg;

	ble_stack_init_wait();

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
