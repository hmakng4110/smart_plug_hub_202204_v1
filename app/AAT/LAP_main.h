/*
 * LAP_main.h
 *
 *  Created on: 2020. 06. 11.
 *      Author: YJPark
 */

#ifndef APPLICATION_LIB_BLUETOOTH_CSOS_LAP_MAIN_H_
#define APPLICATION_LIB_BLUETOOTH_CSOS_LAP_MAIN_H_

typedef struct
{
	uint8_t event;
	uint8_t status;
	uint16_t conn_handle;
	uint16_t handle;
	uint32_t msg_len;
	uint8_t* msg;
}LAPEvt_msgt;

enum
{
	LAP_CENTRAL_EVT = 0,
	LAP_PERIPHERAL_EVT,
};

enum{
	LAP_CENTRAL_ST_SCAN_RESULT = 0,
	LAP_CENTRAL_ST_SCAN_TIMEOUT,
	LAP_CENTRAL_ST_CONN_TIMEOUT,
	LAP_CENTRAL_ST_SCAN_ADV_REPORT,
	LAP_CENTRAL_ST_CONNECTED,
	LAP_CENTRAL_ST_DISCONNECTED,
	LAP_CENTRAL_ST_DATA_RECEIVED,
};

enum
{
	LAP_PERIPHERAL_ST_CONNECTED = 0,
	LAP_PERIPHERAL_ST_DISCONNECTED,
	LAP_PERIPHERAL_ST_DATA_RECEIVED,
	LAP_PERIPHERAL_ST_CCCD_ENABLED,
};

void LAP_main_task_init(void);

int LAP_event_send(uint8_t evt, uint8_t state, uint16_t conn_handle, uint16_t handle,
									uint32_t msg_len, uint8_t* msg);

bool get_ble_peripheral_connected_flag();

#endif /* APPLICATION_LIB_BLUETOOTH_CSOS_LAP_MAIN_H_ */
