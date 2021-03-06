/*
 * sh_uart_wifi_module.h
 *
 *  Created on: 2021. 09. 23.
 *      Author: sayyo, YJPark
 */

#ifndef APP_SH_UART_WIFI_MODULE_SH_UART_WIFI_MODULE_H_
#define APP_SH_UART_WIFI_MODULE_SH_UART_WIFI_MODULE_H_

#include "ubinos.h"
#include <sw_config.h>

#include <stdint.h>
#include <stdbool.h>

#define UART_WIFI_READ_BUF_SIZE		2048
#define UART_WIFI_WRITE_BUF_SIZE	2048
#define UART_WIFI_CMD_BUF_SIZE		64
#define UART_WIFI_RSP_BUF_SIZE		250
#define UART_WIFI_RSP_BUF_MAX_COUNT	5

#define UART_WIFI_READ_DATA_BUF_SIZE	256

#define WIZFI_MODULE_PWR_ON_DELAY_MS	100

// AP List
#define CSOS_AP_SSID 	"AP_DEMO"
#define CSOS_AP_PW		"csos!1234"

#define RTLab_501_SSID	"rtlab501"
#define RTLab_502_SSID	"rtlab502"
#define RTLab_504_SSID	"rtlab503"
#define RTLab_AP_PW		"rtlab!123"

#define TEST_AP_SSID		"AP_DEMO"
#define TEST_AP_PASSWORD	"csos!1234"
//#define TEST_AP_SSID		"KT_GiGA_5C15"
//#define TEST_AP_PASSWORD	"2ad51ff042"

#define PROF_HOME_SSID 	"KT_GiGA_5C15"
#define PROF_HOME_PW	"2ad51ff042"


#define TEST_MQTT_ALIVE_TIME	300

//#define TEST_MQTT_TOPIC_PUBLISH		"CSOS/AB001309/040001D4/ADL_DATA"
//#define TEST_MQTT_TOPIC_SUBSCRIBE		"sensorID"

#define TEST_MQTT_BROKER_IP				"155.230.186.105"
#define TEST_MQTT_BROKER_PORT			"1883"

#define PAAR_ID_LEN			4

#define TEMP_ID_BUFFER_SIZE		30
//#define PAAR_MQTT_PACKET_LENGTH_LIMIT	300
#define PAAR_MQTT_PACKET_LENGTH_LIMIT	500

#define PAAR_MQTT_INDEX_PAAR_ID			0
#define PAAR_MQTT_INDEX_SERVCIE_ID		4
#define PAAR_MQTT_INDEX_BODY_DATA_LEN	5
#define PAAR_MQTT_INDEX_BODY_DATA		6

#define PAAR_MQTT_SIZE_BODY_LENGTH        1
#define PAAR_MQTT_SIZE_LOCATION_LENGTH    1

#define PAAR_MQTT_SIZE_ENV_PACKET		24

#define PAAR_MQTT_INDEX_ENV_DATA_PRESS_0            PAAR_MQTT_INDEX_BODY_DATA
#define PAAR_MQTT_INDEX_ENV_DATA_PRESS_1            PAAR_MQTT_INDEX_BODY_DATA+1
#define PAAR_MQTT_INDEX_ENV_DATA_PRESS_DECIMAL_0    PAAR_MQTT_INDEX_BODY_DATA+2
#define PAAR_MQTT_INDEX_ENV_DATA_PRESS_DECIMAL_1    PAAR_MQTT_INDEX_BODY_DATA+3
#define PAAR_MQTT_INDEX_ENV_DATA_TEMP_0             PAAR_MQTT_INDEX_BODY_DATA+4
#define PAAR_MQTT_INDEX_ENV_DATA_TEMP_1             PAAR_MQTT_INDEX_BODY_DATA+5
#define PAAR_MQTT_INDEX_ENV_DATA_TEMP_DECIMAL_0     PAAR_MQTT_INDEX_BODY_DATA+6
#define PAAR_MQTT_INDEX_ENV_DATA_TEMP_DECIMAL_1     PAAR_MQTT_INDEX_BODY_DATA+7
#define PAAR_MQTT_INDEX_ENV_DATA_HUMIDITY_0         PAAR_MQTT_INDEX_BODY_DATA+8
#define PAAR_MQTT_INDEX_ENV_DATA_HUMIDITY_1         PAAR_MQTT_INDEX_BODY_DATA+9
#define PAAR_MQTT_INDEX_ENV_DATA_GAS_ADC_0          PAAR_MQTT_INDEX_BODY_DATA+10
#define PAAR_MQTT_INDEX_ENV_DATA_GAS_ADC_1          PAAR_MQTT_INDEX_BODY_DATA+11
#define PAAR_MQTT_INDEX_ENV_DATA_GAS_EC02_0         PAAR_MQTT_INDEX_BODY_DATA+12
#define PAAR_MQTT_INDEX_ENV_DATA_GAS_EC02_1         PAAR_MQTT_INDEX_BODY_DATA+13
#define PAAR_MQTT_INDEX_ENV_DATA_GAS_TVOC_0         PAAR_MQTT_INDEX_BODY_DATA+14
#define PAAR_MQTT_INDEX_ENV_DATA_GAS_TVOC_1         PAAR_MQTT_INDEX_BODY_DATA+15
#define PAAR_MQTT_INDEX_ENV_DATA_COLOR_RED_0        PAAR_MQTT_INDEX_BODY_DATA+16
#define PAAR_MQTT_INDEX_ENV_DATA_COLOR_RED_1        PAAR_MQTT_INDEX_BODY_DATA+17
#define PAAR_MQTT_INDEX_ENV_DATA_COLOR_GREEN_0      PAAR_MQTT_INDEX_BODY_DATA+18
#define PAAR_MQTT_INDEX_ENV_DATA_COLOR_GREEN_1      PAAR_MQTT_INDEX_BODY_DATA+19
#define PAAR_MQTT_INDEX_ENV_DATA_COLOR_BLUE_0       PAAR_MQTT_INDEX_BODY_DATA+20
#define PAAR_MQTT_INDEX_ENV_DATA_COLOR_BLUE_1       PAAR_MQTT_INDEX_BODY_DATA+21
#define PAAR_MQTT_INDEX_ENV_DATA_COLOR_CLEAR_0      PAAR_MQTT_INDEX_BODY_DATA+22
#define PAAR_MQTT_INDEX_ENV_DATA_COLOR_CLEAR_1      PAAR_MQTT_INDEX_BODY_DATA+23

typedef struct _uart_wifi_msgq_event_t {
	uint8_t event;
	uint8_t status;
	uint8_t * msg;
} uart_wifi_msgq_event_t;

typedef struct _wifi_processing_msgq_event_t {
	uint8_t event;
	uint8_t status;
	uint8_t * msg;
} wifi_processing_msgq_event_t;

enum{
	WIFI_PROCESSING_EVENT_SEND_MQTT = 0,
	WIFI_PROCESSING_EVENT_ERROR_RESET,
	WIFI_PROCESSING_EVENT_SEND_MQTT_HB,
	WIFI_PROCESSING_EVENT_SEND_MQTT_ENV,
	WIFI_PROCESSING_EVENT_SEND_MQTT_GRID_EYE,
};

typedef enum {
	UART_WIFI_CMD_DEFAULT = 0,
} uart_wifi_evt_list_t;

enum{
	WIZFI360_WIFI_EVENT_RECEIVED,
};

enum{
	WIZFI360_RSP_EVENT_OK,
	WIZFI360_RSP_EVENT_ERROR,
	WIZFI360_RSP_EVENT_FAIL,
	WIZFI360_RSP_EVENT_SEND_OK,
	WIZFI360_RSP_EVENT_SNTP_TIME,
	WIZFI360_RSP_EVENT_WIFI_GOT_IP,
	WIZFI360_RSP_EVENT_UNKNOWN,
	WIZFI360_RSP_EVENT_LINK_IS_NOT_VAILD,
};

#define MQTT_RESET_MAX_COUNT	3

void uart_wifi_module_task_init(void);

void set_test_time(uint8_t YY, uint8_t MM, uint8_t DD, uint8_t hh, uint8_t mm, uint8_t ss);

int wifi_processing_event_send(uint8_t evt, uint8_t state, uint8_t * msg);

bool get_wifi_ready();

bool get_flag_AP_connected();
bool get_flag_MQTT_SERVER_connected();

#endif /* APP_SH_UART_WIFI_MODULE_SH_UART_WIFI_MODULE_H_ */
