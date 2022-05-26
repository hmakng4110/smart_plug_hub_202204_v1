/*
 * sw_config.h
 *
 *  Created on: 2020. 06. 11.
 *      Author: YJPark
 */

#ifndef APPLICATION_GASTAG_EXE_GASTAG_ITF_SW_CONFIG_H_
#define APPLICATION_GASTAG_EXE_GASTAG_ITF_SW_CONFIG_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "SEGGER_RTT.h"

//software mode = Smart Plug Hub
#define SP_SW_MODE_SPH						0

//#define ENABLE_SW_MODE_ENVIRONMENT_SENSORS  1

#define SP_SW_MODE_TEST_CENTRAL				1
#define SP_SW_MODE_TEST_PERIPHERAL			2

#define SP_SW_MODE_TEST_MONITORING          4

#define SP_SW_MODE_SETUP					SP_SW_MODE_SPH

#define SP_ENVIRONMENT_DATA_REPORT_ENABLE   1

//Test code : Demo
#define SP_TEST_LOCATION_LIVINGROOM		1
#define SP_TEST_LOCATION_TOILET		    2
#define SP_TEST_LOCATION_ROOM		    3

#define SP_TEST_LOCATION_GP1309         4
#define SP_TEST_LOCATION_GP1313         5

#define SP_TEST_LOCATION				SP_TEST_LOCATION_GP1309

//Test Information
#if(SP_TEST_LOCATION == SP_TEST_LOCATION_LIVINGROOM)
#define SH_ID "AB001309"
#define SP_ID "FF0100D4"
#define SAAL_LOCATION	"Professor_LIVING"

#define TEST_MQTT_CLIENT_ID		"test_ADL_1"

#define TEST_MQTT_USER_NAME		"Test_User1"
#define TEST_MQTT_USER_PASSWORD	"testtest"

#define TEST_MQTT_TOPIC_PUBLISH			"CSOS/AB001309/FF0100D4/ADL_DATA"
#define TEST_MQTT_TOPIC_SUBSCRIBE		"sensorID"

#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_TOILET)
#define SH_ID "AB001309"
#define SP_ID "FF0200D4"
#define SAAL_LOCATION	"Professor_TOILET"

#define TEST_MQTT_CLIENT_ID		"test_ADL_7"

#define TEST_MQTT_USER_NAME		"Test_User7"
#define TEST_MQTT_USER_PASSWORD	"testtest"

#define TEST_MQTT_TOPIC_PUBLISH			"CSOS/AB001309/FF0200D4/ADL_DATA"
#define TEST_MQTT_TOPIC_SUBSCRIBE		"sensorID"

#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_ROOM)
#define SH_ID "AB001309"
#define SP_ID "FF0300D4"
#define SAAL_LOCATION	"Professor_ROOM"

#define TEST_MQTT_CLIENT_ID		"test_ADL_3"

#define TEST_MQTT_USER_NAME		"Test_User3"
#define TEST_MQTT_USER_PASSWORD	"testtest"

#define TEST_MQTT_TOPIC_PUBLISH			"CSOS/AB001309/FF0300D4/ADL_DATA"
#define TEST_MQTT_TOPIC_SUBSCRIBE		"sensorID"

#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_GP1309)
#define SH_ID "AB001309"
#define SP_ID "FF0400D4"
#define SAAL_LOCATION	"GP_1309"

#define TEST_MQTT_CLIENT_ID		"test_ADL_4"

#define TEST_MQTT_USER_NAME		"Test_User4"
#define TEST_MQTT_USER_PASSWORD	"testtest"

#define TEST_MQTT_TOPIC_PUBLISH			"CSOS/AB001309/FF0400D4/ADL_DATA"
#define TEST_MQTT_TOPIC_SUBSCRIBE		"sensorID"

#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_GP1313)
#define SH_ID "AB001309"
#define SP_ID "FF0500D4"
#define SAAL_LOCATION	"GP_1313"

#define TEST_MQTT_CLIENT_ID		"test_ADL_5"

#define TEST_MQTT_USER_NAME		"Test_User5"
#define TEST_MQTT_USER_PASSWORD	"testtest"

#define TEST_MQTT_TOPIC_PUBLISH			"CSOS/AB001309/FF0500D4/ADL_DATA"
#define TEST_MQTT_TOPIC_SUBSCRIBE		"sensorID"

#endif

#if(SP_SW_MODE_SETUP == SP_SW_MODE_SPH)
    #if(SP_TEST_LOCATION == SP_TEST_LOCATION_LIVINGROOM)
#define TEST_MAX_CONNECTION_DEVICE		6
    #elif(SP_TEST_LOCATION == SP_TEST_LOCATION_TOILET)
#define TEST_MAX_CONNECTION_DEVICE		5
    #elif(SP_TEST_LOCATION == SP_TEST_LOCATION_ROOM)
#define TEST_MAX_CONNECTION_DEVICE		4
    #elif(SP_TEST_LOCATION == SP_TEST_LOCATION_GP1309)
#define TEST_MAX_CONNECTION_DEVICE		6
    #elif(SP_TEST_LOCATION == SP_TEST_LOCATION_GP1313)
#define TEST_MAX_CONNECTION_DEVICE		5
    #endif
#elif(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_CENTRAL || SP_SW_MODE_SETUP == SP_SW_MODE_TEST_PERIPHERAL)
#define TEST_MAX_CONNECTION_DEVICE		1
#elif(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_MONITORING)
#define TEST_MAX_CONNECTION_DEVICE		8
#endif

#if(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_CENTRAL || SP_SW_MODE_SETUP == SP_SW_MODE_TEST_PERIPHERAL)
// Definition : PAAR ID(4 Byte)
#define PAAR_ID_0					(0x01)
#define PAAR_ID_1					(0x00)
#define PAAR_ID_2					(0x00)
#define PAAR_ID_3					(0xE4) //Device ID : Smart Plug Hub
#elif(SP_SW_MODE_SETUP == SP_SW_MODE_SPH)
    #if(SP_TEST_LOCATION == SP_TEST_LOCATION_LIVINGROOM)
#define PAAR_ID_0					0xFF
#define PAAR_ID_1					0x01
#define PAAR_ID_2					0x00
#define PAAR_ID_3				    0xD4 //Device ID : Smart Plug Hub

#define PAAR_ID_STR					"FF0100D4"

    #elif(SP_TEST_LOCATION == SP_TEST_LOCATION_TOILET)
#define PAAR_ID_0					0xFF
#define PAAR_ID_1					0x02
#define PAAR_ID_2					0x00
#define PAAR_ID_3				    0xD4 //Device ID : Smart Plug Hub

#define PAAR_ID_STR					"FF0200D4"

    #elif(SP_TEST_LOCATION == SP_TEST_LOCATION_ROOM)
#define PAAR_ID_0					0xFF
#define PAAR_ID_1					0x03
#define PAAR_ID_2					0x00
#define PAAR_ID_3				    0xD4 //Device ID : Smart Plug Hub

#define PAAR_ID_STR					"FF0300D4"

    #elif(SP_TEST_LOCATION == SP_TEST_LOCATION_GP1309)
#define PAAR_ID_0					0xFF
#define PAAR_ID_1					0x02
#define PAAR_ID_2					0x00
#define PAAR_ID_3				    0xD4 //Device ID : Smart Plug Hub

    #elif(SP_TEST_LOCATION == SP_TEST_LOCATION_GP1313)
#define PAAR_ID_0					0xFF
#define PAAR_ID_1					0x05
#define PAAR_ID_2					0x00
#define PAAR_ID_3				    0xD4 //Device ID : Smart Plug Hub
    #endif

#elif(SP_SW_MODE_SETUP == SP_SW_MODE_TEST_MONITORING)
#define PAAR_ID_0					0xFF
#define PAAR_ID_1					0xFF
#define PAAR_ID_2					0xFF
#define PAAR_ID_3				    0xD4 //Device ID : Smart Plug Hub

#endif 

#define ENABLE_BLE_TARGET_BASE_PAAR_ID      1

#define TEST_TARGET_PAAR_ID_SIZE	3

#define TEST_SEND_MSG_DELAY			500
#define TEST_SCAN_START_DELAY		500
#define TEST_ADV_START_DELAY		500
#define TEST_CONNECTION_DELAY		100

/**< Name of device. Will be included in the advertising data. */
#define PAAR_DEVICE_NAME                	"SP_Hub"

#endif /* APPLICATION_SW_CONFIG_H_ */
