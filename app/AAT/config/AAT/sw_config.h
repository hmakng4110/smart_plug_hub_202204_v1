/*
 * sw_config.h
 *
 *  Created on: 2021. 9. 15.
 *      Author: YuJin Park
 *
 */

#ifndef APPLICATION_CONFIG_SW_CONFIG_H_
#define APPLICATION_CONFIG_SW_CONFIG_H_

#include "ADL_info.h"

#define PAAR_DEVICE_NAME                		"ACC_TAG" /**< Name of device. Will be included in the advertising data. */

//ADL 2 Test...
#define TEST_DEVICE_AST_A_LIVING_FAUSET		0x01 //A
#define TEST_DEVICE_AST_G_LIVING_DRAWER	    0x02 //B

#define TEST_DEVICE_AST_G_TOILET_DOOR		0x03 //C
#define TEST_DEVICE_AST_A_TOILET_FAUSET		0x04 //D
#define TEST_DEVICE_AST_A_TOILET_TOILET		0x05 //E

#define TEST_DEVICE_AST_G_ROOM_DOOR 		0x06 //F
#define TEST_DEVICE_AST_G_ROOM_DRAWER 		0x07 //G

#define TEST_DEVICE_AST_G_LIVING_UTENSIL 	0x08 //H

#define TEST_DEVICE_AST_A_GP1309_FAUSET		0x09 //A
#define TEST_DEVICE_AST_G_GP1309_DOOR	    0x0A //B
#define TEST_DEVICE_AST_G_GP1309_VACCUM		0x0B //C

#define TEST_DEVICE_AST_A_GP1313_FAUSET		0x0C //D
#define TEST_DEVICE_AST_G_GP1313_DOOR		0x0D //E

#define TEST_DEVICE_AST_G_MONITORING		0xFF //E


#define TEST_DEVICE_SELECTION               TEST_DEVICE_AST_A_TOILET_TOILET

//AAT Software Mode
#define	AAT_SW_MODE_ACC 			            0x01
#define	AAT_SW_MODE_GYRO			            0x02
#define	ATT_SW_MODE_SETUP_STREAM_ACC_GYRO_DATA	0x03
#define	ATT_SW_MODE_SETUP_STREAM_GYRO_WINDOW	0x04

#define AAT_ACC_SENSOR_AXIS_X                   0x01
#define AAT_ACC_SENSOR_AXIS_Y                   0x02
#define AAT_ACC_SENSOR_AXIS_Z                   0x03

#define ATT_ACC_SENSOR_AXIS_DEFAULT             AAT_ACC_SENSOR_AXIS_Y

#if(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_A_LIVING_FAUSET)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_ACC

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_FAUCET
#define ADL_DEVICE_TYPE1	0x00

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x01)
#define PAAR_ID_2					(0x01)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "Professor_Living"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_G_LIVING_DRAWER)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_GYRO

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_DRAWER
#define ADL_DEVICE_TYPE1	0x00

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x01)
#define PAAR_ID_2					(0x02)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "Professor_Living"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_G_TOILET_DOOR)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_GYRO

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_DOOR
#define ADL_DEVICE_TYPE1	0x00

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x02)
#define PAAR_ID_2					(0x01)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "Professor_Toilet"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_A_TOILET_FAUSET)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_ACC

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_FAUCET
#define ADL_DEVICE_TYPE1	0x00

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x02)
#define PAAR_ID_2					(0x02)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "Professor_Toilet"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_A_TOILET_TOILET)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_ACC

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_TOILET
#define ADL_DEVICE_TYPE1	0x00

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x02)
#define PAAR_ID_2					(0x01)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "Professor_Toilet"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_G_ROOM_DOOR)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_GYRO

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_DOOR
#define ADL_DEVICE_TYPE1	0x00

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x03)
#define PAAR_ID_2					(0x01)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "Professor_Room"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_G_ROOM_DRAWER)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_GYRO

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_DRAWER
#define ADL_DEVICE_TYPE1	0x00

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x03)
#define PAAR_ID_2					(0x02)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "Professor_Room"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_G_LIVING_UTENSIL)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_GYRO

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_COOKING_UTENSIL
#define ADL_DEVICE_TYPE1	0x00

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x01)
#define PAAR_ID_2					(0x03)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "Professor_Room"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_A_GP1309_FAUSET)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_ACC

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_FAUCET
#define ADL_DEVICE_TYPE1	0x01

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x04)
#define PAAR_ID_2					(0x01)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "GP1309"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_G_GP1309_DOOR)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_GYRO

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_DOOR
#define ADL_DEVICE_TYPE1	0x01

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x04)
#define PAAR_ID_2					(0x02)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "GP1309"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_G_GP1309_VACCUM)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_GYRO

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_VACCUM_CLEANER
#define ADL_DEVICE_TYPE1	0x01

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x04)
#define PAAR_ID_2					(0x03)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "GP1309"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_A_GP1313_FAUSET)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_ACC

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_FAUCET
#define ADL_DEVICE_TYPE1	0x01

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x05)
#define PAAR_ID_2					(0x01)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "GP1313"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_G_GP1313_DOOR)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_GYRO

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_DOOR
#define ADL_DEVICE_TYPE1	0x01

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x05)
#define PAAR_ID_2					(0x02)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "GP1313"

#elif(TEST_DEVICE_SELECTION == TEST_DEVICE_AST_G_MONITORING)

#define AAT_SW_MODE_DEFAULT			AAT_SW_MODE_GYRO

#define ADL_DEVICE_TYPE0	ADL_DEVICE_TYPE_DOOR
#define ADL_DEVICE_TYPE1	0x01

#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0xFF)
#define PAAR_ID_2					(0x02)
#define PAAR_ID_3					(0xE0)

#define SAAL_LOCATION       "TEST_MONITORING"

#endif



#define SAAL_DEFAULT_LOCATION_INFO			"테스트룸A"
//#define SAAL_DEFAULT_LOCATION_INFO			"테스트룸A_화장실"
//#define SAAL_DEFAULT_LOCATION_INFO			"테스트룸A_방1"

#define BLE_DEBUGGING_LED_ENABLE			0

#define AAT_SERVICE_ID					0x12

#define TEST_ADV_START_DELAY		2000

#define GYRO_VARIANCE_THRESHOLD         30//75//150
#define GYRO_OFF_COUNT_REFRESH_VAL      10

#define ATT_CHECK_VAL_ON_MIN            2000
#define ATT_CHECK_VAL_ON_MAX            5000

#define ATT_CHECK_VAL_OFF_MIN            -1000
#define ATT_CHECK_VAL_OFF_MAX            500

#endif /* APPLICATION_CONFIG_SW_CONFIG_H_ */
