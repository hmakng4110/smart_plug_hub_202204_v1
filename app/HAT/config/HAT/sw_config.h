/*
 * sw_config.h
 *
 *  Created on: 2020. 06. 11.
 *      Author: YJPark
 */

#ifndef APP_CONFIG_SW_CONFIG_H_
#define APP_CONFIG_SW_CONFIG_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "ADL_info.h"

/**< Name of device. Will be included in the advertising data. */
#define PAAR_DEVICE_NAME                	"HAT_TAG"

#define TEST_MAX_CONNECTION_DEVICE		3

#define HAT_SW_MODE_HAT_NORMAL          0
#define HAT_SW_MODE_HAT_TEST          0

/*
#define HAT_SW_MODE_TEST_CENTRAL		0
#define HAT_SW_MODE_TEST_PERIPHERAL		1
*/

#define HAT_SW_MODE_SETUP				HAT_SW_MODE_HAT_NORMAL

#define TEST_DEVICE_TYPE_ELECTRIC_FAN		1
#define TEST_DEVICE_TYPE_HAIR_DRYER			2
#define TEST_DEVICE_TYPE_VACUUM_CLEAR		3

#define TEST_DEVICE_TYPE				TEST_DEVICE_TYPE_HAIR_DRYER

#define SAAL_LOCATION       "GP_1309"

#if(TEST_DEVICE_TYPE == TEST_DEVICE_TYPE_ELECTRIC_FAN)
// Definition : PAAR ID(4 Byte)
#define PAAR_ID_0					(0x02)
#define PAAR_ID_1					(0x00)
#define PAAR_ID_2					(0x01)
#define PAAR_ID_3					(0xD8)

#define ADL_DEVICE_TYPE_ID0                 ADL_DEVICE_TYPE_ELECTRIC_FAN
#define ADL_DEVICE_TYPE_ID1                 0x01

#elif(TEST_DEVICE_TYPE == TEST_DEVICE_TYPE_HAIR_DRYER)
// Definition : PAAR ID(4 Byte)
#define PAAR_ID_0					(0xFF)
#define PAAR_ID_1					(0x02)
#define PAAR_ID_2					(0x01)
#define PAAR_ID_3					(0xD8)

#define ADL_DEVICE_TYPE_ID0                 ADL_DEVICE_TYPE_HAIR_DRYER
#define ADL_DEVICE_TYPE_ID1                 0x01


#elif(TEST_DEVICE_TYPE == TEST_DEVICE_TYPE_VACUUM_CLEAR)
// Definition : PAAR ID(4 Byte)
#define PAAR_ID_0					(0x01)
#define PAAR_ID_1					(0x00)
#define PAAR_ID_2					(0x00)
#define PAAR_ID_3					(0xD8)

#define ADL_DEVICE_TYPE_ID0                 ADL_DEVICE_TYPE_VACCUM_CLEANER
#define ADL_DEVICE_TYPE_ID1                 0x01
#endif

#define TEST_SEND_MSG_DELAY			500
#define TEST_SCAN_START_DELAY		500
#define TEST_ADV_START_DELAY		500
#define TEST_CONNECTION_DELAY		100
//////////////////////////////////////////////////////////
//#define APP_TIMER_PRESCALER         0               /**< Value of the RTC1 PRESCALER register. */
//#define APP_TIMER_OP_QUEUE_SIZE			2               /**< Size of timer operation queues. */
//////////////////////////////////////////////////////////
//#define UNDEFINED										0XFF

// Nordic BLE stack communication enable
//#define NRF_NOTIFICATION_ENABLE				0x01
//#define NRF_INDICATION_ENABLE				0x02
//#define NRF_NOTI_INDI_ENABLE				0x03

/* status byte in Advertising packet payloads */
//#define LIDx_STATUS									0x00
//#define PNIP_STATUS									0x01
//#define REQ_CONN_SOSP_STATUS						0x02
//#define REQ_CONN_SMARTDEVICE_STATUS					0x04
//#define REQ_LIDx_STATUS								0x08



//software mode = Smart Plug Hub
//#define SP_SW_MODE_SPH						0

//#define SP_SW_MODE_SETUP							SP_SW_MODE_SPH


//Test code : Demo
#define SP_TEST_LOCATION_BEDROOM		1
#define SP_TEST_LOCATION_LIVINGROOM		2
#define SP_TEST_LOCATION_GP1313		    3

#define SP_TEST_LOCATION				SP_TEST_LOCATION_GP1313

//Test Information
#if(SP_TEST_LOCATION == SP_TEST_LOCATION_BEDROOM)
#define SH_ID "AB001309"
#define SP_ID "010000D4"
#define SAAL_LOCATION	"bedroom"
#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_LIVINGROOM)
#define SH_ID "AB001309"
#define SP_ID "020000D4"
#define SAAL_LOCATION	"livingroom"
#elif(SP_TEST_LOCATION == SP_TEST_LOCATION_GP1313)
#define SH_ID "AB001309"
#define SP_ID "FF0501D4"
#define SAAL_LOCATION	"livingroom"
#endif

///////////////////////////////////////////////////////////////////////////
#endif /* APPLICATION_GASTAG_EXE_GASTAG_ITF_SW_CONFIG_H_ */
