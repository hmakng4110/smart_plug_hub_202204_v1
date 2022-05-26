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

#define PAAR_DEVICE_NAME                		"AA_TAG" /**< Name of device. Will be included in the advertising data. */

#define SAAL_LOCATION				"GP_1309"

#define PAAR_ID_0					(0x01)
#define PAAR_ID_1					(0x09)
#define PAAR_ID_2					(0x09)
#define PAAR_ID_3					(0xE0)

#define TEST_MAX_CONNECTION_DEVICE		3

#define TEST_SEND_MSG_DELAY			500
#define TEST_SCAN_START_DELAY		500
#define TEST_ADV_START_DELAY		2000
#define TEST_CONNECTION_DELAY		100

#endif /* APPLICATION_CONFIG_SW_CONFIG_H_ */
