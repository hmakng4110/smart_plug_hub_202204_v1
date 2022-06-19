/*
 * AAT_BLE_Protocol.h
 *
 *  Created on: 2022. 5. 13.
 *      Author: Yu Jin Park
 */

#ifndef APP_AAT_BLE_PROTOCOL_H_
#define APP_AAT_BLE_PROTOCOL_H_

#include "LAP_api.h"
#include "sw_config.h"

#define AAT_PACKET_INDEX_PACKET_TYPE                    PAAR_PACKET_INDEX_PACKET_TYPE
#define AAT_PACKET_INDEX_PACKET_SERVICE_ID              PAAR_PACKET_INDEX_SERVICE_ID
#define AAT_PACKET_INDEX_SEQUENCE_NUM                   PAAR_PACKET_INDEX_SEQUENCE_NUM
#define AAT_PACKET_INDEX_DATA_LEN                       PAAR_PACKET_INDEX_DATA_LEN
#define AAT_PACKET_INDEX_DATA_CMD                       PAAR_PACKET_INDEX_BODY_DATA_CMD

#define AAT_PACKET_INDEX_ACTIVITY_RESULT_ADL_DEVICE_ID0 PAAR_PACKET_INDEX_BODY_DATA_BODY
#define AAT_PACKET_INDEX_ACTIVITY_RESULT_ADL_DEVICE_ID1 PAAR_PACKET_INDEX_BODY_DATA_BODY+1

#define AAT_PACKET_INDEX_ACITIVITY_RESULT_ACTION        PAAR_PACKET_INDEX_BODY_DATA_BODY+2
#define AAT_PACKET_INDEX_ACITIVITY_LOCATION_LEN         PAAR_PACKET_INDEX_BODY_DATA_BODY+3
#define AAT_PACKET_INDEX_ACITIVITY_LOCATION_DATA        PAAR_PACKET_INDEX_BODY_DATA_BODY+4

#define AAT_PACKET_RESULT_LOCATION_DATA                 SAAL_LOCATION

#define AAT_PACKET_RESULT_HEADER_LEN                    4
#define AAT_PACKET_RESULT_DATA_LEN                      4   

#define AAT_PACKET_RESULT_HEADER_DATA_LEN               AAT_PACKET_RESULT_HEADER_LEN+AAT_PACKET_RESULT_DATA_LEN

#define AAT_PACKET_RESULT_DATA_CMD                      0x03

#define AAT_PACKET_RESULT_LOCATION_LEN_SIZE             1

#define AAT_PACKET_RESULT_PACKET_TYPE                   PAAR_PACKET_TYPE_M2M
#define AAT_PACKET_RESULT_SERVICE_ID                    SAAL_SERVICE_ID_AAT
#define AAT_PACKET_RESULT_SEQUENCE_NUM                  0x11

void send_AAT_result_packet(uint8_t action_result);

#endif /* APP_AAT_BLE_PROTOCOL_H_ */