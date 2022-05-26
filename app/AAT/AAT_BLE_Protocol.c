/*
  * AAT_BLE_Protocol.c
 *
 *  Created on: 2022. 5. 13.
 *      Author: Yu Jin Park
 */

#include <stdio.h>
#include <stdint.h>

#include "sw_config.h"
#include "AAT_BLE_Protocol.h"

void send_AAT_result_packet(uint8_t action_result)
{
    uint8_t* tx_temp_packet = NULL;

    uint8_t location_len = strlen(AAT_PACKET_RESULT_LOCATION_DATA);
    uint8_t packet_len = AAT_PACKET_RESULT_HEADER_DATA_LEN+AAT_PACKET_RESULT_LOCATION_LEN_SIZE+location_len;

    tx_temp_packet = malloc(packet_len);
    if(tx_temp_packet == NULL)
    {
        printf("malloc_error : send_AAT_result_packet \r\n");
        return;
    }

    memset(tx_temp_packet, 0, packet_len);

    tx_temp_packet[AAT_PACKET_INDEX_PACKET_TYPE] = AAT_PACKET_RESULT_PACKET_TYPE;
    tx_temp_packet[AAT_PACKET_INDEX_PACKET_SERVICE_ID] = AAT_PACKET_RESULT_SERVICE_ID;
    tx_temp_packet[AAT_PACKET_INDEX_SEQUENCE_NUM] = AAT_PACKET_RESULT_SEQUENCE_NUM;
    tx_temp_packet[AAT_PACKET_INDEX_DATA_LEN] = AAT_PACKET_RESULT_DATA_LEN;
    tx_temp_packet[AAT_PACKET_INDEX_DATA_CMD] = AAT_PACKET_RESULT_DATA_CMD;
    tx_temp_packet[AAT_PACKET_INDEX_ACTIVITY_RESULT_ADL_DEVICE_ID0] = ADL_DEVICE_TYPE0;
    tx_temp_packet[AAT_PACKET_INDEX_ACTIVITY_RESULT_ADL_DEVICE_ID1] = ADL_DEVICE_TYPE1;
    tx_temp_packet[AAT_PACKET_INDEX_ACITIVITY_RESULT_ACTION] = action_result;
    tx_temp_packet[AAT_PACKET_INDEX_ACITIVITY_LOCATION_LEN] = location_len;

    memcpy(&tx_temp_packet[AAT_PACKET_INDEX_ACITIVITY_LOCATION_DATA], AAT_PACKET_RESULT_LOCATION_DATA, location_len);

    LAP_send_ble_msg_peripheral(tx_temp_packet, packet_len);

    task_sleep(10);

    free(tx_temp_packet);
}