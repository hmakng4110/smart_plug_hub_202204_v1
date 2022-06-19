/*
 * SAAL_packet_process.h
 *
 *  Created on: 2022. 05. 17.
 *      Author: Yu Jin Park
 */

#ifndef APPLICATION_SPH_SAAL_PACKET_PROCESS_H_
#define APPLICATION_SPH_SAAL_PACKET_PROCESS_H_

#include "sw_config.h"
#include "hw_config.h"

#include "LAP_api.h"
#include "LAP_main.h"

//SAAL Packet Processing Functions
void process_SAAL_packet_AAT(LAPEvt_msgt LAP_evt_msg);
void process_SAAL_packet_HAT(LAPEvt_msgt LAP_evt_msg);
void process_SAAL_packet_SPH(LAPEvt_msgt LAP_evt_msg);
void process_SAAL_packet_SmartFarm(LAPEvt_msgt LAP_evt_msg);
void process_SAAL_packet_ENV(LAPEvt_msgt LAP_evt_msg);
void process_SAAL_packet_Footpad(LAPEvt_msgt LAP_evt_msg);
void process_SAAL_packet_Device_Setup(LAPEvt_msgt LAP_evt_msg);

#endif /* APPLICATION_SPH_SAAL_PACKET_PROCESS_H_ */