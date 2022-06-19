/*
 * SPH_environment_sensor.h
 *
 *  Created on: 2022. 05. 24.
 *      Author: Yu Jin Park
 */

#ifndef APPLICATION_SPH_EVIRONMENT_SENSOR_H_
#define APPLICATION_SPH_EVIRONMENT_SENSOR_H_

#include <ubinos.h>

#include <stdio.h>
#include <stdint.h>

#include "sw_config.h"
#include "hw_config.h"

#define SPH_ENV_TASK_START_DELAY    5000


enum{
    SPH_ENV_EVENT_REPORT_SERVER = 0,
};

typedef struct _sph_env_msgq_event_t {
	uint8_t event;
	uint8_t status;
	uint8_t * msg;
} sph_env_msgq_event_t;

int SPH_env_event_send(uint8_t evt, uint8_t state, uint8_t * msg);

void SPH_envrionment_report_task_init(void);

#endif /* APPLICATION_SPH_EVIRONMENT_SENSOR_H_ */