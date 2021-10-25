/*
 * twi_sensor_module.h
 *
 *  Created on: 2021. 2. 8.
 *      Author: HMKANG
 */

#ifndef APP_TWI_SENSOR_READ_TWI_SENSOR_MODULE_H_
#define APP_TWI_SENSOR_READ_TWI_SENSOR_MODULE_H_

#include <ubinos.h>

#include "nrf_drv_twi.h"
#include "twi_include.h"


#define LPS22HB_ADDR	0x5c		//Low Power Accelerometer 	0b 1011100
#define LIS2DH12_ADDR	0x19		//Pressure Sensor			0b 0011000
#define HTS221_ADDR		0x5f		//Humidity Sensor
#define BH1745_ADDR		0x38		//Color Sensor
#define CCS811_ADDR		0x5a		//Gas Sensor

#define PRESSURE_SENSOR_INIT_WAIT_DELAY 			300
#define TEMPERATURE_HUMIDITY_SENSOR_INIT_WAIT_DELAY	300
#define GAS_SENSOR_INIT_WAIT_DELAY 					300
#define COLOR_SENSOR_INIT_WAIT_DELAY 				300

#define NO_INTERRUPT_PIN	0xffffffff


typedef struct _twi433_msgq_event_t {
	uint8_t event;
	uint8_t status;
	uint8_t * msg;
} twi433_msgq_event_t;

typedef struct _twi433_color_data_t {
	uint16_t red;
	uint16_t green;
	uint16_t blue;
	uint16_t clear;
} color_data_t;

typedef struct _twi433_sensor_data_t {
	float pressure;
	float _pressure_temp;
	int16_t humidity;
	float temperature;
	uint16_t gas_adc_raw;
	drv_ccs811_alg_result_t gas_alg_data;
	uint16_t color_data;
} twi433_sensor_data_t;

typedef enum{
	TWI433_DEFAULT_EVT = 0,
	TWI433_SENSOR_SAMPLING,
	TWI433_BLE_REQUEST,
} _twi433_sensor_evt ;

typedef enum {
	TWI433_BLE_DEFAULT  = 0,
	TWI433_BLE_REQUEST_ALL,
	TWI433_BLE_REQUEST_TEMPERATURE,
	TWI433_BLE_REQUEST_HUMIDITY,
	TWI433_BLE_REQUEST_PRESSURE,
	TWI433_BLE_REQUEST_GAS,
	TWI433_BLE_REQUEST_COLOR,
	TWI433_BLE_REQUEST_GRIDEYE,
	TWI433_BLE_REQUEST_ACC,

	TWI433_BLE_ENV_RESPONSE,
} twi433_ble_event;

void twi433_module_task_init(void);
int twi433_event_send(uint8_t evt, uint8_t state, uint8_t * msg);

#endif /* APP_TWI_SENSOR_READ_TWI_SENSOR_MODULE_H_ */
