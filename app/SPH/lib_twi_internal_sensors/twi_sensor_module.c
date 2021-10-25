/*
 * twi_sensor_module.c
 *
 *  Created on: 2021. 2. 8.
 *      Author: HMKANG
 */

#include <malloc.h>
#include "assert.h"
#include "nordic_common.h"

#include "nrf_drv_twi.h"
#include "twi_include.h"

#include "twi_sensor_module.h"

#include "../lib_bluetooth_csos/ble_stack.h"
#include "../lib_bluetooth_csos//LAP_main.h"
#include "../lib_bluetooth_csos//LAP_api.h"

#include "../lib_433_comm/sh_uart_433_module.h"

#include "../config/sw_config.h"
#include "../config/hw_config.h"

#ifndef BLE_STACK_HANDLER_SCHED_EVT_SIZE
#define BLE_STACK_HANDLER_SCHED_EVT_SIZE 0
#endif
#define SCHED_MAX_EVENT_DATA_SIZE	MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE)
#define SCHED_QUEUE_SIZE			60


static const nrf_drv_twi_t twi_433 = NRF_DRV_TWI_INSTANCE(0);

static msgq_pt twi433_module_task_msgq;


#if(DK_DEBUG_ENABLE == 1)
static void button_handler1(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	bsp_busywaitms(10);
	if(nrf_drv_gpiote_in_is_set(pin) ) {
		return;
	}

	LED_toggle(PIN_LED1);
//	twi433_event_send(TWI433_LPS22HB_EVT, 0, NULL);
}

static void button_handler2(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	bsp_busywaitms(10);
	if(nrf_drv_gpiote_in_is_set(pin) ) {
		return;
	}
	LED_toggle(PIN_LED2);
	twi433_event_send(TWI433_SENSOR_SAMPLING, 0, NULL);
}

static void button_handler3(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	bsp_busywaitms(10);
	if(nrf_drv_gpiote_in_is_set(pin) ) {
		return;
	}
	LED_toggle(PIN_LED3);
	if(is_uart_receive_flag_set() == true) {
//		uart433_event_send(UART433_MODULE_SENSOR_PAIRING_MODE, 0, NULL);
		uart433_receive_flag_setting(false);
	}
	else if(is_uart_receive_flag_set() == false) {
		uart433_receive_flag_setting(true);
	}
}
static void button_handler4(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	bsp_busywaitms(10);
	if(nrf_drv_gpiote_in_is_set(pin) ) {
		return;
	}
	uart433_event_send(UART433_MODULE_SEND_CMD_TO_OTHER_HUB, 0, NULL);
	LED_toggle(PIN_LED4);
}
#endif

//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
static int twi_instance_init(void) {

	ret_code_t err;

	nrf_drv_twi_config_t twi_config;
	twi_config.sda = TWI_SCL;
	twi_config.scl = TWI_SDA;
	twi_config.frequency = NRF_DRV_TWI_FREQ_400K;
	twi_config.interrupt_priority = TWI_DEFAULT_CONFIG_IRQ_PRIORITY;
	twi_config.clear_bus_init = TWI_DEFAULT_CONFIG_CLR_BUS_INIT;
	twi_config.hold_bus_uninit = TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT;

	err = nrf_drv_twi_init(&twi_433, &twi_config, NULL, NULL);

	nrf_drv_twi_enable(&twi_433);

	err = NRF_SUCCESS;

	return err;

}

//오랜 기간 DK를 켜두고 PC와 연결되지 않은 상태로 두면
//ERROR 3 [INTERNAL ERROR] 가 뜨며 제대로 동작하지 않는다
//왜그런지는 아직 불명

//--> sensor init하는데 충분한 시간을 task_sleep으로 주고 기다려 줘야함
static void rt_twi_scanner() {
	ret_code_t err_code;
	uint8_t address;
	uint8_t sample_data = 0x0f;
	uint8_t sample_received;
	uint8_t TWI_ADDRESSES = 127;
	bool detected_device = false;

	nrf_drv_twi_enable(&twi_433);

	for (address = 1; address <= TWI_ADDRESSES; address++) {
		err_code = nrf_drv_twi_tx(&twi_433, address, &sample_data, sizeof(sample_data), true);
		if (err_code == NRF_SUCCESS) {
//			if(address == 0x5a) address = 0x5a;
			detected_device = true;
			err_code = nrf_drv_twi_rx(&twi_433, address, &sample_received, 1);
			printf("TWI device detected at address 0x%x.\r\n", address);
			printf("received data = [%d]\r\n", sample_received);
		}

	}

	if (!detected_device) {
		printf("No device was found.");

	}

	nrf_drv_twi_disable(&twi_433);

	printf("\r\n");
}
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////

/*sensor initializing and event handler*/

//////////////////////////////////////////////
//////////////////////////////////////////////
///////  COLOR SENSOR INIT & HANDLER  ////////
//////////////////////////////////////////////
//////////////////////////////////////////////
static void drv_color_data_handler(drv_color_data_t const * p_data)
{
//    (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);

    if (p_data != NULL)
    {
    	drv_bh1745_data_t data;
        printf("color_data_handler r: %d - g: %d - b: %d - c: %d\r\n", p_data->red,
                                                                       p_data->green,
                                                                       p_data->blue,
                                                                       p_data->clear);
        data.red   = p_data->red;
        data.green = p_data->green;
        data.blue  = p_data->blue;
        data.clear = p_data->clear;
//        (void)ble_tes_color_set(&m_tes, &data);
    }
}
static uint32_t color_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    uint32_t err_code;
    drv_color_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.p_twi_instance = p_twi_instance;
    init_params.p_twi_cfg      = &twi_config;
    init_params.twi_addr       = BH1745_ADDR;
    init_params.data_handler   = drv_color_data_handler;

    err_code = drv_color_init(&init_params);
    RETURN_IF_ERROR(err_code);

    task_sleepms(COLOR_SENSOR_INIT_WAIT_DELAY);

    return NRF_SUCCESS;
}
//////////////////////////////////////////////
//////////////////////////////////////////////
/////////  PRESSURE INIT & HANDLER  //////////
//////////////////////////////////////////////
//////////////////////////////////////////////
static void drv_pressure_evt_handler(drv_pressure_evt_t const * p_event)
{
    switch (p_event->type)
    {
        case DRV_PRESSURE_EVT_DATA:
        {
        	//
        }
        break;

        case DRV_PRESSURE_EVT_ERROR:
        {
        	//
        }
        break;

        default:
            break;
    }
}
static uint32_t pressure_sensor_init(const nrf_drv_twi_t * p_twi_instance) {

    drv_pressure_init_t init_params;
    ret_code_t err;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.twi_addr                = LPS22HB_ADDR;
//    init_params.twi_addr                = 0xBA;
    init_params.pin_int                 = LPS_INT;
    init_params.p_twi_instance          = p_twi_instance;
    init_params.p_twi_cfg               = &twi_config;
    init_params.evt_handler             = drv_pressure_evt_handler;
//    init_params.evt_handler				= NULL;
    init_params.mode                    = DRV_PRESSURE_MODE_BAROMETER;

    err = drv_pressure_init(&init_params);

    task_sleepms(PRESSURE_SENSOR_INIT_WAIT_DELAY);

    return err;
}
//////////////////////////////////////////////
//////////////////////////////////////////////
// TEMPERATURE AND HUMIDITY INIT & HANDLER  //
//////////////////////////////////////////////
//////////////////////////////////////////////
static void drv_humidity_evt_handler(drv_humidity_evt_t event)
{
    uint32_t err;

    if (event == DRV_HUMIDITY_EVT_DATA)
    {
        //

    }
    else
    {
    	//
    	err = 0;
    }
}
static uint32_t humidity_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    ret_code_t               err_code = NRF_SUCCESS;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    drv_humidity_init_t    init_params =
    {
        .twi_addr            = HTS221_ADDR,
        .pin_int             = HTS_INT,
        .p_twi_instance      = p_twi_instance,
        .p_twi_cfg           = &twi_config,
        .evt_handler         = drv_humidity_evt_handler
    };

    err_code = drv_humidity_init(&init_params);

    task_sleepms(TEMPERATURE_HUMIDITY_SENSOR_INIT_WAIT_DELAY);

    return err_code;
}
//////////////////////////////////////////////
//////////////////////////////////////////////
////////////  GAS INIT & HANDLER  ////////////
//////////////////////////////////////////////
//////////////////////////////////////////////

static void drv_gas_data_handler(drv_gas_sensor_data_t const * p_data)
{



}
static uint32_t gas_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    uint32_t       err_code;
    drv_gas_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.p_twi_instance = p_twi_instance;
    init_params.p_twi_cfg      = &twi_config;
    init_params.twi_addr       = CCS811_ADDR;
    init_params.data_handler   = drv_gas_data_handler;

    err_code = drv_gas_sensor_init(&init_params);
    RETURN_IF_ERROR(err_code);

    task_sleepms(GAS_SENSOR_INIT_WAIT_DELAY);

    return NRF_SUCCESS;
}

//////////////////////////////////////////////
//////////                          //////////
////////// ENVIRONMENT SENSORS INIT //////////
//////////                          //////////
//////////////////////////////////////////////

static uint32_t sh_environment_init(const nrf_drv_twi_t * p_twi_instance) {

	ret_code_t err;

	err = pressure_sensor_init(p_twi_instance);
	APP_ERROR_CHECK(err);
	err = humidity_sensor_init(p_twi_instance);
	APP_ERROR_CHECK(err);
	err = gas_sensor_init(p_twi_instance);
	APP_ERROR_CHECK(err);
#if(IS_SUB_BOARD_CONNECTED  == 1)
	err = color_sensor_init(p_twi_instance);
	APP_ERROR_CHECK(err);
#endif

	return err;
}
static uint32_t sh_environment_enable() {
	ret_code_t err = 0;

	err = drv_pressure_enable();
	APP_ERROR_CHECK(err);
	err = drv_humidity_enable();
	APP_ERROR_CHECK(err);
	err = drv_gas_sensor_start(DRV_GAS_SENSOR_MODE_1S);
	APP_ERROR_CHECK(err);
#if(IS_SUB_BOARD_CONNECTED == 1)
	err = drv_color_start();
	APP_ERROR_CHECK(err);
#endif

	return err;
}
static uint32_t sh_environment_disable() {
	ret_code_t err = 0;

	err = drv_pressure_disable();
	APP_ERROR_CHECK(err);
	err = drv_humidity_disable();
	APP_ERROR_CHECK(err);
	err = drv_gas_sensor_stop();
	APP_ERROR_CHECK(err);
	err = drv_color_stop();
	APP_ERROR_CHECK(err);

	return err;
}
//////////////////////////////////////////////
/////////////					 /////////////
/////////////  SH SPECIFIC API   /////////////
/////////////                    /////////////
//////////////////////////////////////////////
static float sh_pressure_get(void) {
	ret_code_t err = 0;
	float data_pressure;
	float data_temperature;

//	err = drv_pressure_enable();
	assert(err == 0);
	err = drv_pressure_sample();
	assert(err == 0);

	data_pressure = drv_pressure_get();
	data_temperature = drv_pressure_temperature_get();

//	err = drv_pressure_disable();
	assert(err == 0);

	return data_pressure;
}
static int16_t sh_humidity_get(void) {
	ret_code_t err = 0;
	int16_t data_humidity;

//	err = drv_humidity_enable();
	assert(err == 0);
	err = drv_humidity_sample();
	assert(err == 0);

	data_humidity = drv_humidity_get();

//	err = drv_humidity_disable();
	assert(err == 0);

	return data_humidity;
}
static float sh_temperature_get(void) {
	ret_code_t err = 0;
	float data_temperature;

//	err = drv_humidity_enable();
	assert(err == 0);
	err = drv_humidity_sample();
	assert(err == 0);

//	data_temperature = drv_pressure_temperature_get();
	data_temperature = drv_humidity_temp_get();

//	err = drv_humidity_disable();
	assert(err == 0);

	return data_temperature;
}
static uint16_t sh_gas_raw_get(void) {
	ret_code_t err;
	uint16_t data_gas_raw;
	uint8_t dump_gas_selected;

	err = drv_gas_sensor_raw_data_get(&dump_gas_selected, &data_gas_raw);
	assert(err == 0);

	return data_gas_raw;
}
static drv_ccs811_alg_result_t sh_gas_alg_get(void) {

	return sh_ccs811_alg_result_data_get();
}
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////

static uint32_t twi433_ble_request_handler(uint8_t request_type) {
	twi433_sensor_data_t env_data;
	uint8_t * temp_msg = NULL;
	temp_msg = (uint8_t *)malloc(sizeof(twi433_sensor_data_t));
	memcpy(temp_msg, &env_data, sizeof(twi433_sensor_data_t));

	printf("Sensor Request from BLE Service\r\n");

	env_data.pressure = sh_pressure_get();
	env_data.humidity = sh_humidity_get();
	env_data.temperature = sh_temperature_get();
//	env_data.gas_adc_raw = sh_gas_raw_get();
	env_data.gas_alg_data = sh_gas_alg_get();

	switch( request_type ) {
	case TWI433_BLE_REQUEST_ALL:
//		LAP_event_send(SH_TWI_SENSOR_EVT, TWI433_BLE_ENV_RESPONSE, 0, 0, sizeof(twi433_sensor_data_t), temp_msg);
		break;

	case TWI433_BLE_REQUEST_GAS:

		break;

	default:
		break;
	}

	return NRF_SUCCESS;
}

//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////






static void twi433_module_task(void * arg) {
	ret_code_t err;
	twi433_msgq_event_t twi433_evt_msg;
	twi433_sensor_data_t tmp_data;

	ble_stack_init_wait();

	task_sleepms(500);

	err = sh_environment_enable();
	if(err != NRF_SUCCESS) {
		printf("sh_environment_enable error : %ld\r\n", err);
		assert(err == 0);
	}

	uint32_t task_active_count = 0;

	for(;;) {


		app_sched_execute();
		//for debugging (temp)
		tmp_data.pressure 		= sh_pressure_get();
		tmp_data.humidity 		= sh_humidity_get();
		tmp_data.temperature 	= sh_temperature_get();
		tmp_data.gas_adc_raw	= sh_gas_raw_get();
		tmp_data.gas_alg_data	= sh_ccs811_alg_result_data_get();

		printf("\r\n");
		printf("Pressure 	: [%f]\r\n", tmp_data.pressure);
		printf("Humidity	: [%d]\r\n", tmp_data.humidity);
		printf("Temperature	: [%f]\r\n", tmp_data.temperature);
		printf("Gas Alg		: [%d][%d]\r\n", tmp_data.gas_alg_data.ec02_ppm, tmp_data.gas_alg_data.tvoc_ppb);
		printf("TWI SENSOR ACTIVATED %ld\r\n", task_active_count++);
		printf("\r\n");

//		err = msgq_receive(twi433_module_task_msgq, (unsigned char * ) &twi433_evt_msg);
//		if( err != 0 ) {
//			printf("fail at msgq receive\r\n");
//		} else {
//			switch( twi433_evt_msg.event ) {
//
//			case TWI433_SENSOR_SAMPLING:
//
//
//				break;
//
//			case TWI433_BLE_REQUEST:
//				twi433_ble_request_handler( twi433_evt_msg.status );
//
//
//				break;
//
//			default:
//				printf("Default TWI sensor scanning\r\n");
//
//				//print all twi sensor data
//
//				break;
//			}
//		}

		task_sleepms(10000);
	}//end of for(;;)
}

void twi433_module_task_init(void) {
	ret_code_t err;

	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
	err = app_timer_init();
	APP_ERROR_CHECK(err);

	//TWI instance init
	err = twi_instance_init();
	if( err != 0 ) {
		printf("twi433_init error : %ld\n\r", err);
		assert(err == 0);
	}

	//Sensor Init
	err = sh_environment_init(&twi_433);
	if( err != 0) {
		printf("sh_environment sensor init error : %ld\r\n", err);
		assert(err == 0);
	}

	task_sleepms(500);

	nrf_drv_gpiote_init();
	//DK io Init
#if(DK_DEBUG_ENABLE == 1)
	LED_init();
	button_gpio_init(
			(nrf_drv_gpiote_evt_handler_t) button_handler1,
			(nrf_drv_gpiote_evt_handler_t) button_handler2,
			(nrf_drv_gpiote_evt_handler_t) button_handler3,
			(nrf_drv_gpiote_evt_handler_t) button_handler4
			);
#endif

	//Ubinos kernel api init
	err = msgq_create(&twi433_module_task_msgq, sizeof(twi433_msgq_event_t), 20);
	if( 0 != err ) {
		printf("fail at task_create\r\n");
	}

	err = task_create(NULL, twi433_module_task, NULL, task_getmiddlepriority(), 256, "twi433_module_task");
	if( 0 != err ) {
		printf("fail at task_create\r\n");
	} else {
		printf("== TWI_MODULE_TASK created\r\n");
	}

}

int twi433_event_send(uint8_t evt, uint8_t state, uint8_t * msg) {
	twi433_msgq_event_t tmp_twi433_msgq_event;

	tmp_twi433_msgq_event.event = evt;
	tmp_twi433_msgq_event.status = state;
	tmp_twi433_msgq_event.msg = msg;

	return msgq_send(twi433_module_task_msgq, (unsigned char * ) &tmp_twi433_msgq_event);
}
