/*
 * twi_sensor_module.c
 *
 *  Created on: 2021. 2. 8.
 *      Author: HMKANG
 */

#include <ubinos/bsp.h>
#include <ubinos/bsp/arch.h>
#include <ubinos/bsp_ubik.h>

#include <malloc.h>

#if (CSOS_SAAL__USE_LIB_twi_internal_sensors == 1)

#include "twi_sensor_module.h"

#include <nrf_drv_twi.h>
#include <lib_bluetooth_csos/ble_stack.h>
#include <lib_bluetooth_csos/LAP_api.h>

#include "twi_include.h"

#include "nrf_drv_gpiote.h"

#include "drv_ext_gpio.h"

#include "hw_config.h"
#include "sw_config.h"

#include "hw_config.h"

#include "stdlib.h"
#include "assert.h"
#include "math.h"
#include "lib_433_comm/sh_uart_433_module.h"

#include "sw_config.h"

#include "nordic_common.h"

#define IS_SUB_BOARD_CONNECTED 1
/////////////////////////////////////////
/////////////////////////////////////////

	int grid_eye_pixel_buf_first[GRID_EYE_PIXEL_ARRAY_SIZE] = {0.0, };
	int grid_eye_pixel_buf_second[GRID_EYE_PIXEL_ARRAY_SIZE] = {0.0, };
	int grid_eye_pixel_buf_diff[GRID_EYE_PIXEL_ARRAY_SIZE] = {0.0, };
	int grid_eye_data_raw[GRID_EYE_FRAME_SIZE][2][8];
	int grid_eye_data_diff[GRID_EYE_FRAME_SIZE][2][8];
	int grid_eye_frame = 0;
	grid_eye_queue grid_eye_data_queue_raw;
	grid_eye_queue grid_eye_data_queue_diff;

	grid_eye_list_queue grid_eye_data_list_queue_horizon;
	grid_eye_list_queue grid_eye_data_list_queue_vertical;

	int performance_test = 0;

/////////////////////////////////////////
/////////////////////////////////////////


#ifndef BLE_STACK_HANDLER_SCHED_EVT_SIZE
#define BLE_STACK_HANDLER_SCHED_EVT_SIZE 0

#define SCHED_MAX_EVENT_DATA_SIZE	MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE)
#define SCHED_QUEUE_SIZE			60


static const nrf_drv_twi_t twi_433 = NRF_DRV_TWI_INSTANCE(0);

//extern mutex_pt _g_mutex_peripheral;

static mutex_pt _g_mutex_grid_eye;

static int twi_instance_init(void) {

	ret_code_t err;

	nrf_drv_twi_config_t twi_config;
	twi_config.sda = SPH_TWI_SCL;
	twi_config.scl = SPH_TWI_SDA;
	twi_config.frequency = NRF_DRV_TWI_FREQ_400K;
	twi_config.interrupt_priority = TWI_DEFAULT_CONFIG_IRQ_PRIORITY;
	twi_config.clear_bus_init = TWI_DEFAULT_CONFIG_CLR_BUS_INIT;
	twi_config.hold_bus_uninit = TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT;

	err = nrf_drv_twi_init(&twi_433, &twi_config, NULL, NULL);

	nrf_drv_twi_enable(&twi_433);

	err = NRF_SUCCESS;

	return err;

}

static void twi_instance_uninit(void) {
	nrf_drv_twi_uninit(&twi_433);
}

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
///////                               ///////
///////  COLOR SENSOR INIT & HANDLER  ////////
///////                               ////////
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
        .scl                = SPH_TWI_SCL,
        .sda                = SPH_TWI_SDA,
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
/////////                           //////////
/////////  PRESSURE INIT & HANDLER  //////////
/////////                           //////////
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
        .scl                = SPH_TWI_SCL,
        .sda                = SPH_TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.twi_addr                = LPS22HB_ADDR;
    init_params.pin_int                 = SPH_LPS_INT;
    init_params.p_twi_instance          = p_twi_instance;
    init_params.p_twi_cfg               = &twi_config;
    init_params.evt_handler             = drv_pressure_evt_handler;
    init_params.mode                    = DRV_PRESSURE_MODE_BAROMETER;

    err = drv_pressure_init(&init_params);

    task_sleepms(PRESSURE_SENSOR_INIT_WAIT_DELAY);

    return err;
}
//////////////////////////////////////////////
//                                          //
// TEMPERATURE AND HUMIDITY INIT & HANDLER  //
//                                          //
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
        .scl                = SPH_TWI_SCL,
        .sda                = SPH_TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    drv_humidity_init_t    init_params =
    {
        .twi_addr            = HTS221_ADDR,
        .pin_int             = SPH_HTS_INT,
        .p_twi_instance      = p_twi_instance,
        .p_twi_cfg           = &twi_config,
        .evt_handler         = drv_humidity_evt_handler
    };

    err_code = drv_humidity_init(&init_params);

    task_sleepms(TEMPERATURE_HUMIDITY_SENSOR_INIT_WAIT_DELAY);

    return err_code;
}
//////////////////////////////////////////////
////////////                      ////////////
////////////  GAS INIT & HANDLER  ////////////
////////////                      ////////////
//////////////////////////////////////////////

static void drv_gas_data_handler(drv_gas_sensor_data_t const * p_data)
{
	//do nothing...
}
static uint32_t gas_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    uint32_t       err_code;
    drv_gas_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = SPH_TWI_SCL,
        .sda                = SPH_TWI_SDA,
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

	err = humidity_sensor_init(p_twi_instance);
	APP_ERROR_CHECK(err);

	err = pressure_sensor_init(p_twi_instance);
	APP_ERROR_CHECK(err);

	err = gas_sensor_init(p_twi_instance);
	APP_ERROR_CHECK(err);
	err = color_sensor_init(p_twi_instance);
	APP_ERROR_CHECK(err);

	return err;
}

uint32_t sh_environment_enable() {
	ret_code_t err = 0;

	do {
		err = drv_pressure_enable();
		if(err != 0) {
			break;
		}
		err = drv_humidity_enable();
		if(err != 0) {
			break;
		}
		task_sleepms(5000);
		
		err = drv_gas_sensor_start(DRV_GAS_SENSOR_MODE_1S);
		if(err != 0) {
			break;
		}

		uint16_t humid = sh_humidity_get();
#if(SAAL_HW_DEVICE_TYPE == SAAL_HW_DEVICE_THINGY52)
		float temp = sh_temperature_get();	
#elif(SAAL_HW_DEVICE_TYPE == SAAL_HW_DEVICE_SPH)
		float temp = drv_grid_eye_read_thermistor();
#endif

		drv_gas_sensor_calibrate_humid_temp(humid + 15, temp);
		if(err != 0) {
			break;
		}
		task_sleepms(1000);

		err = drv_color_start();
		if(err != 0) {
			break;
		}
		break;
	} while(1);

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
//////////                          //////////
//////////  GRID_EYE SENSORS INIT   //////////
//////////                          //////////
//////////////////////////////////////////////

static uint32_t sh_grid_eye_init(const nrf_drv_twi_t * p_twi_instance) {
	ret_code_t err;
    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = SPH_TWI_SCL,
        .sda                = SPH_TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    drv_grid_eye_init_t init_params;

    init_params.twi_addr 		= GRID_EYE_ADDR;
    init_params.pin_int 		= GRID_EYE_INT;
    init_params.p_twi_instance 	= p_twi_instance;
    init_params.p_twi_cfg 		= &twi_config;

	err = drv_grid_eye_init(&init_params);
	RETURN_IF_ERROR(err);

	return err;
}

//////////////////////////////////////////////
/////////////					 /////////////
/////////////  SH SPECIFIC API   /////////////
/////////////                    /////////////
//////////////////////////////////////////////
float sh_pressure_get(void) {
	ret_code_t err = 0;
	float data_pressure;
	float data_temperature;

	//mutex_lock(_g_mutex_peripheral);
//	err = drv_pressure_enable();
	err = drv_pressure_sample();
	assert(err == 0);

	data_pressure = drv_pressure_get();
	data_temperature = drv_pressure_temperature_get();

//	err = drv_pressure_disable();
//	assert(err == 0);
	//mutex_unlock(_g_mutex_peripheral);

	return data_pressure;
}
int16_t sh_humidity_get(void) {
	ret_code_t err = 0;
	int16_t data_humidity;

	//mutex_lock(_g_mutex_peripheral);
	err = drv_humidity_sample();
	assert(err == 0);

	data_humidity = drv_humidity_get();
	assert(err == 0);
	//mutex_unlock(_g_mutex_peripheral);

	return data_humidity;
}
float sh_temperature_get(void) {
	ret_code_t err = 0;
	float data_temperature;

	//mutex_lock(_g_mutex_peripheral);
	err = drv_humidity_sample();
	assert(err == 0);

//	data_temperature = drv_pressure_temperature_get();
	data_temperature = drv_humidity_temp_get();
	assert(err == 0);
	//mutex_unlock(_g_mutex_peripheral);

	return data_temperature;
}
drv_bh1745_data_t sh_color_get(void) {
	drv_bh1745_data_t data;

	//mutex_lock(_g_mutex_peripheral);
	data = sh_bh1745_color_data_get();
	//mutex_unlock(_g_mutex_peripheral);

	return data;
}

uint16_t sh_gas_raw_get(void) {
	ret_code_t err;
	uint16_t data_gas_raw;
	uint8_t dump_gas_selected;

//	mutex_lock(_g_mutex_peripheral);
	err = drv_gas_sensor_raw_data_get(&dump_gas_selected, &data_gas_raw);
	assert(err == 0);
//	mutex_unlock(_g_mutex_peripheral);

	return data_gas_raw;
}
drv_ccs811_alg_result_t sh_gas_alg_get(void) {

	drv_ccs811_alg_result_t alg_data;

//	mutex_lock(_g_mutex_peripheral);
	alg_data = sh_ccs811_alg_result_data_get();
//	mutex_unlock(_g_mutex_peripheral);

	return alg_data;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t sh_twi_env_data_processing(void) {
	ret_code_t err = NRF_SUCCESS;
	twi433_sensor_data_t env_data;
	for(;;) {
		env_data.pressure = sh_pressure_get();
		env_data.humidity = sh_humidity_get();
		env_data.temperature = sh_temperature_get();
		env_data.gas_alg_data = sh_gas_alg_get();
	}

	return err;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////
//////////                          //////////
//////////  GRID_EYE SPECIFIC APIS  //////////
//////////                          //////////
//////////////////////////////////////////////
static int sh_grid_eye_is_list_queue_empty(grid_eye_list_queue * queue) {

	return queue->count == 0;
}
static void sh_grid_eye_init_list_queue(grid_eye_list_queue * queue) {
	queue->front = queue->rear = NULL;
	queue->count = 0;
}
static void sh_grid_eye_uninit_list_queue(grid_eye_list_queue * queue) {
	grid_eye_list_node * now;
	if(sh_grid_eye_is_list_queue_empty(queue)) {
		return;
	}
	mutex_lock_timedms(_g_mutex_grid_eye, 100);
	do {
		now = queue->front;
		queue->front = now->next;
		free(now);
		now = NULL;
		queue->count--;
	} while(!sh_grid_eye_is_list_queue_empty(queue));
	mutex_unlock(_g_mutex_grid_eye);
}
static void sh_grid_eye_list_queue_enqueue(grid_eye_list_queue * queue, int data) {
	mutex_lock_timedms(_g_mutex_grid_eye, 100);
	grid_eye_list_node * now = (grid_eye_list_node *)malloc(sizeof(grid_eye_list_node));
	if(now == NULL) {
		return;
	} else {
		//printf("malloc grid_eye_list_queue_enqueue");
	}
	mutex_unlock(_g_mutex_grid_eye);
	now->data = data;
	now->next = NULL;
	if( sh_grid_eye_is_list_queue_empty(queue) ) {
		queue->front = now;
	} else {
		queue->rear->next = now;
	}
	queue->rear = now;
	queue->count++;
}
static int sh_grid_eye_list_check_left_right(grid_eye_list_queue * queue) {
	grid_eye_list_node * start;
	grid_eye_list_node * end;
	if( sh_grid_eye_is_list_queue_empty(queue) ) {
		return -1;
	}
	start = queue->front;
	end = queue->rear;

	if((start->data == GRID_EYE_STATE_LEFT) && (end->data == GRID_EYE_STATE_RIGHT)) {
		printf("\r\n");
		printf("==========================================\r\n");
		printf("========  GRID_EYE_LEFT_TO_RIGHT  ========\r\n");
		printf("==========================================\r\n");
		printf("\r\n");
		return GRID_EYE_LEFT_TO_RIGHT;
	} else if((start->data = GRID_EYE_STATE_RIGHT) && (end->data == GRID_EYE_STATE_LEFT)) {
		printf("\r\n");
		printf("==========================================\r\n");
		printf("========  GRID_EYE_RIGHT_TO_LEFT  ========\r\n");
		printf("==========================================\r\n");
		printf("\r\n");
		return GRID_EYE_RIGHT_TO_LEFT;
	} else {
		return GRID_EYE_NOTHING;
	}

	return 0;
}
static int sh_grid_eye_list_check_up_down(grid_eye_list_queue * queue) {
	grid_eye_list_node * start;
	grid_eye_list_node * end;
	if( sh_grid_eye_is_list_queue_empty(queue) ) {
		return -1;
	}
	start = queue->front;
	end = queue->rear;

	if((start->data == GRID_EYE_STATE_UP) && (end->data == GRID_EYE_STATE_DOWN)) {
		printf("\r\n");
		printf("==========================================\r\n");
		printf("========    GRID_EYE_UP_TO_DOWN   ========\r\n");
		printf("==========================================\r\n");
		printf("\r\n");
		return GRID_EYE_UP_TO_DOWN;
	} else if((start->data == GRID_EYE_STATE_DOWN)  && (end->data == GRID_EYE_STATE_UP)) {
		printf("\r\n");
		printf("==========================================\r\n");
		printf("========    GRID_EYE_DOWN_TO_UP   ========\r\n");
		printf("==========================================\r\n");
		printf("\r\n");
		return GRID_EYE_DOWN_TO_UP;
	} else {
		return GRID_EYE_NOTHING;
	}

	return 0;
}

static int qsort_compare(const void * a, const void * b) {
	int num1 = *(int *)a;
	int num2 = *(int *)b;

	if(num1 < num2) {
		return -1;
	} else if(num1 > num2) {
		return 1;
	} else {
		return 0;
	}
	return 0;
}
static uint32_t sh_grid_eye_is_movement_sensed(grid_eye_list_queue * queue_horizon, grid_eye_list_queue * queue_vertical) {
	ret_code_t err = NRF_SUCCESS;

	int t_count = 0;		//threshold count
	int tmp_col[8] = {0,};
	int tmp_row[8] = {0,};

	uint8_t _count = 0;			//check the biggest line to check where the objet is
	uint8_t top_bottom = 0;
	uint8_t left_right = 0;

	for(int i=1; i<GRID_EYE_PIXEL_ARRAY_SIZE; i++) {
		if(grid_eye_pixel_buf_diff[i] >= GRID_EYE_TEMP_DIFF_THRESHOLD) {				//tt.tt * 100 --> 125 is actually 1.25C.
			t_count++;			//get number of threshold count to activate inference
		}
	}

	for(int i=1; i<=GRID_EYE_PIXEL_ARRAY_SIZE; i++) {

		tmp_row[(i-1)/8] += grid_eye_pixel_buf_diff[i-1];

		switch(i%8) {
		case 0:
			tmp_col[7] += grid_eye_pixel_buf_diff[i-1];
			break;
		case 1:
			tmp_col[0] += grid_eye_pixel_buf_diff[i-1];
			break;
		case 2:
			tmp_col[1] += grid_eye_pixel_buf_diff[i-1];
			break;
		case 3:
			tmp_col[2] += grid_eye_pixel_buf_diff[i-1];
			break;
		case 4:
			tmp_col[3] += grid_eye_pixel_buf_diff[i-1];
			break;
		case 5:
			tmp_col[4] += grid_eye_pixel_buf_diff[i-1];
			break;
		case 6:
			tmp_col[5] += grid_eye_pixel_buf_diff[i-1];
			break;
		case 7:
			tmp_col[6] += grid_eye_pixel_buf_diff[i-1];
			break;
		}
	}
	if(t_count >= GRID_EYE_PIXEL_DIFF_COUNT_THRESHOLD) {
		//sorting column to select horizontal hot spot
		int tmp_sorted_col[8];
		memcpy(tmp_sorted_col, tmp_col, sizeof(tmp_sorted_col));
		qsort(tmp_sorted_col, sizeof(tmp_sorted_col) / sizeof(int), sizeof(int), qsort_compare);
		for(int i=0; i<8; i++) {
			if(tmp_col[i] == tmp_sorted_col[7]) {
				left_right = i;
				break;
			}
		}
		//sorting row to select vertical hot spot
		int tmp_sorted_row[8];
		memcpy(tmp_sorted_row, tmp_row, sizeof(tmp_sorted_row));
		qsort(tmp_sorted_row, sizeof(tmp_sorted_row) / sizeof(int), sizeof(int), qsort_compare);
		for(int i=0; i<8; i++) {
			if(tmp_row[i] == tmp_sorted_row[7]) {
				top_bottom = i;
				break;
			}
		}
		switch(left_right) {
		case GRID_EYE_LEFT_4:
			sh_grid_eye_list_queue_enqueue(queue_horizon, GRID_EYE_STATE_LEFT);
			break;

		case GRID_EYE_LEFT_3:
			sh_grid_eye_list_queue_enqueue(queue_horizon, GRID_EYE_STATE_LEFT);
			break;

		case GRID_EYE_LEFT_2:
			sh_grid_eye_list_queue_enqueue(queue_horizon, GRID_EYE_STATE_LEFT);
			break;

		case GRID_EYE_LEFT_1:
			sh_grid_eye_list_queue_enqueue(queue_horizon, GRID_EYE_STATE_LEFT);
			break;

		case GRID_EYE_RIGHT_1:
			sh_grid_eye_list_queue_enqueue(queue_horizon, GRID_EYE_STATE_RIGHT);
			break;

		case GRID_EYE_RIGHT_2:
			sh_grid_eye_list_queue_enqueue(queue_horizon, GRID_EYE_STATE_RIGHT);
			break;

		case GRID_EYE_RIGHT_3:
			sh_grid_eye_list_queue_enqueue(queue_horizon, GRID_EYE_STATE_RIGHT);
			break;

		case GRID_EYE_RIGHT_4:
			sh_grid_eye_list_queue_enqueue(queue_horizon, GRID_EYE_STATE_RIGHT);
			break;

		default:
			break;
		}
		switch(top_bottom) {
		case GRID_EYE_UP_4:
			sh_grid_eye_list_queue_enqueue(queue_vertical, GRID_EYE_STATE_UP);
			break;

		case GRID_EYE_UP_3:
			sh_grid_eye_list_queue_enqueue(queue_vertical, GRID_EYE_STATE_UP);
			break;

		case GRID_EYE_UP_2:
			sh_grid_eye_list_queue_enqueue(queue_vertical, GRID_EYE_STATE_UP);
			break;

		case GRID_EYE_UP_1:
			sh_grid_eye_list_queue_enqueue(queue_vertical, GRID_EYE_STATE_UP);
			break;

		case GRID_EYE_DOWN_1:
			sh_grid_eye_list_queue_enqueue(queue_vertical, GRID_EYE_STATE_DOWN);
			break;

		case GRID_EYE_DOWN_2:
			sh_grid_eye_list_queue_enqueue(queue_vertical, GRID_EYE_STATE_DOWN);
			break;

		case GRID_EYE_DOWN_3:
			sh_grid_eye_list_queue_enqueue(queue_vertical, GRID_EYE_STATE_DOWN);
			break;

		case GRID_EYE_DOWN_4:
			sh_grid_eye_list_queue_enqueue(queue_vertical, GRID_EYE_STATE_DOWN);
			break;

		default:
			break;
		}


	} else {
		return err;
	}

	return err;
}
static int sh_grid_eye_is_still_moving(int buf[GRID_EYE_PIXEL_ARRAY_SIZE]) {
	int re, t_count = 0;
	float avg = 0.0, var = 0.0, stdev = 0.0;
	float diff_from_avg[GRID_EYE_PIXEL_ARRAY_SIZE];

	//get average
	for(int i=0; i<GRID_EYE_PIXEL_ARRAY_SIZE; i++) {
		avg += ((float)buf[i])/100;
	}
	avg /= GRID_EYE_PIXEL_ARRAY_SIZE;
	for(int i=0; i<GRID_EYE_PIXEL_ARRAY_SIZE; i++) {
		diff_from_avg[i] = pow((avg - ((float)buf[i])/100), 2);
	}
	//get variance
	for(int i=0; i<GRID_EYE_PIXEL_ARRAY_SIZE; i++) {
		var += diff_from_avg[i];
	}
	var /= GRID_EYE_PIXEL_ARRAY_SIZE;
	//get stadard deviation
	stdev = sqrt(var);
	if(stdev == 0) {
		return -1;
	}

	int temp_buf[GRID_EYE_PIXEL_ARRAY_SIZE];
	memcpy(temp_buf, buf, sizeof(temp_buf));
	qsort(temp_buf, sizeof(temp_buf) / sizeof(int), sizeof(int), qsort_compare);
	for(int i=GRID_EYE_PIXEL_ARRAY_SIZE-1; i>=0; i--) {
		float std_score = ( ( ( ((float)temp_buf[i])/100) - avg) / stdev);
		if(std_score > GRID_EYE_STDEV_THRESHOLD){		//std_score = (value - avg) / stdev;
			t_count++;
		}
	}
	if(t_count >= GRID_EYE_PIXEL_ABSOL_COUNT_THRESHOLD) {
		re =  GRID_EYE_HEAT_SENSED;
	} else {
		re  = GRID_EYE_NOTHING_HEAT;
	}

	return re;
}

static uint32_t sh_grid_eye_report_result_to_mqtt(int result) {
	ret_code_t err = NRF_SUCCESS;

	switch(result) {
	case GRID_EYE_NOTHING:
		break;

	case GRID_EYE_LEFT_TO_RIGHT:
		//err = uart_wifi_event_send(UART_TWI_GRID_EYE_ALG_RESULT, GRID_EYE_LEFT_TO_RIGHT, NULL);
		break;

	case GRID_EYE_RIGHT_TO_LEFT:
		//err = uart_wifi_event_send(UART_TWI_GRID_EYE_ALG_RESULT, GRID_EYE_RIGHT_TO_LEFT, NULL);
		break;

	case GRID_EYE_UP_TO_DOWN:
		//err = uart_wifi_event_send(UART_TWI_GRID_EYE_ALG_RESULT, GRID_EYE_UP_TO_DOWN, NULL);
		break;

	case GRID_EYE_DOWN_TO_UP:
		//err = uart_wifi_event_send(UART_TWI_GRID_EYE_ALG_RESULT, GRID_EYE_DOWN_TO_UP, NULL);
		break;

	default:
		break;
	}

	return err;
}
static uint32_t sh_grid_eye_report_result_ble(int result) {
	ret_code_t err = NRF_SUCCESS;
	switch(result) {
	case GRID_EYE_NOTHING:
		break;

	case GRID_EYE_LEFT_TO_RIGHT:
		//err = LAP_event_send(SH_GRID_EYE_RESULT, GRID_EYE_LEFT_TO_RIGHT, 0, 0, SH_TWI_TASK_TO_BLE_MSG_LEN, NULL);
		break;

	case GRID_EYE_RIGHT_TO_LEFT:
		//err = LAP_event_send(SH_GRID_EYE_RESULT, GRID_EYE_RIGHT_TO_LEFT, 0, 0, SH_TWI_TASK_TO_BLE_MSG_LEN, NULL);
		break;

	case GRID_EYE_UP_TO_DOWN:
		//err = LAP_event_send(SH_GRID_EYE_RESULT, GRID_EYE_UP_TO_DOWN, 0, 0, SH_TWI_TASK_TO_BLE_MSG_LEN, NULL);
		break;

	case GRID_EYE_DOWN_TO_UP:
		//err = LAP_event_send(SH_GRID_EYE_RESULT, GRID_EYE_DOWN_TO_UP, 0, 0, SH_TWI_TASK_TO_BLE_MSG_LEN, NULL);
		break;

	default:
		break;
	}

	return err;
}

#define SUB_BOARD_POWER_CTRL	NRF_GPIO_PIN_MAP(0,13)

void Smart_Hub_Sub_Board_Power_On() {
	nrf_gpio_cfg_output(SUB_BOARD_POWER_CTRL);
	nrf_gpio_pin_set(SUB_BOARD_POWER_CTRL);
	bsp_busywaitms(500);
}

void Smart_Hub_Sub_Board_Power_Off() {
	nrf_gpio_cfg_output(SUB_BOARD_POWER_CTRL);
	nrf_gpio_pin_clear(SUB_BOARD_POWER_CTRL);
	bsp_busywaitms(500);
}

static void sh_grid_eye_monitor_task(void * arg) {
	ret_code_t err;

	float grid_eye_thermistor;
	int grid_eye_task_active_count = 0;

	int heat_sensed_count = 0;
	int heat_sensed_threshold = 0;
	int result_horzion = 0;
	int result_vertical = 0;

	memset(grid_eye_data_raw, 0, sizeof(grid_eye_data_raw));

	ble_stack_init_wait();
	//uart_wifi_init_wait();
	/*
	while(get_wifi_ready() == false)
	{
		task_sleep(100);
	}
*/

	sh_grid_eye_init_list_queue(&grid_eye_data_list_queue_horizon);
	sh_grid_eye_init_list_queue(&grid_eye_data_list_queue_vertical);

	//Sense heat source
	for(;;) {


//		grid_eye_thermistor = drv_grid_eye_read_thermistor();
//		printf("GRID EYE Thermistor		: [%.2f]\r\n", grid_eye_thermistor);

		if((grid_eye_task_active_count % 2) == 0) {
			err = drv_grid_eye_get_pixels(grid_eye_pixel_buf_first, GRID_EYE_PIXEL_ARRAY_SIZE);
			APP_ERROR_CHECK(err);
			for (int i = 1; i <= GRID_EYE_PIXEL_ARRAY_SIZE; i++) {
				grid_eye_pixel_buf_diff[i-1] = abs((grid_eye_pixel_buf_second[i-1] - grid_eye_pixel_buf_first[i-1]));
			}
		}
		else if((grid_eye_task_active_count % 2) == 1) {
			err = drv_grid_eye_get_pixels(grid_eye_pixel_buf_second, GRID_EYE_PIXEL_ARRAY_SIZE);
			APP_ERROR_CHECK(err);
			for(int i=1; i<=GRID_EYE_PIXEL_ARRAY_SIZE; i++) {
				grid_eye_pixel_buf_diff[i-1] = abs((grid_eye_pixel_buf_first[i-1] - grid_eye_pixel_buf_second[i-1]));
			}
		}

		sh_grid_eye_is_movement_sensed(&grid_eye_data_list_queue_horizon, &grid_eye_data_list_queue_vertical);
		if((grid_eye_task_active_count % 2) == 0) {
			if(sh_grid_eye_is_still_moving(grid_eye_pixel_buf_second)) {	//heat sensing
				heat_sensed_count++;
				heat_sensed_threshold++;
			} else {
				heat_sensed_threshold++;
				if(heat_sensed_threshold - heat_sensed_count >= 18) {		//image processing
					heat_sensed_count = 0;
					heat_sensed_threshold = 0;
					result_horzion = sh_grid_eye_list_check_left_right(&grid_eye_data_list_queue_horizon);
					result_vertical = sh_grid_eye_list_check_up_down(&grid_eye_data_list_queue_vertical);
					err = sh_grid_eye_report_result_to_mqtt(result_horzion);
			//		err = sh_grid_eye_report_result_to_mqtt(result_vertical);

/*
#if(MQTT_CONNECT_TO_SJKANG_HOME == 0)
					err = sh_grid_eye_report_result_ble(result_horzion);
			//		err = sh_grid_eye_report_result_ble(result_vertical);
#endif
*/
					sh_grid_eye_uninit_list_queue(&grid_eye_data_list_queue_horizon);
					sh_grid_eye_uninit_list_queue(&grid_eye_data_list_queue_vertical);
				}

			}
		} else if((grid_eye_task_active_count % 2) == 1) {
			if(sh_grid_eye_is_still_moving(grid_eye_pixel_buf_first)) {		//heat sensing
				heat_sensed_count++;
				heat_sensed_threshold++;
			} else {
				heat_sensed_threshold++;
				if(heat_sensed_threshold - heat_sensed_count >= 18) {		//image processing
					heat_sensed_count = 0;
					heat_sensed_threshold = 0;
					result_horzion = sh_grid_eye_list_check_left_right(&grid_eye_data_list_queue_horizon);
					result_vertical = sh_grid_eye_list_check_up_down(&grid_eye_data_list_queue_vertical);
					err = sh_grid_eye_report_result_to_mqtt(result_horzion);
			//		err = sh_grid_eye_report_result_to_mqtt(result_vertical);
/*
#if(MQTT_CONNECT_TO_SJKANG_HOME == 0)
					err = sh_grid_eye_report_result_ble(result_horzion);
			//		err = sh_grid_eye_report_result_ble(result_vertical);
#endif
*/
					sh_grid_eye_uninit_list_queue(&grid_eye_data_list_queue_horizon);
					sh_grid_eye_uninit_list_queue(&grid_eye_data_list_queue_vertical);
				}
			}
		}

		grid_eye_task_active_count++;
		result_horzion = 0;
		result_vertical = 0;
		if(grid_eye_data_list_queue_horizon.count >= GRID_EYE_QUEUE_MAX_SIZE) {
			sh_grid_eye_uninit_list_queue(&grid_eye_data_list_queue_horizon);
		}
		if(grid_eye_data_list_queue_vertical.count >= GRID_EYE_QUEUE_MAX_SIZE) {
			sh_grid_eye_uninit_list_queue(&grid_eye_data_list_queue_vertical);
		}
		task_sleepms(100);
	}


}


#if(SAAL_HW_DEVICE_TYPE == SAAL_HW_DEVICE_THINGY52)

static ret_code_t ioext_pin_cfg(uint8_t pin, sx_gpio_cfg_t ioext_sys_pin_cfg)
{
    ret_code_t err_code;
    
    err_code = drv_ext_gpio_cfg(pin,
                     ioext_sys_pin_cfg.dir,
                     ioext_sys_pin_cfg.input_buf,
                     ioext_sys_pin_cfg.pull_config,
                     ioext_sys_pin_cfg.drive_type,
                     ioext_sys_pin_cfg.slew_rate);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

static ret_code_t configure_default_ioext_gpio_state(bool boot)
{
    ret_code_t err_code;
    
    sx_gpio_cfg_t  ioext_sys_pin_cfg[SX_IOEXT_NUM_PINS] = IOEXT_SYSTEM_DEFAULT_PIN_CFG;

    /* Set all IO extender pins in default state. IO extender will be powered down as well,
    Hence, this config will not be retained when VDD is turned off. */
    
    // Prior to setting direction, ensure that all ouput data buffers contains the correct value.
    if (boot)
    {
        uint16_t ext_gpio_init_pin_state = 0;
        
        for (uint8_t i = 0; i < SX_IOEXT_NUM_PINS; i++)
        {
            if (ioext_sys_pin_cfg[i].state == PIN_SET)
            {
                ext_gpio_init_pin_state |= (1 << i);
            }
        }
        
        err_code = drv_ext_gpio_reg_data_init(ext_gpio_init_pin_state);
        RETURN_IF_ERROR(err_code);
    }
    
    for (uint8_t i = 0; i < SX_IOEXT_NUM_PINS; i++)
    {
        err_code = ioext_pin_cfg(i, ioext_sys_pin_cfg[i]); 
        RETURN_IF_ERROR(err_code);
    }

    return NRF_SUCCESS;
}

#define PIN_NUM_SX_OSCIO		5

ret_code_t support_func_configure_io_startup(drv_ext_gpio_init_t const * const p_ext_gpio_init)
{
    ret_code_t err_code;
    
	//config SX_OSCIO - disconnect
	nrf_gpio_cfg(PIN_NUM_SX_OSCIO, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
    
    err_code = drv_ext_gpio_init(p_ext_gpio_init, true);
    RETURN_IF_ERROR(err_code);

    err_code = configure_default_ioext_gpio_state(true);
    RETURN_IF_ERROR(err_code);
    
    return NRF_SUCCESS;
}

#define SX1509_ADDR     0x3E

static void board_init(void)
{
    uint32_t            err_code;
    drv_ext_gpio_init_t ext_gpio_init;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = SPH_TWI_SCL,
        .sda                = SPH_TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = TWI_DEFAULT_CONFIG_IRQ_PRIORITY
    };

    static const drv_sx1509_cfg_t sx1509_cfg =
    {
        .twi_addr       = SX1509_ADDR,
        .p_twi_instance = &twi_433,
        .p_twi_cfg      = &twi_config
    };
    ext_gpio_init.p_cfg = &sx1509_cfg;
    
    err_code = support_func_configure_io_startup(&ext_gpio_init);
    APP_ERROR_CHECK(err_code);
    bsp_busywaitms(100);
}
#endif


void evironment_sensor_module_init()
{
	ret_code_t err;

	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
	err = app_timer_init();
	APP_ERROR_CHECK(err);

	nrf_drv_gpiote_init();

#if(SAAL_HW_DEVICE_TYPE == SAAL_HW_DEVICE_SPH)
	Smart_Hub_Sub_Board_Power_On();
#endif

	bsp_busywaitms(1000);
	//TWI instance init
	err = twi_instance_init();
	if( err != 0 ) {
		printf("twi433_init error : %ld\n\r", err);
		assert(err == 0);
	}

#if(SAAL_HW_DEVICE_TYPE == SAAL_HW_DEVICE_THINGY52)
	//Setup THINGY52 VDD & CCS Power
	nrf_gpio_cfg_output(THING_VDD_PWR_CTRL);
	nrf_gpio_cfg_output(CCS_PWR_CTRL);
	nrf_gpio_pin_clear(CCS_PWR_CTRL);
	nrf_gpio_pin_clear(THING_VDD_PWR_CTRL);
	bsp_busywaitms(1000);

	nrf_gpio_pin_set(THING_VDD_PWR_CTRL);
	nrf_gpio_pin_set(CCS_PWR_CTRL);	
	bsp_busywaitms(1000);
	
	board_init();
	
#endif

	task_sleep(1000);
	
	//Sensor Init
	err = sh_environment_init(&twi_433);
	if( err != 0 ) {
		printf("sh_environment sensor init error : %ld\r\n", err);
		assert(err == 0);
	}

#if(SAAL_HW_DEVICE_TYPE == SAAL_HW_DEVICE_SPH)
	err = sh_grid_eye_init(&twi_433);
	if( err != 0 ) {
		printf("sh_grid_eye_sensor init error %ld\r\n", err);
		assert(err == 0);
	}
#endif
	task_sleep(500);

	err = sh_environment_enable();
	if(err != NRF_SUCCESS) {
		printf("sh_environment_enable error : %ld\r\n", err);
        while(1)
        {
            task_sleep(1000);
        }
	}
}
#endif

#endif /*(CSOS_SAAL__USE_LIB_twi_internal_sensors == 1)*/