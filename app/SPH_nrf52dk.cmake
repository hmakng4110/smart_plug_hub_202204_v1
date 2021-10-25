# ubinos_config_info {"name_base": "CSOS_SAAL", "build_type": "cmake_ubinos", "app": true}

set_cache(UBINOS__BSP__LINK_MEMMAP_RAM_ORIGIN 0x20003400 STRING)
set_cache(UBINOS__BSP__LINK_MEMMAP_RAM_LENGTH 0x0000CC00 STRING)

set_cache(NRF5SDK__SWI_DISABLE0 TRUE BOOL)

include(${PROJECT_UBINOS_DIR}/config/ubinos_nrf52dk_softdevice.cmake)
include(${PROJECT_LIBRARY_DIR}/seggerrtt_wrapper/config/seggerrtt.cmake)
include(${PROJECT_LIBRARY_DIR}/nrf5sdk_wrapper/config/nrf5sdk.cmake)
include(${PROJECT_LIBRARY_DIR}/nrf5sdk_extension/config/nrf5sdk_extension.cmake)

include(${PROJECT_LIBRARY_DIR}/CSOS_SAAL/config/csos_saal.cmake)

####

set(INCLUDE__APP TRUE)
set(APP__NAME "SPH")

get_filename_component(_tmp_source_dir "${CMAKE_CURRENT_LIST_DIR}/${APP__NAME}" ABSOLUTE)
string(TOLOWER ${UBINOS__BSP__BOARD_MODEL} _temp_board_model)
string(TOLOWER ${UBINOS__BSP__NRF52_SOFTDEVICE_NAME} _temp_softdevice_name)

include_directories(${_tmp_source_dir}/arch/arm/cortexm/${_temp_board_model}/${_temp_softdevice_name}/config)
include_directories(${_tmp_source_dir}/arch/arm/cortexm/${_temp_board_model})
include_directories(${_tmp_source_dir})

include_directories(${_tmp_source_dir}/config)
include_directories(${_tmp_source_dir}/lib_433_comm)
include_directories(${_tmp_source_dir}/lib_bluetooth_csos)
include_directories(${_tmp_source_dir}/lib_wifi_wizfi360)
include_directories(${_tmp_source_dir}/lib_twi_internal_sensors)
include_directories(${_tmp_source_dir}/lib_twi_internal_sensors/twi_lib)
include_directories(${_tmp_source_dir}/lib_twi_internal_sensors/twi_lib/include/drv)
include_directories(${_tmp_source_dir}/lib_twi_internal_sensors/twi_lib/include/macros)
include_directories(${_tmp_source_dir}/lib_twi_internal_sensors/twi_lib/include/util)

file(GLOB_RECURSE _tmp_sources
    "${_tmp_source_dir}/*.c"
    "${_tmp_source_dir}/*.cpp"
    "${_tmp_source_dir}/*.S"
    "${_tmp_source_dir}/*.s")

set(PROJECT_APP_SOURCES ${PROJECT_APP_SOURCES} ${_tmp_sources})

