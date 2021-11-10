set(INCLUDE__CSOS_SAAL TRUE)
set(PROJECT_UBINOS_LIBRARIES ${PROJECT_UBINOS_LIBRARIES} CSOS_SAAL)

set_cache_default(CSOS_SAAL__USE_LIB_433 FALSE BOOL "Use 433 library")
set_cache_default(CSOS_SAAL__USE_LIB_bluetooth_csos FALSE BOOL "Use bluetooth_csos library")
set_cache_default(CSOS_SAAL__USE_LIB_twi_internal_sensors FALSE BOOL "Use twi_internal_sensors library")
set_cache_default(CSOS_SAAL__USE_LIB_wifi_wizfi360 FALSE BOOL "Use wifi_wizfi360 library")
