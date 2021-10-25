if(INCLUDE__CSOS_SAAL)

    get_filename_component(_tmp_source_dir "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)

    include_directories(${_tmp_source_dir}/CSOS_SAAL)
    include_directories(${_tmp_source_dir}/CSOS_SAAL/lib_433_comm)
    include_directories(${_tmp_source_dir}/CSOS_SAAL/lib_bluetooth_csos)
    include_directories(${_tmp_source_dir}/CSOS_SAAL/lib_wifi_wizfi360)
    include_directories(${_tmp_source_dir}/CSOS_SAAL/lib_twi_internal_sensors)
    include_directories(${_tmp_source_dir}/CSOS_SAAL/lib_twi_internal_sensors/twi_lib)
    include_directories(${_tmp_source_dir}/CSOS_SAAL/lib_twi_internal_sensors/twi_lib/include/drv)
    include_directories(${_tmp_source_dir}/CSOS_SAAL/lib_twi_internal_sensors/twi_lib/include/macros)
    include_directories(${_tmp_source_dir}/CSOS_SAAL/lib_twi_internal_sensors/twi_lib/include/util)

    file(GLOB_RECURSE _tmpsrc
        "${_tmp_source_dir}/*.c"
        "${_tmp_source_dir}/*.cpp"
        "${_tmp_source_dir}/*.S"
        "${_tmp_source_dir}/*.s")

    set(PROJECT_SOURCES ${PROJECT_SOURCES} ${_tmpsrc})

endif(INCLUDE__CSOS_SAAL)


