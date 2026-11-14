cmake_minimum_required(VERSION 3.16)

project(ecan_igh_demo C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
# Firmware source delivery omits all test sources; standalone demo includes them.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    include(CTest)
elseif(BUILD_TESTING)
    message(FATAL_ERROR "This source package omits tests. Set BUILD_TESTING=OFF or use the standalone IgH demo package.")
else()
    set(BUILD_TESTING OFF)
endif()

set(ECAN_IGH_MCAN_CHANNEL_COUNT "4" CACHE STRING
    "MCAN channel count of the target product (4 for ECAN-Lite, 8 for an HPM6E 8-channel product)")
set_property(CACHE ECAN_IGH_MCAN_CHANNEL_COUNT PROPERTY STRINGS 4 8)
if(NOT ECAN_IGH_MCAN_CHANNEL_COUNT STREQUAL "4" AND
   NOT ECAN_IGH_MCAN_CHANNEL_COUNT STREQUAL "8")
    message(FATAL_ERROR "ECAN_IGH_MCAN_CHANNEL_COUNT must be 4 or 8")
endif()
add_compile_definitions(IGH_MCAN_CHANNEL_COUNT=${ECAN_IGH_MCAN_CHANNEL_COUNT})

option(ECAN_IGH_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
if(MSVC)
    add_compile_options(/W4 /utf-8)
    if(ECAN_IGH_WARNINGS_AS_ERRORS)
        add_compile_options(/WX)
    endif()
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(ECAN_IGH_WARNINGS_AS_ERRORS)
        # add_compile_options(-Werror)
    endif()
endif()

set(ECAN_IGH_BUILD_DEMO "AUTO" CACHE STRING "Build the IgH EtherCAT demos: AUTO, ON, or OFF")
set_property(CACHE ECAN_IGH_BUILD_DEMO PROPERTY STRINGS AUTO ON OFF)
if(DEFINED ECAN_TOOLS_BUILD_IGH)
    set(ECAN_IGH_BUILD_DEMO ${ECAN_TOOLS_BUILD_IGH})
endif()
string(TOUPPER "${ECAN_IGH_BUILD_DEMO}" ECAN_IGH_BUILD_DEMO)
if(ECAN_IGH_BUILD_DEMO STREQUAL "TRUE")
    set(ECAN_IGH_BUILD_DEMO ON)
elseif(ECAN_IGH_BUILD_DEMO STREQUAL "FALSE")
    set(ECAN_IGH_BUILD_DEMO OFF)
endif()
if(NOT ECAN_IGH_BUILD_DEMO STREQUAL "AUTO" AND
   NOT ECAN_IGH_BUILD_DEMO STREQUAL "ON" AND
   NOT ECAN_IGH_BUILD_DEMO STREQUAL "OFF")
    message(FATAL_ERROR "ECAN_IGH_BUILD_DEMO must be AUTO, ON, or OFF")
endif()

if(NOT ECAN_IGH_BUILD_DEMO STREQUAL "OFF")
    find_package(Threads REQUIRED)
    set(ECRT_ROOT "" CACHE PATH "Optional IgH EtherCAT source or install root")
    set(_ECRT_INCLUDE_HINTS
        "${ECRT_ROOT}/include"
        "${ECRT_ROOT}/ethercat/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../ethercat/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../ethercat-install/include"
        "$ENV{ECRT_ROOT}/include"
        "$ENV{ECRT_ROOT}/ethercat/include"
    )
    set(_ECRT_LIBRARY_HINTS
        "${ECRT_ROOT}/lib"
        "${ECRT_ROOT}/lib/.libs"
        "${ECRT_ROOT}/ethercat/lib"
        "${ECRT_ROOT}/ethercat/lib/.libs"
        "${ECRT_ROOT}/ethercat-install/lib"
        "${CMAKE_CURRENT_SOURCE_DIR}/../ethercat/lib"
        "${CMAKE_CURRENT_SOURCE_DIR}/../ethercat/lib/.libs"
        "${CMAKE_CURRENT_SOURCE_DIR}/../ethercat-install/lib"
        "$ENV{ECRT_ROOT}/lib"
        "$ENV{ECRT_ROOT}/lib/.libs"
        "$ENV{ECRT_ROOT}/ethercat/lib"
        "$ENV{ECRT_ROOT}/ethercat/lib/.libs"
    )

    find_path(ECRT_INCLUDE_DIR NAMES ecrt.h HINTS ${_ECRT_INCLUDE_HINTS})
    find_library(ECRT_LIBRARY NAMES ethercat libethercat HINTS ${_ECRT_LIBRARY_HINTS})

    if(NOT ECRT_INCLUDE_DIR OR NOT ECRT_LIBRARY)
        if(ECAN_IGH_BUILD_DEMO STREQUAL "ON")
            message(FATAL_ERROR
                "IgH EtherCAT headers/library not found. Install ethercat-dev, set ECRT_ROOT, "
                "or configure ECRT_INCLUDE_DIR/ECRT_LIBRARY. Use -DECAN_IGH_BUILD_DEMO=OFF "
                "to skip IgH demo targets.")
        endif()
        message(STATUS
            "IgH EtherCAT headers/library not found; skipping IgH/MCAN/CiA402 demo targets. "
            "Set ECRT_ROOT or ECRT_INCLUDE_DIR/ECRT_LIBRARY to build demos.")
        set(ECAN_IGH_BUILD_DEMO OFF)
    else()
        message(STATUS "IgH EtherCAT include: ${ECRT_INCLUDE_DIR}")
        message(STATUS "IgH EtherCAT library: ${ECRT_LIBRARY}")
    endif()
endif()

add_library(ecan_igh_mcan_pdo_codec STATIC
    src/igh_mcan_pdo_common.c
    src/igh_mcan_pdo_parser.c
    src/igh_mcan_pdo_writer.c
)
target_include_directories(ecan_igh_mcan_pdo_codec PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(igh_example_mit_protocol STATIC
    examples/mit/mit_canfd_protocol.c
)
target_include_directories(igh_example_mit_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/mit
)
if(UNIX)
    target_link_libraries(igh_example_mit_protocol PUBLIC m)
endif()

add_library(dm_motor_protocol STATIC
    dm_motor/dm_motor_protocol.c
)
target_include_directories(dm_motor_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/dm_motor
)
if(UNIX)
    target_link_libraries(dm_motor_protocol PUBLIC m)
endif()

if(BUILD_TESTING)
    add_executable(igh_pdo_mutation_test tests/pdo_mutation_test.c)
    target_link_libraries(igh_pdo_mutation_test PRIVATE ecan_igh_mcan_pdo_codec)
    add_test(NAME igh_pdo_mutation COMMAND igh_pdo_mutation_test)
    add_executable(igh_example_args_test tests/example_args_test.c)
    add_test(NAME igh_example_args COMMAND igh_example_args_test)
    foreach(case IN ITEMS "u8;256" "u8;-1" "u32;4294967296" "i32;2147483648"
                          "float;nan" "float;inf" "u8;3junk" "u32;08")
        list(GET case 0 kind)
        list(GET case 1 value)
        string(MAKE_C_IDENTIFIER "args_reject_${kind}_${value}" test_name)
        add_test(NAME ${test_name} COMMAND igh_example_args_test ${kind} ${value})
        set_tests_properties(${test_name} PROPERTIES WILL_FAIL TRUE)
    endforeach()

    add_executable(igh_mcan_pdo_codec_test
        tests/mcan_pdo_codec_test.c
    )
    target_link_libraries(igh_mcan_pdo_codec_test PRIVATE
        ecan_igh_mcan_pdo_codec
    )
    add_test(NAME igh_mcan_pdo_codec COMMAND igh_mcan_pdo_codec_test)

    add_executable(igh_mit_canfd_protocol_test
        examples/mit/mit_canfd_protocol_test.c
    )
    target_link_libraries(igh_mit_canfd_protocol_test PRIVATE
        igh_example_mit_protocol
    )
    add_test(NAME igh_mit_canfd_protocol COMMAND igh_mit_canfd_protocol_test)

    add_executable(dm_motor_protocol_test
        dm_motor/dm_motor_protocol_test.c
    )
    target_link_libraries(dm_motor_protocol_test PRIVATE dm_motor_protocol)
    add_test(NAME dm_motor_protocol COMMAND dm_motor_protocol_test)

    if(UNIX)
        add_executable(igh_rt_support_test
            tests/rt_support_test.c
        )
        target_include_directories(igh_rt_support_test PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
        add_test(NAME igh_rt_support COMMAND igh_rt_support_test)
    endif()
endif()

if(ECAN_IGH_BUILD_DEMO STREQUAL "ON" OR ECAN_IGH_BUILD_DEMO STREQUAL "AUTO")
    add_library(ecan_igh_adapter STATIC
        src/igh_adapter_config.c
        src/igh_adapter_pdo.c
        src/igh_adapter.c
    )
    target_include_directories(ecan_igh_adapter PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${ECRT_INCLUDE_DIR}
    )
    target_link_libraries(ecan_igh_adapter PUBLIC
        ${ECRT_LIBRARY}
        Threads::Threads
    )

    add_library(ecan_igh_mcan_port STATIC
        src/igh_mcan_config.c
        src/igh_mcan_port.c
        src/igh_mcan_stop.c
    )

    target_link_libraries(ecan_igh_mcan_port PUBLIC
        ecan_igh_mcan_pdo_codec
        ecan_igh_adapter
    )

    add_library(dm_motor STATIC
        dm_motor/dm_motor.c
    )
    target_include_directories(dm_motor PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/dm_motor
    )
    target_link_libraries(dm_motor PUBLIC
        dm_motor_protocol
        ecan_igh_mcan_port
    )

    add_library(dm_robot STATIC
        dm_motor/dm_robot.c
    )
    target_include_directories(dm_robot PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/dm_motor
    )
    target_link_libraries(dm_robot PUBLIC
        dm_motor_protocol
        ecan_igh_mcan_port
    )

    add_executable(dm_motor_demo
        dm_motor/dm_motor_demo.c
    )
    set_target_properties(dm_motor_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_include_directories(dm_motor_demo PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/dm_motor
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/common
    )
    target_link_libraries(dm_motor_demo PRIVATE dm_motor)

    add_executable(dm_robot_usb2can_test
        dm_motor/dm_robot_usb2can_test.c
    )
    target_include_directories(dm_robot_usb2can_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/dm_motor
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/common
    )
    target_link_libraries(dm_robot_usb2can_test PRIVATE dm_robot)

    add_executable(dm_robot_motor_test dm_motor/dm_robot_motor_test.c)
    set_target_properties(dm_robot_motor_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_include_directories(dm_robot_motor_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/common
    )
    target_link_libraries(dm_robot_motor_test PRIVATE dm_robot)

    if(BUILD_TESTING AND UNIX)
        add_executable(igh_mcan_stop_test tests/mcan_stop_test.c)
        target_include_directories(igh_mcan_stop_test PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/../include ${ECRT_INCLUDE_DIR})
        target_link_libraries(igh_mcan_stop_test PRIVATE ecan_igh_mcan_pdo_codec Threads::Threads)
        add_test(NAME igh_mcan_stop COMMAND igh_mcan_stop_test)
    endif()

    if(BUILD_TESTING)
        add_executable(igh_example_stop_test tests/example_stop_test.c)
        target_include_directories(igh_example_stop_test PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../include
            ${ECRT_INCLUDE_DIR})
        target_link_libraries(igh_example_stop_test PRIVATE Threads::Threads)
        add_test(NAME igh_example_stop COMMAND igh_example_stop_test)
        add_executable(igh_rx_dump_completion_test tests/rx_dump_completion_test.c)
        target_link_libraries(igh_rx_dump_completion_test PRIVATE ecan_igh_mcan_port)
        foreach(case RANGE 0 9)
            add_test(NAME igh_rx_dump_completion_${case} COMMAND igh_rx_dump_completion_test ${case})
        endforeach()
        add_executable(igh_adapter_config_test
            tests/adapter_config_test.c
        )
        target_link_libraries(igh_adapter_config_test PRIVATE
            ecan_igh_mcan_port
        )
        add_test(NAME igh_adapter_config COMMAND igh_adapter_config_test)
    endif()

    add_library(ecan_igh_250us_pipeline STATIC
        src/igh_250us_pipeline.c
    )
    target_link_libraries(ecan_igh_250us_pipeline PUBLIC
        ecan_igh_mcan_port
    )

    add_library(igh_example_cia402_protocol STATIC
        examples/cia402/igh_cia402_protocol.c
    )
    target_include_directories(igh_example_cia402_protocol PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/cia402
    )

    add_executable(igh_cia402_10axis_demo
        examples/cia402/cia402_10axis_demo.c
    )
    set_target_properties(igh_cia402_10axis_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_cia402_10axis_demo PRIVATE
        igh_example_cia402_protocol
        ecan_igh_mcan_port
    )

    add_executable(igh_simple_loop
        examples/basic/simple_loop.c
    )
    set_target_properties(igh_simple_loop PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_simple_loop PRIVATE
        ecan_igh_mcan_port
    )

    add_executable(igh_raw_pdo_dump
        examples/basic/raw_pdo_dump.c
    )
    set_target_properties(igh_raw_pdo_dump PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_raw_pdo_dump PRIVATE
        ecan_igh_adapter
    )

    add_executable(igh_mcan_multi_channel_demo
        examples/mcan/mcan_multi_channel_demo.c
    )
    set_target_properties(igh_mcan_multi_channel_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_mcan_multi_channel_demo PRIVATE
        ecan_igh_mcan_port
    )

    add_executable(igh_mcan_rx_dump
        examples/mcan/mcan_rx_dump.c
    )
    set_target_properties(igh_mcan_rx_dump PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_mcan_rx_dump PRIVATE
        ecan_igh_mcan_port
    )

    add_executable(igh_cia402_single_axis_demo
        examples/cia402/cia402_single_axis_demo.c
    )
    set_target_properties(igh_cia402_single_axis_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_cia402_single_axis_demo PRIVATE
        igh_example_cia402_protocol
        ecan_igh_mcan_port
    )

    add_executable(igh_cia402_single_axis_sdo_demo
        examples/cia402/cia402_single_axis_demo.c
    )
    set_target_properties(igh_cia402_single_axis_sdo_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_compile_definitions(igh_cia402_single_axis_sdo_demo PRIVATE
        CIA402_SINGLE_AXIS_SDO_DEMO=1
    )
    target_link_libraries(igh_cia402_single_axis_sdo_demo PRIVATE
        igh_example_cia402_protocol
        ecan_igh_mcan_port
    )

    add_executable(igh_cia402_position_watch_demo
        examples/cia402/cia402_position_watch_demo.c
    )
    set_target_properties(igh_cia402_position_watch_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_cia402_position_watch_demo PRIVATE
        igh_example_cia402_protocol
        ecan_igh_mcan_port
    )

    add_executable(igh_canopen_sdo_probe
        examples/canopen/canopen_sdo_probe.c
    )
    set_target_properties(igh_canopen_sdo_probe PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_canopen_sdo_probe PRIVATE
        igh_example_cia402_protocol
        ecan_igh_mcan_port
    )

    add_executable(igh_canopen_heartbeat_probe
        examples/canopen/canopen_heartbeat_probe.c
    )
    set_target_properties(igh_canopen_heartbeat_probe PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_canopen_heartbeat_probe PRIVATE
        igh_example_cia402_protocol
        ecan_igh_mcan_port
    )

    add_executable(igh_mcan_gateway_tx_probe
        examples/mcan/mcan_gateway_tx_probe.c
    )
    set_target_properties(igh_mcan_gateway_tx_probe PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_mcan_gateway_tx_probe PRIVATE
        ecan_igh_mcan_port
    )

    add_executable(igh_mit_canfd_motor_demo
        examples/mit/mit_canfd_motor_demo.c
    )
    set_target_properties(igh_mit_canfd_motor_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_mit_canfd_motor_demo PRIVATE
        igh_example_mit_protocol
        ecan_igh_mcan_port
    )

    add_executable(igh_250us_four_phase_demo
        examples/pipeline/four_phase_250us_demo.c
    )
    set_target_properties(igh_250us_four_phase_demo PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    target_link_libraries(igh_250us_four_phase_demo PRIVATE
        ecan_igh_250us_pipeline
    )

endif()
