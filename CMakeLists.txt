# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(pico_flexray C CXX ASM)

pico_sdk_init()

option(LINK_PICO_CAN_STUB "Link minimal CAN stub object" OFF)
set(PICO_CAN_SRC src/can_bus.c)
if(LINK_PICO_CAN_STUB)
  set(PICO_CAN_SRC src/can_bus_stub.c)
endif()

add_executable(pico_flexray
     src/main.c
     src/replay_frame.c
     src/flexray_frame.c
     src/panda_usb.c
     src/usb_descriptors.c
     src/flexray_bss_streamer.c
     src/flexray_fowarder_with_injector.c
     src/flexray_fifo.c
     ${PICO_CAN_SRC}
     )

pico_set_program_name(pico_flexray "pico_flexray")
pico_set_program_version(pico_flexray "0.1")

pico_generate_pio_header(pico_flexray 
    ${CMAKE_CURRENT_LIST_DIR}/src/flexray_bss_streamer.pio
    ${CMAKE_CURRENT_LIST_DIR}/src/flexray_replay_q8_frame.pio
    ${CMAKE_CURRENT_LIST_DIR}/src/flexray_forwarder_with_injector.pio
    )

pico_enable_stdio_uart(pico_flexray 1)

target_link_libraries(pico_flexray
        hardware_spi
        hardware_gpio
        pico_stdlib
        tinyusb_device
        pico_multicore
        hardware_dma
        hardware_pio
        hardware_interp
        pico_unique_id
        )

target_include_directories(pico_flexray PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_options(pico_flexray PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra -Wstrict-prototypes -Werror>
)

pico_add_extra_outputs(pico_flexray)
