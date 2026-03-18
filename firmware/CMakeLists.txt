cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/board)
set(PICO_BOARD stimjim_rp2354b CACHE STRING "Board type")
add_compile_options(-fstack-usage)

include(pico_sdk_import.cmake)
project(main C CXX ASM)
pico_sdk_init()

add_executable(main
    main.c
    pio_spi.c
    stimjim_context.c
    core1.c
    )

pico_enable_stdio_usb(main 1)

target_link_libraries(main pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pio pico_divider)
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
if (PICO_CYW43_SUPPORTED)
    target_link_libraries(main pico_cyw43_arch_none)
endif()

pico_add_extra_outputs(main)

