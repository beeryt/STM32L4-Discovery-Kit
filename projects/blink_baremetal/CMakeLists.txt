cmake_minimum_required(VERSION 3.3)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/arm-none-eabi.cmake)
project(blink-baremetal C ASM)

# add a target which converts .elf to .hex
add_custom_target(${PROJECT_NAME}.hex
    DEPENDS ${PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    BYPRODUCTS ${PROJECT_NAME}.hex
)

# add a target for flashing .hex using st-link
# TODO

# add bsp for startup and linker script
add_library(bsp OBJECT src/startup_stm32l475vg.s)
target_compile_options(bsp PUBLIC -mcpu=cortex-m4)
target_link_options(bsp PUBLIC -T${CMAKE_SOURCE_DIR}/src/stm32l475vg.ld)

add_executable(${PROJECT_NAME}.elf
    src/main.c
)
target_link_libraries(${PROJECT_NAME}.elf bsp)