# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS "main.c" "mpu6050.c" "motors.c" "ultrasonic.c" "wifi_control.c" "flight_controller.c"
    INCLUDE_DIRS "."
)
