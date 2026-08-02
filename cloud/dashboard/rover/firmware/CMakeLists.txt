# FPMS rover firmware — Yahboom MicroROS Board V2.0 (ESP32-S3)
#
# Requires ESP-IDF v5.2 or newer, and the micro-ROS ESP-IDF component
# cloned into components/micro_ros_espidf_component (see README).

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(fpms_rover_firmware)
