# balancecar — ESP32-P4 项目顶层 CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# 引入 ESP-IDF 的 CMake 工具链
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 项目名称（决定生成的固件名：balancecar.elf / balancecar.bin）
project(balancecar)
