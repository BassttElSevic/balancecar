# ============================================================
#  adc_ab_test —— 复现 GPIO21(ADC1_CH5) 读数卡死的最小诊断工程
#
#  复用 balancecar 仓库根目录下的 components/，所以不需要复制组件。
#  用法:
#      cd diagnostics/adc_ab_test
#      idf.py set-target esp32p4
#      idf.py -p /dev/ttyACM0 flash monitor
# ============================================================
cmake_minimum_required(VERSION 3.16)

# 复用仓库根目录的 components/{board,current,motor,encoder}
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(adc_ab_test)
