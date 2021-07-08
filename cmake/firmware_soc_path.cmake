#    ____  ____
#   /   /\/   /
#  /___/  \  /   Copyright (c) 2021, Xilinx®.
#  \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
#   \   \
#   /   /
#  /___/   /\
#  \   \  /  \
#   \___\/\___\
#
# defines the path to currently deployed firmware System-on-Chip (SoC), which
# should be written in a file under <ws>/acceleration/firmware/xilinx/SOC
#
# This value is used to make the vitis_hls_generate_tcl macro SOC-agnostic and
# purely dependant on the firmware deployed in the ROS 2 workspace.

set(FIRMWARE_SOC_PATH ${CMAKE_INSTALL_PREFIX}/../acceleration/firmware/xilinx/SOC)
      # e.g. <ws>/acceleration/firmware/xilinx/SOC

file (STRINGS ${FIRMWARE_SOC_PATH} FIRMWARE_SOC)
      # e.g. xcvu9p-flga2104-2-i