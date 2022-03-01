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
# set the ROS_VITIS variable
#  used to signal the ROS 2 build system that Vitis is available while constructing binaries and/or libraries
#
# NOTE: refer to the ROS_ACCELERATION CMake variable for
# general hardware acceleration expressions.

if(NOT DEFINED ROS_VITIS)
  set(ROS_VITIS TRUE)
endif()
