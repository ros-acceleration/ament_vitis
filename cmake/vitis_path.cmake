#    ____  ____
#   /   /\/   /
#  /___/  \  /   Copyright (c) 2021, Xilinx®.
#  \   \   \/    Author: Víctor Mayoral Vilches <v.mayoralv@gmail.com>
#   \   \
#   /   /
#  /___/   /\
#  \   \  /  \
#   \___\/\___\
#
# defines the path to the Vitis directory

if(DEFINED ENV{XILINX_VITIS})
  set(XILINX_VITIS $ENV{XILINX_VITIS})
else()
  message("WARNING: Using acceleration but XILINX_VITIS is not defined, acceleration kernels won't be generated.")
  message(STATUS "Defaulting to 'xilinx/vitis' directory.")
  set(XILINX_VITIS ${CMAKE_INSTALL_PREFIX}/../xilinx/vitis)  # <ws>/xilinx/vitis
endif()
