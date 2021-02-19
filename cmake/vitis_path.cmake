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
  message("Deprecated. XILINX_VITIS not defined, defaulting to 'xilinx' directory.")
  set(XILINX_VITIS ${CMAKE_INSTALL_PREFIX}/../xilinx/vitis)  # <ws>/xilinx/vitis
endif()
