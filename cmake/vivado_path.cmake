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
# defines the path to the Vivado directory

if(DEFINED ENV{XILINX_VIVADO})
  set(XILINX_VIVADO $ENV{XILINX_VIVADO})
else()
  message("Deprecated. XILINX_VIVADO not defined, defaulting to 'xilinx' directory.")
  set(XILINX_VIVADO ${CMAKE_INSTALL_PREFIX}/../xilinx/vivado)  # <ws>/xilinx/vivado
endif()
