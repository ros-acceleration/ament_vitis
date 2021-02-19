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
# defines the path to the Vitis HLS directory

if(DEFINED ENV{XILINX_HLS})
  set(XILINX_HLS $ENV{XILINX_HLS})
else()
  message("Deprecated. XILINX_HLS not defined, defaulting to 'xilinx' directory.")
  set(XILINX_HLS ${CMAKE_INSTALL_PREFIX}/../xilinx/hls)  # <ws>/xilinx/hls
endif()
