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
# defines the path to the v++ Vitis compiler

if(DEFINED ENV{XILINX_VITIS})
    set(VPP_PATH $ENV{XILINX_VITIS}/bin/v++)
else()  # assume using "xilinx_vitis" package
  message("Deprecated. XILINX_VITIS not defined, defaulting to 'xilinx' directory.")
  set(VITIS_DIR ${CMAKE_INSTALL_PREFIX}/../xilinx/vitis)  # <ws>/xilinx/vitis
  set(VPP_PATH ${VITIS_DIR}/bin/v++)
endif()
