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
# include vitis extras

find_package(ament_cmake_core QUIET REQUIRED)

# various functions / macros
foreach(filename
  "vitis_acceleration_kernel"
  "vpp_path"
  "vitis_path"
  "vivado_path"
  "hls_path"
  "platform_repo_paths"
  "firmware_data_path"
)
  include(${ament_vitis_DIR}/${filename}.cmake)
endforeach()
