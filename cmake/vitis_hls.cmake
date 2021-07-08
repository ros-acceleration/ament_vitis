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
# Vitis HLS macros to integrate High-Level Synthesis (HLS) 
# capabilities into the  ROS ament build system
#
# See https://www.xilinx.com/html_docs/xilinx2020_2/vitis_doc/gnq1597858079367.html
# for more on Vitis HLS

#
# Generate a Vitis Tcl script from CMake
#
# .. note:: this macro will only command CMake to generate
# the corresponding Tcl script according to the parameters
# designed below. To execute such script (e.g. via 
# "vitis_hls -f <tcl-script>"), refer to ROS 2 colcon extensions.
#
# :param PROJECT: Name of the Vitis HLS project to be created
# :type target: string
#
# :param TOPFUNCTION: Top-level function of the acceleration kernel
# e.g. vadd
# :type target: string
#
# :param SRC: sources for the kernel can be various. Relative paths.
# :type target: string
#
# :param TESTBENCH: source file for the testbench. Relative path.
# :type target: string
#
# :param CLOCK: Clock period in nanoseconds (e.g. 4)
# :type target: string
#
# :param HEADERS: one or muliple relative path directories whereto search for
# header files.
# :type target: string
#
# :param SYNTHESIS (optional): Run synthesis
# :type target: bool
#
# :param RTLSIMULATION (optional): Run RTL Simulation
# :type target: bool
#
# :param RTLIMPLEMENTATION (optional): Run RTL implementation
# :type target: bool
#
macro(vitis_hls_generate_tcl)
    set(options SYNTHESIS RTLSIMULATION RTLIMPLEMENTATION)
    set(oneValueArgs PROJECT TOPFUNCTION TESTBENCH CLOCK)
    set(multiValueArgs SRC HEADERS)

    cmake_parse_arguments(VITIS_HLS "${options}" "${oneValueArgs}"
                          "${multiValueArgs}" ${ARGN} )

    # if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" AND DEFINED CMAKE_SYSROOT})
    if (DEFINED CMAKE_SYSROOT)  # cross compilation, DEFINED CMAKE_SYSROOT
      
      # define template, with or without include directories      
      if(DEFINED VITIS_HLS_HEADERS)
        # convert to "value1 value2"
        foreach(VITIS_HLS_HEADERS_file ${VITIS_HLS_HEADERS})
          set(VITIS_HLS_HEADERS_VALUE "${VITIS_HLS_HEADERS_VALUE} ${CMAKE_SOURCE_DIR}/${VITIS_HLS_HEADERS_file}")
        endforeach()

        FILE(WRITE ${CMAKE_BINARY_DIR}/hls.tcl.in
        "open_project -reset @VITIS_HLS_PROJECT_VALUE@\n"
        "add_files @VITIS_HLS_SRC_VALUE@\n"
        "add_files -tb @VITIS_HLS_TESTBENCH_VALUE@ -cflags \"-isystem @VITIS_HLS_HEADERS_VALUE@\"\n"
        "set_top @VITIS_HLS_TOPFUNCTION_VALUE@\n"
        "open_solution default\n"  # default solution
        "set_part {${FIRMWARE_SOC}}\n"
        "create_clock -period @VITIS_HLS_CLOCK_VALUE@\n"
        "csim_design -ldflags \"-lOpenCL\" -profile\n"
      )
      else()
        FILE(WRITE ${CMAKE_BINARY_DIR}/hls.tcl.in
          "open_project -reset @VITIS_HLS_PROJECT_VALUE@\n"
          "add_files @VITIS_HLS_SRC_VALUE@\n"
          "add_files -tb @VITIS_HLS_TESTBENCH_VALUE@\n"
          "set_top @VITIS_HLS_TOPFUNCTION_VALUE@\n"
          "open_solution default\n"  # default solution
          "set_part {${FIRMWARE_SOC}}\n"
          "create_clock -period @VITIS_HLS_CLOCK_VALUE@\n"
          "csim_design -ldflags \"-lOpenCL\" -profile\n"
        )
      endif()  # VITIS_HLS_HEADERS, add include directories      

      if (${VITIS_HLS_SYNTHESIS})
        FILE(APPEND ${CMAKE_BINARY_DIR}/hls.tcl.in
          "csynth_design\n"
        )
      endif()  # VITIS_HLS_SYNTHESIS

      if (${VITIS_HLS_RTLSIMULATION})
        FILE(APPEND ${CMAKE_BINARY_DIR}/hls.tcl.in
          "cosim_design\n"
        )
      endif()  # VITIS_HLS_RTLSIMULATION

      if (${VITIS_HLS_RTLIMPLEMENTATION})
        FILE(APPEND ${CMAKE_BINARY_DIR}/hls.tcl.in
          "export_design\n"
        )
      endif()  # VITIS_HLS_RTLIMPLEMENTATION

      # exit at the end
      FILE(APPEND ${CMAKE_BINARY_DIR}/hls.tcl.in
      "exit\n"
      )

      # set placeholders
      set(VITIS_HLS_PROJECT_VALUE ${VITIS_HLS_PROJECT})
      # set(VITIS_HLS_SRC_VALUE ${VITIS_HLS_SRC})  # this gets a list as "value1;value2"
      # we need instead: "value1 value2"
      foreach(VITIS_HLS_SRC_file ${VITIS_HLS_SRC})
        set(VITIS_HLS_SRC_VALUE "${VITIS_HLS_SRC_VALUE} ${CMAKE_SOURCE_DIR}/${VITIS_HLS_SRC_file}")
      endforeach()
      set(VITIS_HLS_TESTBENCH_VALUE ${CMAKE_SOURCE_DIR}/${VITIS_HLS_TESTBENCH})
      set(VITIS_HLS_TOPFUNCTION_VALUE ${VITIS_HLS_TOPFUNCTION})
      set(VITIS_HLS_CLOCK_VALUE ${VITIS_HLS_CLOCK})
      
      # replace placeholders and produce final Tcl script
      configure_file(${CMAKE_BINARY_DIR}/hls.tcl.in ${CMAKE_BINARY_DIR}/hls.tcl)

      # install Tcl script
      install(
        FILES
          ${CMAKE_BINARY_DIR}/hls.tcl
        DESTINATION
          lib/${PROJECT_NAME}
      )
    endif()  # cross compilation, DEFINED CMAKE_SYSROOT
endmacro()  # vitis_acceleration_kernel
