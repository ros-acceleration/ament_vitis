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
# base macros and functions to create acceleration kernels

#
# Build acceleration kernels main macro
#
# .. note:: call sub-macros or sub-functions to build, link and/or
# package, as appropriate.
#
# .. note:: kernels will be generated with their corresponding ".xclbin"
# format. A signature of the build type will be appended at the end except
# for the "hardware" build type which includes no signature.
# E.g. "vadd.xclbin.hw_emu" or "vadd.xclbin".
#
# :param NAME: Name of the kernel to compile and link
# :type target: string
#
# :param FILE: CMAKE_SOURCE_DIR relative path to source code of the kernel
# :type target: string
#
# :param CONFIG: CMAKE_SOURCE_DIR relative path to configuration file
# for the kernel e.g. src/zcu102.cfg.
# :type target: string
#
# :param TYPE: type of kernel to generate, could be "sw_emu",
# "hw_emu" or "hardware" (or several of them)
# :type target: string
#
# :param INCLUDE: one or muliple relative path directories whereto search for
# header files. This option is passed to the OpenCL preprocessor. Directories
# are relative to CMAKE_SOURCE_DIR.
# :type target: string
#
# :param PACKAGE: Invoke the -p (package) flag of v++ to generate
# the corresponding artifacts for emulation.
# :type target: bool
#
macro(vitis_acceleration_kernel)
    set(options PACKAGE)
    set(oneValueArgs NAME FILE CONFIG)
    set(multiValueArgs TYPE INCLUDE)

    cmake_parse_arguments(VITIS_KERNEL "${options}" "${oneValueArgs}"
                          "${multiValueArgs}" ${ARGN} )

    # if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" AND DEFINED CMAKE_SYSROOT})
    if (DEFINED CMAKE_SYSROOT)
      # debug()
      foreach(type ${VITIS_KERNEL_TYPE})
        if (${type} STREQUAL "sw_emu")
          vitis_acceleration_kernel_aux(
            NAME ${VITIS_KERNEL_NAME}
            FILE ${VITIS_KERNEL_FILE}
            TYPE "sw_emu"
            CONFIG ${VITIS_KERNEL_CONFIG}
            INCLUDE ${VITIS_KERNEL_INCLUDE}
            PACKAGE ${VITIS_KERNEL_PACKAGE}
          )
        elseif (${type} STREQUAL "hw_emu")
          vitis_acceleration_kernel_aux(
            NAME ${VITIS_KERNEL_NAME}
            FILE ${VITIS_KERNEL_FILE}
            TYPE "hw_emu"
            CONFIG ${VITIS_KERNEL_CONFIG}
            INCLUDE ${VITIS_KERNEL_INCLUDE}
            PACKAGE ${VITIS_KERNEL_PACKAGE}
          )
        elseif (${type} STREQUAL "hw")
          vitis_acceleration_kernel_aux(
            NAME ${VITIS_KERNEL_NAME}
            FILE ${VITIS_KERNEL_FILE}
            TYPE "hw"
            CONFIG ${VITIS_KERNEL_CONFIG}
            INCLUDE ${VITIS_KERNEL_INCLUDE}
            PACKAGE ${VITIS_KERNEL_PACKAGE}
          )
        else()
          message(
            FATAL_ERROR "'" ${type} "' is not a recognized build target \
            for acceleration kernels. Consider 'sw_emu', 'hw_emu' or 'hardware'."
          )
        endif()
      endforeach()
    endif()  # cross compilation, DEFINED CMAKE_SYSROOT

endmacro()  # vitis_acceleration_kernel



#
# Build, link (and package) acceleration kernels for the different targets.
# Auxiliary macro.
#
# :param VITIS_KERNEL_AUX_FILE: CMAKE_SOURCE_DIR relative path to source code of
# the kernel
# :type target: string
#
# :param VITIS_KERNEL_AUX_CONFIG: CMAKE_SOURCE_DIR relative path to configuration
# file for the kernel e.g. src/zcu102.cfg.
# :type target: string
#
# :param VITIS_KERNEL_AUX_INCLUDE: one or muliple relative path directories whereto
# search for header files. This option is passed to the OpenCL preprocessor.
# Directories are relative to CMAKE_SOURCE_DIR.
# :type target: string
#
# :param VITIS_KERNEL_AUX_PACKAGE: Invoke the -p (package) flag of v++ to generate
# the corresponding artifacts for emulation.
# :type target: bool
#
macro(vitis_acceleration_kernel_aux)
  # arguments
  set(options PACKAGE)
  set(oneValueArgs NAME FILE CONFIG TYPE)
  set(multiValueArgs INCLUDE)
  cmake_parse_arguments(VITIS_KERNEL_AUX "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN} )

  message(STATUS "vitis_acceleration_kernel_swemu")

  # variables for the compilation process
  set(ENV{PLATFORM_REPO_PATHS} ${PLATFORM_REPO_PATHS})
  set(ENV{XILINX_VIVADO} ${XILINX_VIVADO})
  set(ENV{XILINX_VITIS} ${XILINX_VITIS})
  set(ENV{XILINX_HLS} ${XILINX_HLS})
  # set(ENV{PATH} "$ENV{PATH}:${XILINX_HLS}/bin")

  # compile
  set(CMD "${VPP_PATH} -c -t ${VITIS_KERNEL_AUX_TYPE} --config " ${CMAKE_SOURCE_DIR} "/"
    ${VITIS_KERNEL_AUX_CONFIG} " -k " ${VITIS_KERNEL_AUX_NAME}
  )
  set(INCLUDE_DIRS "")
  foreach(include_item ${VITIS_KERNEL_AUX_INCLUDE})
    list(APPEND INCLUDE_DIRS "-I${CMAKE_SOURCE_DIR}/${include_item}")
  endforeach()

  list(APPEND CMD ${INCLUDE_DIRS})
  list(APPEND CMD " " ${CMAKE_SOURCE_DIR} "/" ${VITIS_KERNEL_AUX_FILE})
  list(APPEND CMD " -o " ${CMAKE_BINARY_DIR} "/" ${VITIS_KERNEL_AUX_NAME} ".xo")

  # debug()

  # CMake configure time
  # Build
  execute_process(
    COMMAND
      ${VPP_PATH} -c -t ${VITIS_KERNEL_AUX_TYPE}
        --config "${CMAKE_SOURCE_DIR}/${VITIS_KERNEL_AUX_CONFIG}"
        -k ${VITIS_KERNEL_AUX_NAME}
        ${INCLUDE_DIRS}
        "${CMAKE_SOURCE_DIR}/${VITIS_KERNEL_AUX_FILE}"
        -o "${CMAKE_BINARY_DIR}/${VITIS_KERNEL_AUX_NAME}.xo"
    RESULT_VARIABLE
        CMD_ERROR
  )
  message(STATUS "CMD_ERROR: " ${CMD_ERROR})

  # adjust final binary name if not "hw" (only "hw" build target is without suffix)
  set(BINARY_NAME "${VITIS_KERNEL_AUX_NAME}.xclbin")
  if (NOT ${VITIS_KERNEL_AUX_TYPE} STREQUAL "hw")
    set(BINARY_NAME "${VITIS_KERNEL_AUX_NAME}.xclbin.${VITIS_KERNEL_AUX_TYPE}")
    # get a temporary symlink pointing the binary name to allow/permit the
    # v++ compilation process
    # NOTE: v++ is sensitive to the artifact names.
    # See ERROR: [v++ 60-2262] Unsupported input file type specified for
    # --package option.
    execute_process(
      COMMAND
        ln -s ${BINARY_NAME} ${VITIS_KERNEL_AUX_NAME}.xclbin
    )
  endif()

  # Link
  execute_process(
    COMMAND
      ${VPP_PATH} -l -t ${VITIS_KERNEL_AUX_TYPE}
        --config "${CMAKE_SOURCE_DIR}/${VITIS_KERNEL_AUX_CONFIG}"
        "${CMAKE_BINARY_DIR}/${VITIS_KERNEL_AUX_NAME}.xo"
        -o "${CMAKE_BINARY_DIR}/${BINARY_NAME}"
  )
  # package
  if (${VITIS_KERNEL_AUX_PACKAGE})
    execute_process(
      COMMAND
        ${VPP_PATH} -p -t ${VITIS_KERNEL_AUX_TYPE}
          --config "${CMAKE_SOURCE_DIR}/${VITIS_KERNEL_AUX_CONFIG}"
          "${CMAKE_BINARY_DIR}/${VITIS_KERNEL_AUX_NAME}.xclbin"
          --package.out_dir package
          --package.no_image
    )
  endif()  # package

  # install
  if (EXISTS ${CMAKE_BINARY_DIR}/${BINARY_NAME})
    install(
      FILES
        "${CMAKE_BINARY_DIR}/${BINARY_NAME}"
      DESTINATION
        lib/${PROJECT_NAME}
    )

    # package installation
    if (${VITIS_KERNEL_AUX_PACKAGE})
      install(
        DIRECTORY
          "${CMAKE_BINARY_DIR}/package"
        DESTINATION
          lib/${PROJECT_NAME}
      )

      # if hw_emu, symlink to "sim" folder
      if (${VITIS_KERNEL_AUX_TYPE} STREQUAL "hw_emu")
        execute_process(
          COMMAND
            ln -s ${CMAKE_BINARY_DIR}/package/sim ${FIRMWARE_DATA}/../emulation/sim
        )
      endif()  # hw_emu
    endif()  # package installation

    # if sw_emu, deploy "data" from firmware
    if (${VITIS_KERNEL_AUX_TYPE} STREQUAL "sw_emu")
      install(
        DIRECTORY
          "${FIRMWARE_DATA}"
        DESTINATION
          lib/${PROJECT_NAME}
      )
    endif()  # sw_emu
  endif()  # install

  # cleanup symlink for packaging
  if (NOT ${VITIS_KERNEL_AUX_TYPE} STREQUAL "hw")
    execute_process(
      COMMAND
        unlink ${VITIS_KERNEL_AUX_NAME}.xclbin
    )
  endif()
endmacro()  # vitis_acceleration_kernel_swemu

#
# Print relevant variables for debugging purposes
#
macro(debug)
  message("CMAKE_SYSROOT: " ${CMAKE_SYSROOT})
  message("CMAKE_SYSTEM_PROCESSOR: " ${CMAKE_SYSTEM_PROCESSOR})

  message("CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})
  message("CMAKE_BINARY_DIR: " ${CMAKE_BINARY_DIR})
  message("CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})

  message("NAME: " ${VITIS_KERNEL_NAME})
  message("FILE: " ${VITIS_KERNEL_FILE})
  message("CONFIG: " ${VITIS_KERNEL_CONFIG})
  foreach(type ${VITIS_KERNEL_TYPE})
    message("TYPE: " ${type})
  endforeach()
  foreach(include_item ${VITIS_KERNEL_INCLUDE})
    message("INCLUDE: " ${include_item})
  endforeach()
  if (${VITIS_KERNEL_PACKAGE})
    message("PACKAGE: " ${VITIS_KERNEL_PACKAGE})
  endif()
  message("CMD: " ${CMD})
  message("INCLUDE_DIRS: " ${INCLUDE_DIRS})
endmacro()  # debug
