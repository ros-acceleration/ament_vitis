^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ament_vitis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2021-10-19)
------------------
* Make output less verbose
* CMake scripts report only when hardware acceleration disabled

0.5.0 (2021-06-03)
------------------
* Add CHANGELOG.rst
* Remove leftover from HLS support, and set as default
* Add x86 Ubuntu default libraries dir
* Minor improvements for firmware generation
* Adapt code to a non-Xilinx specific environments
* Add support to generate dfx-mgr artifacts
* Various CMake improvements
  * Support multiple include directories with either relative or absolute paths
  * Re-initialize VITIS_HLS_SRC_VALUE to allow for various HLS calls
  * Support multiple include directories
  * etc.

0.4.0 (2021-07-08)
------------------
* Add CMake macro to generate Tcl scripts
* add HLS Tcl generation capabilities

0.3.0 (2021-06-03)
------------------
* Add license, meet ROS 2 QL5, QL4
* Adapt to new folder structure to standardize across vendors

0.2.0 (2021-04-18)
------------------
* Simplify build process of kernels
* Enable hw_emu, hw and sw_emu targets
* Add first iteration on macros, support both Xilinx tools
as well as custom ROS packages.
* Macros work for all build targets
* Variables shield env. variables abstracting them to have
a coherent environment for Vitis v++.
