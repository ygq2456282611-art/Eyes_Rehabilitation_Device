# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/CtrBoard-H7_IMU+CtrBoard_H7_IMU")
  file(MAKE_DIRECTORY "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/CtrBoard-H7_IMU+CtrBoard_H7_IMU")
endif()
file(MAKE_DIRECTORY
  "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/1"
  "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/CtrBoard-H7_IMU+CtrBoard_H7_IMU"
  "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/CtrBoard-H7_IMU+CtrBoard_H7_IMU/tmp"
  "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/CtrBoard-H7_IMU+CtrBoard_H7_IMU/src/CtrBoard-H7_IMU+CtrBoard_H7_IMU-stamp"
  "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/CtrBoard-H7_IMU+CtrBoard_H7_IMU/src"
  "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/CtrBoard-H7_IMU+CtrBoard_H7_IMU/src/CtrBoard-H7_IMU+CtrBoard_H7_IMU-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/CtrBoard-H7_IMU+CtrBoard_H7_IMU/src/CtrBoard-H7_IMU+CtrBoard_H7_IMU-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Keil_v5/CtrBoard-H7_IMU/MDK-ARM/tmp/CtrBoard-H7_IMU+CtrBoard_H7_IMU/src/CtrBoard-H7_IMU+CtrBoard_H7_IMU-stamp${cfgdir}") # cfgdir has leading slash
endif()
