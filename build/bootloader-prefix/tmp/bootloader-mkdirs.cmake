# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "G:/ESP_IDF_PROJECT_2/FINGER_PRINT_INDUSTRIAL_VER/build/bootloader"
  "G:/ESP_IDF_PROJECT_2/FINGER_PRINT_INDUSTRIAL_VER/build/bootloader-prefix"
  "G:/ESP_IDF_PROJECT_2/FINGER_PRINT_INDUSTRIAL_VER/build/bootloader-prefix/tmp"
  "G:/ESP_IDF_PROJECT_2/FINGER_PRINT_INDUSTRIAL_VER/build/bootloader-prefix/src/bootloader-stamp"
  "G:/ESP_IDF_PROJECT_2/FINGER_PRINT_INDUSTRIAL_VER/build/bootloader-prefix/src"
  "G:/ESP_IDF_PROJECT_2/FINGER_PRINT_INDUSTRIAL_VER/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "G:/ESP_IDF_PROJECT_2/FINGER_PRINT_INDUSTRIAL_VER/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "G:/ESP_IDF_PROJECT_2/FINGER_PRINT_INDUSTRIAL_VER/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
