# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/luyetong/esp/esp-idf/components/bootloader/subproject"
  "/Users/luyetong/esp32_code/uart_async_rxtxtasks/build/bootloader"
  "/Users/luyetong/esp32_code/uart_async_rxtxtasks/build/bootloader-prefix"
  "/Users/luyetong/esp32_code/uart_async_rxtxtasks/build/bootloader-prefix/tmp"
  "/Users/luyetong/esp32_code/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/luyetong/esp32_code/uart_async_rxtxtasks/build/bootloader-prefix/src"
  "/Users/luyetong/esp32_code/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/luyetong/esp32_code/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/luyetong/esp32_code/uart_async_rxtxtasks/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
