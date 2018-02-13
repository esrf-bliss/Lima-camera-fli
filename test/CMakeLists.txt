############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2018
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
cmake_minimum_required(VERSION 3.0)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake ${CMAKE_MODULE_PATH})
include(LimaTools)


set(test_src test_fli_list_devices)
set(NAME "fli")

add_executable(${test_src} "${test_src}.cpp")

target_include_directories(${test_src} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../sdk/libfli-1.104")

#LIBRARY
find_library(LIB_FLI fli "${CMAKE_CURRENT_SOURCE_DIR}/../sdk/libfli-1.104/")
if (("${LIB_FLI}" STREQUAL "LIB_FLI-NOTFOUND"))
  message(FATAL_ERROR "${NAME} : libfli not found")
endif()

target_link_libraries(${test_src} ${LIB_FLI})
