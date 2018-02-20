###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2018
#  European Synchrotron Radiation Facility
#  CS40220 38043 Grenoble Cedex 9 
#  FRANCE
# 
#  Contact: lima@esrf.fr
# 
#  This is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
# 
#  This software is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
set(NAME "fli")

if(UNIX)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--add-needed")
endif()


set(${NAME}_srcs src/FliCamera.cpp  src/FliInterface.cpp
	 src/FliDetInfoCtrlObj.cpp src/FliSyncCtrlObj.cpp
	 src/FliBinCtrlObj.cpp src/FliShutterCtrlObj.cpp
	 src/FliRoiCtrlObj.cpp)

add_library(lima${NAME} SHARED ${${NAME}_srcs})

#INCLUDES
target_include_directories(lima${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(lima${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sdk/libfli-1.104")

#LIBRARY
find_library(LIB_FLI fli "${CMAKE_CURRENT_SOURCE_DIR}/sdk/libfli-1.104/")
if (("${LIB_FLI}" STREQUAL "LIB_FLI-NOTFOUND"))
  message(FATAL_ERROR "${NAME} : libfli not found")
endif()

target_link_libraries(lima${NAME} limacore)
target_link_libraries(lima${NAME} ${LIB_FLI})

limatools_set_library_soversion(lima${NAME} "VERSION")
install(TARGETS lima${NAME} LIBRARY DESTINATION lib)

if(LIMA_ENABLE_PYTHON)
  limatools_run_sip_for_camera(${NAME})
  install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Fli")
endif()

if(LIMA_ENABLE_TESTS)
  add_subdirectory(test)
endif()