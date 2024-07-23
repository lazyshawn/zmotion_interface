cmake_minimum_required(VERSION 3.12)
project(ZauxRobot VERSION 1.0.0)

if(WIN32)
  # create symbol for exporting lib for dynmic library
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  # control where the static and shared libraries are built so that on windows
  # we don't need to tinker with the path to run the executable
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  # Remove 'lib' prefix for shared libraries on Windows
  # set(CMAKE_SHARED_LIBRARY_PREFIX "")
  # Build static in windows
  option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
  # Set the runtime lib for MSCV (global)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
else()
  option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
endif()
include(GNUInstallDirs)

add_library(zmotion_interface
  src/zmotion_interface.cpp
)

set_target_properties(zmotion_interface PROPERTIES
  #INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib/zmotion"
  LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib/zmotion"
)
target_include_directories(zmotion_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/lib/zmotion>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Then, link libraries
target_link_libraries(zmotion_interface PUBLIC
  zauxdll
  zmotion
)

# path_planning_lib
add_executable(demo
  src/demo.cpp
)

# Then, link libraries
target_link_libraries(demo PUBLIC
  zmotion_interface
)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################################################################
# Install package
###############################################################################
# Install public include files
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
install(DIRECTORY lib/zmotion/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# We add `yourlibrary` target into the export set.
# The export set will contain all targets to be imported by the other project.
# It also installs the library to the install script so they are installed:
set(zaux_robot_libs
  zmotion_interface
)
install(TARGETS ${zaux_robot_libs} EXPORT ZauxRobotTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(${CMAKE_SOURCE_DIR}/cmake/ExportPackage.cmake)




set(zmotion_libs
  ${CMAKE_SOURCE_DIR}/lib/zmotion/zauxdll.lib
  ${CMAKE_SOURCE_DIR}/lib/zmotion/zauxdll.dll
  ${CMAKE_SOURCE_DIR}/lib/zmotion/zmotion.lib
  ${CMAKE_SOURCE_DIR}/lib/zmotion/zmotion.dll
)
file(COPY ${zmotion_libs} DESTINATION "${PROJECT_BINARY_DIR}")
install(FILES ${zmotion_libs} DESTINATION "${PROJECT_BINARY_DIR}")

###############################################################################
# Build example
###############################################################################

