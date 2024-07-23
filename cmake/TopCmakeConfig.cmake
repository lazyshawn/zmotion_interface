# Check whether current CMakeLists is TOP level one
get_directory_property(hasParent PARENT_DIRECTORY)

if(NOT hasParent)
  # You can tweak some common (for all subprojects) stuff here. For example:
  set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
  set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

  if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(SEND_ERROR "In-source builds are not allowed.")
  endif ()

  # set(CMAKE_VERBOSE_MAKEFILE ON)
  set(CMAKE_COLOR_MAKEFILE   ON)

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

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
  set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

  option(EXPORT_CMAKE_PACKAGE "Export targets with cmake package" ON)

else()
  option(EXPORT_CMAKE_PACKAGE "Export targets with cmake package" OFF)

endif()
