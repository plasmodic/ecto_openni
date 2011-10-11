cmake_minimum_required(VERSION 2.8)
project(ecto_openni)

include(cmake/openni.cmake)
if(NOT ${HAVE_OPENNI})
  MESSAGE(STATUS "Will not be building ecto_openni, as openni has not been found.")
endif()

set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "The build type.")
find_package(ecto REQUIRED)

#all the ecto modules will be put here.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

#generates a python environment script
ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  message(STATUS "OpenCV version: ${OpenCV_VERSION}")
  message(STATUS "OpenCV headers: ${OpenCV_INCLUDE_DIRS}")
endif()
find_package(Boost COMPONENTS serialization filesystem system REQUIRED)


include_directories(openni_wrapper/include)
add_subdirectory(openni_wrapper)
add_subdirectory(modules)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()
