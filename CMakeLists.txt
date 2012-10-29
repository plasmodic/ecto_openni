cmake_minimum_required(VERSION 2.8)
project(ecto_openni)

# Deal with catkin
find_package(catkin REQUIRED ecto)

if (${catkin_VERSION} VERSION_GREATER "0.5.26")
catkin_package(DEPENDS ecto)
else()
catkin_stack()
catkin_project(ecto_openni DEPENDS ecto)
#all the ecto modules will be put here.
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()
endif()

include(cmake/openni.cmake)
if(NOT ${HAVE_OPENNI})
  message(STATUS "Will not be building ecto_openni, as openni has not been found.")
  return()
endif()

find_package(Boost COMPONENTS serialization filesystem system REQUIRED)
find_package(ecto REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(openni_wrapper/include)
add_subdirectory(openni_wrapper)
add_subdirectory(cells)

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()
