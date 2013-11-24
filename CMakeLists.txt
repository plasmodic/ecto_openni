cmake_minimum_required(VERSION 2.8)
project(ecto_openni)

# Deal with catkin
find_package(catkin REQUIRED ecto)
find_package(ecto)

catkin_package(DEPENDS ecto)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

include(cmake/openni.cmake)
if(NOT ${HAVE_OPENNI})
  message(STATUS "Will not be building ecto_openni, as openni has not been found.")
  return()
endif()

find_package(Boost COMPONENTS serialization filesystem system thread REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(openni_wrapper/include)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)
add_subdirectory(openni_wrapper)
add_subdirectory(cells)

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
