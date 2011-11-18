cmake_minimum_required(VERSION 2.8)
PROJECT(openni_wrapper)

include(cmake/openni.cmake)
if(NOT ${HAVE_OPENNI})
  MESSAGE(STATUS "Will not be building ecto_openni, as openni has not been found.")
endif()

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIR})
message(STATUS "OpenNI headers: ${OPENNI_INCLUDE_DIR}")
#INCLUDE_DIRECTORIES("/usr/include/openni")

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  message(STATUS "OpenCV version: ${OpenCV_VERSION}")
  message(STATUS "OpenCV headers: ${OpenCV_INCLUDE_DIRS}")
endif()

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

set (SRCS 
  src/openni_device.cpp
  src/openni_device_xtion.cpp  
  src/openni_image_bayer_grbg.cpp
  src/openni_device_kinect.cpp     
  src/openni_driver.cpp        
  src/openni_image_rgb24.cpp
  src/openni_device_oni.cpp	  
  src/openni_image_yuv_422.cpp
  src/openni_depth_image.cpp	
  src/openni_device_primesense.cpp  
  src/openni_exception.cpp     
  src/openni_ir_image.cpp
  src/openni_acquire.cpp
  )

set_source_files_properties(${SRCS}
  PROPERTIES
  COMPILE_FLAGS "-Wno-deprecated -Wno-strict-aliasing -Wno-unknown-pragmas -Wno-error"
  )

link_directories(${CMAKEDEMO_BINARY_DIR}/w01-cpp)

ADD_LIBRARY(openni_wrapper SHARED ${SRCS})
TARGET_LINK_LIBRARIES(openni_wrapper openni_wrapper boost_thread ${OpenCV_LIBS} OpenNI)

ADD_EXECUTABLE(openni_cap src/openni_cap.cpp)
TARGET_LINK_LIBRARIES(openni_cap openni_wrapper openni_wrapper boost_thread boost_program_options ${OpenCV_LIBS} OpenNI)
