PROJECT(openni_wrapper)

find_package(OpenCV 2.3.1 EXACT REQUIRED)
if (OpenCV_FOUND)
  message(STATUS "OpenCV version: ${OpenCV_VERSION}")
  message(STATUS "OpenCV headers: ${OpenCV_INCLUDE_DIRS}")
endif()
find_package(Boost COMPONENTS serialization filesystem system REQUIRED)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIR})
message(STATUS "OpenNI headers: ${OPENNI_INCLUDE_DIR}")
#INCLUDE_DIRECTORIES("/usr/include/openni")
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

#LINK_DIRECTORIES(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

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
  )

set_source_files_properties(${SRCS}
  PROPERTIES
  COMPILE_FLAGS "-Wno-deprecated -Wno-strict-aliasing -Wno-unknown-pragmas -Wno-error"
  )

ADD_LIBRARY(openni_wrapper SHARED ${SRCS})

ADD_EXECUTABLE(wrapper_test src/openni_example.cpp)
TARGET_LINK_LIBRARIES(wrapper_test opencv_boost_python openni_wrapper OpenNI)
