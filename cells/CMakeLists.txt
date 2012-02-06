include_directories(${OPENNI_INCLUDE_DIR})
set (SRCS 
  module.cpp
  Capture.cpp
  IRGamma.cpp
  KinectKGenerator.cpp
  KinectMaskGenerator.cpp
  openni_driver.cpp
  )

ectomodule(ecto_openni ${SRCS})

set_source_files_properties(${SRCS}
  PROPERTIES
  COMPILE_FLAGS "-Wno-deprecated -Wno-strict-aliasing -Wno-unknown-pragmas -Wno-error"
  )
  

link_ecto(ecto_openni openni_wrapper ${OpenCV_LIBS} ${OPENNI_LIBRARY})
install_ecto_module(ecto_openni)

