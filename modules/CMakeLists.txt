include_directories(${OPENNI_INCLUDE_DIR})

ectomodule(ecto_openni
  module.cpp
  Capture.cpp
  )

set_source_files_properties(Capture.cpp
  PROPERTIES
  COMPILE_FLAGS "-Wno-deprecated -Wno-strict-aliasing"
  )
  

link_ecto(ecto_openni ${OPENNI_LIBRARY})

