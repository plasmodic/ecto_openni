find_file(OPENNI_INCLUDES "XnCppWrapper.h" PATHS $ENV{OPEN_NI_INCLUDE} "/usr/local/include/ni" "/usr/local/include/openni" "/usr/include/ni" "/usr/include/openni" "c:/Program Files/OpenNI/Include" DOC "OpenNI c++ interface header")
find_library(OPENNI_LIBRARY "OpenNI" PATHS $ENV{OPEN_NI_LIB} "/usr/local/lib" "/usr/lib" "c:/Program Files/OpenNI/Lib" DOC "OpenNI library")
mark_as_advanced(OPENNI_INCLUDES)
mark_as_advanced(OPENNI_LIBRARY)

if(OPENNI_LIBRARY AND OPENNI_INCLUDES)
    set(HAVE_OPENNI TRUE)

        # the check: are PrimeSensor Modules for OpenNI installed
    if(WIN32)
        find_file(OPENNI_PRIME_SENSOR_MODULE "XnCore.dll" PATHS "c:/Program Files/Prime Sense/Sensor/Bin" DOC "Core library of PrimeSensor Modules for OpenNI")
    elseif(UNIX OR APPLE)
        find_library(OPENNI_PRIME_SENSOR_MODULE "XnCore" PATHS "/usr/lib" DOC "Core library of PrimeSensor Modules for OpenNI")
    endif()
	mark_as_advanced(OPENNI_PRIME_SENSOR_MODULE)
    if(OPENNI_PRIME_SENSOR_MODULE)
        set(HAVE_OPENNI_PRIME_SENSOR_MODULE TRUE)
    endif()

    get_filename_component(OPENNI_LIB_DIR "${OPENNI_LIBRARY}" PATH CACHE INTERNAL)
    get_filename_component(OPENNI_INCLUDE_DIR ${OPENNI_INCLUDES} PATH CACHE INTERNAL)
    get_filename_component(OPENNI_PRIME_SENSOR_MODULE_BIN_DIR "${OPENNI_PRIME_SENSOR_MODULE}" PATH CACHE INTERNAL)

    try_run(OPENNI_VERSION_RUN_RESULT OPENNI_VERSION_COMPILE_RESULT
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.c
      COMPILE_DEFINITIONS -I${OPENNI_INCLUDE_DIR}
      COMPILE_OUTPUT_VARIABLE OPENNI_VERSION_COMPILE
      RUN_OUTPUT_VARIABLE OPENNI_VERSION
      )
    
    if(NOT OPENNI_VERSION_COMPILE_RESULT)
      message(FATAL_ERROR "Couldn't compile openni version checking program: ${OPENNI_VERSION_COMPILE}")
    endif()
    
    message(STATUS "OpenNI version ${OPENNI_VERSION}")
    
else() #if(OPENNI_LIBRARY AND OPENNI_INCLUDES)
  message(STATUS "OpenNI NOT found.")
  set(HAVE_OPENNI FALSE)
  set(OPENNI_LIB_DIR OPENNI_LIB_DIR-NOTFOUND CACHE PATH "Openni lib dir")
  set(OPENNI_INCLUDE_DIR OPENNI_INCLUDE_DIR-NOTFOUND CACHE PATH "Openni include dir")
  set(OPENNI_PRIME_SENSOR_MODULE_BIN_DIR OPENNI_PRIME_SENSOR_MODULE_BIN_DIR-NOTFOUND "Hmm...")
endif()

