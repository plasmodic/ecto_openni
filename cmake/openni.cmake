find_file(OPENNI_INCLUDES "XnCppWrapper.h" PATHS $ENV{OPEN_NI_INCLUDE} "/usr/include/ni" "/usr/include/openni" "c:/Program Files/OpenNI/Include" DOC "OpenNI c++ interface header")
find_library(OPENNI_LIBRARY "OpenNI" PATHS $ENV{OPEN_NI_LIB} "/usr/lib" "c:/Program Files/OpenNI/Lib" DOC "OpenNI library")

if(OPENNI_LIBRARY AND OPENNI_INCLUDES)
    set(HAVE_OPENNI TRUE)
    # the check: are PrimeSensor Modules for OpenNI installed
    if(WIN32)
        find_file(OPENNI_PRIME_SENSOR_MODULE "XnCore.dll" PATHS "c:/Program Files/Prime Sense/Sensor/Bin" DOC "Core library of PrimeSensor Modules for OpenNI")
    elseif(UNIX OR APPLE)
        find_library(OPENNI_PRIME_SENSOR_MODULE "XnCore" PATHS "/usr/lib" DOC "Core library of PrimeSensor Modules for OpenNI")
    endif()
	
    if(OPENNI_PRIME_SENSOR_MODULE)
        set(HAVE_OPENNI_PRIME_SENSOR_MODULE TRUE)
    endif()
else() #if(OPENNI_LIBRARY AND OPENNI_INCLUDES)
  set(HAVE_OPENNI FALSE)
endif()
get_filename_component(OPENNI_LIB_DIR "${OPENNI_LIBRARY}" PATH CACHE)
get_filename_component(OPENNI_INCLUDE_DIR ${OPENNI_INCLUDES} PATH CACHE)
get_filename_component(OPENNI_PRIME_SENSOR_MODULE_BIN_DIR "${OPENNI_PRIME_SENSOR_MODULE}" PATH CACHE)

