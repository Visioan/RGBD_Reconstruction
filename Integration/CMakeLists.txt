
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


add_executable(Integration
helper.h
TSDF.hpp
TSDF.cpp
main.cpp
)

   
# Set the build postfix extension according to what configuration is being built.  
IF (CMAKE_BUILD_TYPE MATCHES "Release")  
  	target_link_libraries(Integration "../lib/SIFTGPU")
ENDIF()
IF(CMAKE_BUILD_TYPE MATCHES "Debug")  
	target_link_libraries(Integration "../lib/SIFTGPU-d")
ENDIF()



