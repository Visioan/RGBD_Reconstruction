cmake_minimum_required(VERSION 2.8)
project(SIFT_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

#MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -W0 -MT -bigobj")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W0 -MT -bigobj")
SET(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/bin" CACHE STRING "installation path")

  #这个就是定义各个版本对应的后缀，例如d，debug版本后缀，当然你想定义为其他，   #自己修改这块就可以了。  
SET(CMAKE_DEBUG_POSTFIX "-d" CACHE STRING "add a postfix, usually d on windows")  
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a postfix, usually empty on windows")  
SET(CMAKE_RELWITHDEBINFO_POSTFIX "rd" CACHE STRING "add a postfix, usually empty on windows")  
SET(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add a postfix, usually empty on windows")  
   
# Set the build postfix extension according to what configuration is being built.  
IF (CMAKE_BUILD_TYPE MATCHES "Release")  
    SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")  
ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")  
    SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")  
ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")  
    SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")  
ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")  
    SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")  
ELSE()  
    SET(CMAKE_BUILD_POSTFIX "")  
ENDIF() 
 



# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
SET(DBUILD_SHARED true)

#if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#   add_definitions(-DCOMPILEDWITHC11)
#   message(STATUS "Using flag -std=c++11.")
#elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#   add_definitions(-DCOMPILEDWITHC0X)
#   message(STATUS "Using flag -std=c++0x.")
#else()
#	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#find_package(OpenCV 3.0 QUIET)
#if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.3 QUIET)
#   if(NOT OpenCV_FOUND)
#      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#   endif()
#endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Pangolin REQUIRED)
#find_package(Eigen3 3.1.0 REQUIRED)
#find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/src
${EIGEN3_INCLUDE_DIR}
#${Pangolin_INCLUDE_DIRS}
#${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/include
)



#ADD_LIBRARY(SIFT_SLAM2 SHARED ${LIB_SRC})
add_subdirectory("${PROJECT_SOURCE_DIR}/src")
add_subdirectory("${PROJECT_SOURCE_DIR}/RGB-D")
add_subdirectory("${PROJECT_SOURCE_DIR}/Integration")



