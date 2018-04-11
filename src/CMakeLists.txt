

 # Set the library extension according to what configuration is being built.  
IF(CMAKE_DEBUG_POSTFIX)  
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DRW_LIBRARY_POSTFIX=${CMAKE_DEBUG_POSTFIX}")  
ENDIF()  
IF(CMAKE_RELEASE_POSTFIX)  
SET(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -DRW_LIBRARY_POSTFIX=${CMAKE_RELEASE_POSTFIX}")  
ENDIF()  
IF(CMAKE_RELWITHDEBINFO_POSTFIX)  
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DRW_LIBRARY_POSTFIX=${CMAKE_RELWITHDEBINFO_POSTFIX}")  
ENDIF()  
IF(CMAKE_MINSIZEREL_POSTFIX)  
    SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -DRW_LIBRARY_POSTFIX=${CMAKE_MINSIZEREL_POSTFIX}")  
ENDIF() 

MACRO(RW_LINK_LIBRARY BASE_LIBRARY_NAME DEBUGSUFFIX EXSUFFIX)  
     set(DEBUG_LIB ${PROJECT_SOURCE_DIR}/${BASE_LIBRARY_NAME}${DEBUGSUFFIX})  
     set(RELEASE_LIB ${PROJECT_SOURCE_DIR}/${BASE_LIBRARY_NAME}${EXSUFFIX})  
     IF(EXISTS ${RELEASE_LIB}) 
		target_link_libraries(SIFT_SLAM2 optimized  ${RELEASE_LIB})  
		IF(EXISTS ${DEBUG_LIB})  
			target_link_libraries(SIFT_SLAM2  debug ${DEBUG_LIB})  
		ELSE()  
			target_link_libraries(SIFT_SLAM2  debug ${RELEASE_LIB})  
		ENDIF(EXISTS ${DEBUG_LIB})  
	 ENDIF(EXISTS ${RELEASE_LIB})  
ENDMACRO() 

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#ADD_LIBRARY(SIFT_SLAM2 SHARED ${LIB_SRC})
add_library(SIFT_SLAM2 SHARED ${LIB_SRC}
#include/Converter
helper.h
common.h
Converter.h
Converter.cc
Frame.h
Frame.cc
FrameDrawer.h
FrameDrawer.cc
GPU_Siftextractor.h
GPU_Siftextractor.cpp
Initializer.h
Initializer.cc
KeyFrame.h
KeyFrame.cc
KeyFrameDatabase.h
KeyFrameDatabase.cc
LocalMapping.h
LocalMapping.cc
LoopClosing.h
LoopClosing.cc
Map.h
Map.cc
MapDrawer.h
MapDrawer.cc
MapPoint.h
MapPoint.cc
Optimizer.h
Optimizer.cc
ORBVocabulary.h
PnPsolver.h
PnPsolver.cc
SIFTextractor.h
SIFTextractor.cpp
SiftGPU.h
SIFTmatcher.h
SIFTmatcher.cc
Sim3Solver.h
Sim3Solver.cc
System.h
System.cc
Tracking.h
Tracking.cc
Viewer.h
Viewer.cc
)
set_target_properties(SIFT_SLAM2 PROPERTIES OUTPUT_NAME "SIFT_SLAM2")
LINK_DIRECTORIES(
#${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib
#${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/lib
${PROJECT_SOURCE_DIR}/bin)

ADD_DEFINITIONS(-DUSE_GPU_SIFT -DSLAM_DLL_EXPORT)
target_link_libraries(
SIFT_SLAM2
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
#DBoW2.lib
#g2o.lib
#pangolin.lib
)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
 

RW_LINK_LIBRARY(../lib/SIFTGPU "-d" "")
#MACRO(RW_LINK_3RD_PART_LIBRARY FULL_LIBRARY_DEBUGNAME FULL_LIBRARY_RELEASENAME)  
#    IF(EXISTS ${FULL_LIBRARY_RELEASENAME})  
#        target_link_libraries(${PROJECT_NAME} optimized  ${FULL_LIBRARY_RELEASENAME})  
#        IF(NOT EXISTS ${FULL_LIBRARY_DEBUGNAME})  
#            target_link_libraries(${PROJECT_NAME} debug  ${FULL_LIBRARY_RELEASENAME})  
#        ELSE()  
#             target_link_libraries(${PROJECT_NAME}  debug ${FULL_LIBRARY_DEBUGNAME})  
#        ENDIF(NOT EXISTS ${FULL_LIBRARY_DEBUGNAME})  
#    ENDIF(EXISTS ${FULL_LIBRARY_RELEASENAME})  
#ENDMACRO() 
#RW_LINK_3RD_PART_LIBRARY(${GDAL_LIBRARY_DEBUG} ${GDAL_LIBRARY})
# Build examples


