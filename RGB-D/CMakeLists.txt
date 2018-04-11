
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(rgbd_reconstruction
helper.h
main.cpp
)
target_link_libraries(rgbd_reconstruction SIFT_SLAM2)

