cmake_minimum_required(VERSION 2.8.3)
project(rtsp_ffmpeg)
add_definitions(-std=c++11)

#set(CMAKE_CXX_FLAGS "-D__STDC_CONSTANT_MACROS")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)
find_package(glog)
find_package(OpenCV 4)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)

find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIR} 
  ${AVFORMAT_INCLUDE_DIR} 
  ${AVUTIL_INCLUDE_DIR} 
  ${AVDEVICE_INCLUDE_DIR}
  ${SWSCALE_INCLUDE_DIR}
  ${AVCODEC_INCLUDE_DIR}
)

message("OpenCV_INCLUDE_DIRS:"  ${OpenCV_INCLUDE_DIRS})
message("OpenCV_LIBRARIES:"  ${OpenCV_LIBRARIES})

add_executable(ros_rtsp_ffmpeg_node src/main_rtsp_ffmpeg.cpp    src/ffmpegdecoder.cpp)
target_link_libraries(ros_rtsp_ffmpeg_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY} ${SWSCALE_LIBRARY} ${AVCODEC_LIBRARY}  glog)

add_executable(ros_rtsp_ffmpeg_state_node src/main_rtsp_ffmpeg_state.cpp    src/ffmpegdecoder.cpp)
target_link_libraries(ros_rtsp_ffmpeg_state_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY} ${SWSCALE_LIBRARY} ${AVCODEC_LIBRARY}  glog)

