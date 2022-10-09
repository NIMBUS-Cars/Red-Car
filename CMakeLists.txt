cmake_minimum_required(VERSION 3.0.2)
project(car)
set(OpenCV_DIR /home/ryan/opencv/build)
set(Torch_DIR /home/ryan/libtorch-cpp/libtorch)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_runtime geometry_msgs sensor_msgs cv_bridge image_transport message_generation)
add_message_files(
	DIRECTORY
	msg
	FILES
	CarObject.msg
  CarObjects.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
	CATKIN_DEPENDS roscpp rospy message_runtime
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)
find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
  ${Torch_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(real_car_lane_follow src/fastLaneFollow.cpp)
add_dependencies(real_car_lane_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(real_car_lane_follow ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${TORCH_LIBRARIES}) 
