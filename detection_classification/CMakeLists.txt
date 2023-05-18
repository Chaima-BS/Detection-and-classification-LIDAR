cmake_minimum_required(VERSION 3.0.2)
project(detection_classification)

# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs 
  std_msgs
  message_generation
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


add_message_files(
  FILES
  trackbox.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_generation std_msgs message_runtime sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
)

set(SOURCE_FILES1 src/Detecter/ground_rmv/main.cpp src/Detecter/ground_rmv/gaussian_blur.cpp src/Detecter/ground_rmv/ground_removal.cpp )
add_executable(remove_ground ${SOURCE_FILES1})
target_link_libraries(remove_ground ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

set(SOURCE_FILES2 src/Detecter/cluster/main.cpp src/Detecter/cluster/clustering.cpp src/Detecter/cluster/box_fitting.cpp )
add_executable(cluster ${SOURCE_FILES2})
target_link_libraries(cluster ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

set(SOURCE_FILES3 src/Tracker/main.cpp src/Tracker/ukf.cpp src/Tracker/imm_ukf_jpda.cpp)
add_executable(tracker ${SOURCE_FILES3})
target_link_libraries(tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
