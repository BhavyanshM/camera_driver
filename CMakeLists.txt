cmake_minimum_required(VERSION 3.0.2)
project(camera_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        image_transport
        cv_bridge
        )

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge
)

find_package( OpenCV REQUIRED )

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/camera_driver_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

