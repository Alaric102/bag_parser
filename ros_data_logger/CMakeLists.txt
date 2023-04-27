cmake_minimum_required(VERSION 3.1.0)
project(bag_parser)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
    COMPONENTS 
        roscpp
        sensor_msgs
        cv_bridge
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp nodelet
   DEPENDS
)

add_executable(image_parser_node ${CMAKE_CURRENT_SOURCE_DIR}/src/image_parser_node.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_data_logger.cpp)

target_include_directories(image_parser_node
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${catkin_INCLUDE_DIRS}
        # ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(image_parser_node
    PUBLIC
        ${catkin_LIBRARIES}
        data_logger
        # ${OPENCV_LIBS}
        yaml-cpp
        stdc++fs
)