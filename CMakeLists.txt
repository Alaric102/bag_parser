cmake_minimum_required(VERSION 3.1.0)
project(bag_parser)

find_package(catkin REQUIRED
    COMPONENTS 
        roscpp
        sensor_msgs
        cv_bridge
)

find_library(OpenCV REQUIRED)
find_library(yaml-cpp REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp nodelet
   DEPENDS
)

add_library(data_logger STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/data_logger.cpp)
target_compile_features(data_logger PRIVATE cxx_std_17)
target_include_directories(data_logger 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
    data_logger
    PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        yaml-cpp
        stdc++fs
)

add_executable(image_parser_node src/image_parser_node.cpp)
target_compile_features(image_parser_node PRIVATE cxx_std_17)
target_include_directories( image_parser_node
    PRIVATE
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(image_parser_node
    PRIVATE
        ${catkin_LIBRARIES}
        data_logger
)