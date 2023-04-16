cmake_minimum_required(VERSION 3.1.0)
project(bag_parser)

find_package(catkin REQUIRED
    COMPONENTS 
        roscpp
        cv_bridge
)

find_library(OpenCV REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp nodelet
   DEPENDS
)

add_executable(image_parser_node src/image_parser_node.cpp)

target_compile_features(image_parser_node PRIVATE cxx_std_17)

target_include_directories( image_parser_node
    PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(image_parser_node
    PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        stdc++fs
)