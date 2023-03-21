cmake_minimum_required(VERSION 3.1.0)
project(bag_parser)

find_package(catkin REQUIRED
    COMPONENTS roscpp
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp nodelet
   DEPENDS
)

add_executable(image_parser_node src/image_parser_node.cpp)

target_include_directories( image_parser_node
    PRIVATE
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries(image_parser_node
    PRIVATE
        ${catkin_LIBRARIES}
)