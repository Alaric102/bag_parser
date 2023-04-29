cmake_minimum_required(VERSION 3.1.0)
project(data_logger)

set(CMAKE_CXX_STANDARD 17)

find_library(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_library(yaml-cpp REQUIRED)

add_library(data_logger SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/data_logger.cpp)

target_include_directories(data_logger
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    PRIVATE
        
)

target_link_libraries(data_logger
    PUBLIC
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
    PRIVATE
        yaml-cpp
        stdc++fs
)

install(
    TARGETS data_logger
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/data_logger
)