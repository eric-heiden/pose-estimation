project(PoseEstimation)
cmake_minimum_required(VERSION 2.8)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES
    *.h
    *.hpp
    *.cpp
    */*
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} )
