cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_SOURCE_DIR}
                    ${CMAKE_SOURCE_DIR}/descriptors
                    ${CMAKE_SOURCE_DIR}/downsamplers
                    ${CMAKE_SOURCE_DIR}/featurematcher
                    ${CMAKE_SOURCE_DIR}/keypointextractors
                    ${CMAKE_SOURCE_DIR}/lrfestimators
                    ${CMAKE_SOURCE_DIR}/poserefiners
                    ${CMAKE_SOURCE_DIR}/transformationestimators)

add_executable(run_tests run_tests.cpp ParameterTest.hpp)
#SET_TARGET_PROPERTIES(run_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${COMMON_RUNTIME_OUTPUT_DIRECTORY}")

ADD_CUSTOM_COMMAND(TARGET run_tests POST_BUILD
  COMMAND "./run_tests")
ADD_CUSTOM_TARGET(tests "./run_tests" DEPENDS run_tests COMMENT "Running CPPUNIT tests...")


#ADD_CUSTOM_COMMAND(TARGET run_tests POST_BUILD
#  COMMAND "${COMMON_RUNTIME_OUTPUT_DIRECTORY}/run_tests")

target_link_libraries(run_tests ${PROJECT_NAME}_lib cppunit ${PCL_LIBRARIES} ${OpenCV_LIBS})
install_programs(/bin FILES run_tests)
