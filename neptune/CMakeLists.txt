cmake_minimum_required(VERSION 3.0.2)
project(neptune)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
)
cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project(neptune)
find_package(Eigen3 REQUIRED)
set(CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/../Cmake/)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(GTest  REQUIRED)



find_package(Glog REQUIRED)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "*.cc")
file(GLOB_RECURSE EXECUTABLES "*_main.cc")
list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_LIBRARY_SRCS})

add_subdirectory(GeographicLib)

add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

add_executable(${PROJECT_NAME}_main  ${EXECUTABLES})
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_main
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   libGeographiccc
   ${EIGEN3_LIBS}
   ${GLOG_LIBRARIES}
   ${GTEST_LIBRARIES}
    ${CERES_LIBRARIES}
		${OpenCV_LIBS}
   -pthread
 )
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/neptune.cpp
# )

