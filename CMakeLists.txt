cmake_minimum_required(VERSION 3.0.2)
project(neptune)
set(NOT_BUILD_WITH_ROS OFF)
set(CMAKE_BUILD_TYPE "Debug")
if(NOT ${NOT_BUILD_WITH_ROS})
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    rosbag
    nav_msgs
    tf
  )
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Eigen3 REQUIRED)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/Cmake/)
find_package(Glog REQUIRED)
find_package(Ceres REQUIRED)
find_package(Lua REQUIRED)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "*.cc")

file(GLOB_RECURSE EXECUTABLES "*_main.cc")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${EXECUTABLES})

add_subdirectory(GeographicLib)

add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${LUA_INCLUDE_DIR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
include_directories(
  ${PROJECT_SOURCE_DIR}
)

if(NOT $(NOT_BUILD_WITH_ROS))
  target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

if(NOT ${NOT_BUILD_WITH_ROS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC -lceres)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC -pthread)
target_link_libraries(${PROJECT_NAME} PUBLIC libGeographiccc)

if(NOT ${NOT_BUILD_WITH_ROS})
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    "${catkin_INCLUDE_DIRS}"
  )
  foreach(ABS_FIL ${EXECUTABLES})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  string(REPLACE "/" "." TARGET_NAME "${DIR}/${FIL_WE}")
  add_executable(${TARGET_NAME} ${ABS_FIL})
  target_link_libraries(${TARGET_NAME}
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
  )
endforeach()
endif()

# install(TARGETS ${PROJECT_NAME} DESTINATION lib/locator)

# # Declare a C++ library
# add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/neptune.cpp
# )
