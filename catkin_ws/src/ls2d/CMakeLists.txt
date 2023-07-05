cmake_minimum_required(VERSION 3.10)

project(ls2d)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
 

# find pkgs
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    message_generation
    roslaunch
)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED)

roslaunch_add_file_check(launch)

# add custom msgs 
add_message_files(
  FILES
  Sensor_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


# catkin settings
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    message_runtime
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)


# boost settings
add_definitions(-DBOOST_ASIO_STANDALONE)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_SYSTEM_NO_LIB)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.62.0)

if (NOT ${Boost_FOUND})
    message(FATAL_ERROR "Boost not found")
endif()

include_directories(${Boost_INCLUDE_DIR})

# boost debug
# message(STATUS "Boost_VERSION: " ${Boost_VERSION}) 
# message(STATUS "Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIR})


# bin output directory 
math(EXPR platform_bits "${CMAKE_SIZEOF_VOID_P} * 8")
set (platform_dir "x${platform_bits}-${CMAKE_BUILD_TYPE}")
set (output_directory "${CMAKE_SOURCE_DIR}/bin/${platform_dir}")
message(STATUS "platform_dir: " ${platform_dir})
message(STATUS "output_directory: " ${output_directory})
message (STATUS "CMAKE_PREFIX_PATH: " "${CMAKE_PREFIX_PATH}")

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${output_directory})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${output_directory})
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${output_directory})


# prism lib
set (PRISM_LIB
    include/prism/utils/byte_manip.h
    include/prism/utils/timer.h
    src/prism/utils/timer.cpp
    include/prism/s6/ls2d_device.h
    src/prism/s6/ls2d_device.cpp
    include/prism/s6/ls2d_settings.h
    src/prism/s6/ls2d_settings.cpp
    include/prism/s6/sensor_search.h
    src/prism/s6/sensor_search.cpp
    include/prism/s6/sensor_info.h
    src/prism/s6/sensor_info.cpp
)


# add executable files
set(SENSOR_INIT_NODE src/ls2d_init_node.cpp)
add_executable(sensor_init_node ${SENSOR_INIT_NODE} ${PRISM_LIB})
target_include_directories(sensor_init_node PUBLIC include/prism src/)
target_link_libraries (sensor_init_node ${catkin_LIBRARIES} Threads::Threads)
add_dependencies(sensor_init_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker ${catkin_EXPORTED_TARGETS})
#install(TARGETS sensor_init_node
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})