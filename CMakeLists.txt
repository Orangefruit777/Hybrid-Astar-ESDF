cmake_minimum_required(VERSION 3.10)
project(hybrid_a_star)

message(STATUS "Hybrid A star")
message(STATUS "Author: Zhang Zhimeng")

#set(CMAKE_VERBOSE_MAKEFILE "true")

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")



# Eigen
# include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# glog
# include(cmake/glog.cmake)
# include_directories(${GLOG_INCLUDE_DIRS})

# catkin
find_package(
    catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    tf
    pcl_conversions
    pcl_ros
)
find_package(PCL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES path_searcher
    #  CATKIN_DEPENDS nav_mags roscpp tf
    #  DEPENDS system_lib
)

add_library(
    PATH_SEARCHER_LIB SHARED
    src/util/rs_path.cpp
    src/util/dubins_curve.cpp
    src/util/costmap_subscriber.cpp
    src/util/goal_pose_subscriber.cpp
    src/util/init_pose_subscriber.cpp
    src/hybrid_a_star.cpp
    src/plan_env/sdf_map_2d.cpp
    src/hybrid_a_star_node.cpp  
    src/smooth.cpp 
    src/quintic_spline.cpp
    src/Astar_path/a_star.cpp
    src/Astar_path/a_star_node.cpp
)

target_link_libraries(
    PATH_SEARCHER_LIB
    ${catkin_LIBRARIES}
    # ${GLOG_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(map_publish src/plan_env/map_publish.cpp)
target_link_libraries(map_publish 
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
)

add_executable(map_publish_y src/plan_env/map_publish_y.cpp)
target_link_libraries(map_publish_y 
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
)

add_executable(run_hybrid_astar app/run_hybrid_astar.cpp)
target_link_libraries(run_hybrid_astar PATH_SEARCHER_LIB)

add_executable(run_astar app/run_astar.cpp)
target_link_libraries(run_astar PATH_SEARCHER_LIB)