cmake_minimum_required(VERSION 3.0.2)
project(range_to_pointcloud)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)
set(CMAKE_SHARED_LINKER_FLAGS "-static-libstdc++")

find_package(PCL REQUIRED)
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  laser_geometry
  pcl_conversions
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp pcl_conversions pcl_ros
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
  include_directories(${PCL_INCLUDE_DIRS})
  include_directories(${EIGEN3_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  
set(SOURCES1 src/range_to_pointcloud.cpp node/range_to_pointcloud_node.cpp)
add_executable(range_to_pointcloud_node ${SOURCES1})
target_link_libraries(range_to_pointcloud_node
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
)

# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})
# enable_testing()
# add_executable(
#     utest
#     test/utest.cpp
#   )
#   target_link_libraries(
#   utest
#   ${GTEST_LIBRARIES}
#   pthread
# )

# gtest_discover_tests(utest)

install(TARGETS range_to_pointcloud_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)