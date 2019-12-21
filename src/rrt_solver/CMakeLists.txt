cmake_minimum_required(VERSION 2.8.3)
project(rrt_solver)
add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  kdl_parser
  urdf

)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    kdl_parser
    urdf


  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ library
add_library(${PROJECT_NAME}
	src/rrt_solver.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# add_dependencies(rrt_node ${PROJECT_NAME})

add_executable(rrt_node nodes/rrt_node.cpp)
target_link_libraries(rrt_node ${catkin_LIBRARIES} ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(rrt_solver-test launch/gtest.test tests/tests_1.cpp tests/tests_2.cpp)
  #add_rostest_gtest(rrt_solver-test2 launch/gtest.test tests/tests_2.cpp)
  target_link_libraries(rrt_solver-test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()