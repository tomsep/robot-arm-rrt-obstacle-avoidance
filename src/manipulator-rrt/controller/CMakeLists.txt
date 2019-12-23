cmake_minimum_required(VERSION 2.8.3)
project(arm_controllers)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  realtime_tools
  dynamic_reconfigure
  geometry_msgs
  tf
)

# Dynamics reconfigure
generate_dynamic_reconfigure_options(
  cfg/PassivityControllerParams.cfg
  )

# message
add_message_files(FILES ControllerJointState.msg)
generate_messages(DEPENDENCIES std_msgs)

# include
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    urdf
    kdl_parser
    realtime_tools
    dynamic_reconfigure
    tf
    
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/manipulator_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

