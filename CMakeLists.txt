cmake_minimum_required(VERSION 3.8)
project(epuck_movement_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(epuck_driver_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)


# Building
add_executable(${PROJECT_NAME}_node src/epuck_movement_controller.cpp src/pid.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)

# Installing node
install(TARGETS
${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME}
)

# Installing config files
# install(FILES
# 	config/safety_state_machine.yml
#   	DESTINATION share/${PROJECT_NAME}/config/
# )

# Installing launch files
# install(DIRECTORY launch
# DESTINATION share/${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}_node 
rclcpp
epuck_driver_interfaces 
rclcpp_action
nav_msgs
geometry_msgs
tf2
std_srvs
)

ament_package()
