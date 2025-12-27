cmake_minimum_required(VERSION 3.8)
project(nav2_launcher)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robots/turtlebot3_waffle/models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  robots
  worlds
  config
  models
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
