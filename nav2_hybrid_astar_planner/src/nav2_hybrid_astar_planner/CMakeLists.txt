cmake_minimum_required(VERSION 3.8)
project(nav2_hybrid_astar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# 外部算法库
find_package(ompl REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. 设置头文件路径
include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/nav2_smac_planner
)

# 3. 创建插件库
# 注意：移除了.tpp 文件，只编译.cpp
add_library(hybrid_astar_planner_plugin SHARED
  src/hybrid_astar_planner.cpp
  src/node_se2.cpp
  #include/nav2_smac_planner/a_star.tpp
  #include/nav2_smac_planner/footprint_collision_checker.tpp
)

target_link_libraries(hybrid_astar_planner_plugin
  ${OMPL_LIBRARIES}   
  ${CERES_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

ament_target_dependencies(hybrid_astar_planner_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2_ros
  geometry_msgs
  nav_msgs
  visualization_msgs
  ompl
  Eigen3
)

install(TARGETS hybrid_astar_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# 5. 导出插件与安装
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)


install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(hybrid_astar_planner_plugin)
ament_package()
