#include "nav2_hybrid_astar_planner/hybrid_astar_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace nav2_hybrid_astar_planner
{

HybridAStarPlanner::HybridAStarPlanner()
: costmap_(nullptr) 
{
}

HybridAStarPlanner::~HybridAStarPlanner()
{
}

void HybridAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = parent.lock();
  
  // --- 1. 读取参数 ---

  double min_turning_radius_meters;
  nav2_util::declare_parameter_if_not_declared(node, name + ".min_turning_radius", rclcpp::ParameterValue(0.40));
  node->get_parameter(name + ".min_turning_radius", min_turning_radius_meters);

// --- 修改开始：将米转换为栅格数 ---
  double resolution = costmap_ros_->getCostmap()->getResolution();
  search_info_.minimum_turning_radius = static_cast<float>(min_turning_radius_meters / resolution);
  
  nav2_util::declare_parameter_if_not_declared(node, name + ".reverse_penalty", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".change_penalty", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));

  node->get_parameter(name + ".reverse_penalty", search_info_.reverse_penalty);
  node->get_parameter(name + ".change_penalty", search_info_.change_penalty);
  node->get_parameter(name + ".non_straight_penalty", search_info_.non_straight_penalty);
  node->get_parameter(name + ".cost_penalty", search_info_.cost_penalty);
  // 其他惩罚参数
  //search_info_.reverse_penalty = 2.0;
  //search_info_.change_penalty = 0.05;
  //search_info_.non_straight_penalty = 1.2;
  //search_info_.cost_penalty = 1.0;
  //search_info_.analytic_expansion_ratio = 3.5;

  // --- 2. 初始化算法 ---
  // 注意：MotionModel::DUBIN (仅前进) 或 REEDS_SHEPP (可倒车)
  // 这里我们假设使用 Reeds-Shepp 模型
  planner_ = std::make_unique<AlgorithmT>(nav2_smac_planner::MotionModel::REEDS_SHEPP, search_info_);
  
  // 设置最大迭代次数等
  int max_iterations = 100000;
  int max_on_approach_iterations = 1000;
  planner_->initialize(false, max_iterations, max_on_approach_iterations);

  // 初始化平滑器
  smoother_ = std::make_unique<SmootherT>();
  smoother_params_.debug = false;
  smoother_->initialize(smoother_params_);
  
  RCLCPP_INFO(node->get_logger(), "Hybrid A* Planner Configured!");
}

void HybridAStarPlanner::cleanup()
{
  planner_.reset();
  smoother_.reset();
}

void HybridAStarPlanner::activate()
{
  RCLCPP_INFO(node_.lock()->get_logger(), "Hybrid A* Planner Activated");
}

void HybridAStarPlanner::deactivate()
{
  RCLCPP_INFO(node_.lock()->get_logger(), "Hybrid A* Planner Deactivated");
}

nav_msgs::msg::Path HybridAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_.lock()->now();
  path.header.frame_id = global_frame_;

  // 1. 锁住 Costmap，防止规划时地图更新导致崩溃
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // 2. 坐标转换：World (米) -> Map (栅格索引)
  unsigned int mx_start, my_start, mx_goal, my_goal;
  
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
     !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    RCLCPP_ERROR(node_.lock()->get_logger(), "Start or Goal is out of map bounds!");
    return path;
  }

  // 3. 角度量化 (Pose Quaternion -> Yaw -> Bin Index)
  // 混合 A* 将 360 度分为 72 个方向 (每个 5 度)
  unsigned int quantization = 72;
  double start_yaw = tf2::getYaw(start.pose.orientation);
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  
  // 将 (-PI, PI) 映射到 [0, 72)
  auto angle_to_bin = [&](double angle) {
    // 归一化到 [0, 2PI]
    while(angle < 0) angle += 2.0 * M_PI;
    while(angle >= 2.0 * M_PI) angle -= 2.0 * M_PI;
    return static_cast<unsigned int>(angle / (2.0 * M_PI / quantization));
  };

  unsigned int start_yaw_bin = angle_to_bin(start_yaw);
  unsigned int goal_yaw_bin = angle_to_bin(goal_yaw);

  // 4. 设置算法输入
  // 获取 Costmap 设置的 footprint
  std::vector<geometry_msgs::msg::Point> ros_footprint = costmap_ros_->getRobotFootprint();
  std::vector<WrapperPoint> converted_footprint;
  
  // 转换类型
  for(auto& pt : ros_footprint){
      converted_footprint.push_back({pt.x, pt.y});
  }

  // 如果 footprint 为空（通常意味着在 params 中设置了 robot_radius 而不是 footprint），则回退到半径模式
  bool use_radius = converted_footprint.empty();
  
  planner_->setFootprint(converted_footprint, use_radius);
  //planner_->setFootprint(std::vector<WrapperPoint>(), true); // true 表示使用半径圆碰撞检测（简化）
  
  // 这一步很重要：初始化搜索图
  planner_->createGraph(
    costmap_->getSizeInCellsX(), 
    costmap_->getSizeInCellsY(), 
    quantization, 
    costmap_
  );

  planner_->setStart(mx_start, my_start, start_yaw_bin);
  planner_->setGoal(mx_goal, my_goal, goal_yaw_bin);

  // 5. 执行规划
  nav2_smac_planner::NodeSE2::CoordinateVector plan_path;
  int num_iterations = 0;
  float tolerance = 10.0; // 允许目标点有一定的误差

  // 增加 try-catch 块，防止算法内部抛出异常导致整个节点崩溃
  try {
    if (planner_->createPath(plan_path, num_iterations, tolerance)) {
      // 6. 后处理：转换回 ROS 消息格式
      path.poses.reserve(plan_path.size());
      
      for (int i = plan_path.size() - 1; i >= 0; --i) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header = path.header;
          
          double wx, wy;
          costmap_->mapToWorld(
              static_cast<unsigned int>(plan_path[i].x), 
              static_cast<unsigned int>(plan_path[i].y), 
              wx, wy
          );
          
          pose.pose.position.x = wx;
          pose.pose.position.y = wy;
          pose.pose.position.z = 0.0;
          
          double yaw = plan_path[i].theta * (2.0 * M_PI / quantization);
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          pose.pose.orientation.x = q.x();
          pose.pose.orientation.y = q.y();
          pose.pose.orientation.z = q.z();
          pose.pose.orientation.w = q.w();
          
          path.poses.push_back(pose);
      }
      
      RCLCPP_INFO(node_.lock()->get_logger(), "Plan found with %zu points! Iterations: %d", path.poses.size(), num_iterations);
    } else {
      RCLCPP_WARN(node_.lock()->get_logger(), "Hybrid A* failed to create a plan.");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.lock()->get_logger(), "Hybrid A* threw exception: %s", e.what());
  }

  return path;
}

} // namespace nav2_hybrid_astar_planner

// 导出插件
PLUGINLIB_EXPORT_CLASS(nav2_hybrid_astar_planner::HybridAStarPlanner, nav2_core::GlobalPlanner)