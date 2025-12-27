#ifndef NAV2_HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER_HPP_
#define NAV2_HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/types.hpp"

namespace nav2_hybrid_astar_planner
{
  // 预先声明模板实例，简化代码可读性
  using CostmapT = nav2_costmap_2d::Costmap2D;
  // 定义一个简单的 Point 结构，或者直接使用 geometry_msgs::Point
  // 为了解耦，建议定义内部使用的 PointMock 或复用算法库中的定义
  struct WrapperPoint { double x; double y; }; 
  
  using CollisionCheckerT = nav2_smac_planner::GridCollisionChecker<CostmapT, WrapperPoint>;
  using AlgorithmT = nav2_smac_planner::AStarAlgorithm<CostmapT, CollisionCheckerT>;
  using SmootherT = nav2_smac_planner::Smoother<CostmapT>;

  class HybridAStarPlanner : public nav2_core::GlobalPlanner
  {
  public:
    HybridAStarPlanner();
    ~HybridAStarPlanner();

    // 标准 Nav2 接口
    void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override;

  private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string name_;
    // 成员变量持有算法实例，避免重复内存分配
    std::unique_ptr<AlgorithmT> planner_;
    std::unique_ptr<SmootherT> smoother_;
    
    // 关键：持有代价地图的裸指针
    nav2_costmap_2d::Costmap2D * costmap_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    
    // 配置参数缓存
    nav2_smac_planner::SearchInfo search_info_;
    nav2_smac_planner::OptimizerParams smoother_params_;
    std::string global_frame_;
    //...
  };
}
#endif