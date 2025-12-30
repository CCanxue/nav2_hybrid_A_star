# nav2_hybrid_A_star
- 自定义 hybrid A star 算法包实现
- 模块化机器人导航仿真启动
- 与 nav2 相关内建算法对比测试
### 文件结构
- **/ collected_data** ：在相同条件下针对三种算法分别进行的15次测试的测试结果

- **/ data_collect_scripts**：用于进行测试的 ROS2 Python 包

- **/ nav2_hybrid_astar_planner**：用于实现 hybrid A star 算法的 ROS2 C++ 包，作为插件参数载入 **/nav2_launcher** 中

- **/ nav2_launcher**：用于主程序执行（包括启动 gazebo 仿真，Nav2 和 ROS2 图形化界面 Rviz ）的 ROS2 Launch 包

- **/ other_data**：用于 PPT 制作与查看程序运行情况的数据

- **README.md**：说明文档

### 环境设置

> **WSL2 Ubuntu 22.04 + ROS2 Humble + Gazebo Classic + Nav2**

- **Shell 配置**

```bash
# 声明机器人模型为 Turtlebot3 waffle
export TURTLEBOT3_MODEL=waffle

# 声明仿真环境模型文件地址
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/nav2_launcher/src/nav2_launcher/models/house

# 更改 DDS 通信中间层
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 激活 ROS2 Humble 环境
source /opt/ros/humble/setup.bash

# 强制使用软件渲染（针对集显）
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
```

- **代码运行**

```bash
# 产生当前文件夹下所有包的构建文件
colcon build

# 激活当前环境
source install/setup.bash

# 运行程序
ros2 launch nav2_launcher nav2_launcher.launch.py

# 收集数据
ros2 test_scripts nav2_metrics_collector_v2.py
```

### 更改规划算法

1. **/nav2_launcher/robots/turtlebots_waffle/config/nav2_params.yaml** 中更改 Global Planner 部分的插件参数名
2. **/nav2_launcher/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml** 中更改 ComputePathToPose 部分的规划器名
3. **/nav2_launcher/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml** 中更改 ComputePathThroughPoses 部分的规划器名
