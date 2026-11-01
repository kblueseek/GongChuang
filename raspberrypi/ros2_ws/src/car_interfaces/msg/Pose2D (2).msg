# 平面位姿：x、y 单位米，theta 单位弧度（逆时针为正）。
# 约定和 F407 里程计 / 陀螺仪一致：车正前方为 +x，左侧为 +y。
# （ROS2 的 geometry_msgs 里没有 Pose2D，这里自己定义一个，替代 ROS1 的 geometry_msgs/Pose2D）
float64 x
float64 y
float64 theta
