# 转盘外圆定位结果。
# 生产方：vision_node（turntable_detector）
# 消费方：flow_node（第 2 步「转盘对准」、第 8 步「回转盘重新校准」）
#
# 约定：
#   - 坐标为 1280x720 畸变矫正后图像的像素。
#   - 只有 detected=true 且 stable=true 的新鲜结果才可以直接用；
#     断开或未稳定时不发布，消费方应按「超时未更新」处理。
std_msgs/Header header

bool detected
bool stable

float32 center_x_px
float32 center_y_px

# 外圆拟合半径。已知转盘直径 300mm，可反推相机到台面的距离：
#   Z_mm = fx * 150 / radius_px
# 注意 turntable_detector.json 的 maximum_radius_px 会卡住近距离。
float32 radius_px

# local（局部跟踪）/ hough（全图粗搜）/ three_material（三物料快速路径）/ reacquired
string tracking_mode
