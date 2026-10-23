# 三圆环工位识别结果。
# 生产方：vision_node（普通三圆环 ring_station_detector / 码垛 stacked_ring_detector）
# 消费方：flow_node（13 步状态机的第 4~7、10~13 步）
#
# 约定：
#   - 坐标为 1280x720 畸变矫正后图像的像素，左上角 (0,0)，x 向右、y 向下。
#   - 误差为「工位平面」上的毫米值，由 image_to_station_affine 换算。
#   - detected=false 时三个圆心全部填 0，消费方必须先看 detected 再用坐标。
std_msgs/Header header

bool detected
bool stable

# R1 / R2 / R3，固定三组。第 5/6/7 步要用全部三个中心去够不同的环，
# 不要只取 R2 再靠 150mm 间距反推（反推会放大像素→毫米的误差）。
float32[3] center_x_px
float32[3] center_y_px

# 实际连线角度，不是 correction_yaw。单位度，逆时针为正。
float32 station_yaw_deg

# 2 号环在工位平面上的偏差，以及相对标准工位（0°）的角度误差。
float32 error_x_mm
float32 error_y_mm
float32 angle_error_deg

# 圆心抖动与角度误差是否都落在 acceptance 门槛内。
bool within_tolerance
