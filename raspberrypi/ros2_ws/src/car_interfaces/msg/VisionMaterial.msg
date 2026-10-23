# 高位物料颜色识别结果。
# 生产方：vision_node（material_pickup_detector）
# 消费方：flow_node（第 3 步「识别 down 区物料」、第 9 步「识别第二批物料」）
std_msgs/Header header

# false 表示活动区和左/右/下三个抓取区还没标定过，此时 slot_colors 一律无效。
bool calibrated

bool detected

# true 才允许把 slot_colors 当成有效结果使用。判据是：
# 本周期已确认转过（>=30px/s），且接近停稳（<=25px/s 连续 3 帧）且位置颜色不变。
bool ready

# [左, 右, 下] 三个抓取位置的颜色编号。0=该位置为空，1~6 见串口通信协议。
uint8[3] slot_colors

# 当前活动区内检测到的物料件数（1~3）。
uint8 count

# 所有物料中的最大像素速度，用于调试「转没转起来 / 停没停稳」。
float32 max_speed_px_s

# SEARCHING / UNCALIBRATED / MOVING / SETTLING / POSITIONING / NEAR_STOP / READY
string state
