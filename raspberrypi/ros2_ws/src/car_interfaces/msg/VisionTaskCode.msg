# 二维码任务码。
# 生产方：qr_node（平视相机）
# 消费方：flow_node（第 1 步「扫二维码」），用于决定两个批次的摆放顺序
#
# 格式：15 字符 ASCII，形如 156+123+651+213
#   第 1、3 组：三位颜色编号，取值 1~6，组内不重复，两批集合相同（顺序可变）
#   第 2、4 组：圆环编号，必须是 1/2/3 的不重复排列
# 校验规则见 vision_frames.validate_task_code_body()
std_msgs/Header header

string code

# 同一条码要连续若干帧解出一致才置 true，用来滤掉 QRCodeDetector 的偶发误读。
bool valid
