# vision_node

统一视觉节点：**一个进程、一个节点、一台摄像头**，视觉动作作为"业务"挂在注册表下面。

```
vision_node
├── camera            唯一持有 /dev/video0，启动时预热一次
├── /vision/command   收业务请求（一行 JSON）
├── /vision/result    回业务结果（一行 JSON）
├── /vision/status    启动/忙/摄像头状态
└── businesses/
      ├── color_pick   已完成（复用 vision/gimbal_track_pick.py）
      ├── track_debug  已完成（color_pick 的调参模式）
      ├── vision_put   已完成（复用 vision/vision_put_align.py）
      └── calibrate    已完成（从 ring_fix 搬来的检测 + 纠偏逻辑）
```

## 为什么这样拆

- 摄像头只能有一个主人：`calibrate` / `color_pick` / `vision_put` 以前分别是
  常驻节点、常驻子进程、一次性子进程，互相会抢摄像头；
- 反复开关摄像头要重新预热、重新收敛自动曝光，切换业务会浪费 2~4 秒；
- 统一后切换业务只是改一个状态，不用起进程、不用关开摄像头。

## 接口

发到 `/vision/command`（`std_msgs/String`）：

```jsonc
{"action":"color_pick","args":{"color":4,"timeout_s":60,"still_time":0.3}}
{"action":"track_debug","args":{"color":4}}
{"action":"vision_put","args":{"ring":0,"target_x":640,"target_y":360}}
{"action":"calibrate","args":{"x_m":-2.169,"y_m":1.061}}
{"action":"stop"}
{"action":"status"}
```

`/vision/result` 回一条：

```jsonc
{"action":"color_pick","ok":true,"color":4,"yaw_deg":...,"lift_deg":...,"extend_deg":...}
{"action":"color_pick","ok":false,"reason":"stopped"}
{"action":"color_pick","ok":false,"reason":"忙：vision_put"}
```

空闲/运行期间节点会发 `/motion_active=True`，让 `joy_control` 让出云台；
业务结束发 `False` 交还。

## 新增一个视觉动作

在 `vision_node/businesses/` 下加一个文件，不需要改 `node.py`：

```python
from ..business import Business, register

@register
class QrAlignBusiness(Business):
    name = 'qr_align'

    def run(self, ctx, args, stop) -> dict:
        # ctx.camera.read()      —— 共享摄像头的最新帧
        # ctx.node               —— send_gimbal_vel / _gimbal / _gimbal_ok
        # ctx.vision_dir         —— vision/ 目录（算法模块和 config 都在里面）
        # stop.is_set()          —— 收到 {"action":"stop"} 时返回 True
        return {'ok': True}
```

`businesses/__init__.py` 会自动 import 目录下所有模块。

## 构建

```bash
cd ~/GongChuang/raspberrypi/ros2_ws
colcon build --symlink-install --packages-select vision_node
source install/setup.bash
```

## 单独测试（不要同时跑旧的 vision 脚本）

```bash
ros2 run vision_node vision_node --ros-args \
  -p vision_dir:=$HOME/GongChuang/raspberrypi/vision \
  -p headless:=true
```

另开终端：

```bash
# 看状态
ros2 topic pub --once /vision/command std_msgs/String \
  "{data: '{\"action\":\"status\"}'}"

# 开始 color_pick 颜色 4
ros2 topic pub --once /vision/command std_msgs/String \
  "{data: '{\"action\":\"color_pick\",\"args\":{\"color\":4,\"timeout_s\":60}}'}"

# 取消
ros2 topic pub --once /vision/command std_msgs/String \
  "{data: '{\"action\":\"stop\"}'}"

# 看结果
ros2 topic echo /vision/result
```

参数（`--ros-args -p`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `vision_dir` | 自动从源码树向上找，找不到用 `$HOME/GongChuang/raspberrypi/vision` | 算法模块和 config 所在目录 |
| `device` | `/dev/video0` | 摄像头设备 |
| `width` / `height` / `fps` | 1280 / 720 / 30 | 必须和标定时的分辨率一致 |
| `cam_rotation` | 180 | 摄像头安装旋转角 |
| `headless` | true | 比赛时保持 true，不弹预览窗口 |
| `idle_grab_hz` | 10 | 空闲时排空摄像头缓冲的频率 |

## 迁移状态

1. ✅ `color_pick` / `track_debug` 挂到本节点；
2. ✅ `vision_put` 挂到本节点（复用 `vision_put_align.run_align`）；
3. ✅ `calibrate` 挂到本节点，`ring_fix` 包已删除；
4. ✅ `motion_planner` 默认 `vision_backend: 'node'`，发 `/vision/command`、
   收 `/vision/result`；`vehicle.yaml` 里改回 `'subprocess'` 可退回旧方式。

`calibrate` 的 PID / 对位参数存在 `vision/config/calibrate.json`，上位机 PID 页
新增了『视觉矫正 PID（calibrate 用）』面板可以直接拖；每次 calibrate 开始前读一次，
动作命令 `args` 里的同名项优先级更高。`align_yaw_deg` 没有滑杆，直接改 JSON。
位置/航向环 Kp（`chassis_pos_kp` / `chassis_heading_kp`）不在面板里重复调：
calibrate 自动取 F407 调参页经 `/tune_state` 回传的 `0x05` / `0x08`，JSON 里只是兜底值。

也可以直接在命令的 `args` 里逐项覆盖，例如：

```jsonc
{"action":"calibrate","args":{
  "x_m": -2.169, "y_m": 1.061,
  "ctrl_kp_xy": 1.2, "align_yaw_deg": ""
}}
```
