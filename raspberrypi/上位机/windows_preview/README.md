# Windows 离线预览

不装 ROS、不接树莓派/407，直接在本机看上位机的四个界面和完整流程。

## 运行

双击 `启动预览.bat`，或在 PowerShell 里：

```powershell
cd "E:\Desktop\工创赛\电控代码\raspberrypi\上位机\windows_preview"
python preview.py
```

只需要 Python 3（自带 Tkinter）。不需要 `rclpy`、numpy、pyserial。

## 能演示什么

- 四个页签：当前动作序列 / 雷达·地图 / 节点动作序列 / PID 调参
- 【规划全程】→ 底部【执行】：假车会按九点路线在地图上跑，波形实时更新
- PID 页拖动滑杆：模拟的 407 会立即回显，状态栏显示“407 已确认”
- PID 页的云台零点 / 限位 / 松轴：模拟 407 会回传一组假限位；
  【设为零点】会把模拟云台三轴清零并重新下发限位；松轴只发指令，不驱动任何东西
- 雷达页勾【用模拟雷达】也能扫描/识别（用的是真正的 `lidar_d6.SimLidar`）
- 底部【停止】立刻停在原地（真机上还会同时整车失能）

## 边界

- 这是 **纯软件模拟**，不会发送任何真实指令，也不连接 F407。
- 预览里的 `rclpy` / `std_msgs` / `nav_msgs` / `car_interfaces` 是
  `stubs/` 下的空壳，只在预览进程的 `sys.path` 里生效。
- `script` / `align` / `vision_pick` 在预览里只做示意停顿，不会真的启动程序；
  `pick` / `place` 只动云台和夹爪，不展开完整宏。这些在真机上由
  motion_planner 执行。
- 真机仍然跑树莓派上的 `scripts/run_gui.sh`。
