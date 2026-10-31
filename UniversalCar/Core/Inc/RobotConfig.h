#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       整车配置宏
 * @details     集中管理所有可调参数，调车只改这里
 */

/* ==================== 电机（ZDT Y42 闭环步进，CAN，Emm 固件） ==================== */

/* CAN 地址：1-4 底盘麦轮，5-7 云台三轴 */
enum
{
    ZDT_MOTOR_FL = 1,   /* 底盘 前左 */
    ZDT_MOTOR_FR = 2,   /* 底盘 前右 */
    ZDT_MOTOR_RL = 3,   /* 底盘 后左 */
    ZDT_MOTOR_RR = 4,   /* 底盘 后右 */
    ZDT_MOTOR_YAW = 5,  /* 云台 Yaw 旋转轴 */
    ZDT_MOTOR_LIFT = 6, /* 云台 Z 升降轴 */
    ZDT_MOTOR_EXTEND = 7, /* 云台 X 伸缩轴 */
};

#define ZDT_ACC_DEFAULT     222    /* 云台位置模式加速度档位（0-255，越大越快，0=无曲线） */
#define ZDT_POS_SPEED_MAX   100     /* 云台位置模式速度上限，单位 RPM（整数）。
                                       400=2400°/s 会让位置模式任何一次误目标在 0.2s 内冲出去、
                                       根本来不及停（"切模式云台失控无法停止"的放大器）。
                                       降到 100=600°/s，误目标也慢到能看清、能按停。 */
/* 底盘轮加速度档位（0-255）。Emm 固件曲线加减速公式：
     每隔 (256-acc)*50us 变化 1 RPM  =>  加速度 = 20000/(256-acc) RPM/s
   加速、减速、反向共用此档位，固件自动走曲线，无法分开设置。
     0   : 不使用曲线，直接以设定速度启动（最猛）
     250 : 3333 RPM/s，0→300RPM 用 0.09s（原值，起步/停车顿挫的根源）
     156 :  200 RPM/s，0→300RPM 用 1.5s（当前值）
     100 :  128 RPM/s，0→300RPM 用 2.3s
   注意：急停命令 0xFE 不受此档位影响，永远是立即刹车。 */
#define ZDT_WHEEL_ACC       222
#define ZDT_PULSE_PER_REV   3200   /* Emm 固件：1.8°电机 + 16细分 = 3200 脉冲/圈 */

/* 电机失联判定阈值（无任何反馈视为离线） */
#define ZDT_OFFLINE_MS      200

/* 电机编码器反馈周期（ms）：底盘 4 轮定时返回实时位置(0x36)的间隔，用于里程计/位置闭环。
   值越小反馈越密、速度估计越准，但 CAN 总线占用越高（500K 下每轴返回约占 1000/周期 帧/s）。
   本轮改版把云台三轴的指令从 0xAA 广播改成单机帧、底盘也降到 100Hz，总线腾出了余量，
   先统一用 10ms=100Hz：4 轮 + 3 轴 7 路反馈 = 700 帧/s，加上速度指令仍在线性区间。
   里程计位置用 RawCount 差分积分、与反馈疏密无关、仍然精确；先按 10ms 跑，稳定后再提频率。 */
#define ZDT_FEEDBACK_PERIOD_MS  10

/* ==================== 底盘（麦克纳姆 X 布局） ==================== */

#define WHEEL_RADIUS_M          0.03925f /* 轮半径 m（实测直径 78.5mm） */
#define WHEEL_HALF_WHEELBASE_M  0.15f   /* 半轴距 m（前后轮距/2） */
#define WHEEL_HALF_TRACK_M      0.15f   /* 半轮距 m（左右轮距/2） */

#define CHASSIS_MAX_SPEED_X     1.0f    /* 前进最大速度 m/s（摇杆满量程对应的值） */
#define CHASSIS_MAX_SPEED_Y     1.0f    /* 平移最大速度 m/s（摇杆满量程对应的值） */
#define CHASSIS_MAX_SPEED_W     3.0f    /* 自转最大角速度 rad/s（自旋滑杆拨到两端对应的值） */
#define CHASSIS_MAX_WHEEL_RPM   300     /* 单轮最大转速 RPM（软件限幅） */

/* 每轮转向系数（按安装方向标定 ±1）：单轮发正速度时车应"前进"，反向取反 */
#define WHEEL_FL_DIR    1
#define WHEEL_FR_DIR   -1
#define WHEEL_RL_DIR    1
#define WHEEL_RR_DIR   -1

/* ==================== 舵机（3 个，均 180°） ==================== */

#define SERVO_GRIPPER_CH    TIM_CHANNEL_1   /* 夹爪     TIM3_CH1 / PA6 */
#define SERVO_PLATE_CH      TIM_CHANNEL_2   /* 载物盘   TIM3_CH2 / PA7 */
#define SERVO_CAM_CH        TIM_CHANNEL_3   /* 摄像头   TIM3_CH3 / PB0 */

#define SERVO_TRAVEL        180.0f
#define SERVO_PULSE_MIN     500     /* 0°   脉宽 us */
#define SERVO_PULSE_MAX     2500    /* 180° 脉宽 us */

/* 上电初始角度 */
#define SERVO_GRIPPER_INIT  90.0f
#define SERVO_PLATE_INIT    0.0f
#define SERVO_CAM_INIT      90.0f

/* ==================== 补光灯（LED PWM 调光） ==================== */

#define LED_PWM_CH       TIM_CHANNEL_1   /* 补光灯  TIM2_CH1 / PA5 */
#define LED_BRIGHT_MAX   100.0f          /* 亮度满量程 % */

/* ==================== 云台 ==================== */

/* 云台电机总开关：0 = 完全不碰 5/6/7（云台整个没装时用）。
   装好云台后置 1，再按轴单独开关下面的 GIMBAL_YAW_INSTALLED。
   没装的轴一条帧都不发——向空地址发的帧无 ACK，会抬高 TEC、白占 3 个发送邮箱。 */
#define GIMBAL_MOTOR_INSTALLED   1

/* Yaw 轴（地址 5）是否已装。三轴现在全部安装，置 1。
   ⚠️ 电机侧地址必须和本文件的枚举一一对应，改完 ID 要整车上电重启：
        Yaw = 地址 5，Lift（Z 升降）= 地址 6，Extend（X 伸缩）= 地址 7。
      两个电机绝不能共用同一个地址（会同时动作、无法单独控制）。 */
#define GIMBAL_YAW_INSTALLED     1

/* 三轴软限位（相对「上电位置」的角度，单位 °），往越界方向推不发速度，防止撞结构。
   云台任务在每轴第一次收到位置反馈时把那一刻锁存成 0 点，所以这里的 ± 值就是
   "相对上电位置"的行程。下面是出厂默认值（实测行程）：
     Extend（X 伸缩，地址 7）：行程 1160° → ±580°
     Lift  （Y 升降，地址 6）：行程 1750° → ±875°
     Yaw   （旋转，地址 5）  ：行程未量，先给 ±1000° 兜底。
   ⚠️ 正常运行不用改这里：树莓派面板节点会在开机示教时把人为拨到的两端通过
      PI_CMD_GIMBAL_LIMIT 下发，运行时值以树莓派下发的为准（见 PiComm.h / GimbalTask.c）。
      这组宏只是"树莓派没下发过限位"时的兜底。
   云台只发速度帧，不存在位置模式；限位靠"相对中位的位置 + 运动方向"在速度指令上掐掉。 */
#define GIMBAL_YAW_MIN      -1000.0f
#define GIMBAL_YAW_MAX      1000.0f
#define GIMBAL_LIFT_MIN      -875.0f
#define GIMBAL_LIFT_MAX       875.0f
#define GIMBAL_EXTEND_MIN    -580.0f
#define GIMBAL_EXTEND_MAX     580.0f

/* 轴指令方向符号（+1 / -1）：把"下发速度的正方向"换算到"编码器位置增大的方向"。
   +1 = 正速度指令让位置反馈变大（Lift / Extend 属于这种，实车已验证）。
   -1 = 正速度指令让位置反馈变小（编码器计数方向和电机转向相反，Yaw 就是这种）。
   ⚠️ 只影响限位判断，不改电机实际转向：限位执行的是"这一拍的速度是往里推还是往外拉"，
      所以必须先把速度换到编码器坐标里再比。符号填反的表现是
      "顶到限位后，往回拨不动，继续往越界方向反而能动"——哪个轴出现就把哪个轴改号。
   示教出来的最小/最大点仍然是编码器原始坐标，不需要跟着改。 */
#define GIMBAL_YAW_CMD_SIGN     (-1.0f)
#define GIMBAL_LIFT_CMD_SIGN     1.0f
#define GIMBAL_EXTEND_CMD_SIGN   1.0f

/* ==================== 树莓派链路（无线） ==================== */
/* 上行：树莓派 --BLE--> F407（USART2 / PA2 PA3，二进制 PiComm 帧）
   下行：F407 --LoRa--> 树莓派（UART4 / PC10=TX PC11=RX，只发不收） */
#define BLE_UPLINK_ENABLE    0      /* 1=USART2 作为 PiComm 指令上行 0=关闭 */
#define LORA_DOWNLINK_ENABLE 0      /* 1=UART4 作为 PiComm 遥测下行 0=关闭（回落 USART6 有线） */
/* 必须与两块 LoRa 模组的串口波特率一致（本项目实测 115200）。
   ⚠️ 改这个宏之后，如果再从 CubeMX 重新生成代码，usart.c 里那行会被 .ioc 的值覆盖回去，
   所以 .ioc 的 UART4.BaudRate 必须同步改（当前已设为 115200）。 */
#define LORA_UART_BAUD       115200

#define PI_CMD_TIMEOUT_MS   500     /* 超过此时长收不到指令 → 底盘停车、云台保持 */

/* 遥测频率（USART6 115200，11.52 字节/ms）：
     里程计 17B × 50Hz ≈ 850B/s
     云台位置 13B × 20Hz ≈ 260B/s
     状态 10B × 5Hz     ≈  50B/s
   合计约 1.2kB/s，占用约 10%，余量充足。
   里程计 50Hz 是上位机位置闭环/状态机的采样依据；云台位置只给状态屏和限位示教看，
   20Hz 足够，频率再高只会让阻塞式发送占用接收任务更多 CPU。 */
#define PI_RPT_FREQ_HZ          50   /* 里程计回传频率 */
#define PI_GIMBAL_RPT_FREQ_HZ   20   /* 云台三轴位置回传频率 */
#define PI_STATUS_FREQ_MS       200  /* 状态回传周期（5Hz） */

/* ==================== JY901S 陀螺仪（USART3） ==================== */

#define JY901S_OFFLINE_MS   200     /* 陀螺仪失联判定 */

/* ==================== 旧版小程序蓝牙遥控（USART2，已停用） ==================== */
/* 这个是早期的微信小程序文本协议（[joystick,...]）。现在 USART2 已经用作
   BLE_UPLINK_ENABLE 的二进制 PiComm 上行，两者互斥：BLE_UPLINK_ENABLE=1 时本开关无效。 */

#define BLE_CONTROL_ENABLE  0       /* 1=启用小程序文本协议 0=关闭 */

#define BLE_OFFLINE_MS      500     /* 小程序蓝牙失联判定（遥控包频率低，阈值放宽） */

/* ==================== 调试输出（USART1） ==================== */

#define DEBUG_HEAD          0x55AA55AA
#define DEBUG_TAIL          0xAA55AA55
#define DEBUG_VALUE_NUM     12      /* 每帧 float 数量，可改 */
#define DEBUG_RPT_FREQ_HZ   100

#endif
