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

#define ZDT_ACC_DEFAULT     245    /* 云台位置模式加速度档位（0-255，越大越快，0=无曲线） */
#define ZDT_POS_SPEED_MAX   50     /* 云台位置模式速度上限，单位 RPM（整数） */
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
   值越小反馈越密、速度估计越准，但 CAN 总线占用越高（500K 下 4 轮返回约占 4*1000/周期 帧/s）。
   5ms=200Hz 是底盘常用的折中；要更精确可降到 2ms，但需注意总线负载。 */
#define ZDT_FEEDBACK_PERIOD_MS  5

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

/* 云台三轴电机(5/6/7)是否已安装。未安装时置 0，跳过所有 5/6/7 电机的 CAN 控制，
   避免向空地址发帧（这些帧无 ACK 会拖累 CAN 总线）。装好云台后改回 1。 */
#define GIMBAL_MOTOR_INSTALLED   0

/* 三轴软限位（绝对角度，单位 °），超限不发命令，防止撞结构 */
#define GIMBAL_YAW_MIN      -180.0f
#define GIMBAL_YAW_MAX      180.0f
#define GIMBAL_LIFT_MIN     0.0f
#define GIMBAL_LIFT_MAX     200.0f
#define GIMBAL_EXTEND_MIN   0.0f
#define GIMBAL_EXTEND_MAX   200.0f

/* ==================== 树莓派指令（USART6） ==================== */

#define PI_CMD_TIMEOUT_MS   500     /* 超过此时长收不到指令 → 底盘停车、云台保持 */
#define PI_RPT_FREQ_HZ      50      /* 遥测回传频率 */

/* ==================== JY901S 陀螺仪（USART3） ==================== */

#define JY901S_OFFLINE_MS   200     /* 陀螺仪失联判定 */

/* ==================== 蓝牙遥控（USART2，类比 DBUS） ==================== */

#define BLE_OFFLINE_MS      500     /* 蓝牙失联判定（遥控包频率低，阈值放宽） */

/* ==================== 调试输出（USART1） ==================== */

#define DEBUG_HEAD          0x55AA55AA
#define DEBUG_TAIL          0xAA55AA55
#define DEBUG_VALUE_NUM     12      /* 每帧 float 数量，可改 */
#define DEBUG_RPT_FREQ_HZ   100

#endif
