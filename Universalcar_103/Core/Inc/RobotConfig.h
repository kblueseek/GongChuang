#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       整车配置宏
 * @details     集中管理所有可调参数，调车只改这里
 */

/* ==================== 电机传输方式（C8T6 只能二选一：CAN 或 串口） ==================== */

/* 0 = CAN 电机（ZdtMotor.c），USART1（PA9/PA10）空出来做调试口
   1 = 串口电机（ZdtMotorUart.c，走 USART1）
   C8T6 只有 USART1/2/3：USART3 固定给 JY901S；
   USART2 平时接树莓派，树莓派不用时可改接蓝牙模块（见下方 BLE_CONTROL_ENABLE）。
   两个驱动 .c 都加进了 Keil 工程，但各自整体用条件编译包住：
     ZdtMotor.c     -> #if !MOTOR_USE_UART
     ZdtMotorUart.c -> #if  MOTOR_USE_UART
   只有被选中的那份参与编译，改这一个宏即可切换，不用动 Keil 工程。
   （两者定义了同名符号，若同时编译会报 L6200E multiply defined。） */
#define MOTOR_USE_UART      0

/* 电机地址（CAN 或串口通用）：1-4 底盘麦轮，5-7 云台三轴 */
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
   CAN 500K 下 5ms(200Hz) 绰绰有余；串口 115200 下不行：
     4 轮位置帧 = 4×8B，底盘速度帧 = 37B。
     5ms 周期 → (32+37)B/5ms ≈ 13.8KB/s > 115200 的 11.5KB/s，总线撑不住；
     20ms 周期 → (32/4+37)B/5ms ≈ 9.0KB/s，约 78% 占用，可用。
   若把 USART1 和电机都改成 460800，可把此值降回 5。 */
#if MOTOR_USE_UART
#define ZDT_FEEDBACK_PERIOD_MS  20
#else
#define ZDT_FEEDBACK_PERIOD_MS  5
#endif

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

#define LED_PWM_CH       TIM_CHANNEL_1   /* 补光灯  TIM2_CH1 / PA0（F103 上由 PA5 改到 PA0） */
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

/* ==================== 树莓派指令（USART2） ==================== */

#define PI_CMD_TIMEOUT_MS   500     /* 超过此时长收不到指令 → 底盘停车、云台保持 */
#define PI_RPT_FREQ_HZ      50      /* 遥测回传频率 */

/* ==================== JY901S 陀螺仪（USART3） ==================== */

#define JY901S_OFFLINE_MS   200     /* 陀螺仪失联判定 */

/* ==================== 遥控源选择（树莓派 / 蓝牙 二选一） ====================
   C8T6 只有 3 个串口：USART3 固定给 JY901S，剩下 USART2(原树莓派) 和 USART1(原调试口)。
   树莓派暂时不用 → 蓝牙模块直接插原来的树莓派 4 针口（USART2），一根线都不用改。
   1 = 蓝牙遥控：PiComm 协议停止解析、停止回传（二进制遥测会污染蓝牙模块）
   0 = 树莓派 PiComm 协议（原逻辑，树莓派恢复后改回 0 即可） */
#define BLE_CONTROL_ENABLE  1

/* 蓝牙模块接在哪个串口：
   2 = USART2 (PA2=TX, PA3=RX)，原树莓派接口，推荐
   1 = USART1 (PA9=TX, PA10=RX)，原调试口；此模式要求 MOTOR_USE_UART=0 */
#define BLE_UART_PORT       2

#if BLE_CONTROL_ENABLE && (BLE_UART_PORT == 1) && MOTOR_USE_UART
#error "USART1 conflict: set MOTOR_USE_UART=0 when BLE_UART_PORT=1"
#endif

#define BLE_OFFLINE_MS      500     /* 蓝牙失联判定（遥控包频率低，阈值放宽） */

/* ---- 波形推送（MCU → 小程序）----
   帧格式（ASCII，与小程序波形页一致）：
     数据：[p,v1,v2,...v6]\r\n    每个值两位小数，例如 [p,12.34,-5.67,89.01]
     清屏：[p-c]\r\n
   当前推送 JY901S 三轴角度：v1=Yaw  v2=Pitch  v3=Roll（单位 °）
   20Hz × 约 30B ≈ 600B/s，115200 下每帧阻塞约 2.6ms，可按需调低 */
#define BLE_PLOT_ENABLE     1
#define BLE_PLOT_FREQ_HZ    20

/* ==================== 调试输出（已停用） ====================
   C8T6 版 DebugTask 已删除，USART1 改作电机总线，以下是 F407 版遗留宏，
   目前没有任何 .c 引用，仅保留以便将来换到有富余串口的板子。 */

#define DEBUG_HEAD          0x55AA55AA
#define DEBUG_TAIL          0xAA55AA55
#define DEBUG_VALUE_NUM     12      /* 每帧 float 数量，可改 */
#define DEBUG_RPT_FREQ_HZ   100

#endif
