/**
 * @attention   采用UTF-8字符集编码
 * @brief       全项目总头文件集合（STM32F103C8T6 版）
 * @details     适合在每个.c文件开头调用。
 *              注意本文件没有ifndef，要防止递归调用（不要在任何头文件中包含此文件）。
 *              虽然降低编译效率，但可以避免手动解耦调用的麻烦。
 */

//系统
#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"
#include "math.h"
#include "cmsis_os.h"

//宏
#include "RobotConfig.h"
#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef FLOAT_MAX_VAL
#define FLOAT_MAX_VAL 3.4028234663852886e+38f
#endif

//外设
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "tim.h"

//外部硬件
#if MOTOR_USE_UART
#include "ZdtMotorUart.h"
#else
#include "ZdtMotor.h"
#endif
#include "Servo.h"
#include "Led.h"
#include "JY901S.h"
#include "PiComm.h"
#if BLE_CONTROL_ENABLE
#include "BleControl.h"
#endif

//软件
#include "Universal.h"
#include "UniversalPID.h"

//任务
#include "ReceiveTask.h"
#include "ImuTask.h"
#include "ChassisTask.h"
#include "GimbalTask.h"
/* DebugTask 已删除：USART1 改作串口电机总线，没有调试串口了 */
