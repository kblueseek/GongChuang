/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ==================== 转盘参数 ==================== */
#define ZDT_ADDR            1       /* 电机地址（电机侧需配成 1） */
#define ZDT_CHECKSUM        0x6B    /* 协议固定校验码 */

/* 120° = 3200/3 = 1066.67 脉冲。按 1067/1067/1066 循环，
   三次相加正好 3200（一整圈），长期运行不会累积角度误差。 */
#define TURN_PULSE_A        1067u
#define TURN_PULSE_B        1066u

#define TURN_PERIOD_MS      4000u   /* 每 4 秒转一次 */
#define TURN_SPEED_RPM      30u     /* 位置模式运行速度（整数 RPM，0~3000） */
#define TURN_ACC            100u    /* 加速度档位 0~255，越大越快；0=不加曲线直接启动 */

#define TURN_ALTERNATE      0       /* 1=正反交替（120°→回原位→120°…）  0=始终同向 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t s_RxByte;              /* UART5 单字节中断接收缓冲 */

volatile uint32_t g_MotorAckCount;    /* 合法应答帧计数（Keil Watch 里观察） */
volatile uint8_t  g_LastFunc;         /* 最近一次应答的功能码 */
volatile uint8_t  g_LastResult;       /* 最近一次应答的结果码：02=成功 E2/EE=错误 */

static uint32_t s_Step = 0;           /* 已转次数（用于 1067/1067/1066 循环） */
static uint32_t s_LastTurnTick;       /* 上次转向时刻 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== 张大头（ZDT）电机串口协议 ====================
   帧格式：[地址] [功能码] [命令数据...] [校验码 0x6B]
   接线：PC12(TX) → 电机 RX，PD2(RX) ← 电机 TX，GND 共地
   电机侧配置：通讯端口=串口，地址=1，波特率=115200 */

/* 发送一帧：地址 + 功能码 + n 字节数据 + 校验码 */
static void Zdt_Send(uint8_t func, const uint8_t* data, uint8_t n)
{
    uint8_t buf[16];
    uint8_t i = 0;

    buf[i++] = ZDT_ADDR;
    buf[i++] = func;
    for (uint8_t k = 0; k < n; k++)
        buf[i++] = data[k];
    buf[i++] = ZDT_CHECKSUM;

    HAL_UART_Transmit(&huart5, buf, i, 50);
}

/* 使能 / 锁轴：01 F3 AB 01 00 6B */
static void Zdt_Enable(void)
{
    uint8_t d[] = { 0xAB, 0x01, 0x00 };
    Zdt_Send(0xF3, d, sizeof(d));
}

/* 位置模式（FD）：相对当前实时位置转 pulse 个脉冲
   dir: 0=正向(CW)  1=反向(CCW)
   帧：01 FD dir speed(2B) acc pulse(4B) mode=02 sync=00 6B */
static void Zdt_MovePulses(uint32_t pulse, uint8_t dir)
{
    uint16_t spd = TURN_SPEED_RPM;
    uint8_t d[10];

    d[0] = dir;
    d[1] = (uint8_t)(spd >> 8);
    d[2] = (uint8_t)(spd & 0xFF);
    d[3] = TURN_ACC;
    d[4] = (uint8_t)(pulse >> 24);
    d[5] = (uint8_t)(pulse >> 16);
    d[6] = (uint8_t)(pulse >> 8);
    d[7] = (uint8_t)(pulse & 0xFF);
    d[8] = 0x02;        /* 2 = 相对当前实时位置 */
    d[9] = 0x00;        /* 0 = 立即执行 */

    Zdt_Send(0xFD, d, sizeof(d));
}

/* 应答帧长度（含地址、功能码、校验码） */
static uint8_t Zdt_FrameLen(uint8_t func)
{
    switch (func) {
    case 0x35: return 6;   /* 实时转速：addr + 35 + 符号 + 速度2B + 校验 */
    case 0x36: return 8;   /* 实时位置：addr + 36 + 符号 + 位置4B + 校验 */
    case 0x3C: return 5;   /* 状态：addr + 3C + 回零状态 + 电机状态 + 校验 */
    default:   return 4;   /* 通用应答：addr + func + 02/E2/EE + 6B */
    }
}

/* 串口是字节流，用状态机按帧长切分。仅用于调试观察，不参与控制 */
static void Zdt_RxByte(uint8_t b)
{
    static uint8_t state = 0;
    static uint8_t buf[8];
    static uint8_t idx = 0;
    static uint8_t len = 0;

    if (state == 0) {                       /* 等地址字节 1~7 */
        if (b >= 1 && b <= 7) {
            buf[0] = b;
            idx = 1;
            state = 1;
        }
        return;
    }

    if (state == 1) {                       /* 等功能码，据此确定整帧长度 */
        buf[1] = b;
        len = Zdt_FrameLen(b);
        idx = 2;
        state = 2;
        return;
    }

    buf[idx++] = b;                         /* 收集剩余字节 */
    if (idx >= len) {
        if (buf[len - 1] == ZDT_CHECKSUM) {
            g_MotorAckCount++;
            g_LastFunc = buf[1];
            g_LastResult = buf[2];          /* 通用应答：02=成功 E2/EE=错误 */
        }
        state = 0;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart5, &s_RxByte, 1);   /* 开启单字节中断接收，用于观察电机应答 */

  HAL_Delay(500);          /* 等电机上电稳定 */
  Zdt_Enable();            /* 使能锁轴 */
  HAL_Delay(100);

  s_LastTurnTick = HAL_GetTick();   /* 使能后开始计 4 秒 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - s_LastTurnTick >= TURN_PERIOD_MS)
    {
        uint32_t pulse;
        uint8_t  dir;

        s_LastTurnTick += TURN_PERIOD_MS;    /* 累加而不是重取时间，长时间运行周期不漂移 */

        /* 1067 / 1067 / 1066：每 3 次正好 3200 脉冲 = 360°，零累积误差 */
        pulse = (s_Step % 3 == 2) ? TURN_PULSE_B : TURN_PULSE_A;

#if TURN_ALTERNATE
        dir = (s_Step & 1) ? 1 : 0;          /* 奇数次反向（转过去再转回来） */
#else
        dir = 0;                             /* 0 = 始终正向 */
#endif

        Zdt_MovePulses(pulse, dir);
        s_Step++;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* 每收到 1 字节进一次中断；处理完必须重新武装，否则只会收到第一个字节 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) {
        Zdt_RxByte(s_RxByte);
        HAL_UART_Receive_IT(&huart5, &s_RxByte, 1);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
