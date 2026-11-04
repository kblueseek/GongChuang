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
/* ==================== 转盘参数（只改这里） ==================== */

/* 目标周期：转一圈用多少毫秒。8000 = 8 秒一圈。运行时也能用 Turntable_SetPeriodMs() 改 */
#define TURNTABLE_PERIOD_MS   8000u

/* 标定常数：满速（脉宽打到 1000us 或 2000us）时转一圈用多少毫秒。
   这个值决定「8 秒一圈」准不准，标定方法见下面 USER CODE 0 的说明。 */
#define SERVO_FULL_REV_MS     1000u

/* 方向：1 = 正转，-1 = 反转。装反了改这里，不用重新接线 */
#define TURNTABLE_DIR         1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 转盘运行时状态（Keil 的 Watch 窗口里可以直接看这几个变量） */
static uint32_t s_PeriodMs  = TURNTABLE_PERIOD_MS;   /* 当前目标周期 ms/圈 */
static uint32_t s_FullRevMs = SERVO_FULL_REV_MS;     /* 标定常数：满速一圈 ms */
static int8_t   s_Dir       = TURNTABLE_DIR;         /* +1 / -1 */
static uint8_t  s_Running   = 0;                     /* 1 = 正在转 */
static uint16_t s_PulseUs   = 1500u;                 /* 当前下发脉宽 us（调试用） */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==================== 转盘驱动：360° 连续旋转舵机 ====================
   接线：舵机信号线（橙/白）→ PA6（TIM3_CH1）
         舵机电源（红）     → 独立 5V（不要从单片机的 3.3V 取，堵转能到 1A）
         舵机地（棕/黑）    → 与单片机 GND 共地（必须！不共地信号无效）

   原理：50Hz PWM（20ms 周期），脉宽决定转向和转速：
           1000us ──── 1500us ──── 2000us
           满速反转     停        满速正转
         偏离 1500us 越多转越快，转速大致和偏移量成正比。

   为什么这里有标定常数：360° 舵机没有位置/速度反馈，单片机不知道它转多快，
   只能"给一个脉宽，假设它以某个速度转"。所以：
       目标周期 → 偏移量 = 满速偏移(500us) × 满速一圈用时(SERVO_FULL_REV_MS) ÷ 目标周期
   SERVO_FULL_REV_MS 就是唯一的标定量，标定一次就准了。

   标定步骤：
     1. 把 SERVO_FULL_REV_MS 保持默认 1000（=假设满速 1 秒一圈），烧进去。
     2. 在转盘上贴一个记号，秒表计 10 圈的总时间，除以 10 得到实际一圈 T_实测(ms)。
     3. 新值 = 1000 × T_实测 ÷ 8000，填回 SERVO_FULL_REV_MS，重新烧录。
        例：实测一圈 12s → 1000 × 12000 ÷ 8000 = 1500，把 1000 改成 1500。
     4. 再测一次微调，一般一到两轮就能对到 ±0.1s。

   注意：8 秒一圈只需要 62us 左右的偏移量，接近很多舵机的死区边缘。如果发现
   舵机根本不转或时转时停，说明这个舵机慢不到这么慢，处理办法见文末说明。
   ================================================================ */

/* ---- 电气参数：按你的舵机手册改 ---- */
#define SERVO_STOP_US   1500u                            /* 停止脉宽 */
#define SERVO_MIN_US    1000u                            /* 满速反向 */
#define SERVO_MAX_US    2000u                            /* 满速正向 */
#define SERVO_HALF_US   (SERVO_STOP_US - SERVO_MIN_US)   /* 满速偏移量 = 500us */

/* 定时器时钟（Hz）。F1 的 TIM2/3/4 挂在 APB1，TIM1 在 APB2；
   APB 预分频 ≠ 1 时定时器时钟是总线时钟的 2 倍。 */
static uint32_t TimerClockHz(TIM_TypeDef* tim)
{
    uint32_t pclk = (tim == TIM1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    return (pclk == HAL_RCC_GetHCLKFreq()) ? pclk : pclk * 2u;
}

/* 把定时器统一成「1 个计数 = 1us、周期 20ms」。
   这样 CCR 的数值就直接等于脉宽(us)，和舵机手册对得上；
   即使 CubeMX 里 PSC/ARR 填错了，这里也会纠正过来。 */
static void Servo_SetTimebase(void)
{
    uint32_t timClk = TimerClockHz(htim3.Instance);

    htim3.Instance->PSC = timClk / 1000000u - 1u;
    htim3.Instance->ARR = 20000u - 1u;      /* 20000us = 20ms = 50Hz */
    htim3.Instance->EGR = TIM_EGR_UG;       /* 立即装载新的 PSC/ARR */
}

static void Servo_WriteUs(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
}

/* 由目标周期反算脉宽：转速 ∝ 偏移量，周期 ∝ 1/偏移量 */
static uint16_t Servo_PulseForPeriod(uint32_t periodMs)
{
    uint32_t offset;

    if (periodMs < 10u)
        periodMs = 10u;                     /* 防除零；10ms 一圈已远超舵机能力 */

    offset = (uint32_t)(((uint64_t)SERVO_HALF_US * s_FullRevMs + periodMs / 2u) / periodMs);
    if (offset > SERVO_HALF_US)
        offset = SERVO_HALF_US;             /* 转到满速就饱和在 1000/2000us */

    return (s_Dir >= 0) ? (uint16_t)(SERVO_STOP_US + offset)
                        : (uint16_t)(SERVO_STOP_US - offset);
}

/* ---------------- 对外接口 ---------------- */

/* 初始化：配时基、开 PWM，脉宽先停在 1500us（不转） */
static void Turntable_Init(void)
{
    Servo_SetTimebase();
    Servo_WriteUs(SERVO_STOP_US);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

/* 主要接口：运行时改周期（ms/圈），转着的时候改立即生效 */
static void Turntable_SetPeriodMs(uint32_t periodMs)
{
    s_PeriodMs = periodMs;
    if (s_Running) {
        s_PulseUs = Servo_PulseForPeriod(s_PeriodMs);
        Servo_WriteUs(s_PulseUs);
    }
}

/* 标定接口：满速一圈的毫秒数 */
static void Turntable_SetFullRevMs(uint32_t ms)
{
    s_FullRevMs = (ms < 10u) ? 10u : ms;
    Turntable_SetPeriodMs(s_PeriodMs);      /* 重新算一遍脉宽 */
}

/* 方向接口：+1 正转 / -1 反转 */
static void Turntable_SetDir(int8_t dir)
{
    s_Dir = (dir >= 0) ? 1 : -1;
    Turntable_SetPeriodMs(s_PeriodMs);
}

static void Turntable_Start(void)
{
    s_PulseUs = Servo_PulseForPeriod(s_PeriodMs);
    Servo_WriteUs(s_PulseUs);
    s_Running = 1;
}

static void Turntable_Stop(void)
{
    s_PulseUs = SERVO_STOP_US;
    Servo_WriteUs(s_PulseUs);
    s_Running = 0;
}

/*
 * 【如果 8 秒一圈舵机转不动怎么办】
 * 大多数 360° 舵机在 1500us 附近有 30~60us 的死区，8 秒一圈算出来只有 62us 偏移，
 * 正好卡在边缘。两个办法：
 *   ① 机械减速：舵机轴上套一级齿轮/同步带，减速比 N，舵机转 N 圈转盘转 1 圈。
 *      代码里把 TURNTABLE_PERIOD_MS 改成 8000/N（舵机自己的周期），最省事。
 *   ② 走走停停：让舵机以 4 秒一圈转 4 秒、停 4 秒，平均也是 8 秒一圈。
 *      缺点是动作不连续，转盘会一顿一顿的。
 */
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
  Turntable_Init();        /* 配时基 + 开 PWM，脉宽先停在 1500us（不转） */
  HAL_Delay(500);          /* 等舵机上电自检稳定（有的舵机上电会抖一下） */
  Turntable_Start();       /* 按 TURNTABLE_PERIOD_MS 开始转 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 转盘是纯硬件 PWM，主循环里没有周期性的活要干。
       下面是几个可以在调试时临时打开用的接口：

         Turntable_SetPeriodMs(5000);   // 改成 5 秒一圈，立即生效
         Turntable_SetFullRevMs(1200);  // 改标定常数（满速一圈 ms）
         Turntable_SetDir(-1);          // 反转
         Turntable_Stop();              // 停
         Turntable_Start();             // 按当前周期重新开始
    */
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
/* 转盘不需要串口：全部逻辑都在上面 USER CODE 0 里，主循环空转即可。 */
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
