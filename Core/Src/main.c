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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* ---- 开火链路协议（与 Jetson 侧 fire_link.hpp 逐字节一致）---- */
#define FIRE_CMD_HEAD0   0xA5u
#define FIRE_CMD_HEAD1   0x5Au
#define FIRE_FB_HEAD0    0x5Au
#define FIRE_FB_HEAD1    0xA5u
#define FIRE_FB_TYPE     0x81u
#define FIRE_FB_TAIL     0x0Du
#define FIRE_CMD_WORD    0x10u   /* 开火命令字 */
#define FIRE_CMD_LEN     6u      /* A5 5A 10 seq flags crc8 */
#define FIRE_FB_LEN      7u      /* 5A A5 81 seq state crc8 0D */

#define FIRE_FLAG_ON     0x01u   /* 命令 flags bit0：请求开火 */

#define FIRE_ST_FIRE_ON  0x01u   /* 反馈 state bit0：PA12 当前高 */
#define FIRE_ST_TIMEOUT  0x02u   /* 反馈 state bit1：链路看门狗已触发 */
#define FIRE_ST_OVERTIME 0x04u   /* 反馈 state bit2：连续导通保护已触发（锁存） */

/* ---- 安全参数 ---- */
#define FIRE_LINK_TIMEOUT_MS   100u    /* 100 ms 收不到合法命令 -> 强制停火 */
#define FIRE_FB_PERIOD_MS      20u     /* 50 Hz 回状态帧 */
#define FIRE_MAX_ON_MS         5000u   /* 连续导通上限 5 s，超时强制关断，松手解锁 */
#define FIRE_DEBUG_LOG         1       /* USART2 打印事件日志（与开火数据同一口，帧解析会自动跳过 ASCII） */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    volatile uint8_t rx_byte;        /* HAL_UART_Receive_IT 的单字节落点 */
    uint8_t  parse_buf[FIRE_CMD_LEN];
    uint8_t  parse_state;            /* 0=等 A5, 1=等 5A, 2..5=数据, 6=CRC 待判 */
    uint32_t last_rx_ms;             /* 最近一条合法命令的时刻 */
    uint32_t rx_frames, bad_frames;
    uint8_t  last_seq;               /* 回显给 Jetson */
    uint8_t  fire_req;               /* 命令请求（MOS 应导通） */
    uint8_t  fire_on;                /* PA12 实际状态 */
    uint8_t  overtime;               /* 连续导通保护锁存 */
    uint32_t on_since_ms;
    uint32_t fb_seq;
    uint32_t last_fb_ms;
} fire_ctx_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 开火链路宏定义见上方 USER CODE BEGIN Includes（typedef 在其后要用） */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static fire_ctx_t g_fire;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t fire_crc8(const uint8_t *data, uint32_t len);
static void    fire_rx_byte(uint8_t b);
static void    fire_apply_cmd(uint8_t seq, uint8_t flags);
static void    fire_apply_output(uint32_t now_ms);
static void    fire_send_feedback(uint32_t now_ms);
static void    dbg_print(const char *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* CRC-8/ATM（poly 0x07, init 0），与 Jetson 侧 fire_link.hpp 一致 */
static uint8_t fire_crc8(const uint8_t *data, uint32_t len)
{
  uint8_t crc = 0x00;
  while (len-- > 0)
  {
    crc ^= *data++;
    for (int i = 0; i < 8; ++i)
      crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
  }
  return crc;
}

/* 逐字节状态机，由 USART2 接收中断喂入 */
static void fire_rx_byte(uint8_t b)
{
  switch (g_fire.parse_state)
  {
    case 0:
      if (b == FIRE_CMD_HEAD0) { g_fire.parse_buf[0] = b; g_fire.parse_state = 1; }
      break;
    case 1:
      if (b == FIRE_CMD_HEAD1)      { g_fire.parse_buf[1] = b; g_fire.parse_state = 2; }
      else if (b != FIRE_CMD_HEAD0) { g_fire.parse_state = 0; }
      break;
    default:
      g_fire.parse_buf[g_fire.parse_state] = b;
      ++g_fire.parse_state;
      if (g_fire.parse_state >= FIRE_CMD_LEN)
      {
        if (g_fire.parse_buf[2] == FIRE_CMD_WORD &&
            fire_crc8(g_fire.parse_buf, FIRE_CMD_LEN - 1) ==
                g_fire.parse_buf[FIRE_CMD_LEN - 1])
          fire_apply_cmd(g_fire.parse_buf[3], g_fire.parse_buf[4]);
        else
          ++g_fire.bad_frames;
        /* 重同步：当前字节若又是帧头，下一帧从它开始 */
        g_fire.parse_state = (b == FIRE_CMD_HEAD0) ? 1 : 0;
      }
      break;
  }
}

/* 合法命令帧：刷新看门狗 + 记录扳机请求 */
static void fire_apply_cmd(uint8_t seq, uint8_t flags)
{
  g_fire.last_rx_ms = HAL_GetTick();
  g_fire.last_seq   = seq;
  ++g_fire.rx_frames;
  g_fire.fire_req   = (flags & FIRE_FLAG_ON) ? 1u : 0u;
}

/* 扳机 -> PA12（低电平开火，高电平关火），带连续导通保护 */
static void fire_apply_output(uint32_t now_ms)
{
  if (!g_fire.fire_req)
  {
    g_fire.overtime = 0;                 /* 松扳机解除锁存 */
    if (g_fire.fire_on)
    {
      g_fire.fire_on = 0;
      HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);
      dbg_print("[biu] FIRE OFF\r\n");
    }
    return;
  }

  if (g_fire.overtime) return;           /* 保护锁存中：等松扳机 */

  if (!g_fire.fire_on)
  {
    g_fire.fire_on     = 1;
    g_fire.on_since_ms = now_ms;
    HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
    dbg_print("[biu] FIRE ON\r\n");
    return;
  }

  if ((now_ms - g_fire.on_since_ms) > FIRE_MAX_ON_MS)
  {
    g_fire.overtime = 1;
    g_fire.fire_on  = 0;
    HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);
    dbg_print("[biu] OVERTIME -> fire OFF (release trigger to reset)\r\n");
  }
}

/* 状态帧（50 Hz）：回真实 PA12 电平与保护状态 */
static void fire_send_feedback(uint32_t now_ms)
{
  uint8_t f[FIRE_FB_LEN];
  uint8_t state = 0;

  if ((now_ms - g_fire.last_fb_ms) < FIRE_FB_PERIOD_MS) return;
  g_fire.last_fb_ms = now_ms;

  if (g_fire.fire_on)                                      state |= FIRE_ST_FIRE_ON;
  if ((now_ms - g_fire.last_rx_ms) > FIRE_LINK_TIMEOUT_MS) state |= FIRE_ST_TIMEOUT;
  if (g_fire.overtime)                                     state |= FIRE_ST_OVERTIME;

  f[0] = FIRE_FB_HEAD0;
  f[1] = FIRE_FB_HEAD1;
  f[2] = FIRE_FB_TYPE;
  f[3] = g_fire.last_seq;
  f[4] = state;
  f[5] = fire_crc8(f, FIRE_FB_LEN - 2);
  f[6] = FIRE_FB_TAIL;
  (void)HAL_UART_Transmit(&huart2, f, FIRE_FB_LEN, 10);
}

/* USART2(DAP-LINK) 事件日志：只在状态变化时打，避免阻塞刷屏 */
static void dbg_print(const char *s)
{
#if FIRE_DEBUG_LOG
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 20);
#else
  (void)s;
#endif
}

/* UART2 接收中断回调：喂解析器并重新挂上 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    fire_rx_byte(g_fire.rx_byte);
    (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_fire.rx_byte, 1);
  }
}

/* 噪声/溢出等错误会让 HAL 停掉接收，这里重新挂上，避免永久失联 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_fire.rx_byte, 1);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ---- 开火链路初始化：PA12 先拉高(关火，低电平有效)，UART2 开中断接收 ---- */
  memset(&g_fire, 0, sizeof(g_fire));
  g_fire.last_rx_ms = HAL_GetTick();
  g_fire.last_fb_ms = HAL_GetTick() - FIRE_FB_PERIOD_MS;
  HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_fire.rx_byte, 1);
  dbg_print("[biu] ready: UART2 115200 8N1, PA12=FIRE, FB 50Hz\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    const uint32_t now = HAL_GetTick();

    /* 1. 链路看门狗：100 ms 没有合法命令 -> 强制停火（最硬的一道保险） */
    if (g_fire.fire_req && (now - g_fire.last_rx_ms) > FIRE_LINK_TIMEOUT_MS)
    {
      g_fire.fire_req = 0;
      dbg_print("[biu] LINK TIMEOUT -> fire OFF\r\n");
    }

    /* 2. 输出到 MOS（带连续导通保护） */
    fire_apply_output(now);

    /* 3. 50 Hz 回状态帧 */
    fire_send_feedback(now);

    /* 4. LED：链路正常 1 Hz 慢闪，链路丢失 5 Hz 快闪 */
    {
      static uint32_t last_led_ms = 0;
      static uint8_t  link_prev  = 0xFF;
      const uint8_t   link_ok    = ((now - g_fire.last_rx_ms) <= FIRE_LINK_TIMEOUT_MS);
      const uint32_t  period     = link_ok ? 500u : 100u;
      if (link_ok != link_prev)
      {
        link_prev = link_ok;
        dbg_print(link_ok ? "[biu] LINK OK\r\n" : "[biu] LINK LOST\r\n");
      }
      if (now - last_led_ms >= period)
      {
        last_led_ms = now;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1);   /* 主循环 1 kHz 轮询足够；接收走中断不受影响 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
