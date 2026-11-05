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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "balance.h"
#include "uart_cmd.h"
#include "display_link.h"
#include "motor.h"
#include "dmp_platform.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*==============================重定义printf===============================*/
int fputc(int ch, FILE *stream)
{
    (void)stream;
    return UART_SendByte((uint8_t)ch) ? (int)(uint8_t)ch : EOF;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /* =====================	关 JTAG，释放 PB3/PB4 等引脚，留 SWD 调试用 ===================*/
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
/*
 * 先开启调参串口,后续好输出报告
 * 校准ADC
 */
  if (UART_Start(&huart2) != HAL_OK) Error_Handler();
  /* The display link is optional and must not stop the balance controller. */
  (void)DisplayLink_Init(&huart1);
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) Error_Handler();
  /* 防止读取DMP时候卡顿明显 */
  /* Screen drawing now runs on the ESP32-S3. */
  UART_Printf("I2C MPU6050/MPU6500 v11 CAL-STATS AUTO-START TILT45 RECOVER200ms; SCL=PB10 SDA=PB11 addr=0x68\r\n");
  /*
   * Balance_Init 初始化控制器和硬件，并进入姿态校准阶段，尚不立即转动电机
   * 启动 TIM1 中断，200Hz（5ms）控制节拍，到点触发 Balance_OnTickISR 提交控制节拍
   */
  if (Balance_Init() != HAL_OK)
      UART_Printf("Balance init FAILED; STATE=%u FAULT=%u\r\n",
                  (unsigned)balance_state, (unsigned)balance_fault);
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) Error_Handler();
  UART_Printf("Balance 200Hz; PWM 20kHz; PB1 active-low press toggles START/STOP\r\n");
  UART_Printf("Hold upright and still; auto-start after calibration; PB1 toggles STOP/START\r\n");
  uint32_t last_display_link = HAL_GetTick() - 200U;
  uint32_t last_remote_command = 0U;
  uint8_t remote_active = 0U;
  UART_Printf("I2C1 PB8/PB9 100k; I2C2 MPU PB10/PB11 400k; display USART1 PA9/PA10\r\n");
  uint32_t last_print = HAL_GetTick();                                              //VOFA 上位机绘图数据的时间标志
  uint32_t last_led = HAL_GetTick();                                                //PC13 状态灯翻转的时间标志
  uint32_t last_diag_print = HAL_GetTick();                                         //打完整诊断信息的时间标志
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
   * 主循环快速轮询，不固定延时5ms：Balance_Poll 内部由 TIM1 中断提交的节拍决定是否执行PID计算，
   * 按键扫描、串口命令处理也在主循环轮询，避免在中断里做耗时操作。
   */
while (1)
  {
    Balance_Poll();
    Balance_KeyPoll();
    UART_Poll();
    uint32_t now = HAL_GetTick();
    float remote_speed, remote_turn;
    if (DisplayLink_PollCommand(&remote_speed, &remote_turn) && balance_state == BALANCE_RUNNING) {
        target_speed = remote_speed;
        target_turn = remote_turn;
        last_remote_command = now;
        remote_active = 1U;
    }
    if (remote_active && (balance_state != BALANCE_RUNNING ||
        (uint32_t)(now - last_remote_command) >= 500U)) {
        target_speed = 0.0f;
        target_turn = 0.0f;
        remote_active = 0U;
    }
    /*
     * 基础信息如果出故障就5s打印一次，没有就1s打印一次，防止数据打印太多，波特率太低卡死
     * 100ms发一次数据，但是在报错情况下不发送数据
     */
    if ((uint32_t)(now - last_diag_print) >=
        (balance_state == BALANCE_RUNNING ? 5000U : 1000U)) {
        last_diag_print = now;
        Balance_PrintDiagnostics();
    }
    if (balance_state != BALANCE_FAULT && (uint32_t)(now - last_print) >= 100U) {
        last_print = now;
        /*
         *  输出当前角度，角速度，A轮速度，B轮速度，平均速度，直立环输出的PWM
         *  速度目标，速度环输出量，速度环积分输出量，直立环P输出量，直立环D输出量
         *  位置环输出量，转向环输出量,转向环误差
         */
        UART_Printf(":%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                    angle, raw_gx, speedA, speedB, speed_filt, balance_pwm,
                    position_speed_target, pid_speed.output, pid_speed.integral,
                    pid_balance.kp * pid_balance.error, -pid_balance.kd * raw_gx,
                    pid_position.output,pid_turn.output,pid_turn.error);
    }
    /*
     * 如果正常校准好，就一直常亮，否则用标志位判断防止堵塞，实现约250ms反转一次电平
     */
    if (balance_state == BALANCE_RUNNING)
        HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
    else if ((uint32_t)(now - last_led) >= 250U) {
        last_led = now;
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    }
    /* Latest status snapshot, nonblocking USART1 transmission every 200 ms. */
    if ((uint32_t)(now - last_display_link) >= 200U) {
        last_display_link = now;
        DisplayStatus d;
        d.state = (uint8_t)balance_state;
        d.fault = balance_fault;
        d.angle = angle;
        d.gyro = raw_gx;
        d.encoder_a = speedA;
        d.encoder_b = speedB;
        d.pwm_a = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
        d.pwm_b = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);
        d.stby = HAL_GPIO_ReadPin(STBY_PORT, STBY_PIN) == GPIO_PIN_SET;
        d.key = HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET;
        (void)DisplayLink_SendStatus(&d);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
/*
 * 初始化等关键操作失败会到这里：先停机，再关中断并等待复位�??
 */
Motor_Disable();
  __disable_irq();
  while (1) { }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
