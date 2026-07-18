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
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad9959.h"
#include "Phase.h"
#include "ZPN_Uart.h"
#include "adf4002.h"
#include "Phase.h"
#include "DFT.h"
#include "WindowFunction.h"
#include "Modulation.h"
#include "ZPN_Hmi.h"
#include "ModRecognizer.h"
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
static void Relay_SetByType(MR_ModType_t type);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  ZPN_UART_Init();
  Init_AD9959();

  InitADF4002();
  RDivideTest(2);
  NDivideTest(2);

  FFT_App_Init();
  AD9959_SetChannel(0,1000,0,1950000);
  AD9959_IO_Update();

  MR_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

    /* PB12 按键: 上升沿翻转 PB2 LED */
    {
        static uint8_t last_key = 0;
        uint8_t key = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
        if (key && !last_key)
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        last_key = key;
    }

    MR_Result_t r = MR_Process();
    if (r.type != MR_TYPE_NONE) {
        MR_SendResult(&r);

        static const char *names[] = {"CW","AM","2ASK","2PSK","FM","2FSK"};
        static MR_ModType_t last_display = (MR_ModType_t)0xFF;
        static MR_ModType_t type_hist[20] = {0};
        static uint8_t hist_idx = 0;

        /* 20帧历史记录 */
        type_hist[hist_idx] = r.type;
        hist_idx = (hist_idx + 1) % 20;

        /* 每帧统计20帧窗口内各类型次数 */
        uint8_t cnt[6] = {0};
        for (int i = 0; i < 20; i++)
            if (type_hist[i] <= MR_TYPE_2FSK) cnt[type_hist[i]]++;

        /* 找最多的类型 */
        MR_ModType_t display_type = MR_TYPE_FM;
        uint8_t max_cnt = 0;
        for (int t = 0; t < 6; t++)
            if (cnt[t] > max_cnt) { max_cnt = cnt[t]; display_type = (MR_ModType_t)t; }

        /* FM/2FSK 同时存在 → FM需>70%才判FM, 否则2FSK */
        if (cnt[MR_TYPE_FM] > 0 && cnt[MR_TYPE_2FSK] > 0)
            display_type = (cnt[MR_TYPE_FM] > 14) ? MR_TYPE_FM : MR_TYPE_2FSK;
        /* 仅 2PSK+FM 存在 (无2FSK) → 选 2PSK */
        if (cnt[MR_TYPE_2PSK] > 0 && cnt[MR_TYPE_FM] > 0 && cnt[MR_TYPE_2FSK] == 0)
            display_type = MR_TYPE_2PSK;

        /* 类型变化时才更新 HMI */
        if (display_type != last_display) {
            last_display = display_type;
            HMI_SetText("t1", names[display_type]);
            Relay_SetByType(display_type);

            switch (display_type) {
            case MR_TYPE_CW:
                HAL_GPIO_WritePin(GPIOE, Relay_2_Pin|Relay_3_CT1_Pin|Relay_3_CT2_Pin|Relay_4_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOE, Relay_1_Pin, GPIO_PIN_SET);
                HMI_SetFloat("x0", 0.0f, 1);
                HMI_SetFloat("x1", 0.0f, 1);
                HMI_SetFloat("x2", 0.0f, 1);
                HMI_SetFloat("x3", 0.0f, 1);
                HMI_SetFloat("x5", 0.0f, 1);
                break;
            case MR_TYPE_AM:
                {
                float ma_cal = r.ma;
if (r.ma < 85.0f) ma_cal += 10.0f;
if (r.ma < 60.0f) ma_cal += 5.0f;
                HMI_SetFloat("x0", ma_cal, 1);
                }
                HMI_SetFloat("x1", 0.0f, 1);
                HMI_SetFloat("x2", 0.0f, 1);
                HMI_SetFloat("x3", 0.0f, 1);
                HMI_SetFloat("x5", roundf(r.fm / 1000.0f) * 10.0f, 2);
                break;
            case MR_TYPE_FM:
                HMI_SetFloat("x0", r.mf * 100.0f, 1);
                HMI_SetFloat("x1", r.df / 100.0f, 2);
                HMI_SetFloat("x2", 0.0f, 1);
                HMI_SetFloat("x3", 0.0f, 1);
                HMI_SetFloat("x5", roundf(r.fm / 1000.0f) * 10.0f, 2);
                break;
            case MR_TYPE_2ASK:
                HMI_SetFloat("x0", 0.0f, 1);
                HMI_SetFloat("x1", 0.0f, 1);
                HMI_SetFloat("x2", roundf(r.baud / 1000.0f) * 20.0f, 2);
                HMI_SetFloat("x3", 0.0f, 1);
                HMI_SetFloat("x5", 0.0f, 1);
                break;
            case MR_TYPE_2PSK:
                HMI_SetFloat("x0", 0.0f, 1);
                HMI_SetFloat("x1", 0.0f, 1);
                HMI_SetFloat("x2", roundf(r.baud / 1000.0f) * 20.0f, 2);
                HMI_SetFloat("x3", 0.0f, 1);
                HMI_SetFloat("x5", 0.0f, 1);
                break;
            case MR_TYPE_2FSK: {
                HMI_SetFloat("x0", 0.0f, 1);
                HMI_SetFloat("x1", 0.0f, 1);
                HMI_SetFloat("x2", roundf(r.baud / 1000.0f) * 20.0f, 2);
                HMI_SetFloat("x3", r.h, 2);
                HMI_SetFloat("x5", 0.0f, 1);
                break;
            }
            default:
                break;
            }
        }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void Relay_SetByType(MR_ModType_t type)
{
    switch (type) {
    case MR_TYPE_CW:
        HAL_GPIO_WritePin(GPIOE, Relay_2_Pin|Relay_3_CT1_Pin|Relay_3_CT2_Pin|Relay_4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, Relay_1_Pin, GPIO_PIN_SET);
        break;
    case MR_TYPE_AM:
        HAL_GPIO_WritePin(GPIOE, Relay_1_Pin|Relay_2_Pin|Relay_3_CT1_Pin|Relay_3_CT2_Pin|Relay_4_Pin, GPIO_PIN_RESET);
        break;
    case MR_TYPE_FM:
        HAL_GPIO_WritePin(GPIOE, Relay_2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, Relay_1_Pin|Relay_3_CT1_Pin|Relay_3_CT2_Pin|Relay_4_Pin, GPIO_PIN_SET);
        break;
    case MR_TYPE_2ASK:
        HAL_GPIO_WritePin(GPIOE, Relay_1_Pin|Relay_3_CT1_Pin|Relay_3_CT2_Pin|Relay_4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, Relay_2_Pin, GPIO_PIN_SET);
        break;
    case MR_TYPE_2PSK:
        HAL_GPIO_WritePin(GPIOE, Relay_2_Pin|Relay_3_CT2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, Relay_1_Pin|Relay_3_CT1_Pin|Relay_4_Pin, GPIO_PIN_SET);
        break;
    case MR_TYPE_2FSK:
        HAL_GPIO_WritePin(GPIOE, Relay_2_Pin|Relay_3_CT2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, Relay_1_Pin|Relay_3_CT1_Pin|Relay_4_Pin, GPIO_PIN_SET);
        break;
    default:
        break;
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
