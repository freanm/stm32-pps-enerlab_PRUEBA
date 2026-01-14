/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PER_ADC_CHANNEL_COUNT 4U
#define TOTAL_CHANNELS        8U

#define MAX_SAMPLES           130
#define MAX_RMS               10

//#define OFFSET                2048 + 45
#define HYST                  40
//#define ADC_SCALE_FACTOR  (311.f / 0.94f) * (3.24f / 4095.f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* DMA multimode buffer */
static uint32_t adc_dma_buf[PER_ADC_CHANNEL_COUNT];
static ADC_MeasurementData_t adcIncData;

/* Buffers */
static int16_t sample_buffer[TOTAL_CHANNELS][MAX_SAMPLES];
static float rms_result[TOTAL_CHANNELS][MAX_RMS];
static float rms_voltage[TOTAL_CHANNELS][MAX_RMS];
static float valor_medio[TOTAL_CHANNELS][MAX_RMS];

/*flags */
uint8_t flag_adc_ready = 0;
uint8_t  en_region_alta = 0;
uint8_t  muestreo = 0;
uint8_t  primer_periodo = 1;

uint16_t sample_index = 0;
uint16_t rms_index = 0;

int32_t v1 = 0;

uint8_t flag_rms_ready = 0;


static float calculate_rms(int16_t *buffer, uint8_t samples);
static float adc_to_voltage(float adc_value);
static float calculate_mean(int16_t *buffer, uint8_t samples);


static float vdda = 3.3f;

//static int16_t offset = 2048;
static uint8_t adc_calibrated = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Calibración ADC */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);

  HAL_TIM_Base_Start(&htim3);

  //HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_dma_buf, PER_ADC_CHANNEL_COUNT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if (!adc_calibrated && flag_adc_ready) {
      uint16_t adc_vrefint = adcIncData.channels[3];

      vdda = (1.21 * 4095.0f) / adc_vrefint;

      adc_calibrated = 1;
      flag_adc_ready = 0;
      continue;
    }



    if (!flag_adc_ready) continue;

    ADC_MeasurementData_t adcData;

    __disable_irq();
    adcData = adcIncData;
    flag_adc_ready = 0;
    __enable_irq();

    // === 1. Leer muestra ADC (referencia para inicio de periodo) ===
    v1 = adcData.channels[0] - adcData.channels[7];

    // === 2. Detección de cruce por cero con histéresis ===
    uint8_t cruce_ascendente = 0;

    if (!en_region_alta && (v1 > HYST)) {
      en_region_alta = 1;
      cruce_ascendente = 1;
    }
    else if (en_region_alta && (v1 < -HYST)) {
      en_region_alta = 0;
    }

    // === 3. Gestión de inicio / fin de período ===
    if (cruce_ascendente){
      if (!muestreo) {
        // ---- INICIO DE PERÍODO ----
        muestreo = 1;
        sample_index = 0;
      }
      else {
        // ---- FIN DE PERÍODO ----
        muestreo = 0;

        if (primer_periodo) {
          // descartar el primer período
          primer_periodo = 0;
        }
        else {
          for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ch++) {
            rms_result[ch][rms_index] = calculate_rms(sample_buffer[ch], sample_index);
            
            valor_medio[ch][rms_index] = calculate_mean(sample_buffer[ch], sample_index);

            rms_voltage[ch][rms_index] = adc_to_voltage(rms_result[ch][rms_index]);
          }
          rms_index = (rms_index + 1) % MAX_RMS;
          flag_rms_ready = 1;
  

        }
      }
    }

    // === 4. Acumulación de muestras SOLO dentro del período ===
    if (muestreo) {
      if (sample_index < MAX_SAMPLES) {
        for (uint8_t ch = 0; ch < TOTAL_CHANNELS - 2; ch++) {
          sample_buffer[ch][sample_index] = (int16_t)adcData.channels[ch] - (int16_t)adcData.channels[7];
        }
        sample_index++;
      }
      else {
        // seguridad: overflow del buffer
        muestreo = 0;
        sample_index = 0;
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance != ADC1) return;

    for (uint32_t i = 0; i < PER_ADC_CHANNEL_COUNT; i++) {
      uint32_t packed = adc_dma_buf[i];
      adcIncData.channels[i] = (uint16_t)(packed & 0xFFFF);
      adcIncData.channels[i + PER_ADC_CHANNEL_COUNT] = (uint16_t)((packed >> 16) & 0xFFFF);
    }

    flag_adc_ready = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        //uartReady = 1;
    }
}


static float calculate_rms(int16_t *buffer, uint8_t samples)
{
    if (samples == 0) return 0.0f;

    float sum = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
        sum += buffer[i] * buffer[i];
    }
    return (sqrtf((float) sum / samples));
}

static float adc_to_voltage(float adc_value)
{
    return ((float)adc_value * vdda * 0.08153905715);//adc_value*(197.7/05842)*(vdda/2095)
}

static float calculate_mean(int16_t *buffer, uint8_t samples)
{
    if (samples == 0) return 0.0f;

    int64_t acc = 0;
    for (uint16_t i = 0; i < samples; i++) {
        acc += buffer[i];
    }

    return (float)acc / (float)samples;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
