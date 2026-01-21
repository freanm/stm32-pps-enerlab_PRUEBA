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
#include "mcp4131.c"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PER_ADC_CHANNEL_COUNT 4U
#define TOTAL_CHANNELS        6U

#define MAX_SAMPLES           130
#define MAX_RMS               128
#define MAX_RMS_PROM1         6

#define HYST                  40

#define I_MAX                 1638 //80% = 0.08 * 4095 / 2
#define I_MIN                 512  //25%


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
static float rms_prom1[TOTAL_CHANNELS][MAX_RMS_PROM1];
static float rms_prom2[TOTAL_CHANNELS];
static float rms_voltage[TOTAL_CHANNELS/2];
static float rms_current[TOTAL_CHANNELS/2];

static float P_inst[3][MAX_RMS];
static float P_prom1[3][MAX_RMS_PROM1];
static float P_prom2[3]; 

/*buffer para configurar wiper potenciometro Digital*/
int16_t I_max[TOTAL_CHANNELS/2] = {0,0,0};
int16_t I_min[TOTAL_CHANNELS/2] = {0,0,0};


/*flags */
uint8_t flag_adc_ready = 0;
uint8_t  en_region_alta = 0;
uint8_t  muestreo = 0;
uint8_t  primer_periodo = 1;
uint8_t flag_rms_ready = 0;

/*indices*/
uint16_t sample_index = 0;
int16_t rms_index = -1;
int16_t rms_prom1_index = -1;

int16_t v1 = 0;


static const uint8_t valid_channels[] = {0, 1, 2, 4, 5, 6};

static float calculate_rms(int16_t *buffer, uint8_t samples);
static float calculate_mean(float *buffer, uint8_t samples);
static float adc_to_voltage(float adc_value);
static float adc_to_current(float adc_value, uint8_t gain);

static float calculate_active_power(int16_t *v_buf, int16_t *i_buf, uint8_t samples);


static float vdda = 3.3f;

//static int16_t offset = 2048;
static uint8_t adc_calibrated = 0;


/* MCP4131 Digital Potentiometer handles */
static MCP4131_HandleTypeDef hpot3;  /* CS3 */
static MCP4131_HandleTypeDef hpot4;  /* CS4 */
static MCP4131_HandleTypeDef hpot5;  /* CS5 */

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

  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_dma_buf, PER_ADC_CHANNEL_COUNT);





// POTENCIONEMTRO DIGITAL REVISAR

  /* Initialize MCP4131 digital potentiometers */
  MCP4131_Init(&hpot3, &hspi1, SPI1_CS3_GPIO_Port, SPI1_CS3_Pin);
  MCP4131_Init(&hpot4, &hspi1, SPI1_CS4_GPIO_Port, SPI1_CS4_Pin);
  MCP4131_Init(&hpot5, &hspi1, SPI1_CS5_GPIO_Port, SPI1_CS5_Pin);

  uint8_t cambio_wiper = 0;
  int8_t wiper[TOTAL_CHANNELS/2] = {0,0,0};

  /* Wiper gain levels: x1, x2, x4, x8, x16, x32, x64 */
  const uint8_t gain_wiper[7] = {64, 86, 103, 114, 121, 125, 127};
  const uint8_t gain_table[7] = {1, 2, 4, 8, 16, 32, 64};


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // === Calibración de Vdda ===
    if (!adc_calibrated && flag_adc_ready) {
      ADC_MeasurementData_t adcData;
      __disable_irq();
      adcData = adcIncData;
      flag_adc_ready = 0;
      __enable_irq();

      uint16_t adc_vrefint = adcData.channels[3];

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

          rms_index = (rms_index + 1) % MAX_RMS;

          //  ---- Calcula y almacena RMS de MAX_RMS periodos  ----
          for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ch++) {
            rms_result[ch][rms_index] = calculate_rms(sample_buffer[ch], sample_index);
            //rms_voltage[ch] = adc_to_voltage(rms_result[ch][rms_index]);            
          }
          //  ---- Calcula y almacena P_Activa MAX_RMS periodos  ----
          for(uint8_t phase = 0; phase < 3; phase++) {
            P_inst[phase][rms_index] = calculate_active_power(sample_buffer[phase], sample_buffer[phase+3], sample_index);
          }
          //rms_index = (rms_index + 1) % MAX_RMS;
          
          //  ---- Promedio RMS y P_activa de MAX_RMS periodos  ----
          if(rms_index == MAX_RMS - 1){
            rms_prom1_index = (rms_prom1_index + 1) % MAX_RMS_PROM1;
            for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ch++) {
              rms_prom1[ch][rms_prom1_index] = calculate_mean(rms_result[ch], MAX_RMS);
              //rms_voltage[ch] = adc_to_voltage(rms_prom1[ch][rms_prom1_index]);
            }
            for (uint8_t phase = 0; phase < 3; phase++) {
              P_prom1[phase][rms_prom1_index] = calculate_mean(P_inst[phase], MAX_RMS);
            }
            //rms_prom1_index = (rms_prom1_index + 1) % MAX_RMS_PROM1;
          }

          //  ---- Promedio RMS y P_activa de MAX_RMS_PROM1 promedios de RMS y P_Activa  ----
          if(rms_prom1_index == MAX_RMS_PROM1 - 1){
            for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ch++) {
              rms_prom2[ch] = calculate_mean(rms_prom1[ch], MAX_RMS_PROM1);
            }

            for(uint8_t phase = 0; phase < 3; phase++) {
              rms_voltage[phase] = adc_to_voltage(rms_prom2[phase]);
              rms_current[phase] = adc_to_current(rms_prom2[phase+3], gain_table[wiper[phase]]);
            }



            for (uint8_t phase = 0; phase < 3; phase++) {
              P_prom2[phase] = calculate_mean(P_prom1[phase], MAX_RMS_PROM1);
            }
            //rms_prom1_index = 0;
            flag_rms_ready = 1;
          }

          //ajuste de wiper
          for(uint8_t ch=0; ch<TOTAL_CHANNELS/2; ch++){
            cambio_wiper = 0;

            if(I_max[ch] < I_MIN || I_min[ch] > -I_MIN){  //menor al 25% del rango de medicion -> aumento ganancia
              cambio_wiper = 1;
              wiper[ch]++;
              if(wiper[ch] > 6){
                wiper[ch] = 6;
              }
            }else{
              if(I_max[ch] > I_MAX || I_min[ch] < -I_MAX){  //mayor al 80% del rango de medicion -> disminuyo ganancia
                cambio_wiper = 1;
                wiper[ch]--;
                if(wiper[ch] < 0){
                  wiper[ch] = 0;
                }
                //AGREGAR: si hubo una saturacion de ADC, descartar este periodo
              }
            }
            

            if(cambio_wiper){
              switch(ch){
                case 0:
                  if (MCP4131_IsReady(&hpot3)) {
                    MCP4131_WriteWiper_DMA(&hpot3,gain_wiper[wiper[ch]]);
                  }
                  break;
                case 1:
                  if (MCP4131_IsReady(&hpot4)) {
                    MCP4131_WriteWiper_DMA(&hpot4,gain_wiper[wiper[ch]]);
                  }
                  break;
                case 2:
                  if (MCP4131_IsReady(&hpot5)) {
                    MCP4131_WriteWiper_DMA(&hpot5,gain_wiper[wiper[ch]]);
                  }
                  break;
              }
            }
            



          }

        }
      }
    }

    // === 4. Acumulación de muestras SOLO dentro del período ===
    if (muestreo) {
      if (sample_index < MAX_SAMPLES) {
        for (uint8_t i = 0; i < TOTAL_CHANNELS; i++) {
        //en el buffer "valid_channels" se tienen los canales de tension y corriente (se omite ch3 con VREFINT y ch7 con OFFSET)
          sample_buffer[i][sample_index] = (int16_t)adcData.channels[valid_channels[i]] - (int16_t)adcData.channels[7];
        }

        if(sample_index == 0){
          for(uint8_t ch=0; ch<TOTAL_CHANNELS/2; ch++){
            I_max[ch] = 0;
            I_min[ch] = 0;
          }
        }else{
          for(uint8_t ch=0; ch<TOTAL_CHANNELS/2; ch++){
            if(sample_buffer[ch+3][sample_index] > I_max[ch]){
              I_max[ch] = sample_buffer[ch+3][sample_index];
            }
            if(sample_buffer[ch+3][sample_index] < I_min[ch]){
              I_min[ch] = sample_buffer[ch+3][sample_index];
            }
          }
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
    return ((float)adc_value * vdda * 0.08153905715f);//adc_value*(197.7/0.5842)*(vdda/4095)
}

static float adc_to_current(float adc_value, uint8_t gain)
{
    return ((float)adc_value * (vdda / 2048) * (30.303 / gain) * (20.0f));
}


static float calculate_mean(float *buffer, uint8_t samples)
{
    if (samples == 0) return 0.0f;

    float acc = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
        acc += buffer[i];
    }

    return acc / (float)samples;
}


static float calculate_active_power(int16_t *v_buf, int16_t *i_buf, uint8_t samples)
{
    if (samples == 0) return 0.0f;

    float acc = 0.0f;
    for (uint16_t n = 0; n < samples; n++) {
        acc += (float)v_buf[n] * (float)i_buf[n];
    }

    return acc / (float)samples;
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
