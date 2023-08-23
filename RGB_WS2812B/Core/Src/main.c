/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include <stdlib.h>
#include "ws2812b.h"
#include <stdio.h>
#include <stdlib.h>
#include "ws2812b.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NUM_BANDS 8
#define SAMPLE_RATE 40000
#define F_SIZE 1024
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

const uint8_t gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };
uint16_t fourier[F_SIZE];
float fftBufIn[F_SIZE];
float fftBufOut[F_SIZE];

volatile uint8_t data_ready = 0;
uint8_t fftFlag = 0;
float peakVal = 0.0f;
uint16_t peakHz = 0;
arm_rfft_fast_instance_f32 fftHandler;

//------------------------------------------------------------------------------


#define SAMPLE_BUFFER_LENGTH        1024
#define SAMPLE_BUFFER_LENGTH_HALF   (SAMPLE_BUFFER_LENGTH/2)
#define SAMPLING_RATE               40000

float fft_input[SAMPLE_BUFFER_LENGTH];
float fft_output[SAMPLE_BUFFER_LENGTH];
float fft_power[SAMPLE_BUFFER_LENGTH_HALF];

uint8_t     ifftFlag                = 0;
float       frequency_resolution    = (float)SAMPLING_RATE / (float)SAMPLE_BUFFER_LENGTH;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

/*void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	__NOP();
}*/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    for(int i=0; i<F_SIZE; i++)
    {
    	//fft_input[i] = (float)(fourier[i]*(3.3/4096));
    }
    data_ready = 1;

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start(&htim3);
  //HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, &test, sizeof(test));
  /* USER CODE END 2 */
  ws2812b_init();
  //arm_rfft_fast_init_f32(&fftHandler, F_SIZE);
  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)fourier, F_SIZE);
  HAL_TIM_Base_Start(&htim6);


  while (1)
  {
	  /* write signal to array */
	     for (int i = 0; i < SAMPLE_BUFFER_LENGTH; i++) {
	         float r = (float)i / (float)SAMPLING_RATE;
	         r *= 3.14159265359 * 2;
	         r *= 10000; // frequency in Hz
	         float s = sin(r) + sin(r * 4) * 0.5 + sin(r * 3) * 0.25;
	         fft_input[i] = s;
	     }
	    // if(data_ready)
	     //{
	    	 arm_rfft_fast_instance_f32 fft;
	    		     arm_rfft_fast_init_f32(&fft, SAMPLE_BUFFER_LENGTH);
	    		     arm_rfft_fast_f32(&fft, fft_input, fft_output, ifftFlag);
	    		     arm_cmplx_mag_f32(fft_output, fft_power, SAMPLE_BUFFER_LENGTH_HALF);
	    		     float suma = 0;
	    		     for (int i = 1; i < SAMPLE_BUFFER_LENGTH_HALF; i++) {
	    		         /*printf("%i\tfrq: %.1f\tenergy %.6f\r\n", i, i * frequency_resolution, fft_power[i]);*/
	    		         if(i<20)
	    		         {
	    		         suma+=fft_power[i];
	    		         }
	    		         float32_t   maxValue;
	    		         uint16_t    maxIndex;
	    		         arm_max_f32(fft_power, SAMPLE_BUFFER_LENGTH_HALF, &maxValue, &maxIndex);
	    		         printf("max power: %f\r\n", maxValue);
	    		         printf("max index: %i\r\n", maxIndex);
	    		         printf("frequency: %f\r\n", (maxIndex * frequency_resolution));
	    		         //HAL_Delay(100);
	    		     }
	    		    // int sum = suma;
	    		     /*for(int i=0; i <2; i++)
	    		     {
	    		     printf("2suma %f\n", suma);
	    		     //HAL_Delay(10);
	    		     }*/
	    		     if(suma>100)
	    		     	        	  {
	    		     	        		  uint8_t r = gamma8[rand() % 256];
	    		     	        		  	  uint8_t g = gamma8[rand() % 256];
	    		     	        		  	  uint8_t b = gamma8[rand() % 256];

	    		     	        		      for (int led = 0; led < 24; led++)
	    		     	        		      {
	    		     	        		        ws2812b_set_color(led, r, g, b);
	    		     	        		        ws2812b_update();

	    		     	        		      }
	    		     	        		     HAL_Delay(10);

	    		     	        	  }

	    		     /* find dominant frequency */
	    		    // float32_t   maxValue;
	    		     //uint16_t    maxIndex;
	    		   //  uint16_t sum=0;

	    		    //
	    		    /*for (int i =1; i<10; i++)
	    		     {
	    		    	 sum+=fft_power[i];
	    		    	 if(sum>20)
	    		    	 {
	    		    		 printf("test sum = %lu\n", sum);
	    		    		 //HAL_Delay(1000);
	    		    	 }
	    		    	 else
	    		    	 {
	    		    		 printf("wiecej niz 1kHz = %lu\n", sum);
	    		    	 }
	    		     }
	    		     printf("\r\n");*/
	    		     //printf("max power: %f\r\n", maxValue);
	    		     //printf("max index: %i\r\n", maxIndex);
	    		     //printf("frequency: %f\r\n", (maxIndex * frequency_resolution));

	     /*} data_ready = 0;*/

	     /* analyze signal */

  }
	        // HAL_Delay(500);
	     //    data_ready = 0;
	        /* int val = maxValue;
	        	  if(val<1000)
	        	  {
	        		  uint8_t r = gamma8[rand() % 256];
	        		  	  uint8_t g = gamma8[rand() % 256];
	        		  	  uint8_t b = gamma8[rand() % 256];

	        		      for (int led = 0; led < 24; led++)
	        		      {
	        		        ws2812b_set_color(led, r, g, b);
	        		        ws2812b_update();
	        		        HAL_Delay(1);
	        		      }
	        	  }*/
	        // HAL_Delay(500);

	 /* if(data_ready)
	  {
		  arm_rfft_fast_f32(&fftHandler, &fftBufIn, &fftBufOut, 0);
	  }
	  data_ready = 0;

	  fftFlag = 1;

	  if(fftFlag)
	  {
		  peakVal = 0.0f;
		  peakHz = 0.0f;

		  uint16_t freq_Index = 0;

		  for(int i = 0; i<F_SIZE; i+=2)
		  {
			  float CurVal = sqrtf((fftBufOut[i]*fftBufOut[i])+(fftBufOut[i+1]*fftBufOut[i+1]));

			  if(CurVal>peakVal)
			  {
				  peakVal = CurVal;
				  peakHz = (uint16_t) (freq_Index*SAMPLE_RATE)/((float)F_SIZE);
			  }
			  freq_Index++;
		  }
	  }
	  printf("peakHz = %lu\n", peakHz);
	  HAL_Delay(10);*/
	//  HAL_Delay(100);

	  /*for(int i=0; i<1024; i++)
	  {
		  printf("f[%d] = %lu\n",i,fourier[i]);
	  }*/
	  /*uint8_t r = gamma8[rand() % 256];
	  uint8_t g = gamma8[rand() % 256];
	  uint8_t b = gamma8[rand() % 256];

    for (int led = 0; led < 24; led++)
    {
      ws2812b_set_color(led, r, g, b);
      ws2812b_update();
      HAL_Delay(100);
    }*/



   // HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
