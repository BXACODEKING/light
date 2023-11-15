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
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "xensiv_bgt60trxx.h"
#include "presence_radar_settings.h"
#include <inttypes.h>
#include "usbd_cdc_if.h"
#include "interpeaks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define XENSIV_BGT60TRXX_SPI_FREQUENCY (25000000UL) //define clk = 25MHZ
#define NUM_SAMPLES_PER_FRAME (XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS *      \
                               XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME * \
                               XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP)
#define NUM_SAMPLES_PER_FRAME_ADC_RAW_DATA (NUM_SAMPLES_PER_FRAME * 3 / 2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	static  xensiv_bgt60trxx_t sensor;
	static volatile bool data_available = false;

/* Allocate enough memory for the radar dara frame. */
static uint16_t samples[NUM_SAMPLES_PER_FRAME];
static uint8_t adcRawData[NUM_SAMPLES_PER_FRAME_ADC_RAW_DATA];
	bool tranformSpeed = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	void ti2infineon(float input[800],float output[1600] ) {
			for (int i = 0; i < 800; i++) {
					output[2 * i] = input[i];
			}
	}

	void BIT12_TO_BIT16(uint8_t *A, uint16_t *B, uint16_t Alen)
	{
		uint32_t tmp = 0;
		uint16_t *samplesPt = B;
		for (size_t i = 0; i < Alen; i = i + 3)
		{
			tmp = ((A[i] << 16) | (A[i + 1] << 8) | A[i + 2]);
			*samplesPt = ((tmp >> 12) & 0xfff);
			samplesPt++;
			*samplesPt = tmp & 0xfff;
			samplesPt++;
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
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_QUADSPI_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, LED_GREEN_Pin | LED_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_LDO_GPIO_Port, EN_LDO_Pin, GPIO_PIN_SET);
  static xensiv_bgt60trxx_st_iface_t spiface;
  spiface.spi = &hspi1;
  spiface.selpin.GPIOx = SPI1_NSS_GPIO_Port;
  spiface.selpin.GPIO_Pin = SPI1_NSS_Pin;
  spiface.rstpin.GPIOx = BGT_RST_GPIO_Port;
  spiface.rstpin.GPIO_Pin = BGT_RST_Pin;
	

  uint32_t rslt;
  rslt = xensiv_bgt60trxx_init(&sensor, &spiface, tranformSpeed);
  xensiv_bgt60trxx_get_device(&sensor);
//	usb_printf("milli: %D,AMPLITUDE: %D",sensor.iface)

  uint32_t Nchipid;
  uint32_t NADC0;
	//uint32_t NSTAT1;
 // HAL_Delay(5000);
  xensiv_bgt60trxx_get_reg(&sensor, XENSIV_BGT60TRXX_REG_CHIP_ID, &Nchipid);
  usb_printf("chipid:%08X\n", Nchipid);
  xensiv_bgt60trxx_get_reg(&sensor, XENSIV_BGT60TRXX_REG_ADC0, &NADC0);
  usb_printf("ADC0:%08X\n", NADC0);
	 //xensiv_bgt60trxx_get_reg(&sensor, XENSIV_BGT60TRXX_REG_ADC0, &NSTAT1);
	//usb_printf("ADC0:%08X\n", NSTAT1);
  if (rslt == XENSIV_BGT60TRXX_STATUS_OK)
  {
    xensiv_bgt60trxx_hard_reset(&sensor);
  }
  if (rslt == XENSIV_BGT60TRXX_STATUS_OK)
  {
    rslt = xensiv_bgt60trxx_config(&sensor, register_list, XENSIV_BGT60TRXX_CONF_NUM_REGS);
  }
  if (rslt != XENSIV_BGT60TRXX_STATUS_OK)
  {
    goto Error_Assert;
  }
  if (xensiv_bgt60trxx_enable_data_test_mode(&sensor, false) != XENSIV_BGT60TRXX_STATUS_OK)
  {
    goto Error_Assert;
  }
  
  if (xensiv_bgt60trxx_start_frame(&sensor, true) != XENSIV_BGT60TRXX_STATUS_OK)
  {
    goto Error_Assert;
  }
HAL_Delay(1000);
  uint32_t frame_idx = 0;
  // uint16_t test_word = XENSIV_BGT60TRXX_INITIAL_TEST_WORD;
  // uint32_t tmp = 0;
  // uint16_t *samplesPt = samples;
  uint8_t headflag[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Wait for the radar device to indicate the availability of the data to fetch. */
    while (data_available == false){
		};     
    data_available = false;
    //CDC_Transmit_FS(headflag, 8);

    if (xensiv_bgt60trxx_get_fifo_data(&sensor, adcRawData,
                                       NUM_SAMPLES_PER_FRAME_ADC_RAW_DATA) == XENSIV_BGT60TRXX_STATUS_OK)
    {
      //xensiv_bgt60trxx_start_frame(&sensor, false);
      // for (size_t i = 0; i < NUM_SAMPLES_PER_FRAME_ADC_RAW_DATA; i = i + 3)
      // {
      //   tmp = ((adcRawData[i] << 16) | (adcRawData[i + 1] << 8) | adcRawData[i + 2]);
      //   *samplesPt = ((tmp >> 12) & 0xfff);
      //   samplesPt++;
      //   *samplesPt = tmp & 0xfff;
      //   samplesPt++;
      // }
      // for (int32_t sample_idx = 0; sample_idx < NUM_SAMPLES_PER_FRAME; ++sample_idx)
      // {
      //   if (test_word != samples[sample_idx])
      //   {
      //     usb_printf("Sam: error_1__  ");
      //     usb_printf("Frame %" PRIu32 " error detected. "
      //            "Expected: %" PRIu16 ". "
      //            "Received: %" PRIu16 "\n",
      //            frame_idx, test_word, samples[sample_idx]);
      //     // goto Error_Assert;
      //   }

      //   // Generate next test_word
      // test_word = xensiv_bgt60trxx_get_next_test_word(test_word);
      // }
      //CDC_Transmit_FS(adcRawData, NUM_SAMPLES_PER_FRAME_ADC_RAW_DATA);
     // xensiv_bgt60trxx_start_frame(&sensor, true);
	
		static unsigned short uv[2]={0,};
		float breath_count;
		float heart_count;
		static float test_data[1600] ={0.0,};
		uint16_t adcRaw_u16[128];
		
		BIT12_TO_BIT16(adcRawData,adcRaw_u16,192);
		
		
		//static unsigned short fv[128]= {1546, 1344, 1196, 887, 579, 374, 209, 68, 81, 233, 393, 505, 573, 629, 733, 772, 701, 556, 323, 189, 247, 353, 434, 552, 678, 762, 795, 757, 708, 653, 610, 640, 691, 701, 719, 710, 691, 728, 811, 929, 1007, 887, 737, 640, 525, 478, 533, 705, 870, 989, 1112, 1210, 1176, 1053, 982, 981, 970, 826, 638, 583, 627, 781, 964, 1050, 1078, 1133, 1180, 1230, 1277, 1303, 1236, 1051, 934, 910, 942, 1050, 1124, 1156, 1175, 1131, 1094, 1056, 1014, 1060, 1129, 1124, 1126, 1237, 1441, 1620, 1670, 1644, 1531, 1284, 970, 693, 650, 757, 860, 1017, 1202, 1309, 1354, 1404, 1391, 1248, 1147, 1150, 1145, 1169, 1269, 1435, 1587, 1720, 1847, 1888, 1840, 1820, 1761, 1604, 1401, 1268, 1238, 1246, 1274, 1304, 1297, 1271};	
			
		volatile int start = HAL_GetTick();
		interpeaks(adcRaw_u16, &breath_count, &heart_count);
		volatile int end = HAL_GetTick();
		//HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);
		//HAL_Delay(1000);
		usb_printf("time:%dms heart:%lf breath:%lf  %d-----\n",end-start,heart_count,breath_count,adcRaw_u16[0]);
//	 for(int i=0;i<128;i++){
//		usb_printf("%d \t",adcRaw_u16[i]);
//		}
//	 usb_printf("----------------------------\n");
	 
    }
    else
    {
      goto Error_Assert;
    }

    // usb_printf("Frame %" PRIu32 " received correctly\n", frame_idx);
    frame_idx++;

    // goto Error_Assert;
    // HAL_Delay(1000);


//	
//		
//	void ti2infineon(float input[800],float test_data[1600] ) {
//    for (int i = 0; i < 800; i++) {
//        output[2 * i] = input[i];
//    }
//}	
	
    



  }
Error_Assert:
  while (true)
  {
   // HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    CDC_Transmit_FS("Error_Assert\n", 13);
    HAL_Delay(100);
  };
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  data_available = true;
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
