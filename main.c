/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//USE your sample images
#include "panda2.h"
#include "bro3.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE         ((uint32_t)0xFF00)
#define WRITE_READ_ADDR     ((uint32_t)0x0000)
#define BUFFER1_ADDR		0xC0000000
#define BUFFER2_ADDR		0xC0200000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

/* USER CODE BEGIN PV */
uint32_t uwIndex = 0;
uint32_t test = 0;
__IO uint32_t uwWriteReadStatus = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void MX_DMA2D_Init(void);
/* USER CODE BEGIN PFP */
void MPU_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t Buffer=0x0;
_Bool Frontbuffer= 0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	FMC_Bank1->BTCR[0] = 0x000030D2;			//Disable FMC Bank1: Prevent the cpu booting from external device this prevents the speculative read accesses
	/* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  BSP_SDRAM_Init();
  MPU_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LTDC_Init();
  MX_DMA2D_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	//TEST CASE1:  Read two pictures from the internal flash WITH LTDC, changing first layer frame buffer address
//		  HAL_Delay(1000);
//		  HAL_LTDC_SetAddress(&hltdc,(uint32_t) &image_data_bro3,0);
//		  HAL_Delay(1000);
//		  HAL_LTDC_SetAddress(&hltdc, (uint32_t) &image_data_panda2,0);

//	//TEST CASE2:  Write the image to the external RAM and read back to verify the transfer using a simple for loop
//		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//			{
//			  *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00200000) + 4*uwIndex) = (uint32_t) image_data_bro3[uwIndex];//(uint32_t) image_data_panda2[uwIndex];
//			}
//
//		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//			{
//			  if( (uint32_t) image_data_bro3[uwIndex] != *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00200000) + 4*uwIndex))
//					{
//						test = 0x01;//&*(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + 4*uwIndex);
//					}
//			}

//	//TEST CASE3:  Write the image to the external RAM and read back to verify the transfer using a simple for loop, AND MPU unit activated
//		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//			{
//			  *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00200000) + 4*uwIndex) = (uint32_t) image_data_panda2[uwIndex];//(uint32_t) image_data_panda2[uwIndex];
//			}
//
//		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//			{
//			  if( (uint32_t) image_data_panda2[uwIndex] != *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00200000) + 4*uwIndex))
//					{
//						test = 0x01;//&*(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + 4*uwIndex);
//					}
//			}

//	//TEST CASE4:  Display an image from the external RAM. First fill the BUFFER on the RAM after that we change the first layer buffer address to the RAM
//		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//			{
//			  *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00200000) + 4*uwIndex) = (uint32_t) image_data_bro3[uwIndex];//(uint32_t) image_data_panda2[uwIndex];
//			}
//		HAL_Delay(1000);
//		HAL_LTDC_SetAddress(&hltdc, 0xC0200000,0);
//		HAL_Delay(1000);
//		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//					{
//					  *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00000000) + 4*uwIndex) = (uint32_t) image_data_panda2[uwIndex];//(uint32_t) image_data_panda2[uwIndex];
//					}
//		HAL_LTDC_SetAddress(&hltdc, 0xC0000000,0);

	  //Blending issues appeared after regenerating the code for DMA2D  the blending value was 0 for the first layer this makes the image 100% transparent

//	//TEST CASE5:  DMA2D testing the buffer area on the RAM for concurrent access issues  //OK
//		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//			{
//			  *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00200000) + 4*uwIndex) = (uint32_t) image_data_panda2[uwIndex];//(uint32_t) image_data_panda2[uwIndex];
//			}

//	//TEST CASE6: DMA2D transfer to the buffer area and verify the data // Not Working the image is is not visible, but the RAM get the data properly
//		HAL_DMA2D_Start(&hdma2d,(uint32_t) &image_data_panda2, 0xC0000000, 480, 272);
//		HAL_DMA2D_PollForTransfer(&hdma2d, 100);			//Waiting for the transfer complete, after that unlock the dma2d module 100msec timeout
//		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//			{
//			  if( (uint32_t) image_data_panda2[uwIndex] != *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00000000) + 4*uwIndex))
//					{
//						test = 0x01;//&*(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + 4*uwIndex);
//					}
//			}
//		HAL_LTDC_SetAddress(&hltdc, 0xC0000000,0);

//	//TEST CASE7: DMA2D TURN on but not used, transfer image to ram by for loop than display it // Not working Image not visible, Possible AHB bus issues concurrent access
//	  		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//	  			{
//	  			  *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00200000) + 4*uwIndex) = (uint32_t) image_data_bro3[uwIndex];//(uint32_t) image_data_panda2[uwIndex];
//	  			}
//	  		HAL_Delay(1000);
//	  		HAL_LTDC_SetAddress(&hltdc, 0xC0200000,0);
//	  		HAL_Delay(1000);
//	  		for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//	  					{
//	  					  *(__IO uint32_t*) (SDRAM_BANK_ADDR + (0x00000000) + 4*uwIndex) = (uint32_t) image_data_panda2[uwIndex];//(uint32_t) image_data_panda2[uwIndex];
//	  					}
//	  		HAL_LTDC_SetAddress(&hltdc, 0xC0000000,0);

//	//TEST CASE8: DMA2D turn on , Try to display an image from internal Flash  //Not Work-> Work problem was the blending value 0-> the first layer was 100% transparent
//	  HAL_LTDC_SetAddress(&hltdc, (uint32_t)&image_data_panda2,0);

	  //Blending issues end all previous tests work properly

//	//TEST CASE9: Two image transfer to the RAM by DMA2D and display it //Ok Blinking issue-> need apply the vertical blanking reload technique and frame buffer changing
//	  HAL_DMA2D_Start(&hdma2d,(uint32_t) &image_data_panda2, 0xC0000000, 480, 272);
//	  HAL_DMA2D_PollForTransfer(&hdma2d, 100);
//	  HAL_LTDC_SetAddress(&hltdc, 0xC0000000,0);
//	  HAL_DMA2D_Start(&hdma2d,(uint32_t) &image_data_bro3, 0xC0200000, 480, 272);
//	  HAL_DMA2D_PollForTransfer(&hdma2d, 100);
//	  HAL_LTDC_SetAddress(&hltdc, 0xC0200000,0);

//	//TEST CASE10: Frame buffer with pointers  // Not work, maybe you have to change differently the layer address
//	  HAL_DMA2D_Start(&hdma2d,(uint32_t) &image_data_panda2, BUFFER1_ADDR, 480, 272);
//	  Buffer=BUFFER1_ADDR;
//	  HAL_Delay(1000);
//	  HAL_DMA2D_Start(&hdma2d,(uint32_t) &image_data_bro3, BUFFER2_ADDR, 480, 272);
//	  Buffer=BUFFER2_ADDR;
//	  HAL_Delay(1000);

//	//TEST CASE11: Frame buffer switching during vertical blanking // OK
//	  if(Frontbuffer==0)
//		  {
//			  LTDC_Layer1->CFBAR = BUFFER1_ADDR;
//			  Frontbuffer =1;
//		  }
//	  else
//		  {
//			  LTDC_Layer1->CFBAR = BUFFER2_ADDR;
//			  Frontbuffer =0;
//		  }
//	  LTDC->SRCR = LTDC_SRCR_VBR;                     // reload shadow registers on vertical blank
//	  while ((LTDC->CDSR & LTDC_CDSR_VSYNCS) == 0)    // wait for reload
//			{
//			  ;
//			}
//	  HAL_Delay(200);

	//TEST CASE12: DAM2D + Vertical blanking frame reload
	  if(Frontbuffer==0)
		  {
		  	  HAL_DMA2D_Start(&hdma2d,(uint32_t) &image_data_panda2, BUFFER1_ADDR, 480, 272);
		  	  HAL_DMA2D_PollForTransfer(&hdma2d, 10);
		  }
	  else
		  {
			  HAL_DMA2D_Start(&hdma2d,(uint32_t) &image_data_bro3, BUFFER2_ADDR, 480, 272);
			  HAL_DMA2D_PollForTransfer(&hdma2d, 10);
		  }
	  if(Frontbuffer==0)
		  {
			  LTDC_Layer1->CFBAR = BUFFER1_ADDR;
			  Frontbuffer =1;
		  }
	  else
		  {
			  LTDC_Layer1->CFBAR = BUFFER2_ADDR;
			  Frontbuffer =0;
		  }
	  LTDC->SRCR = LTDC_SRCR_VBR;                     // reload shadow registers on vertical blank
	  while ((LTDC->CDSR & LTDC_CDSR_VSYNCS) == 0)    // wait for reload
			{
			  ;
			}
	  HAL_Delay(500);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 190;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 0;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 43;
  hltdc.Init.AccumulatedVBP = 21;
  hltdc.Init.AccumulatedActiveW = 523;
  hltdc.Init.AccumulatedActiveH = 293;
  hltdc.Init.TotalWidth = 531;
  hltdc.Init.TotalHeigh = 297;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EXT_RST_GPIO_Port, EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DISP_Pin */
  GPIO_InitStruct.Pin = LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DISP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_RST_Pin */
  GPIO_InitStruct.Pin = EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EXT_RST_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* MPU Configuration */
void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct;
	/* Disables the MPU */
	HAL_MPU_Disable();
	/* Configure the MPU attributes for region 0 */
	/* Configure the MPU attributes for SDRAM to normal memory*/
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0xC0000000;
	MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
	MPU_InitStruct.SubRegionDisable = 0x0;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Configure the MPU attributes for region 1 */
	/* Configure the MPU attributes for the frontbuffer to normal memory*/
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER1;
	MPU_InitStruct.BaseAddress = 0xC0000000;
	MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
	MPU_InitStruct.SubRegionDisable = 0x0;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Configure the MPU attributes for region 2 */
	/* Configure the MPU attributes for the backbuffer to normal memory*/
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER2;
	MPU_InitStruct.BaseAddress = 0xC0200000;
	MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
	MPU_InitStruct.SubRegionDisable = 0x0;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
