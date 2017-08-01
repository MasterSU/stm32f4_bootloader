/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* Definition Flash Sector----------------------------------------------------*/
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
/* Definition system error code-------------------------------------------------------*/
#define FLASHERASE_ERROR        0
#define LINKDISK_ERROR          1
#define FILE_ERROR              2
#define COPY_ERROR              3
#define CLKCONF_ERROR           4
/*******************************************************************************/
#define FLASH_APP_ADDR        0x080A0000
typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;
uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t SectorError = 0;
FATFS fs;												
FIL file;									
FRESULT f_res;           
uint32_t br,bw;
uint32_t copytime=0;
uint8_t i = 0; 
//uint8_t buffer[PAGE_SIZE];  
uint8_t   SystemError;
uint8_t   ReadAppBuffer[512];
//uint16_t  ChangeBuffer_HALFWORD[256];
//uint32_t  ChangeBuffer_WORD[256];

__asm void MSR_MSP(uint32_t addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE BEGIN 2 */
	/*uint32_t wbytes; 
	uint8_t wtext[] = "text to write logical disk";
	if(f_mount(&fs,(TCHAR *const)SD_Path,0)==FR_OK)
	{
		if(f_open(&file, "test.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
		{
			if(f_write(&file, wtext, sizeof(wtext), (void *)&wbytes) == FR_OK)
			{
				f_close(&file);
			}
		}
	}*/
	//FATFS_UnLinkDriver(SD_Path);
	
	/*Open file--------------------------------------------------*/
	int j=0,i=0;
	uint32_t  APP_Sector = 0;
	uint16_t  APP_Byte = 0;
	printf("Link Disk...\r\n");
	if(f_mount(&fs,(TCHAR *const)SD_Path,0) == FR_OK)
		printf("Link Disk successful...\r\n");
	else
	{
		SystemError = LINKDISK_ERROR;
		Error_Handler();
	}
	printf("Open File...\r\n");
	if(f_open(&file, "0:/app.bin", FA_OPEN_EXISTING | FA_READ) == FR_OK)
	{
		printf("Open File successful...\r\n");
		/*FLASH Erase-------------------------------------------------*/
		FirstSector = 9;
		NbOfSectors = 3;
		FLASH_EraseInitTypeDef pEraseInit;
		pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		pEraseInit.Sector = FirstSector;
		pEraseInit.NbSectors = NbOfSectors;
		HAL_FLASH_Unlock();  
		copytime=0;
		if(HAL_FLASHEx_Erase(&pEraseInit, &SectorError) == HAL_OK)
		{
			printf("FLASH Erase successful...\r\n");
			printf("Usage time: %.6fs\r\n",((float)copytime)/1000000.0);
		}
		else 
		{
			SystemError = FLASHERASE_ERROR;
			Error_Handler();
		}
		//f_read(&file, buffer, PAGE_SIZE, &br);  
		APP_Sector = file.fsize / 512;
		APP_Byte = file.fsize % 512;
		printf("app.bin size = %ld bytes\r\n",file.fsize);
		copytime=0;
		for(i = 0;i < APP_Sector;i++)
		{
			f_read(&file,ReadAppBuffer,512,(UINT *)&bw);
			for(j = 0;j < 256;j++)  
			{
				//ChangeBuffer_HALFWORD[j] = (ReadAppBuffer[j * 2 + 1] << 8) + ReadAppBuffer[j * 2];	
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,FLASH_APP_ADDR + i * 512 + j * 2,(ReadAppBuffer[j * 2 + 1] << 8) + ReadAppBuffer[j * 2])!= HAL_OK)
				{
					SystemError = COPY_ERROR;
					Error_Handler();
				}
			}
			//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,FLASH_APP_ADDR + i * 512,);	 
		}
		if(APP_Byte != 0)
		{
			f_read (&file,ReadAppBuffer,APP_Byte,(UINT *)&bw);
			for(j = 0;j < (APP_Byte / 2);j++)  
			{
				//ChangeBuffer_HALFWORD[j] = (ReadAppBuffer[j * 2 + 1] << 8) + ReadAppBuffer[j * 2];	
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,FLASH_APP_ADDR + APP_Sector * 512 + j * 2,(ReadAppBuffer[j * 2 + 1] << 8) + ReadAppBuffer[j * 2]) != HAL_OK)
				{
					SystemError = COPY_ERROR;
					Error_Handler();
				}
			}
			//HAL_FLASH_Program(FLASH_APP_ADDR + i * 512,ChangeBuffer_HALFWORD,APP_Byte / 2);
		}
		f_close(&file); 
		printf("Reload bin file successful...\r\n");
		printf("Usage time: %.6fs\r\n",((float)copytime)/1000000.0);
		HAL_TIM_Base_Stop_IT(&htim2);
		/*Jump to Application------------------------------------*/
		if(((*(__IO uint32_t*)FLASH_APP_ADDR)&0x2FFE0000)==0x20000000)	
		{ 
			Jump_To_Application = (pFunction)(*(__IO uint32_t*)(FLASH_APP_ADDR+4));	
			printf("Jump to Application...\r\n");				
			MSR_MSP(*(__IO uint32_t*)FLASH_APP_ADDR);					
			//printf("Jump to Application...\r\n");		
			Jump_To_Application();									   
		}
		else //Almost impossible so no definition
		{
			printf("Application address error...\r\n");			
			Error_Handler();
		}
	}
	else //printf("File not found...\r\n");
	{
		SystemError = FILE_ERROR;
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	copytime++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
	switch(SystemError)
	{
		case FLASHERASE_ERROR:
			printf("FLASH Erase error...\r\n");
			break;
		case LINKDISK_ERROR:
			printf("Link Disk error...\r\n");
			break;
		case FILE_ERROR:
			HAL_TIM_Base_Stop_IT(&htim2);
			printf("Open File error...\r\n");
			if(((*(__IO uint32_t*)FLASH_APP_ADDR)&0x2FFE0000)==0x20000000)	
			{ 
				Jump_To_Application = (pFunction)(*(__IO uint32_t*)(FLASH_APP_ADDR+4));	
				printf("Jump to Application...\r\n");				
				MSR_MSP(*(__IO uint32_t*)FLASH_APP_ADDR);					
				//printf("Jump to Application...\r\n");		
				Jump_To_Application();									   
			}
			break;
		case COPY_ERROR:
			printf("Copy File error...\r\n");
			break;
		case CLKCONF_ERROR:
			printf("Configure the system clock error...\r\n");
			break;
		default :
			printf("Magic error ^_^...\r\n");
			break;
	}
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
