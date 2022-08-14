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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "rtc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "at.h"
#include "lowpower.h"
#include "nbInit.h"
#include "ultrasound.h" // AMS - remove once tested
#include "stm32l0xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* debug macro definition ----------------------------------------------------*/

/* NBIOT task macro definition, the task in NBTASK will not be executed after commenting out -*/
//#define NBIOT

//#define ST_DEBUG	
#ifndef ST_DEBUG
	#define lowpower_enter
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AMS_DEBUG
#define uart1_nibble_size 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// uint8_t	1 byte	max 255
// uint16_t	2 bytes	max 65,535 unsigned - A 2-byte signed integer can have a range from -32,768 to 32,767
// uint32_t 4 bytes 	max 4,294,967,295 unsigned, -2,147,483,648 to 2,147,483,647 signed
// https://developer.arm.com/documentation/dui0472/k/C-and-C---Implementation-Details/Basic-data-types-in-ARM-C-and-C--

#ifdef AMS_DEBUG
	static uint32_t loopcount = 0; // AMS - remove once debugged
#endif // AMS_DEBUG

static uint16_t distance_uart1; // max 4000mm
uint16_t get_distance_uart1(void) // return to extern
{
	return distance_uart1;
}


// USART1
static uint8_t uart1_nibble[uart1_nibble_size] = {0};
static uint8_t uart1_rx = 0; // flag

// USART2
static uint8_t  rxbuf_2 = 0;			
static uint16_t rxlen_2 = 0;
static uint8_t  rxDATA_2[300]={0};
static uint8_t  uart2_recieve_flag = 0;

static uint8_t rxbuf_lp;
static uint8_t lpuart_recieve_flag = 0;

static uint8_t pwd_time_count = 0;			//Password time count times

static uint8_t task_num = _AT_IDLE;			//NB task directory

static uint8_t error_num = 0;				    //Error count

static uint8_t uplink_time_num = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void USERTASK(void);
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
	#ifdef AMS_DEBUG
		__HAL_DBGMCU_FREEZE_IWDG(); // AMS - remove once debugged
	#endif // AMS_DEBUG
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
  MX_LPUART1_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
//	new_firmware_update();
//	config_Get();
	reboot_information_print();
//	product_information_print();
//	List_Init(sys.list);
	//HAL_Delay(3000);
//	HAL_UART_Receive_IT(&hlpuart1,(uint8_t*)&rxbuf_lp,1);
//	HAL_UART_Receive_IT(&huart2,(uint8_t*)&rxbuf_2,1);
//	My_UARTEx_StopModeWakeUp(&huart2);				//Enable serial port wake up
//	My_UARTEx_StopModeWakeUp(&hlpuart1);			//Enable serial port wake up
//	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	#ifdef AMS_DEBUG
		HAL_GPIO_WritePin(Power_5v_GPIO_Port, Power_5v_Pin, GPIO_PIN_RESET); // AMS - set back after debugging pin off = 5v on
		HAL_UART_Receive_DMA(&huart1,(uint8_t*)&uart1_nibble,uart1_nibble_size); // circular
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_nibble,uart1_nibble_size);
//		HAL_StatusTypeDef res = HAL_UART_Receive_DMA(&huart1, (uint8_t*)&uart1_nibble,uart1_nibble_size);
//		printf("start HAL_UART_Receive_DMA res = %u\r\n", res);
		led_on(5000);
	#else
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)&uart1_nibble,uart1_nibble_size);
	#endif // AMS_DEBUG
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef NBIOT
	if((*(__IO uint8_t *)EEPROM_USER_START_FDR_FLAG) == 0x01)
	{
		task_num = _AT_IDLE;
		HAL_FLASHEx_DATAEEPROM_Unlock();
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,EEPROM_USER_START_FDR_FLAG,0x00);
		HAL_FLASHEx_DATAEEPROM_Lock();
	}
	else
		task_num = _AT_FLAG_INIT;
#else
	task_num = _AT_IDLE;
#endif
  while (1)
  {
		led_on(100);
		#ifdef AMS_DEBUG
			HAL_IWDG_Refresh(&hiwdg);
			loopcount++;
			if ((loopcount % 100) < 1)
			{
				printf("loop %i |", loopcount);
			}
		HAL_Delay(20);
		#endif // AMS_DEBUG
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(uart1_rx)
		{
			for (int i = 0; i < uart1_nibble_size; i++)
			{
				printf("%02x |", uart1_nibble[i]);
	//				uart1_nibble[id] = 0;
			}
			if(uart1_nibble[0] == 0xff) // header
			{
				uint8_t checksum = (uart1_nibble[0]+uart1_nibble[1]+uart1_nibble[2])&0x00FF; ;
				if(checksum==uart1_nibble[3])
				{
					distance_uart1=(uart1_nibble[1]<<8) + uart1_nibble[2];
					printf(" = %u mm\r\n",distance_uart1);
				}
				else
				{
					printf("\r\n");
				}
			}
			else if(uart1_nibble[2] == 0xff) // header
			{
				uint8_t checksum = (uart1_nibble[2]+uart1_nibble[3]+uart1_nibble[0])&0x00FF; ;
				if(checksum==uart1_nibble[1])
				{
					distance_uart1=(uart1_nibble[3]<<8) + uart1_nibble[0];
					printf(" = %u mm\r\n",distance_uart1);
				}
				else
				{
					printf("\r\n");
				}
			}
			else
			{
				printf(" unknown order\r\n");
	//			distance_uart1=(uint8_t)(uart1_nibble[3]<<8)+uart1_nibble[0];
			}
			led_on(200);
			uart1_rx = 0;
		}

		if (uart2_recieve_flag)
			{
				printf("rxDATA_2 = %s", (char*)rxDATA_2); 
				memset(rxDATA_2, 0, rxlen_2);
				rxlen_2 = 0;
				uart2_recieve_flag	 = 0;
				HAL_UART_Receive_IT(&huart1,(uint8_t*)&uart1_nibble,uart1_nibble_size);
			}			
		
#ifdef NBIOT		
		
		if(task_num != _AT_IDLE)
		{
			HAL_Delay(1000);
			
			if(NBTASK(&task_num) == _AT_ERROR)
			{
				error_num++;
			}
	  }
		if(error_num > 3 && nb.uplink_flag == no_status)
		{
			user_main_printf("Restart the module...");
			task_num = _AT_NRB;
			error_num = 0;
		}
		
		if(nb.recieve_flag == NB_RECIEVE && nb.uplink_flag == send)
		{
			task_num = _AT_URI;
			nb.recieve_flag = NB_IDIE;
		}
#endif
		
//		USERTASK();

//#ifdef lowpower_enter
//		if(task_num == _AT_IDLE && uart2_recieve_flag==0)
//		{
//			LPM_EnterStopMode(SystemClock_Config);
//		}
//#endif
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void USERTASK(void)
{
	if(sys.pwd_flag==1 && sys.pwd_flag == 0 && pwd_time_count == 0)
	{
		HAL_UART_Transmit(&huart2,(uint8_t*)"Password timeout\r\n",20,20);
	}
	
	if(sys.pwd_flag == 0 && uart2_recieve_flag)
	{
		rtrim((char*)rxDATA_2);
		if(strcmp((char*)rxDATA_2,(char*)sys.pwd) == 0 && strlen((char*)rxDATA_2) < 9)
		{
			user_main_printf("Password Correct");
			sys.pwd_flag = 1;
		}
		else
		{
			user_main_printf("Password Incorrect");
		}
	}
	else if(sys.pwd_flag != 0 && uart2_recieve_flag)
	{
		ATEerror_t AT_State = ATInsPro((char*)rxDATA_2);
		if(AT_State == AT_OK)
			printf("%s\r\n",ATError_description[AT_OK]);
		else if(AT_State == AT_PARAM_ERROR)
			printf("%s",ATError_description[AT_PARAM_ERROR]);
		else if(AT_State == AT_BUSY_ERROR)
			printf("%s",ATError_description[AT_BUSY_ERROR]);
		else if(AT_State == AT_TEST_PARAM_OVERFLOW)
			printf("%s",ATError_description[AT_TEST_PARAM_OVERFLOW]);
		else if(AT_State == AT_RX_ERROR)
			printf("%s",ATError_description[AT_RX_ERROR]);
		else if(AT_State == AT_ERROR)
		{
			nb.usart.len = 0;
			memset(nb.usart.data,0,NB_RX_SIZE);
			rxDATA_2[strlen((char*)rxDATA_2)] = '\n';
			HAL_UART_Transmit_DMA(&hlpuart1,(uint8_t*)rxDATA_2,strlen((char*)rxDATA_2));		
			HAL_Delay(500);					//Waiting to Send
		}
	}
		
	if(uart2_recieve_flag == 1)
	{
		rxlen_2 = 0;
		uart2_recieve_flag = 0;
		memset(rxDATA_2,0,sizeof(rxDATA_2));
	}
	if(lpuart_recieve_flag == 1)
	{
		printf("%s",nb.usart.data);
		nb.usart.len = 0;
		lpuart_recieve_flag = 0;
		memset(nb.usart.data,0,NB_RX_SIZE);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &hlpuart1) // modem
	{
		nb.usart.data[nb.usart.len++] = rxbuf_lp;	
		if(task_num == _AT_IDLE)
		{
			if(rxbuf_lp == '\r' || rxbuf_lp == '\n')
			{
				lpuart_recieve_flag = 1;
			}
		}
		HAL_UART_Receive_IT(&hlpuart1,(uint8_t*)&rxbuf_lp,1);		
	}
	else if(huart == &huart1) // Async sensor A02YYUW/A0221AU 
	{
		uart1_rx = 1;
		//		HAL_UART_DMAPause(&huart1);
	}
	else if(huart == &huart2) // console
	{
		rxDATA_2[rxlen_2++] = rxbuf_2;
		if(rxbuf_2 == '\r' || rxbuf_2 == '\n')
		{
			uart2_recieve_flag = 1;		
		}
		HAL_UART_Receive_IT(&huart2,(uint8_t*)&rxbuf_2,1);
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	printf("HAL_RTC_AlarmAEventCallback - kick off upload\r\n"); //AMS should be every 5 sec
	if(nb.net_flag == no_status)
		task_num = _AT_FLAG_INIT;
	else if(nb.net_flag == success && nb.uplink_flag == no_status)
		task_num = _AT_CSQ;
	
	LPM_DisableStopMode();
}

void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
	HAL_IWDG_Refresh(&hiwdg);
	printf("HAL_RTCEx_AlarmBEventCallback fed the dog\r\n"); //AMS should be every 5 sec
	
#ifdef NBIOT	
	if(sys.pwd_flag==1)
	{
		pwd_time_count++;
		if(pwd_time_count == 17)
		{
			sys.pwd_flag = 0;
			pwd_time_count = 0;
		}
	}
#endif	

	if(nb.net_flag == fail)
		task_num = _AT_CSQ;
	if(nb.net_flag == success && nb.uplink_flag == send)
	{
		uplink_time_num++;
		if(uplink_time_num>=7)
		{
			uplink_time_num = 0;
			error_num++;
			if(sys.protocol == COAP_PRO)			{task_num = _AT_COAP_CLOSE;}
			else if(sys.protocol == UDP_PRO)	{task_num	=	_AT_UDP_CLOSE;}
			else if(sys.protocol == MQTT_PRO)	{task_num	=	_AT_MQTT_CLOSE;}
			else if(sys.protocol == TCP_PRO)	{task_num	=	_AT_TCP_CLOSE;}
		}
	}
	else 
		uplink_time_num = 0;
	
	My_AlarmInit(18,1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{	
	if(GPIO_Pin == GPIO_PIN_14)
	{		
		if(sys.mod == model6)
			sensor.exit_count++;
		else if(nb.net_flag == success && sys.mod != model6 && (task_num < _AT_COAP_CONFIG || task_num > _AT_TCP_CLOSE))
		{
			task_num = _AT_CSQ;
			sys.exit_flag = 1;
			//sensor.exit_state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==1?1:2;
			sensor.exit_state = 1;
			LPM_DisableStopMode();	
		}
	}
	else if(GPIO_Pin == GPIO_PIN_13)
	{		
		nb.recieve_flag = NB_RECIEVE;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
