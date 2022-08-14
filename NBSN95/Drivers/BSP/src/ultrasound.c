#include "ultrasound.h"
#include "main.h"

void GPIO_ULT_INPUT_Init(void)	//ECHO
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode =GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  HAL_GPIO_Init( GPIOA, &GPIO_InitStruct);
}

void GPIO_ULT_OUTPUT_Init(void)	//TRIG
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init( GPIOB,&GPIO_InitStruct);
}
/////////////////////////////////////////////////
void GPIO_ULT_INPUT_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode =GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init( GPIOA, &GPIO_InitStruct);
}

void GPIO_ULT_OUTPUT_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;

  HAL_GPIO_Init( GPIOB,&GPIO_InitStruct);
}
///////////////////////////////////////////
uint16_t ULT_distance(void)
{
	GPIO_ULT_INPUT_Init();
	GPIO_ULT_OUTPUT_Init();
	
	uint16_t distance = 0;
	uint8_t  ult_flags=0;
	
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)==0)
	{
		 ult_flags=0;
	}
	else
	{
		 ult_flags=1;
	}
	
	if(ult_flags==0)
	{
		uint8_t error_num=0;
		while(error_num<10)
		{		
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
			HAL_Delay(1);	
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			
			while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11));
			uint32_t time = HAL_GetTick();
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11));
			time= HAL_GetTick() - time;

			distance=(time*10000)/58;
			user_main_debug("time:%d",time);
			user_main_debug("distance:%d",distance);
			if((distance<240)||(distance>6000))
			{
				user_main_printf("Distance is out of range");
				error_num++;
				if(error_num==9)				
					break;
			}
			else
			{
				user_main_printf("Distance=%d mm",distance);
				return distance;
			}
		}
	}
	
	else
	{
		user_main_printf("ULT is not connect");
		return distance;
	}	
	
	return distance=0;
}

// AMS could not get HAL_UART_Receive working
// HAL_UART_Receive_DMA only fires if set to half-duplex
uint16_t ULT_distance_async(void) // sensor sends reading via serial
{
//	uint8_t  rxbuf[4] = {0};			
	float distance;
//	for(int i =0; i < 10; i++)
//	{
//		HAL_UART_Receive_DMA(&huart1,(uint8_t*)&rxbuf,4);
////		while( ! uart1_recieve_flag ) 
////		{	
////			for(int attempts = 0; attempts <20; attempts++){}
////			break;
////		}
////		if(uart1_recieve_flag)
////		{
////			led_on(200);
////			if (rxbuf[0] == 0xFF)
////			{
////		//					printf("%x | %x\r\n", rxDATA_1[1], rxDATA_1[2]);
////				distance=(rxbuf[1]<<8)+rxbuf[2];
////				printf("MSB %.0f\r\n", distance);
////			}
////			else if (rxbuf[2] == 0xFF)
////			{
////		//					printf("%x | %x\r\n", rxDATA_1[3], rxDATA_1[0]);
////				distance=(rxbuf[3]<<8)+rxbuf[0];
////				printf("LSB %.0f\r\n", distance);
////			}
////			else
////			{
////				printf("invalid packet\r\n");
////				printf("%02hhx | %02hhx | %02hhx | %02hhx\r\n", rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
////			}
////			//	return distance;
////		}
//	}
	return distance;
}
////		// add buf to end of array
//		if (rxlen_1 + 4 < sizeof(rxDATA_1)) // another reading will fit
//			{
//				for(int i =0; i < 4; i++)
//				{
//					rxDATA_1[rxlen_1++] = rxbuf_1[i];
//				}
//			HAL_UART_Receive_DMA(&huart1,(uint8_t*)&rxbuf_1,4);
//			}
//			else
