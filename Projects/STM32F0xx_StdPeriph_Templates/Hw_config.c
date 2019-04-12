/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(AudioControl_USART, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(AudioControl_USART, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef 	RCC_ClockFreq;

/*TIMER variable*/
__IO uint16_t 		CCR1_Val = 6000;	//	6000000/6000 = 1000Hz = 1ms
__IO uint16_t 		CCR2_Val = 1200;	//	6000000/1200 = 5000Hz = 5ms

/*FLASH read/write variable*/
uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t Data = 0x3210ABCD;
uint32_t NbrOfPage = 0x00;
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO TestStatus MemoryProgramStatus = PASSED;
uint8_t TransBuffer[FLASH_USER_END_ADDR-FLASH_USER_START_ADDR];

typedef void (*f1_p)(uint32_t,uint32_t);
f1_p Flash_Erase_p = ((void (*) (uint32_t,uint32_t)) (0x08008000+1))	;
typedef void (*f2_p)(uint32_t,uint32_t,uint32_t);
f2_p Flash_Write_p = ((void (*) (uint32_t,uint32_t,uint32_t)) (0x08008100+1));
f2_p Flash_Read_p = ((void (*) (uint32_t,uint32_t,uint32_t)) (0x08008200+1));

/*
    TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to APB1 clock (PCLK1),  
      => TIM3CLK = PCLK1 = SystemCoreClock = 48 MHz
          
    To get TIM3 counter clock at 6 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = (PCLK1 /6 MHz) - 1
                                              
    CC1 update rate = TIM3 counter clock / CCR1_Val = 1000 Hz
    CC2 update rate = TIM3 counter clock / CCR2_Val = 5000 Hz    
*/
void TIM_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	uint16_t PrescalerValue = 0;
	
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	PrescalerValue = (uint16_t) ((SystemCoreClock) / 6000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);
	
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/* Private functions ---------------------------------------------------------*/
	

/*
	[IAR stm32f030_flash.icf file add]
	place at address mem:0x08008000 { readonly section .Flash_Erase};
	place at address mem:0x08008100 { readonly section .Flash_Read};
	place at address mem:0x08008200 { readonly section .Flash_Write};


	CALL Flash_Erase function :
	((void  (void)) (0x08008000+1)) (); 

	CALL Flash_Read function : 
	((void  (void)) (0x08008100+1)) (); 

	CALL Flash_Write function : 
	((void  (void)) (0x08008200+1)) (); 

*/

void Flash_Erase(uint32_t start , uint32_t end) @".Flash_Erase"
{
	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

	/* Define the number of page to be erased */
	NbrOfPage = (end - start) / FLASH_PAGE_SIZE;

	/* Erase the FLASH pages */
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		if (FLASH_ErasePage(start + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
		{
			/* Error occurred while sector erase. 
			User can add here some code to deal with this error  */
			while (1)
			{
			}
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock(); 
	printf("Flash_Erase!\r\n");
}

void Flash_Read(uint32_t start , uint32_t end , uint32_t DataRead) @".Flash_Read"
{
	/* Check if the programmed data is OK 
	  MemoryProgramStatus = 0: data programmed correctly
	  MemoryProgramStatus != 0: number of words not programmed correctly ******/
//	uint32_t i=0,j=0;
	Address = start;
	MemoryProgramStatus = PASSED;

	while (Address < end)
	{
		Data = *(__IO uint32_t *)Address;

		if (Data != DataRead)
		{
			MemoryProgramStatus = FAILED;  
		}

		Address = Address + 4;
	}
	printf("MemoryProgramStatus = %d\r\n" ,MemoryProgramStatus );

	#if 0
	printf("Flash_Read!\r\n");	
	for (i=0,j=0;i<(end-start);i++)
	{
		printf("0x%8X,",*(__IO uint32_t *)start+i);
		j++;
		if (j>=4)
		{
			printf("\r\n");
			j=0;
		}
	}
	printf("\r\n");	
	#endif
}

void Flash_Write(uint32_t start , uint32_t end , uint32_t DataWrite) @".Flash_Write"
{
//	uint32_t i=0;
	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();

	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	Address = start;

	#if 0	//array
	for (i=0;i<(FLASH_USER_END_ADDR-FLASH_USER_START_ADDR);i++)
	{
		TransBuffer[i] = 0x01+i;
	};
	
	i=0;
	while (Address < end)
	{
		if (FLASH_ProgramWord(Address, (uint32_t)TransBuffer[i]) == FLASH_COMPLETE)
		{
			Address = Address + 4;
		}
		else
		{ 
			/* Error occurred while writing data in Flash memory. 
			User can add here some code to deal with this error */
			while (1)
			{
			}
		}
		i++;
	}
	#else	//single
	while (Address < end)
	{
		if (FLASH_ProgramWord(Address, DataWrite) == FLASH_COMPLETE)
		{
			Address = Address + 4;
		}
		else
		{ 
			/* Error occurred while writing data in Flash memory. 
			User can add here some code to deal with this error */
			while (1)
			{
			}
		}
	}
	#endif

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock(); 
	printf("Flash_Write!\r\n");
}

void Flash_MEMTEST(uint32_t StartAddress , uint32_t Length)
{

	Flash_Erase_p(FLASH_USER_START_ADDR+0x1000,FLASH_USER_END_ADDR+0x1000);
	Flash_Write_p(FLASH_USER_START_ADDR+0x1000,FLASH_USER_END_ADDR+0x1000,DATA_32r);
	Flash_Read_p(FLASH_USER_START_ADDR+0x1000,FLASH_USER_END_ADDR+0x1000,DATA_32r);

	printf("Flash_Erase Add:0x%8X \r\n" , (uint32_t) &Flash_Erase);
	printf("Flash_Read Add:0x%8X \r\n" , (uint32_t) &Flash_Read);
	printf("Flash_Write Add:0x%8X \r\n" , (uint32_t) &Flash_Write);	

	#if 0
	uint16_t i=0;
	uint8_t j=0;
	
	memcpy(TransBuffer,(uint32_t*)&Flash_Erase,Length);

	printf("TransBuffer = \r\n");
	for (i=0,j=0;i<0xFF;i++)
	{
		printf("0x%8X ,",TransBuffer[i]);
		j++;
		if (j>=4)
		{
			printf("\r\n");
			j=0;
		}
	}
	printf("\r\n");	
	#endif

	#if 0
	uint16_t i=0;	
	Flash_Erase(StartAddress , StartAddress+Length);
	Flash_Write(StartAddress,StartAddress+4,TransBuffer[0]);
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

	NbrOfPage = ((Length+StartAddress)-StartAddress) / FLASH_PAGE_SIZE; // start 32k address 

	/* Erase the FLASH pages */
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		if (FLASH_ErasePage(StartAddress + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
		{
			/* Error occurred while sector erase. 
			User can add here some code to deal with this error  */
			while (1)
			{
			}
		}
	}

	Address = StartAddress;
	i= 0;
	while (Address < (Length+StartAddress))
	{
		if (FLASH_ProgramWord(Address, TransBuffer[i]) == FLASH_COMPLETE)
		{
			Address = Address + 4;
		}
		else
		{ 
			/* Error occurred while writing data in Flash memory. 
			User can add here some code to deal with this error */
			while (1)
			{
			}
		}
		i++;
	}

	FLASH_Lock(); 	
	#endif
}

uint32_t Z_BootFunc_9 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i ,j ;
	
	for (i=0;i<Data;i++)
	{
		j=i+Data;
	}

	return j + Data;
}

uint32_t Z_BootFunc_8 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i ,j ;
	
	for (i=0;i<Data;i++)
	{
		j=i+Data;
	}

	return j + Data;
}

uint32_t Z_BootFunc_7 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i ,j ;
	
	for (i=0;i<Data;i++)
	{
		j=i+Data;
	}

	return j + Data;
}

uint32_t Z_BootFunc_6 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i ,j ;
	
	for (i=0;i<Data;i++)
	{
		j=i+Data;
	}

	return j + Data;
}

uint32_t Z_BootFunc_5 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i ,j ;
	
	for (i=0;i<Data;i++)
	{
		j=i+Data;
	}

	return j + Data;
}

uint32_t Z_BootFunc_4 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i ,j ;
	
	for (i=0;i<Data;i++)
	{
		j=i+Data;
	}

	return j + Data;
}

uint32_t Z_BootFunc_3 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i ,j ;
	
	for (i=0;i<Data;i++)
	{
		j=i+Data;
	}

	return j + Data;
}

uint32_t Z_BootFunc_2 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i ,j ;
	
	for (i=0;i<Data;i++)
	{
		j=i+Data;
	}

	return j + Data;
}

uint32_t F_BootFunc_1 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i ,j ;
	
	for (i=0;i<Data;i++)
	{
		j=i+Data;
	}

	return j + Data;
}

uint32_t E_BootFunc_1 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i = 0 ;
	for (i=0;i<Data;i++)
	{
		i=i+Data;
	}

	return (i+Data) - 50;
}

uint32_t D_BootFunc_1 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i = 0 ;
	for (i=0;i<Data;i++)
	{
		i=i+Data;
	}
	return i*Data * 22;
}

uint32_t C_BootFunc_1 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i = 0 ;
	for (i=0;i<Data;i++)
	{
		i=i+Data;
	}
	return i*Data/100;
}

uint32_t B_BootFunc_1 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i = 0 ;
	for (i=0;i<Data;i++)
	{
		i=i+Data;
	}
	return (i + Data)*100;
}

uint32_t A_BootFunc_1 (uint8_t Data)@".BootLoaderSection"
{
	uint16_t i = 0 ;
	for (i=0;i<Data;i++)
	{
		i=i+Data;
	}
	return (i + Data)%100;
}

void LED_Config(void)
{
  	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void LED3_Toggle(void)
{
	GPIOC ->ODR ^= GPIO_Pin_9;
}

void LED4_Toggle(void)
{
	GPIOC ->ODR ^= GPIO_Pin_8;
}

void USART_TEST(void)
{
	__IO uint8_t temp;
	
	if(USART_GetFlagStatus(AudioControl_USART, USART_FLAG_RXNE) == SET)
	{
			temp = USART_ReceiveData(AudioControl_USART);
			printf("Press KEY : %c \n\r",temp);

			switch (temp)
			{
				case '1': 
//					EXTI_GenerateSWInterrupt(EXTI_Line2);
					EXTI_GenerateSWInterrupt(EXTI_Line0);					
					printf("EXTI_GenerateSWInterrupt(EXTI_Line0) \r\n");	

					break;

				case '2': 
//					EXTI_GenerateSWInterrupt(EXTI_Line2);
					EXTI_GenerateSWInterrupt(EXTI_Line1);					
					printf("EXTI_GenerateSWInterrupt(EXTI_Line1) \r\n");	

					break;

				case '3':
					EXTI_GenerateSWInterrupt(EXTI_Line3);					
					printf("EXTI_GenerateSWInterrupt(EXTI_Line3) \r\n");					
					break;

				case '4':
					
					break;					
					
				default : 
					printf("INPUT CMD not support !\r\n");
					break;
			}
	}
}

void USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Clock configuration ---------------------------------------------------*/
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(AudioControl_USART_GPIO_CLK, ENABLE);

	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(AudioControl_USART_CLK, ENABLE);

	/* GPIO configuration ----------------------------------------------------*/
	GPIO_DeInit(AudioControl_USART_GPIO_PORT);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(AudioControl_USART_GPIO_PORT, AudioControl_USART_TX_SOURCE, AudioControl_USART_TX_AF);
	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(AudioControl_USART_GPIO_PORT, AudioControl_USART_RX_SOURCE, AudioControl_USART_RX_AF);

	/* Configure USARTx_Tx,USARTx_Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = AudioControl_USART_TX_PIN|AudioControl_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(AudioControl_USART_GPIO_PORT, &GPIO_InitStructure);

	/* USART configuration ---------------------------------------------------*/
	USART_DeInit(AudioControl_USART);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(AudioControl_USART, &USART_InitStructure);

	/* Enable USARTy Receive and Transmit interrupts */
	USART_ITConfig(AudioControl_USART, USART_IT_RXNE, ENABLE); 
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	/* The software must wait until TC=1. The TC flag remains cleared during all data
	transfers and it is set by hardware at the last frame’s end of transmission*/	
	while (USART_GetFlagStatus(AudioControl_USART, USART_FLAG_TC) == RESET)
	{}

	/* NVIC configuration */
	/* Enable the USARRx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = AudioControl_USART_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	/* Enable the USARRx */
	USART_Cmd(AudioControl_USART, ENABLE);
}

void SysTickTimer_Config(void)
{
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	#if 1 //debug
	printf("===========================\r\n");
	printf("SYSCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.SYSCLK_Frequency);
	printf("HCLK_Frequency = %d \r\n" , 			RCC_ClockFreq.HCLK_Frequency);
	printf("PCLK_Frequency = %d \r\n" , 			RCC_ClockFreq.PCLK_Frequency);
	printf("ADCCLK_Frequency= %d \r\n" , 		RCC_ClockFreq.ADCCLK_Frequency);
	printf("CECCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.CECCLK_Frequency);
	printf("I2C1CLK_Frequency = %d \r\n" , 		RCC_ClockFreq.I2C1CLK_Frequency);
	printf("USART1CLK_Frequency = %d \r\n" , 	RCC_ClockFreq.USART1CLK_Frequency); 
	#endif /*debug*/
	
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}
	
	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x01);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay_ms(__IO uint32_t uTime)
{ 
	uwTimingDelay = uTime;
	while(uwTimingDelay != 0);
}

void Delay_s(__IO uint32_t mTime)
{ 
	uint32_t i;
	for(i=0;i<mTime;i++)
		Delay_ms(1000);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

//currently not use
/*

void SystemClkDelay(void)
{
	uint32_t i;
	i = 0xffff;
	while(i--);
}

void wtPutChar(uint8_t ccc)
{
	UART1_SendData8(ccc);
	// Loop until the end of transmission 
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	
}

u16 GetAbsTime(u16 a,u16 b)
{
	u16 c;
	if(a>=b) c=(a-b);
	else c=65535-(b-a);	
	
	return c;
}
*/

void UART_SendByte(uint8_t Data)
{
	USART_SendData(AudioControl_USART , (unsigned char)Data);
	while (USART_GetFlagStatus(AudioControl_USART , USART_FLAG_TXE)==RESET);
	{
	}
}

void UART_SendString(uint8_t* Data,uint16_t len)
{
	uint16_t i=0;
	for(i=0;i<len;i++ )
	{
		UART_SendByte(Data[i]);
	}
}

void SystemClkDelay(uint32_t u32Delay)
{
	//uint32_t i;
	//i = 0xffff;
	while(u32Delay--);
}


