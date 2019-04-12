/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include <stdio.h>
#include "string.h"

/* Define config -------------------------------------------------------------*/
#define TRUE			1
#define FALSE           0
#define 	ON				1
#define 	OFF				0
typedef unsigned char   BOOL;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

#define FLASH_PAGE_SIZE         					((uint32_t)0x00000400)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   					((uint32_t)0x08005000)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     					((uint32_t)0x08006000)   /* End @ of user Flash area */
#define DATA_32                 					((uint32_t)0x12345678)
#define DATA_32r                 					((uint32_t)0x87654321)

#define AudioControl_USART                         	USART1
#define AudioControl_USART_CLK                     	RCC_APB2Periph_USART1
#define AudioControl_USART_GPIO_PORT          		GPIOA
#define AudioControl_USART_GPIO_CLK         		RCC_AHBPeriph_GPIOA

#define AudioControl_USART_TX_PIN                	GPIO_Pin_9
#define AudioControl_USART_TX_SOURCE            	GPIO_PinSource9
#define AudioControl_USART_TX_AF                  	GPIO_AF_1

#define AudioControl_USART_RX_PIN                 	GPIO_Pin_10
#define AudioControl_USART_RX_SOURCE             	GPIO_PinSource10
#define AudioControl_USART_RX_AF                  	GPIO_AF_1

#define AudioControl_USART_IRQn                   	USART1_IRQn

#define AudioControl_T1_PIN                   		GPIO_Pin_0
#define AudioControl_T1_GPIO_PORT             		GPIOB
#define AudioControl_T1_GPIO_CLK              		RCC_AHBPeriph_GPIOB
#define AudioControl_T1_EXTI_LINE             		EXTI_Line0
#define AudioControl_T1_EXTI_PORT_SOURCE      		EXTI_PortSourceGPIOB
#define AudioControl_T1_EXTI_PIN_SOURCE       		EXTI_PinSource0
#define AudioControl_T1_EXTI_IRQn             		EXTI0_1_IRQn 

#define AudioControl_T2_PIN                   		GPIO_Pin_1
#define AudioControl_T2_GPIO_PORT             		GPIOB
#define AudioControl_T2_GPIO_CLK              		RCC_AHBPeriph_GPIOB
#define AudioControl_T2_EXTI_LINE             		EXTI_Line1
#define AudioControl_T2_EXTI_PORT_SOURCE      		EXTI_PortSourceGPIOB
#define AudioControl_T2_EXTI_PIN_SOURCE       		EXTI_PinSource1
#define AudioControl_T2_EXTI_IRQn             		EXTI0_1_IRQn 

extern __IO uint16_t CCR1_Val;
extern __IO uint16_t CCR2_Val;
extern uint32_t Data;
/* Macro ---------------------------------------------------------------------*/
/*
#define UartTxPutChar(x)		\
{	\
     UART1_SendData8(x);	\
     while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	\
}*/

/* Exported types ------------------------------------------------------------*/
void TIM_Config(void);
void EXTI_Config(void);

void Flash_Erase(uint32_t start , uint32_t end) @".Flash_Erase";
void Flash_Write(uint32_t start , uint32_t end , uint32_t DataWrite) @".Flash_Write";
void Flash_Read(uint32_t start , uint32_t end , uint32_t DataRead) @".Flash_Read";
void Flash_MEMTEST(uint32_t StartAddress , uint32_t Length);

uint32_t Z_BootFunc_9 (uint8_t Data)@".BootLoaderSection";
uint32_t Z_BootFunc_8 (uint8_t Data)@".BootLoaderSection";
uint32_t Z_BootFunc_7 (uint8_t Data)@".BootLoaderSection";
uint32_t Z_BootFunc_6 (uint8_t Data)@".BootLoaderSection";
uint32_t Z_BootFunc_5 (uint8_t Data)@".BootLoaderSection";
uint32_t Z_BootFunc_4 (uint8_t Data)@".BootLoaderSection";
uint32_t Z_BootFunc_3 (uint8_t Data)@".BootLoaderSection";
uint32_t Z_BootFunc_2 (uint8_t Data)@".BootLoaderSection";

uint32_t F_BootFunc_1 (uint8_t Data)@".BootLoaderSection";
uint32_t E_BootFunc_1 (uint8_t Data)@".BootLoaderSection";
uint32_t D_BootFunc_1 (uint8_t Data)@".BootLoaderSection";
uint32_t C_BootFunc_1 (uint8_t Data)@".BootLoaderSection";
uint32_t B_BootFunc_1 (uint8_t Data)@".BootLoaderSection";
uint32_t A_BootFunc_1 (uint8_t Data)@".BootLoaderSection";

void LED_Config(void);
void LED3_Toggle(void);
void LED4_Toggle(void);

void USART_TEST(void);
void USART_Config(void);  

void SysTickTimer_Config(void);

void Delay_ms(__IO uint32_t uTime);
void Delay_s(__IO uint32_t mTime);

void TimingDelay_Decrement(void);
void UART_SendByte(uint8_t Data);
void UART_SendString(uint8_t* Data,uint16_t len);
void SystemClkDelay(uint32_t u32Delay);
/* Exported constants --------------------------------------------------------*/

#endif  /* __HW_CONFIG_H */

