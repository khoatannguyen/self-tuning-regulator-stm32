/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
#define BUFFERED // Use buffer and interrupt for USART
#define USARTBUFFSIZE	24
#define Mode_Position 1
#define Mode_Velocity 0
void TIM3_Config_PWM(void);
void TIM4_Config_DeBug(void); // For DeBug Timing 
void TIM5_Config_Velocity(void);
void TIM5_Config_Position(void);
void DMA_Config(void); // For system performance to improve
void GPIO_Config(void);
void SystemTick_Config(void);
void NVIC_Config_Velocity(void);
void NVIC_Config_Position(void);
void Display_Matrix( double x[6][6], unsigned char m, unsigned char n); // Display matrix to debug
void Multiply_Matrix( double x[6][6], unsigned char m, unsigned char n,  double y[6][6], unsigned char p, unsigned char q,  double z[6][6]);
void Add_Matrix( double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6]);
void Sub_Matrix( double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6]);
void Multiply_Matrix_Vector( double x[6][6],unsigned char m, unsigned char n,double y,  double z[6][6]);
void Divide_Matrix_Vector( double x[6][6],unsigned char m, unsigned char n,double y,  double z[6][6]);
void Tranpose_Matrix( double x[6][6],unsigned char m, unsigned char n,  double y[6][6]);
void Initial_Condition_Velocity(void); // Genarate initial condition for the estimation argorithm
void Initial_Condition_Position(void); // Genarate initial condition for the estimation argorithm
void RNG_Config(void);
//*******************************************************************************************************************************************

typedef struct{
	uint8_t in;
	uint8_t out;
	uint8_t count;
	uint8_t buff[USARTBUFFSIZE];
}FIFO_TypeDef;
void BufferInit(__IO FIFO_TypeDef *buffer);
ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch);
ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch);
ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer);
#ifdef BUFFERED
extern  FIFO_TypeDef U3Rx, U3Tx;
#endif
void Usart3Init(void);
void Usart3Put(uint8_t ch);
//*******************************************************************************************************************************************
#ifdef __cplusplus
 extern "C" {
#endif 

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

	 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdio.h>
#if defined (USE_STM324xG_EVAL)
  #include "stm324xg_eval.h"

#elif defined (USE_STM324x7I_EVAL) 
  #include "stm324x7i_eval.h"
   
#else
 #error "Please select first the Evaluation board used in your application (in Project Options)"
#endif

/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
