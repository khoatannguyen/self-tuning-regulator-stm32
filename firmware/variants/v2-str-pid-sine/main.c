/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "main.h"
#include <stdio.h> 
#include <stdint.h> 
/* PIN DESCRIPTION:

*/
/*  USEFUL NVIC FUNCTIONS AND INFOMATION:
 NVIC_SetPriorityGrouping,NVIC_GetPriorityGrouping,NVIC_EnableIRQ,NVIC_DisableIRQ,NVIC_SetPriority,NVIC_GetPriority,NVIC_SetPendingIRQ
 NVIC_GetPendingIRQ,NVIC_ClearPendingIRQ,NVIC_DecodePriority,NVIC_EncodePriority
NVIC_PriorityGroup 	NVIC_IRQChannelPreemptionPriority 	NVIC_IRQChannelSubPriority
0 									0 																	0-15
1 									0-1 																0-7
2 									0-3 																0-3
3 									0-7 																0-1
4 									0-15 																0
STM32 Interrupt Service Routine is based on Nested Vectored Interrupt Controller (NVIC) and supports nesting (stacking) of interrupts, 
allowing an interrupt to be serviced earlier by exerting higher priority. The followings summarize important points:
    *The lower the number the higher the priority.
    *NVIC_PriorityGroup has the highest priority. All other interrupts with lower NVIC_PriorityGroup that occur after are put in a stack (queue).
    *NVIC_IRQChannelPreemptionPriority is used to determine if an interrupt that occurs after can overtake previous interrupt that is being serviced.
    *NVIC_IRQChannelSubPriority is used to determine priority if two interrupts occur at the same time. 
		(If NVIC_IRQChannelSubPriority is not determined, the position in the NVIC table is used to determine the priority.) 
*/
/* Private typedef -----------------------------------------------------------*/
TIM_ICInitTypeDef  TIM_ICInitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef  GPIO_InitStructure;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile unsigned char Velocity_Position; 
unsigned int i,j,k; // Counter variable for if function
double phi[6][6], theta[6][6], theta_temp[6][6], L[6][6], P[6][6], phiT[6][6]; 
double temp[6][6], temp2[6][6], Kalman[6][6], denominator[6][6], numerator[6][6], Temp_mul;
double lamda_predict = 100, lamda = 0.98, phiTPphi = 0, trace_P = 0;// The higher lamda the more stable, the lower the faster from 0.95 to 1
double epsilon = 0; 
uint32_t random32bit; // For creating initial random theta
unsigned int NVIC_Priority_Number; // To set the priority of NVIC 
/* Private function prototypes -----------------------------------------------*/
void TIM3_Config_PWM(void); // For PWM output
void TIM4_Config_DeBug(void); // For DeBug Timing 
void TIM5_Config_Velocity(void); // For encoder frequency measurement
void TIM5_Config_Position(void); // For encoder frequency measurement
void DMA_Config(void); // For system performance to improve
void GPIO_Config(void); // For general GPIO init
void SystemTick_Config(void); // Generate sample time interrupt to control DC motor
void NVIC_Config_Velocity(void); // Set the priority of each interrupt
void NVIC_Config_Position(void); // Set the priority of each interrupt
void Display_Matrix( double x[6][6], unsigned char m, unsigned char n); // Display matrix to debug
void Multiply_Matrix( double x[6][6], unsigned char m, unsigned char n,  double y[6][6], unsigned char p, unsigned char q,  double z[6][6]);
void Add_Matrix( double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6]);
void Sub_Matrix( double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6]);
void Multiply_Matrix_Vector( double x[6][6],unsigned char m, unsigned char n,double y,  double z[6][6]);
void Divide_Matrix_Vector( double x[6][6],unsigned char m, unsigned char n,double y,  double z[6][6]);
void Tranpose_Matrix( double x[6][6],unsigned char m, unsigned char n,  double y[6][6]);
void Initial_Condition_Velocity(void); // Genarate initial condition for the estimation argorithm
void Initial_Condition_Position(void); // Genarate initial condition for the estimation argorithm
void RNG_Config(void); // Random number generator
void BufferInit(__IO FIFO_TypeDef *buffer); // Init buffer for USART output
ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch); // Put character to buffer
ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch); //Get Character from buffer
ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer); // Buffer is empty or not
void USART3Init(void); 
void USART3Put(uint8_t ch); // Put character to USART
/* Private main functions ---------------------------------------------------------*/
int main(void)
{
	Usart3Init();
	TIM3_Config_PWM();
	GPIO_Config();
  SystemTick_Config();
	RNG_Config();
	if(Velocity_Position == Mode_Position){
		TIM5_Config_Position();
		Initial_Condition_Position();
		NVIC_Config_Position();
	}
	else{
		TIM5_Config_Velocity();
		Initial_Condition_Velocity();
		NVIC_Config_Velocity();
	}
  while (1){}
}
/* Private sub functions ---------------------------------------------------------*/
void TIM3_Config_PWM(void) // For PWM output
{
	  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
    To get TIM3 counter clock at 84 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /84 MHz) - 1
    To get TIM3 output clock at 21 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM3 counter clock / TIM3 output clock) - 1
           = 3999
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */   
	/* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* GPIOC Configuration: TIM3 CH1 (PC6) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 3999;
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock /2) / 84000000) - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}
void TIM4_Config_DeBug(void) // For DeBug Timing 
{
  /* --------------------------------------------------------------------------- 
    TIM4 configuration: Input Capture mode
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM4CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM4CLK = HCLK / 2 = SystemCoreClock /2
		Prescaler = 47 So TIM4CLK = SystemCoreClock /2 / (1+47) = SystemCoreClock / 96
		TIM4 counter clock also = SystemCoreClock / 96 = 1750000
		Frequency = TIM4 counter clock / TIM4_CCR2 in Hz, 
		The minimum frequency value to measure is 50 Hz (TIM4 counter clock / CCR MAX = 35000).
		Prescaler is 1 so the frequency is measured every 1 event.
  --------------------------------------------------------------------------- */
	GPIO_InitTypeDef GPIO_InitStructure;
  //TIM4 clock enable 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  //GPIOA clock enable 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  //TIM4 channel2 configuration : PB7 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //Connect TIM pin to AF2 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
  //Select the TIM4 Input Trigger: TI2FP2
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
  /*Choose the Prescaler*/
	TIM_PrescalerConfig(TIM4, 47, TIM_PSCReloadMode_Immediate);
  //Select the slave Mode: Reset Mode 
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);
	TIM_SetAutoreload (TIM4, 35000);
  //TIM enable counter 
  TIM_Cmd(TIM4, ENABLE);
  //Enable the CC2 Interrupt Request 
  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_UpdateRequestConfig(TIM4, TIM_UpdateSource_Regular);
	NVIC_DisableIRQ(TIM4_IRQn);
}
void TIM5_Config_Velocity(void) // For encoder frequency measurement
{
  /* --------------------------------------------------------------------------- 
    TIM5 configuration: Input Capture mode
    In this example TIM5 input clock (TIM5CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM5CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM5CLK = HCLK / 2 = SystemCoreClock /2
		Prescaler = 63 So TIM5CLK = SystemCoreClock /2 / (1+63) = SystemCoreClock / 128
		TIM5 counter clock also = SystemCoreClock / 128
	= 1875000
		Frequency = TIM5 counter clock / TIM5_CCR2 in Hz, 
		DutyCycle = (TIM5_CCR1*100)/(TIM5_CCR2) in %.
		The minimum frequency value to measure is 53 Hz (TIM5 counter clock / CCR MAX = 35000).
		Prescaler is 1 so the frequency is measured every 1 event.
  --------------------------------------------------------------------------- */
	GPIO_InitTypeDef GPIO_InitStructure;
  //TIM5 clock enable 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  //GPIOA clock enable 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  //TIM5 channel2 configuration : PA1 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//Configure PA0
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //Connect TIM pin to AF2 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
  //Select the TIM5 Input Trigger: TI2FP2
  TIM_SelectInputTrigger(TIM5, TIM_TS_TI2FP2);
  /*Choose the Prescaler*/
	TIM_PrescalerConfig(TIM5, 47, TIM_PSCReloadMode_Immediate);
  //Select the slave Mode: Reset Mode 
  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM5,TIM_MasterSlaveMode_Enable);
	TIM_SetAutoreload (TIM5, 35000);
  //TIM enable counter 
  TIM_Cmd(TIM5, ENABLE);
  //Enable the CC2 Interrupt Request 
  TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_UpdateRequestConfig(TIM5, TIM_UpdateSource_Regular);
	NVIC_DisableIRQ(TIM5_IRQn);
}
void TIM5_Config_Position(void) // For encoder position measurement
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//Configure peripheral clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
	//Configure pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //GPIO_PuPd_UP GPIO_PuPd_NOPULL
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);  //A0 -TIM5_CH1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);  //A1 -TIM5_CH2
	//Setup quadrature encoder and enable timer
	TIM_EncoderInterfaceConfig(TIM5,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM_SetAutoreload (TIM5, 0xFFFFFFFF);
	TIM_Cmd(TIM5, ENABLE);  
}
void DMA_Config(void)
{
}
void GPIO_Config(void) // For general GPIO init
{
	/* STM32F4-Discovery
  LED4, LED3, LED5 and LED6 are connected respectively to PD.12, PD.13, PD.14 and PD.15.*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Configure PD11*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Configure PD12 output for motor direction control*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	/* Configure PD13 output for timing debug*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	/* Configure PD14 output for timing debug*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	/* Configure PD15 output for timing debug*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_15);
}
void SystemTick_Config(void) // Generate sample time interrupt to control DC motor
{
		/* Setup SysTick Timer for 5 msec interrupts.
     ------------------------------------------
    1. The SysTick_Config() function is a CMSIS function which configure:
       - The SysTick Reload register with value passed as function parameter.
       - Configure the SysTick IRQ priority to the highest value (0x00).
       - Reset the SysTick Counter register.
       - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
       - Enable the SysTick Interrupt.
       - Start the SysTick Counter.
    2. You can change the SysTick Clock source to be HCLK_Div8 by calling the
       SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8) just after the
       SysTick_Config() function call. The SysTick_CLKSourceConfig() is defined
       inside the misc.c file.
    3. You can change the SysTick IRQ priority by calling the
       NVIC_SetPriority(SysTick_IRQn,...) just after the SysTick_Config() function 
       call. The NVIC_SetPriority() is defined inside the core_cm4.h file.
    4. To adjust the SysTick time base, use the following formula:
         Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
       - Reload Value is the parameter to be passed for SysTick_Config() function
       - Reload Value should not exceed 0xFFFFFF */
	if (SysTick_Config(SystemCoreClock * 0.005)){ 
    printf("SysTick_Config Error");
    while (1){}
  }
	NVIC_DisableIRQ(SysTick_IRQn);
}
void NVIC_Config_Velocity(void) // Set the priority of each interrupt
{
	NVIC_Priority_Number = NVIC_EncodePriority(0,0,0);
	NVIC_SetPriority(SysTick_IRQn,NVIC_Priority_Number);
	NVIC_Priority_Number = NVIC_EncodePriority(2,2,2);
	NVIC_SetPriority(TIM5_IRQn,NVIC_Priority_Number);
	NVIC_Priority_Number = NVIC_EncodePriority(1,1,1);
	NVIC_SetPriority(USART3_IRQn,NVIC_Priority_Number);
	TIM_SetCounter(TIM5, 0);
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
	GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
}
void NVIC_Config_Position(void) // Set the priority of each interrupt
{
	NVIC_Priority_Number = NVIC_EncodePriority(0,0,0);
	NVIC_SetPriority(SysTick_IRQn,NVIC_Priority_Number);
	NVIC_Priority_Number = NVIC_EncodePriority(2,2,2);
	NVIC_SetPriority(USART3_IRQn,NVIC_Priority_Number);
	TIM_SetCounter(TIM5, 0);
	GPIO_SetBits(GPIOD, GPIO_Pin_15);
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
}
void Display_Matrix(double x[6][6], unsigned char m, unsigned char n)
{
  for (i=0;i<m;i++)
    {
		for(j=0;j<n;j++)
		printf(" %10.2f",x[i][j]);
		printf("\n");
    }
    printf("\n");
}
void Multiply_Matrix(double x[6][6], unsigned char m, unsigned char n,  double y[6][6], unsigned char p, unsigned char q,  double z[6][6])
{
    for ( i = 0 ; i < m ; i++ )
    {
      for ( j = 0 ; j < q ; j++ )
      {
        for ( k = 0 ; k < p ; k++ )
        {
          Temp_mul = Temp_mul + x[i][k]*y[k][j];
        }
        z[i][j] = Temp_mul;
        Temp_mul = 0;
      }
    }
}
void Add_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6])
{
    for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]+y[i][j];
}
void Sub_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6])
{
    for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]-y[i][j];
}
void Multiply_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6])
{
		   for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]*y;
}
void Divide_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6])
{
	    for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]/y;
}
void Tranpose_Matrix(double x[6][6],unsigned char m, unsigned char n,  double y[6][6])
{
   for(i=0;i<n;i++)
      for(j=0;j<m;j++)
           y[i][j]=0;
  for(i=0;i<n;i++){
      for(j=0;j<m;j++){
           y[i][j]=x[j][i];
}}}
void Initial_Condition_Velocity(void) 
{
	theta[0][0]= -8.49670851058287e-18 ; 		//a1
	theta[1][0]= 1.80485138785099e-35;  			//a2
	theta[2][0]= 1; 			//b1
	theta[3][0]= 1.65685815956384e-16; 			//b2
	theta_temp[2][0]= 1; 			//b1
	// Init P matrix
	P[0][0] = 1000;
	P[1][1] = 1000;
	P[2][2] = 1000;
	P[3][3] = 1000;
	trace_P = 4000;
}
void Initial_Condition_Position(void) 
{	/*while(RNG_GetFlagStatus(RNG_FLAG_DRDY)== RESET){}
  random32bit = RNG_GetRandomNumber();
	theta[1][0]= ((random32bit % 100) + 1.0) / 100 ;*/
	theta[0][0]= -4.12230724487712e-09;			//a1
	theta[1][0]= 4.24835425529119e-18;				//a2
	theta[2][0]= 3.62206672972073e-41;				//a3
	theta[3][0]= 0.999999952160871;				//b1
	theta[4][0]= 4.37168138727063e-08;				//b2
	theta[5][0]= 5.24488179668768e-20;				//b3
	theta_temp[3][0]= 0.999999952160871;				//b1
	// Init P matrix
	P[0][0] = 1000;
	P[1][1] = 1000;
	P[2][2] = 1000;
	P[3][3] = 1000;
	P[4][4] = 1000;
	P[5][5] = 1000;
	trace_P = 6000;
}
void RNG_Config(void) // Random number generator
{
 /* Enable RNG clock source */
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
  /* RNG Peripheral enable */
  RNG_Cmd(ENABLE);
}
PUTCHAR_PROTOTYPE /* Place your implementation of fputc here */
{
  /* Place your implementation of fputc here */
	USART3Put((uint8_t) ch);
	return ch;
}
void BufferInit(__IO FIFO_TypeDef *buffer) // Init buffer for USART output
{
buffer->count = 0;//0 bytes in buffer
buffer->in = 0;//index points to start
buffer->out = 0;//index points to start
}
ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch)
{
if(buffer->count==USARTBUFFSIZE)
{return ERROR;}//buffer full
buffer->buff[buffer->in++]=ch;
buffer->count++;
if(buffer->in==USARTBUFFSIZE)
{buffer->in=0;}//start from beginning
return SUCCESS;
}
ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch)
{
if(buffer->count==0)
{return ERROR;}//buffer empty
*ch=buffer->buff[buffer->out++];
buffer->count--;
if(buffer->out==USARTBUFFSIZE)
{buffer->out=0;}//start from beginning
return SUCCESS;
}
ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer)
{
	if(buffer.count==0)
	{return SUCCESS;}//buffer full
	return ERROR;
}
#ifdef BUFFERED
 FIFO_TypeDef U3Tx;
#endif
void Usart3Init(void)
{
  USART_InitTypeDef USART_InitStructure;
  /* USARTx configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  STM_EVAL_COMInit(COM1, &USART_InitStructure);	
#ifdef BUFFERED
	//initialize buffers
	BufferInit(&U3Tx);
	//disable Transmit Data Register empty interrupt
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	//enable Receive Data register not empty interrupt
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
#endif
}
void USART3Put(uint8_t ch)
{
#ifdef BUFFERED
	//put char to the buffer
	BufferPut(&U3Tx, ch);
	//enable Transmit Data Register empty interrupt
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
#else
	  USART_SendData(USART3, (uint8_t) ch);
	  //Loop until the end of transmission
	  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
	  {
	  }
#endif
}
//*******************************************************************************************************************************************
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1)
  {}
}
#endif
