/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define Dynamic_STR
//#define Frequency_Debug_Pin13
#define STR 1
#define PID 0
#define Variable_forgetting_factor 1
#define Constant_forgetting_factor 0
#define MAX_STRLEN 11 // this is the maximum string length of our string in characters
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile unsigned char Position_Model = 1, Velocity_Model = 1;
volatile unsigned char Controller = STR, Forgetting_Type = Variable_forgetting_factor;
volatile unsigned char Velocity_Position = Mode_Position; // Mode_Position Mode_Velocity
char received_string[MAX_STRLEN+1]; // this will hold the received string
double Set_Frequency = 0, pre_Set_Frequency = 0;
double Frequency = 0, Frequency_Debug = 0, pre_Frequency = 0, pre_pre_Frequency = 0;
double Error_value = 0, pre_Error_value = 0;
long Set_Position = 0, pre_Set_Position = 0, pre_pre_Set_Position = 0;
long Position = 0, pre_Position = 0, pre_pre_Position = 0, pre_pre_pre_Position = 0;
double out = 0, pre_out = 0, pre_pre_out = 0, pre_pre_pre_out = 0, out_temp = 0, out_real = 0;
unsigned int out_int;
double s0,s1,s2,t0,t1,t2,r1,r2;
double out1, out2, out3, out4, out5, out6, out7;
double P_part = 0, I_part = 0, D_part = 0;
double const pKp = 0.05, pKd = 0.075, pKi = 0; //pKp = 0.05, pKd = 0.075, pKi = 0;
double const vKp = 0.02, vKd = 0.015, vKi = 0.00005; //vKp = 0.02, vKd = 0.015, vKi = 0.00005;
extern double phi[6][6], theta[6][6], L[6][6], P[6][6], phiT[6][6]; 
extern double temp[6][6], temp2[6][6], denominator[6][6], numerator[6][6];
extern double lamda, lamda_predict, phiTPphi, trace_P;
extern double epsilon;
unsigned char Send2PC_Counter = 0;
double IC1Value = 0, IC2Value = 0, Motor_Direction = 0;
/* Private function prototypes -----------------------------------------------*/
void MRC_Design(void);
void Estimation_Algorithm(void);
void Send2PC(void);
void STR_Position(void);
void PID_Position(void);
void Model_Position(void);
void STR_Velocity(void);
void PID_Velocity(void);
void Model_Velocity(void);
/* Private functions ---------------------------------------------------------*/
void SysTick_Handler(void) // Generate sample time interrupt to control DC motor
{
	#ifdef Frequency_Debug_Pin13
	GPIO_SetBits(GPIOD, GPIO_Pin_13);
	GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	#endif
	if(Velocity_Position == Mode_Position){ //STR Position Control
		if(Controller == STR){
			STR_Position();
		}
		else{ //PID Position Control
			PID_Position();
		}
	}
	else{ //STR Velocity Control
		if(Controller == STR){
			STR_Velocity();
		}
		else{ //PID Velocity Control
			PID_Velocity();
		}
	}	
	#ifdef Frequency_Debug_Pin13
	GPIO_SetBits(GPIOD, GPIO_Pin_13);
	GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	#endif
}
void MRC_Design(void) // Base on Estimation_Algorithm this function generate suitable control signal for the motor similar to created model
{
	if(Velocity_Position == Mode_Position){
		Error_value = Set_Position - Position;
		#ifdef Dynamic_STR
		if (labs(Position)< 0.98*labs(Set_Position)){
			Position_Model = 2;
		}
		else{
			Position_Model = 1;	
		}	
		#endif
		Model_Position();
		Position = TIM_GetCounter(TIM5);
		/*out = - (r1 * pre_out) - (r2 * pre_pre_out)
					+ (t0 * Set_Position) + (t1 * pre_Set_Position) 
					- (s0 * Position) - (s1 * pre_Position)- (s2 * pre_pre_Position);*/
		/*		r1 = theta[4][0] / theta[3][0];
		r2 = theta[5][0] / theta[3][0];
		t0 = 0.999999999999999 / theta[3][0];
		t1 = 2.75395408819753e-85 / theta[3][0];
		s0 = (-2.76779305358719e-87 - theta[0][0]) / theta[3][0];
		s1 = (1.91516959667544e-174 - theta[1][0]) / theta[3][0];
		s2 = -theta[2][0] / theta[3][0];*/
		out = - (r1 * pre_out) - (r2 * pre_pre_out)
					+ (t0 * Set_Position) + (t1 * pre_Set_Position) 
					- (s0 * Position) - (s1 * pre_Position)- (s2 * pre_pre_Position);
		
		out_real = out;
		if (out<0) {
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			if (out < -3999) {out = -3999;}
			out_temp = out;
			out = -out;
		}
	else{			
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		if (out > 3999) {out = 3999;}
		out_temp = out;
	}
	out_int = (unsigned)(int)(out);
	TIM_SetCompare1(TIM3,out_int);
	out = out_temp;
}
	else{
		Model_Velocity();
		if (IC2Value > 0){
			if (Motor_Direction == 1){
				Frequency = 1750000 / IC2Value;
			}
			else{
				Frequency = -1750000 / IC2Value;
			}
		TIM_SetCounter(TIM5, 0);
		}
		out = -(r1 * pre_out) 
					+ (t0 * Set_Frequency) + (t1 * pre_Set_Frequency) 
					- (s0 * Frequency) - (s1 * pre_Frequency);
		out_real = out;
		if (out<0) {
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			if (out < -3999) {out = -3999;}
			out_temp = out;
			out = -out;
		}
		else{			
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			if (out > 3999) {out = 3999;}
			out_temp = out;
		}
		out_int = (unsigned)(int)(out);
		TIM_SetCompare1(TIM3,out_int);
		out = out_temp;
}}
void Estimation_Algorithm(void)
{
	if(Velocity_Position == Mode_Position){
		if(Forgetting_Type == Variable_forgetting_factor){
			/*
			epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)
			L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))
			P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda
			theta(k) = theta(k-1) + L(k)*epsilon(k)
			phi(k) = [ -y(k-1) -y(k-2) u(k-1) u(k-2)]^T*/
			phi[0][0] = -pre_Position;
			phi[1][0] = -pre_pre_Position;
			phi[2][0] = -pre_pre_pre_Position;
			phi[3][0] = pre_out;
			phi[4][0] = pre_pre_out;
			phi[5][0] = pre_pre_pre_out;
			Tranpose_Matrix(phi,6,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/ 
			Multiply_Matrix(phiT,1,6,theta,6,1,temp);//temp = phiT(k)*theta(k-1)
			epsilon = Position - temp[0][0];
			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
			Multiply_Matrix(P,6,6,phi,6,1,numerator);//(P(k-1)*phi(k))
			Multiply_Matrix(phiT,1,6,P,6,6,temp);//temp = phiT(k)*P(k-1)
			Multiply_Matrix(temp,1,6,phi,6,1,denominator);//phiT(k)*P(k-1)*phi(k)
			phiTPphi = denominator[0][0];
			denominator[0][0] = denominator[0][0] + 1;//(1 + phiT(k)*P(k-1)*phi(k))
			Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
			/*lamda*/
			lamda = epsilon * epsilon;
			lamda = lamda / (1 + phiTPphi);
			lamda = lamda / lamda_predict;
			lamda = 1 - lamda;
			if(lamda>1){lamda = 1;}
			if(lamda <0.95){lamda =0.95;}
			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
		//	Multiply_Matrix(numerator,6,1,phiT,1,6,temp);
		//	Multiply_Matrix(temp,6,6,P,6,6,numerator);//(P(k-1)*phi(k)*phiT(k)*P(k-1))
		//	Divide_Matrix_Vector(numerator,6,6,denominator[0][0],temp);
		//	Sub_Matrix(P,temp,6,6,temp2);
		//	Divide_Matrix_Vector(temp2,6,6,lamda,P);
			Multiply_Matrix(L,6,1,temp,1,6,temp2);
			Sub_Matrix(P,temp2,6,6,temp);
			Divide_Matrix_Vector(temp,6,6,lamda,P);
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
			Multiply_Matrix_Vector(L,6,1,epsilon,temp);
			Add_Matrix(theta,temp,6,1,theta);
			/*Recursion*/
			pre_pre_pre_Position = pre_pre_Position;
			pre_pre_Position = pre_Position;
			pre_Position = Position;
			pre_pre_pre_out = pre_pre_out;
			pre_pre_out = pre_out;
			pre_out = out;
			pre_pre_Set_Position = pre_Set_Position;
			pre_Set_Position = Set_Position;
			pre_Error_value = Error_value;
			trace_P = P[0][0] + P[1][1]+ P[2][2]+ P[3][3]+ P[4][4]+ P[5][5];
			if (theta[3][0] == 0) {theta[3][0] = 1;}
		}
		else{
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)
			L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))
			P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda
			theta(k) = theta(k-1) + L(k)*epsilon(k)
			phi(k) = [ -y(k-1) -y(k-2) u(k-1) u(k-2)]^T*/
			phi[0][0] = -pre_Position;
			phi[1][0] = -pre_pre_Position;
			phi[2][0] = -pre_pre_pre_Position;
			phi[3][0] = pre_out;
			phi[4][0] = pre_pre_out;
			phi[5][0] = pre_pre_pre_out;
			Tranpose_Matrix(phi,6,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/ 
			Multiply_Matrix(phiT,1,6,theta,6,1,temp);//temp = phiT(k)*theta(k-1)
			epsilon = Position - temp[0][0];
			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
			Multiply_Matrix(P,6,6,phi,6,1,numerator);//(P(k-1)*phi(k))
			Multiply_Matrix(phiT,1,6,P,6,6,temp);//temp = phiT(k)*P(k-1)
			Multiply_Matrix(temp,1,6,phi,6,1,denominator);//phiT(k)*P(k-1)*phi(k)
			denominator[0][0] = denominator[0][0] + lamda;//(1 + phiT(k)*P(k-1)*phi(k))
			Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
		//	Multiply_Matrix(numerator,6,1,phiT,1,6,temp);
		//	Multiply_Matrix(temp,6,6,P,6,6,numerator);//(P(k-1)*phi(k)*phiT(k)*P(k-1))
		//	Divide_Matrix_Vector(numerator,6,6,denominator[0][0],temp);
		//	Sub_Matrix(P,temp,6,6,temp2);
		//	Divide_Matrix_Vector(temp2,6,6,lamda,P);
			Multiply_Matrix(L,6,1,temp,1,6,temp2);
			Sub_Matrix(P,temp2,6,6,temp);
			Divide_Matrix_Vector(temp,6,6,lamda,P);
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
			Multiply_Matrix_Vector(L,6,1,epsilon,temp);
			Add_Matrix(theta,temp,6,1,theta);
			/*Recursion*/
			pre_pre_pre_Position = pre_pre_Position;
			pre_pre_Position = pre_Position;
			pre_Position = Position;
			pre_pre_pre_out = pre_pre_out;
			pre_pre_out = pre_out;
			pre_out = out;
			pre_pre_Set_Position = pre_Set_Position;
			pre_Set_Position = Set_Position;
			pre_Error_value = Error_value;
			trace_P = P[0][0] + P[1][1]+ P[2][2]+ P[3][3]+ P[4][4]+ P[5][5];
			if (theta[3][0] == 0) {theta[3][0] = 1;}
		}}
	else{
		if(Forgetting_Type == Variable_forgetting_factor){
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)
			L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))
			P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda
			theta(k) = theta(k-1) + L(k)*epsilon(k)
			phi(k) = [ -y(k-1) -y(k-2) u(k-1) u(k-2)]^T*/
			phi[0][0] = -pre_Frequency;
			phi[1][0] = -pre_pre_Frequency;
			phi[2][0] = pre_out;
			phi[3][0] = pre_pre_out;
			Tranpose_Matrix(phi,4,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)*/
			Multiply_Matrix(phiT,1,4,theta,4,1,temp);//phiT(k) * theta(k-1)
			epsilon = Frequency - temp[0][0];
			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
			Multiply_Matrix(P,4,4,phi,4,1,numerator);//(P(k-1)*phi(k))
			Multiply_Matrix(phiT,1,4,P,4,4,temp);
			Multiply_Matrix(temp,1,4,phi,4,1,denominator);//phiT(k)*P(k-1)*phi(k)
			phiTPphi = denominator[0][0];
			denominator[0][0] = denominator[0][0] + 1;//(lamda + phiT(k)*P(k-1)*phi(k))
			Divide_Matrix_Vector(numerator,4,1,denominator[0][0],L);
			/*lamda*/
			lamda = epsilon * epsilon;
			lamda = lamda / (1 + phiTPphi);
			lamda = lamda / lamda_predict;
			lamda = 1 - lamda;
			if(lamda>1){lamda = 1;}
			if(lamda <0.95){lamda =0.95;}
			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
		//	Multiply_Matrix(numerator,4,1,phiT,1,4,temp);
		//  Multiply_Matrix(temp,4,4,P,4,4,numerator);//(P(k-1)*phi(k)*phiT(k)*P(k-1))
		//	Divide_Matrix_Vector(numerator,4,4,denominator[0][0],temp);
		//  Sub_Matrix(P,temp,4,4,temp2);
		//	Divide_Matrix_Vector(temp2,4,4,lamda,P);
			Multiply_Matrix(L,4,1,temp,1,4,temp2);
			Sub_Matrix(P,temp2,4,4,temp);
			Divide_Matrix_Vector(temp,4,4,lamda,P);
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
			Multiply_Matrix_Vector(L,4,1,epsilon,temp);
			Add_Matrix(theta,temp,4,1,theta);
			pre_pre_Frequency = pre_Frequency;
			pre_Frequency = Frequency;
			pre_pre_out = pre_out;
			pre_out = out;
			pre_Set_Frequency = Set_Frequency;
			trace_P = P[0][0] + P[1][1]+ P[2][2]+ P[3][3];
			if (theta[2][0] == 0) {theta[2][0] = 1;}
		}
		else{
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)
			L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))
			P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda
			theta(k) = theta(k-1) + L(k)*epsilon(k)
			phi(k) = [ -y(k-1) -y(k-2) u(k-1) u(k-2)]^T*/
			phi[0][0] = -pre_Frequency;
			phi[1][0] = -pre_pre_Frequency;
			phi[2][0] = pre_out;
			phi[3][0] = pre_pre_out;
			Tranpose_Matrix(phi,4,1,phiT);
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)*/
			Multiply_Matrix(phiT,1,4,theta,4,1,temp);//phiT(k) * theta(k-1)
			epsilon = Frequency - temp[0][0];
			/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
			Multiply_Matrix(P,4,4,phi,4,1,numerator);//(P(k-1)*phi(k))
			Multiply_Matrix(phiT,1,4,P,4,4,temp);
			Multiply_Matrix(temp,1,4,phi,4,1,denominator);//phiT(k)*P(k-1)*phi(k)
			denominator[0][0] = denominator[0][0] + lamda;//(lamda + phiT(k)*P(k-1)*phi(k))
			Divide_Matrix_Vector(numerator,4,1,denominator[0][0],L);
			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
		//	Multiply_Matrix(numerator,4,1,phiT,1,4,temp);
		//  Multiply_Matrix(temp,4,4,P,4,4,numerator);//(P(k-1)*phi(k)*phiT(k)*P(k-1))
		//	Divide_Matrix_Vector(numerator,4,4,denominator[0][0],temp);
		//  Sub_Matrix(P,temp,4,4,temp2);
		//	Divide_Matrix_Vector(temp2,4,4,lamda,P);
			Multiply_Matrix(L,4,1,temp,1,4,temp2);
			Sub_Matrix(P,temp2,4,4,temp);
			Divide_Matrix_Vector(temp,4,4,lamda,P);
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
			Multiply_Matrix_Vector(L,4,1,epsilon,temp);
			Add_Matrix(theta,temp,4,1,theta);
			pre_pre_Frequency = pre_Frequency;
			pre_Frequency = Frequency;
			pre_pre_out = pre_out;
			pre_out = out;
			pre_Set_Frequency = Set_Frequency;
			trace_P = P[0][0] + P[1][1]+ P[2][2]+ P[3][3];
			if (theta[2][0] == 0) {theta[2][0] = 1;}
}}}
void STR_Position(void)
{
	MRC_Design();
	Estimation_Algorithm();
	Send2PC();
}
void PID_Position(void)
{
	Position = TIM_GetCounter(TIM5);
	/*PID Calculation*/
	Error_value = Set_Position - Position;
	P_part = pKp * Error_value;
	I_part += pKi * Error_value / 200;
	D_part = pKd * (Error_value - pre_Error_value) * 200;
	out+= P_part + I_part + D_part;
	if (out<0) 
	{
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		if (out < -3999) {out = -3999;}
		out_temp = out;
		out = -out;
	}
	else
	{			
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		if (out > 3999) {out = 3999;}
		out_temp = out;
	}
	TIM_SetCompare1(TIM3,out);
	pre_Error_value = Error_value;
	out = out_temp;	
	Send2PC();
}
void STR_Velocity(void)
{
	NVIC_DisableIRQ(TIM5_IRQn);
	MRC_Design();
	Estimation_Algorithm();
	Send2PC();
	NVIC_EnableIRQ(TIM5_IRQn);	
}
void PID_Velocity(void)
{
	NVIC_DisableIRQ(TIM5_IRQn);
	if (IC2Value > 0){
		if (Motor_Direction == 1){
			Frequency = 1750000 / IC2Value;
		}
		else{
			Frequency = -1750000 / IC2Value;
		}
		TIM_SetCounter(TIM5, 0);
	}
	/*PID Calculation*/
	Error_value = Set_Frequency - Frequency;
	P_part = vKp * Error_value;
	I_part += vKi * Error_value / 200;
	D_part = vKd * (Error_value - pre_Error_value) * 200;
	out+= P_part + I_part + D_part;
	if (out<0) 
	{
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		if (out < -3999) {out = -3999;}
		out_temp = out;
		out = -out;
	}
	else
	{			
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		if (out > 3999) {out = 3999;}
		out_temp = out;
	}
	TIM_SetCompare1(TIM3,out);
	pre_Error_value = Error_value;
	out = out_temp;	
	Send2PC();
	NVIC_EnableIRQ(TIM5_IRQn);
}
void Model_Position(void)
{
	/* MODEL 1			POT = 0% 		Tqd < 0.0001		0.005 seconds    
	epsilon = 0.826085 ~ 1                   POT = 0%                     POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
	Wn = 40000 ~ 40000                               Tqd < 0.0001								 	Tqd = 4 / (epsilon * Wn)
	*/ 
	if (Position_Model == 1){
		r1 = theta[4][0] / theta[3][0];
		r2 = theta[5][0] / theta[3][0];
		t0 = 0.999999999999999 / theta[3][0];
		t1 = 2.75395408819753e-85 / theta[3][0];
		s0 = (-2.76779305358719e-87 - theta[0][0]) / theta[3][0];
		s1 = (1.91516959667544e-174 - theta[1][0]) / theta[3][0];
		s2 = -theta[2][0] / theta[3][0];
	}
	/* MODEL 2			POT = 0% 		Tqd < 2		0.005 seconds   
	epsilon = 0.826085 ~ 1                   POT = 0%                 POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
	Wn = 2 ~ 2                               Tqd < 2								 	Tqd = 4 / (epsilon * Wn)
	*/ 
	if (Position_Model == 2){
		r1 = theta[4][0] / theta[3][0];
		r2 = theta[5][0] / theta[3][0];
		t0 = 4.96679133402659e-05 / theta[3][0];
		t1 = 4.93378950789292e-05 / theta[3][0];
		s0 = (-1.98009966749834 - theta[0][0]) / theta[3][0];
		s1 = (0.980198673306755 - theta[1][0]) / theta[3][0];
		s2 = -theta[2][0] / theta[3][0];
	}
}
void Model_Velocity(void)
{
/* MODEL 1			POT = 0% 		Tqd < 0.001		0.005 seconds    
epsilon = 0.826085 ~ 1                 POT = 0%                     POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
Wn = 4000 ~ 4000                       Tqd < 0.001								 	Tqd = 4 / (epsilon * Wn)
*/
if (Velocity_Model == 1){
	s0 = (-4.12230724487735e-09 - theta[0][0]) / theta[2][0];//a1
	s1 = (4.24835425529272e-18 - theta[1][0]) / theta[2][0];//a2
	t0 = 0.999999956715774 / theta[2][0];//b1  
	t1 = 3.91619188305815e-08 / theta[2][0];//b2
	r1 = theta[3][0] / theta[2][0];
}
/* MODEL 2			POT = 0% 		Tqd < 0.002		0.005 seconds    
epsilon = 0.826085 ~ 1                   POT = 0%                     POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
Wn = 2 ~ 2                               Tqd < 2								 	Tqd = 4 / (epsilon * Wn)
*/
if (Velocity_Model == 2){
	s0 = (-1.98009966749834 - theta[0][0]) / theta[2][0];//a1
	s1 = (0.980198673306755 - theta[1][0]) / theta[2][0];//a2
	t0 = 4.96679133402659e-05 / theta[2][0];//b1  
	t1 = 4.93378950789292e-05 / theta[2][0];//b2
	r1 = theta[3][0] / theta[2][0];
}	
}
void Send2PC(void)
{
	if(Velocity_Position == Mode_Position){
		if (Send2PC_Counter == 0){
			printf("%10ld@%10.1f\r\n", Position, out);
			Send2PC_Counter++;
		}
		else{
			Send2PC_Counter++;
			if (Send2PC_Counter == 39) Send2PC_Counter =0;
		}
	}
	else{
		if (Send2PC_Counter == 0){
			printf ("%10.2f@%10.1f\r\n", Frequency, out);
			Send2PC_Counter++;
		}
		else{
			Send2PC_Counter++;
			if (Send2PC_Counter == 39) Send2PC_Counter =0;
}}}
void TIM4_IRQHandler(void) 
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		Frequency_Debug = 0;
	}
	if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET){
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
		IC1Value = TIM_GetCapture2(TIM4);
		if (IC1Value > 0){
					Frequency_Debug = 1750000 / IC1Value;
}}}
void TIM5_IRQHandler(void) // For encoder velocity measurement
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		Frequency = 0;
	}
	if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET){
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
		IC2Value = TIM_GetCapture2(TIM5);
		Motor_Direction = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
	}
}
void USART3_IRQHandler(void)
{
	uint8_t ch;
	//if Receive interrupt
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART3->DR; /*the character from the USART1 data register is saved in t
		check if the received character is not the LF character (used to determine end of string)
		or the if the maximum string length has been been reached*/
		if( (t != '*') && (t != '#') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			if(t == '*'){
				if (Velocity_Position == Mode_Position){
					Set_Position = atoi(received_string);
				}
				else {
					Set_Frequency = atoi(received_string);
				}
				cnt = 0;
				memset(received_string, '\0', sizeof received_string);
			}
			if(t == '#'){
				if(Velocity_Position == Mode_Position){
						NVIC_DisableIRQ(SysTick_IRQn);
						TIM_SetCompare1(TIM3,0);
						Velocity_Position = Mode_Velocity;
						TIM_DeInit(TIM5);
						TIM5_Config_Velocity();
						cnt = 0;
						memset(received_string, '\0', sizeof received_string);
						Set_Frequency = 0, pre_Set_Frequency = 0;
					  Frequency = 0, pre_Frequency = 0, pre_pre_Frequency = 0;
						out = 0, pre_out = 0, pre_pre_out = 0, out_temp = 0;
						Error_value = 0, pre_Error_value = 0;
						Initial_Condition_Position();
						NVIC_Config_Velocity();
				}
				else{
						NVIC_DisableIRQ(SysTick_IRQn);
						TIM_SetCompare1(TIM3,0);
						Velocity_Position = Mode_Position;
						TIM_DeInit(TIM5);
					  TIM5_Config_Position();
						cnt = 0;
						memset(received_string, '\0', sizeof received_string);
						Set_Position = 0, pre_Set_Position = 0;
						Position = 0, pre_Position = 0, pre_pre_Position = 0;
						out = 0, pre_out = 0, pre_pre_out = 0, out_temp = 0;
						Error_value = 0, pre_Error_value = 0;
					  Initial_Condition_Velocity();
						NVIC_Config_Position();
				}
				if((cnt >= MAX_STRLEN)){
					cnt = 0;
				  memset(received_string, '\0', sizeof received_string);					
}}}}
	//if Transmit interrupt
	if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET){
	#ifdef BUFFERED
			if (BufferGet(&U3Tx, &ch) == SUCCESS)//if buffer read
			{
				USART_SendData(USART3, ch);
			}
			else//if buffer empty
	#endif
			{
				//disable Transmit Data Register empty interrupt
				USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
}}}
void NMI_Handler(void)
{
}
void HardFault_Handler(void)
{
  while (1)
  {}
}
void MemManage_Handler(void)
{
  while (1)
  {}
}
void BusFault_Handler(void)
{
  while (1)
  {}
}
void UsageFault_Handler(void)
{
  while (1)
  {}
}
void DebugMon_Handler(void)
{
}
void SVC_Handler(void)
{
}
void PendSV_Handler(void)
{
}
