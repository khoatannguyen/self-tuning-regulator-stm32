/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* Private define ------------------------------------------------------------*/
#define STR 1
#define PID 0
//#define Motor_delay_effect // For position
//#define Integrator_Roburst
//#define Sinoid_Setpoint
#define Sinoid_vAmplitude 3000
#define Sinoid_pAmplitude 8000
#define Sinoid_Period 3
#define Sinoid_Counter_Max Sinoid_Period*200
//#define Frequency_Debug_Pin13
#define MAX_STRLEN 11 // this is the maximum string length of our string in characters
//##############################################################################################
volatile unsigned char Position_Model = 1, Velocity_Model = 0, lamda_Model = 1;
//0: fucking fast /5	1: fast /2		2: normal =		3: slow *2
volatile unsigned char Controller = STR; // STR PID
volatile unsigned char Velocity_Position = Mode_Velocity; // Mode_Position Mode_Velocity
double const pKp = 0.2, pKd = 0.075/0.005, pKi = 0.00005*0.005; //pKp = 0.2, pKd = 0.075, pKi = 0.00005;
double const vKp = 0.02, vKd = 0.015/0.005, vKi = 0.00005*0.005;//vKp = 0.02, vKd = 0.015, vKi = 0.00005;
double const vE = vKp + 0.5*vKi + vKd, vpE = -vKp + 0.5*vKi - 2*vKd, vppE = vKd;
double const inside_Sine = 2*3.14159265358979323*0.005/Sinoid_Period;
//##############################################################################################
char received_string[MAX_STRLEN+1]; // this will hold the received string
double Set_Frequency = 0, pre_Set_Frequency = 0, pre_pre_Set_Frequency = 0;
double Frequency = 0, Frequency_Debug = 0, pre_Frequency = 0, pre_pre_Frequency = 0, pre_pre_pre_Frequency = 0;
double Error_value = 0, pre_Error_value = 0 , pre_pre_Error_value = 0, pre_pre_pre_Error_value = 0, pre_pre_pre_pre_Error_value = 0;
long Set_Position = 0, pre_Set_Position = 0, pre_pre_Set_Position = 0, pre_pre_pre_Set_Position = 0, pre_pre_pre_pre_Set_Position = 0;
long Position = 0, pre_Position = 0, pre_pre_Position = 0, pre_pre_pre_Position = 0, pre_pre_pre_pre_Position = 0;
double out = 0, pre_out = 0, pre_pre_out = 0, pre_pre_pre_out = 0, pre_pre_pre_pre_out = 0, out_temp = 0, out_real = 0;
unsigned int out_int;
double s0,s1,s2,s3,t0,t1,t2,r1,r2,r3;
double P_part = 0, I_part = 0, D_part = 0;
extern double phi[6][6], theta[6][6], theta_temp[6][6], L[6][6], P[6][6], phiT[6][6]; 
extern double temp[6][6], temp2[6][6], Kalman[6][6], denominator[6][6], numerator[6][6];
extern double lamda, lamda_predict, phiTPphi, trace_P;
extern double epsilon;
unsigned char Send2PC_Counter = 0; 
unsigned int Sinoid_Counter = 0;
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
		#ifdef Sinoid_Setpoint
		Set_Position = Sinoid_pAmplitude*sin(inside_Sine*Sinoid_Counter);
		Sinoid_Counter++;
		if (Sinoid_Counter == Sinoid_Counter_Max) Sinoid_Counter = 0;
		#endif
		if(Controller == STR){
			STR_Position();
		}
		else{ //PID Position Control
			PID_Position();
		}
	}
	else{ //STR Velocity Control
		#ifdef Sinoid_Setpoint
		Set_Frequency = Sinoid_vAmplitude*sin(inside_Sine*Sinoid_Counter);
		Sinoid_Counter++;
		if (Sinoid_Counter == Sinoid_Counter_Max) Sinoid_Counter = 0;
		#endif
		if(Controller == STR){
			STR_Velocity();
		}
		else{ //PID Velocity Control
			PID_Velocity();
		}
	}	
	/*#ifdef Frequency_Debug_Pin13
	GPIO_SetBits(GPIOD, GPIO_Pin_13);
	GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	#endif*/
}
void MRC_Design(void) // Base on Estimation_Algorithm this function generate suitable control signal for the motor similar to created model
{
	if(Velocity_Position == Mode_Position){
		Error_value = Set_Position - Position;
		Model_Position();
		Position = TIM_GetCounter(TIM5);
		#ifdef Integrator_Roburst
		/*r1=b2-b1/b1
		r2=b3-b2/b1
		r3=-b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1+1)/b1
		s1=(a2m-a2+a1)/b1
		s2=(a3m-a3+a2)/b1
		s3=a3/b1*/
		out = - (r1 * pre_out) - (r2 * pre_pre_out) - (r3 * pre_pre_pre_out)
					+ (t0 * Set_Position) + (t1 * pre_Set_Position) + (t2 * pre_pre_Set_Position)
					- (s0 * Position) - (s1 * pre_Position)- (s2 * pre_pre_Position) - (s3 * pre_pre_pre_Position);
		#else
		/*r1=b2/b1
		r2=b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1)/b1
		s1=(a2m-a2)/b1
		s2=(a3m-a3)/b1*/
		out = - (r1 * pre_out) - (r2 * pre_pre_out)
					+ (t0 * Set_Position) + (t1 * pre_Set_Position) + (t2 * pre_pre_Set_Position)
					- (s0 * Position) - (s1 * pre_Position)- (s2 * pre_pre_Position);
		#endif
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
		out_int = (unsigned)(int)(rint(out));
		TIM_SetCompare1(TIM3,out_int);
		out = out_temp;
	}
	else{
		Model_Velocity();
		if (IC2Value > 0){
			if (Motor_Direction == 1){
				Frequency = 1750000 / IC2Value;
				if (Frequency < 200) Frequency = 0;
			}
			else{
				Frequency = -1750000 / IC2Value;
				if (Frequency > -200) Frequency = 0;
			}
		}
	  #ifdef Integrator_Roburst
		out = - (r1 * pre_out) -(r2 * pre_pre_out)
					+ (t0 * Set_Frequency) + (t1 * pre_Set_Frequency) 
					- (s0 * Frequency) - (s1 * pre_Frequency) - (s2 * pre_pre_Frequency);
		#else
		out = -(r1 * pre_out) 
					+ (t0 * Set_Frequency) + (t1 * pre_Set_Frequency) 
					- (s0 * Frequency) - (s1 * pre_Frequency);
		#endif
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
		out_int = (unsigned)(int)(rint(out));
		TIM_SetCompare1(TIM3,out_int);
		out = out_temp;
}}
void Estimation_Algorithm(void)
{
	if(Velocity_Position == Mode_Position){
		if(lamda_Model == 1){
			/*
			epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)
			L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))
			P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda
			theta(k) = theta(k-1) + L(k)*epsilon(k)
			phi(k) = [ -y(k-1) -y(k-2) u(k-1) u(k-2)]^T*/
			//Added the number of time steps before the input u affects the output y
			#ifdef Motor_delay_effect
			phi[0][0] = -pre_pre_Position;
			phi[1][0] = -pre_pre_pre_Position;
			phi[2][0] = -pre_pre_pre_pre_Position;
			phi[3][0] = pre_pre_out;
			phi[4][0] = pre_pre_pre_out;
			phi[5][0] = pre_pre_pre_pre_out;
			#else
			phi[0][0] = -pre_Position;
			phi[1][0] = -pre_pre_Position;
			phi[2][0] = -pre_pre_pre_Position;
			phi[3][0] = pre_out;
			phi[4][0] = pre_pre_out;
			phi[5][0] = pre_pre_pre_out;
			#endif
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
			if(lamda<0.95){lamda = 0.95;}
			/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
			Multiply_Matrix(L,6,1,temp,1,6,temp2);
			Sub_Matrix(P,temp2,6,6,temp);
			Divide_Matrix_Vector(temp,6,6,lamda,P);
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
			theta_temp[3][0] = theta[3][0];
			Multiply_Matrix_Vector(L,6,1,epsilon,temp);
			Add_Matrix(theta,temp,6,1,theta);
			/*Recursion*/
			pre_pre_pre_pre_Position = pre_pre_pre_Position;
			pre_pre_pre_Position = pre_pre_Position;
			pre_pre_Position = pre_Position;
			pre_Position = Position;
			pre_pre_pre_pre_out = pre_pre_pre_out;
			pre_pre_pre_out = pre_pre_out;
			pre_pre_out = pre_out;
			pre_out = out;
			pre_pre_pre_Set_Position = pre_pre_Set_Position;
			pre_pre_Set_Position = pre_Set_Position;
			pre_Set_Position = Set_Position;
			pre_pre_Error_value = pre_Error_value;
			pre_Error_value = Error_value;
			trace_P = P[0][0] + P[1][1]+ P[2][2]+ P[3][3]+ P[4][4]+ P[5][5];
			if (theta[3][0] == 0) {theta[3][0] = theta_temp[3][0];}
		}
		if(lamda_Model == 2){
				
		}
}
	else{
		if(lamda_Model == 1){
			/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)
			L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))
			P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda
			theta(k) = theta(k-1) + L(k)*epsilon(k)
			phi(k) = [ -y(k-1) -y(k-2) u(k-1) u(k-2)]^T*/
			//Added the number of time steps before the input u affects the output y
			#ifdef Motor_delay_effect
			phi[0][0] = -pre_pre_Frequency;
			phi[1][0] = -pre_pre_pre_Frequency;
			phi[2][0] = pre_pre_out;
			phi[3][0] = pre_pre_pre_out;
			#else
			phi[0][0] = -pre_Frequency;
			phi[1][0] = -pre_pre_Frequency;
			phi[2][0] = pre_out;
			phi[3][0] = pre_pre_out;
			#endif
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
			Multiply_Matrix(L,4,1,temp,1,4,temp2);
			Sub_Matrix(P,temp2,4,4,temp);
			Divide_Matrix_Vector(temp,4,4,lamda,P);
			/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
			theta_temp[2][0] = theta[2][0];
			Multiply_Matrix_Vector(L,4,1,epsilon,temp);
			Add_Matrix(theta,temp,4,1,theta);
			/*Recursion*/
			pre_pre_pre_Frequency = pre_pre_Frequency;
			pre_pre_Frequency = pre_Frequency;
			pre_Frequency = Frequency;
			pre_pre_pre_out = pre_pre_out;
			pre_pre_out = pre_out;
			pre_out = out;
			pre_pre_Set_Frequency = pre_Set_Frequency;
			pre_Set_Frequency = Set_Frequency;
			trace_P = P[0][0] + P[1][1]+ P[2][2]+ P[3][3];
			if (theta[2][0] == 0) {theta[2][0] = theta_temp[2][0];}
		}
		if(lamda_Model == 2){
				
		}
}}
void STR_Position(void)
{
	//MRC_Design();
	Estimation_Algorithm();
	MRC_Design();
	Send2PC();
}
void PID_Position(void)
{
	Position = TIM_GetCounter(TIM5);
	/*PID Calculation*/
	Error_value = Set_Position - Position;
	P_part = pKp * Error_value;
	I_part += pKi * Error_value;
	D_part = pKd * (Error_value - pre_Error_value);
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
	out_int = (unsigned)(int)(rint(out));
	TIM_SetCompare1(TIM3,out_int);	
	out = out_temp;	
	pre_pre_Error_value = pre_Error_value;
	pre_Error_value = Error_value;
	pre_out = out;
	Send2PC();
}
void STR_Velocity(void)
{
	NVIC_DisableIRQ(TIM5_IRQn);
	//MRC_Design();
	Estimation_Algorithm();
	MRC_Design();
	Send2PC();
	NVIC_EnableIRQ(TIM5_IRQn);	
}
void PID_Velocity(void)
{
	NVIC_DisableIRQ(TIM5_IRQn);
	if (IC2Value > 0){
		if (Motor_Direction == 1){
			Frequency = 1750000 / IC2Value;
			if (Frequency < 200) Frequency = 0;
		}
		else{
			Frequency = -1750000 / IC2Value;
			if (Frequency > -200) Frequency = 0;
		}
	}
	Error_value = Set_Frequency - Frequency;
	/*P_part = (Error_value - pre_Error_value) * vKp;
	I_part = (Error_value + pre_Error_value) * 0.5 * vKi;
	D_part = vKd* (Error_value - (2*pre_Error_value) + pre_pre_Error_value);
	vE = vKp + 0.5*vKi + vKd, vpE = -vKp + 0.5*vKi - 2*vKd, vppE = vKd
	*/
	//out+= (vE*Error_value) + (vpE*pre_Error_value) + (vppE*pre_pre_Error_value);
	P_part = vKp * Error_value;
	I_part += vKi * Error_value;
	D_part = vKd * (Error_value - pre_Error_value);
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
	out_int = (unsigned)(int)(rint(out));
	TIM_SetCompare1(TIM3,out_int);
	out = out_temp;
	pre_pre_Error_value = pre_Error_value;
	pre_Error_value = Error_value;
	Send2PC();
	NVIC_EnableIRQ(TIM5_IRQn);
}
void Model_Position(void)
{
	/* MODEL 0			POT = 0% 								Tqd < 0.0001		0.005 seconds    
	epsilon = 0.826085 ~ 1                  POT = 0%                     POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
	Wn = 4000 ~ 4000                        Tqd < 0.001								 	Tqd = 4 / (epsilon * Wn)
	Gd =
          z^2 + 4.372e-08 z + 5.245e-20
  ---------------------------------------------
  z^3 - 4.122e-09 z^2 + 4.248e-18 z + 3.622e-41
	*/ 
	if (Position_Model == 0){
		#ifdef Integrator_Roburst
		/*r1=b2-b1/b1
		r2=b3-b2/b1
		r2=-b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1+1)/b1
		s1=(a2m-a2+a1)/b1
		s2=(a3m-a3+a2)/b1
		s3=a3/b1*/
		r1 = (theta[4][0]-theta[3][0])/ theta[3][0];
		r2 = (theta[5][0]-theta[4][0]) / theta[3][0];
		r3 = -theta[5][0] / theta[3][0];
		t0 = 0.999999952160871 / theta[3][0];
		t1 = 4.37168138727063e-08 / theta[3][0];
		t2 = 5.24488179668768e-20 / theta[3][0];
		s0 = (-4.12230724487712e-09 - theta[0][0] + 1) / theta[3][0];
		s1 = (4.24835425529119e-18 - theta[1][0] + theta[0][0]) / theta[3][0];
		s2 = (3.62206672972073e-41 - theta[2][0] + theta[1][0]) / theta[3][0];
		s3 = theta[2][0] / theta[3][0];
		#else
		/*r1=b2/b1
		r2=b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1)/b1
		s1=(a2m-a2)/b1
		s2=(a3m-a3)/b1*/
		r1 = theta[4][0] / theta[3][0];
		r2 = theta[5][0] / theta[3][0];
		t0 = 0.999999952160871 / theta[3][0];
		t1 = 4.37168138727063e-08 / theta[3][0];
		t2 = 5.24488179668768e-20 / theta[3][0];
		s0 = (-4.12230724487712e-09 - theta[0][0]) / theta[3][0];
		s1 = (4.24835425529119e-18 - theta[1][0]) / theta[3][0];
		s2 = (3.62206672972073e-41 - theta[2][0]) / theta[3][0];
		#endif
	}
	/* MODEL 1			POT = 0% 						Tqd < 2		0.005 seconds   
	epsilon = 0.826085 ~ 1              POT = 0%                POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
	Wn = 1600 ~ 1600                    Tqd < 0.0025							Tqd = 4 / (epsilon * Wn)
       0.9967 z^2 + 0.002642 z + 1.389e-09
  ---------------------------------------------
  z^3 - 0.0006709 z^2 + 1.125e-07 z - 7.117e-27
	*/ 
	if (Position_Model == 1){
		#ifdef Integrator_Roburst
		/*r1=b2-b1/b1
		r2=b3-b2/b1
		r2=-b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1+1)/b1
		s1=(a2m-a2+a1)/b1
		s2=(a3m-a3+a2)/b1
		s3=a3/b1*/
		r1 = (theta[4][0]-theta[3][0])/ theta[3][0];
		r2 = (theta[5][0]-theta[4][0]) / theta[3][0];
		r3 = -theta[5][0] / theta[3][0];
		t0 = 0.996686788860220 / theta[3][0];
		t1 = 0.00264239702982444 / theta[3][0];
		t2 = 1.38932314468172e-09 / theta[3][0];
		s0 = (-0.000670925255805015 - theta[0][0] + 1) / theta[3][0];
		s1 = (1.12535174719261e-07 - theta[1][0] + theta[0][0]) / theta[3][0];
		s2 = (-7.11704102524349e-27 - theta[2][0] + theta[1][0]) / theta[3][0];
		s3 = theta[2][0] / theta[3][0];
		#else
		/*r1=b2/b1
		r2=b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1)/b1
		s1=(a2m-a2)/b1
		s2=(a3m-a3)/b1*/
		r1 = theta[4][0] / theta[3][0];
		r2 = theta[5][0] / theta[3][0];
		t0 = 0.996686788860220 / theta[3][0];
		t1 = 0.00264239702982444 / theta[3][0];
		t2 = 1.38932314468172e-09 / theta[3][0];
		s0 = (-0.000670925255805015 - theta[0][0]) / theta[3][0];
		s1 = (1.12535174719261e-07 - theta[1][0]) / theta[3][0];
		s2 = (-7.11704102524349e-27 - theta[2][0]) / theta[3][0];
		#endif
		/*
		r1=b2/b1
		r2=b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1)/b1
		s1=(a2m-a2)/b1
		s2=(a3m-a3)/b1
		*/
	}
		/* MODEL 2			POT = 0% 						Tqd < 2		0.005 seconds   
	epsilon = 0.826085 ~ 1              POT = 0%                POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
	Wn = 800 ~ 800                   	  Tqd < 0.005							Tqd = 4 / (epsilon * Wn)
      0.9005 z^2 + 0.06319 z + 4.142e-06
  -------------------------------------------
  z^3 - 0.03663 z^2 + 0.0003355 z - 3.145e-22
	*/ 
	if (Position_Model == 2){
		#ifdef Integrator_Roburst
		/*r1=b2-b1/b1
		r2=b3-b2/b1
		r2=-b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1+1)/b1
		s1=(a2m-a2+a1)/b1
		s2=(a3m-a3+a2)/b1
		s3=a3/b1*/
		r1 = (theta[4][0]-theta[3][0])/ theta[3][0];
		r2 = (theta[5][0]-theta[4][0]) / theta[3][0];
		r3 = -theta[5][0] / theta[3][0];
		t0 = 0.900507640604406 / theta[3][0];
		t1 = 0.0631924027321025 / theta[3][0];
		t2 = 4.14151392472197e-06 / theta[3][0];
		s0 = (-0.0366312777774684 - theta[0][0] + 1) / theta[3][0];
		s1 = (0.000335462627902516 - theta[1][0] + theta[0][0]) / theta[3][0];
		s2 = (-3.14463895119926e-22 - theta[2][0] + theta[1][0]) / theta[3][0];
		s3 = theta[2][0] / theta[3][0];
		#else
		/*r1=b2/b1
		r2=b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1)/b1
		s1=(a2m-a2)/b1
		s2=(a3m-a3)/b1*/
		r1 = theta[4][0] / theta[3][0];
		r2 = theta[5][0] / theta[3][0];
		t0 = 0.900507640604406 / theta[3][0];
		t1 = 0.0631924027321025 / theta[3][0];
		t2 = 4.14151392472197e-06 / theta[3][0];
		s0 = (-0.0366312777774684 - theta[0][0]) / theta[3][0];
		s1 = (0.000335462627902516 - theta[1][0]) / theta[3][0];
		s2 = (-3.14463895119926e-22 - theta[2][0]) / theta[3][0];
		#endif
		/*
		r1=b2/b1
		r2=b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1)/b1
		s1=(a2m-a2)/b1
		s2=(a3m-a3)/b1
		*/
	}
		/* MODEL 3			POT = 0% 						Tqd < 2		0.005 seconds   
	epsilon = 0.826085 ~ 1              POT = 0%                POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
	Wn = 400 ~ 400                    Tqd < 0.01							Tqd = 4 / (epsilon * Wn)
     0.5656 z^2 + 0.1818 z + 0.0002261
  ----------------------------------------
  z^3 - 0.2707 z^2 + 0.01832 z - 3.775e-11
	*/ 
	if (Position_Model == 3){
		#ifdef Integrator_Roburst
		/*r1=b2-b1/b1
		r2=b3-b2/b1
		r2=-b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1+1)/b1
		s1=(a2m-a2+a1)/b1
		s2=(a3m-a3+a2)/b1
		s3=a3/b1*/
		r1 = (theta[4][0]-theta[3][0])/ theta[3][0];
		r2 = (theta[5][0]-theta[4][0]) / theta[3][0];
		r3 = -theta[5][0] / theta[3][0];
		t0 = 0.565590448844686 / theta[3][0];
		t1 = 0.181828503413316 / theta[3][0];
		t2 = 0.000226118616496226 / theta[3][0];
		s0 = (-0.270670568534380 - theta[0][0] + 1) / theta[3][0];
		s1 = (0.0183156394466279 - theta[1][0] + theta[0][0]) / theta[3][0];
		s2 = (-3.77513450014425e-11 - theta[2][0] + theta[1][0]) / theta[3][0];
		s3 = theta[2][0] / theta[3][0];
		#else
		/*r1=b2/b1
		r2=b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1)/b1
		s1=(a2m-a2)/b1
		s2=(a3m-a3)/b1*/
		r1 = theta[4][0] / theta[3][0];
		r2 = theta[5][0] / theta[3][0];
		t0 = 0.565590448844686 / theta[3][0];
		t1 = 0.181828503413316 / theta[3][0];
		t2 = 0.000226118616496226 / theta[3][0];
		s0 = (-0.270670568534380 - theta[0][0]) / theta[3][0];
		s1 = (0.0183156394466279 - theta[1][0]) / theta[3][0];
		s2 = (-3.77513450014425e-11 - theta[2][0]) / theta[3][0];
		#endif
		/*
		r1=b2/b1
		r2=b3/b1
		t0=b1m/b1
		t1=b2m/b1
		t2=b3m/b1
		s0=(a1m-a1)/b1
		s1=(a2m-a2)/b1
		s2=(a3m-a3)/b1
		*/
	}
}
void Model_Velocity(void)
{
/* MODEL 0			POT = 0% 						Tqd < 0.001		0.005 seconds    
epsilon = 0.826085 ~ 1              POT = 0%                     POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
Wn = 4000 ~ 4000                    Tqd < 0.001								 	 Tqd = 4 / (epsilon * Wn)
          z + 3.916e-08
  -----------------------------
  z^2 - 4.122e-09 z + 4.248e-18
*/
if (Velocity_Model == 0){
	#ifdef Integrator_Roburst
	/*r1=b2-b1/b1
	r1=-b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1+1)/b1
	s1=(a2m-a2+a1)/b1
	s2=a2/b1*/
	r1 = (theta[3][0]-theta[2][0])/theta[2][0];	
	r2 = -theta[3][0] / theta[2][0];	
	t0 = 0.999999956715774 / theta[2][0];
	t1 = 3.91619188305815e-08 / theta[2][0];
	s0 = (-4.12230724487735e-09- theta[0][0]+ 1) / theta[2][0];
	s1 = (4.24835425529272e-18 - theta[1][0]+ theta[0][0]) / theta[2][0];
	s2 = theta[1][0] / theta[2][0];
	#else
	/*r1=b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1)/b1
	s1=(a2m-a2)/b1*/
	r1 = theta[3][0] / theta[2][0];
	t0 = 0.999999956715774 / theta[2][0]; 
	t1 = 3.91619188305815e-08 / theta[2][0];
	s0 = (-4.12230724487735e-09 - theta[0][0]) / theta[2][0];
	s1 = (4.24835425529272e-18 - theta[1][0]) / theta[2][0];
	#endif
	/*
	r1=b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1)/b1
	s1=(a2m-a2)/b1
	*/
}
/* MODEL 1			POT = 0% 						Tqd < 0.005		0.005 seconds    
epsilon = 0.826085 ~ 1              POT = 0%                     POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
Wn = 1600 ~ 1600                    Tqd < 0.0025								 	 Tqd = 4 / (epsilon * Wn)
       0.997 z + 0.002348
  -----------------------------
  z^2 - 0.0006709 z + 1.125e-07
*/
if (Velocity_Model == 1){
	#ifdef Integrator_Roburst
	/*r1=b2-b1/b1
	r1=-b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1+1)/b1
	s1=(a2m-a2+a1)/b1
	s2=a2/b1*/
	r1 = (theta[3][0]-theta[2][0])/theta[2][0];	
	r2 = -theta[3][0] / theta[2][0];	
	t0 = 0.996980836348877 / theta[2][0];
	t1 = 0.00234835093049230 / theta[2][0];
	s0 = (-0.000670925255805016- theta[0][0]+ 1) / theta[2][0];
	s1 = (1.12535174719257e-07 - theta[1][0]+ theta[0][0]) / theta[2][0];
	s2 = theta[1][0] / theta[2][0];
	#else
	/*r1=b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1)/b1
	s1=(a2m-a2)/b1*/
	r1 = theta[3][0] / theta[2][0];
	t0 = 0.996980836348877 / theta[2][0]; 
	t1 = 0.00234835093049230 / theta[2][0];
	s0 = (-0.000670925255805016 - theta[0][0]) / theta[2][0];
	s1 = (1.12535174719257e-07 - theta[1][0]) / theta[2][0];
	#endif
	/*
	r1=b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1)/b1
	s1=(a2m-a2)/b1
	*/
}
/* MODEL 3			POT = 0% 								Tqd < 0.01		0.005 seconds    
epsilon = 0.826085 ~ 1                  POT = 0%                    POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
Wn = 400 ~ 400                        Tqd < 0.01				 		  		Tqd = 4 / (epsilon * Wn)
      0.594 z + 0.1537
  ------------------------
  z^2 - 0.2707 z + 0.01832
*/
if (Velocity_Model == 3){
	#ifdef Integrator_Roburst
	/*r1=b2-b1/b1
	r1=-b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1+1)/b1
	s1=(a2m-a2+a1)/b1
	s2=a2/b1*/
	r1 = (theta[3][0]-theta[2][0])/theta[2][0];	
	r2 = -theta[3][0] / theta[2][0];	
	t0 = 0.593994150290162 / theta[2][0];
	t1 = 0.153650922125347 / theta[2][0];
	s0 = (-0.270670566473225- theta[0][0]+ 1) / theta[2][0];
	s1 = (0.0183156388887342 - theta[1][0]+ theta[0][0]) / theta[2][0];
	s2 = theta[1][0] / theta[2][0];
	#else
	/*r1=b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1)/b1
	s1=(a2m-a2)/b1*/
	r1 = theta[3][0] / theta[2][0];	
	t0 = 0.593994150290162 / theta[2][0];
	t1 = 0.153650922125347 / theta[2][0];
	s0 = (-0.270670566473225 - theta[0][0]) / theta[2][0];
	s1 = (0.0183156388887342 - theta[1][0]) / theta[2][0];
	#endif
}	
/* MODEL 2			POT = 0% 						Tqd < 0.005		0.005 seconds    
epsilon = 0.826085 ~ 1              POT = 0%                     POT = e^[(-epsilon * pi) / sqrt(1 - epsilon^2)]
Wn = 800 ~ 800                    Tqd < 0.005								 	 Tqd = 4 / (epsilon * Wn)
    0.1437 z + 0.0922
  ----------------------
  z^2 - 1.029 z + 0.2645
*/
if (Velocity_Model == 2){
	#ifdef Integrator_Roburst
	/*r1=b2-b1/b1
	r1=-b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1+1)/b1
	s1=(a2m-a2+a1)/b1
	s2=a2/b1*/
	r1 = (theta[3][0]-theta[2][0])/theta[2][0];	
	r2 = -theta[3][0] / theta[2][0];	
	t0 = 0.908421805556329 / theta[2][0];
	t1 = 0.0552823792941050 / theta[2][0];
	s0 = (-0.0366312777774683- theta[0][0]+ 1) / theta[2][0];
	s1 = (0.000335462627902512 - theta[1][0]+ theta[0][0]) / theta[2][0];
	s2 = theta[1][0] / theta[2][0];
	#else
	/*r1=b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1)/b1
	s1=(a2m-a2)/b1*/
	r1 = theta[3][0] / theta[2][0];
	t0 = 0.908421805556329 / theta[2][0]; 
	t1 = 0.0552823792941050 / theta[2][0];
	s0 = (-0.0366312777774683 - theta[0][0]) / theta[2][0];
	s1 = (0.000335462627902512 - theta[1][0]) / theta[2][0];
	#endif
	/*
	r1=b2/b1
	t0=b1m/b1
	t1=b2m/b1
	s0=(a1m-a1)/b1
	s1=(a2m-a2)/b1
	*/
}
}
void Send2PC(void)
{ 	if(Velocity_Position == Mode_Position){
		if (Send2PC_Counter == 0){
			//printf("%6ld%6.1f\r\n", Position, out);
			printf("%6ld%6ld\r\n", Position, Set_Position);
			//printf("%6.0f%6.0f\r\n", Frequency, Set_Frequency);//custom	
			Send2PC_Counter++;
		}
		else{
			Send2PC_Counter++;
			if (Send2PC_Counter == 39) Send2PC_Counter =0;
		}
	}
	else{
		if (Send2PC_Counter == 0){
			//printf ("%6.0f%6.0f\r\n", Frequency, out);
			printf("%6.0f%6.0f\r\n", Frequency, Set_Frequency);
			//printf("%6.0f%6.0f\r\n", Frequency, Set_Frequency);custom
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
						memset(P, '\0', sizeof P);
						Set_Frequency = 0, pre_Set_Frequency = 0;
					  Frequency = 0, pre_Frequency = 0, pre_pre_Frequency = 0;
						out = 0, pre_out = 0, pre_pre_out = 0, out_temp = 0;
						Error_value = 0, pre_Error_value = 0, pre_pre_Error_value = 0;
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
						memset(P, '\0', sizeof P);
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
