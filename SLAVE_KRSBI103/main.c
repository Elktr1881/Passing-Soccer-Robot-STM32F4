
#include "stm32f10x.h"
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_usart.h>
#include <misc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


const float keliling_roda = 15.00;

float 	Temp, Temp1, GYRO_SAVE, HMC_SAVE, GYRO, HMC8553L;
float 	Sudut, Sudut1, KOY, KOX, arah = 90;
float 	Encoder_X_Sin, Encoder_X_Cos;
float 	Encoder_Y_Sin, Encoder_Y_Cos;
int 	Pbutton,ArduST;

float 	Kalib_R, Kalib_L,Kalib_B;
int 	data, ToArdu;

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// USART 3 /////////////////////////////////////
#define MAX_STRLEN2 150
volatile char str2[MAX_STRLEN2+1];
volatile char Received_MASTER[MAX_STRLEN2+1];
void USART_MASTER(uint32_t baudrate)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART3_IRQHandler(void)
{
	if( USART_GetITStatus(USART3, USART_IT_RXNE) )
	{
		static uint8_t cnt2 = 0;
		char t = USART3->DR;
		if( (t != '\n') && (cnt2 < MAX_STRLEN2) )
		{
			str2[cnt2] = t;
			cnt2++;
		}
		else
		{
			strcpy(Received_MASTER,str2);
			cnt2 = 0;
		}
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}

void USART_TO_MASTER(char *pucBuffer)
{
    while (*pucBuffer)
    {
        USART_SendData(USART3, *pucBuffer++);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
        {
        }
    }
}

////////////////////////////////////// USART 1 /////////////////////////////////////
#define MAX_STRLEN1 50
volatile char str1[MAX_STRLEN1+1];
volatile char Received_ARDUINO[MAX_STRLEN1+1];
void USART_ARDUINO(uint32_t baudrate)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler(void)
{
	if( USART_GetITStatus(USART1, USART_IT_RXNE) )
	{
		static uint8_t cnt1 = 0;
		char t = USART1->DR;
		if( (t != '\n') && (cnt1 < MAX_STRLEN1) )
		{
			str1[cnt1] = t;
			cnt1++;
		}
		else
		{
			strcpy(Received_ARDUINO,str1);
			cnt1 = 0;
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

void USART_TO_ARDUINO(char *pucBuffer)
{
    while (*pucBuffer)
    {
        USART_SendData(USART1, *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}

//////////////////////////////////////ENCODER MAPPING/////////////////////////////////////
void ENCODER(void)
{
	GPIO_InitTypeDef GPIOB_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIOB_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14;
	GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIOB_InitStruct);

	EXTI_InitStruct.EXTI_Line = EXTI_Line13 | EXTI_Line14;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

float ENCODER_X, ENCODER_Y;
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line14))
    {
        if (GPIOB->IDR & GPIO_Pin_15)
        {
        	ENCODER_X = ENCODER_X - Encoder_X_Cos;
        	ENCODER_Y = ENCODER_Y - Encoder_Y_Sin;
        }
        else
        {
        	ENCODER_X = ENCODER_X + Encoder_X_Cos;
        	ENCODER_Y = ENCODER_Y + Encoder_Y_Sin;
        }
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line13))
    {
        if (GPIOB->IDR & GPIO_Pin_12)
        {
        	ENCODER_X = ENCODER_X - Encoder_X_Sin;
        	ENCODER_Y = ENCODER_Y + Encoder_Y_Cos;
        }
        else
        {
        	ENCODER_X = ENCODER_X + Encoder_X_Sin;
        	ENCODER_Y = ENCODER_Y - Encoder_Y_Cos;
        }
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

////////////////////////////////// TOMBOL ///////////////////////////////////////
#define BUTTON_1  	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)
#define BUTTON_2  	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11)
#define BUTTON_3  	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)
#define BUTTON_4  	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)
#define BUTTON_5  	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)
#define BUTTON_6  	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)
#define BUTTON_7 	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)
#define BUTTON_8  	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)
#define BUTTON_9  	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)
#define BUTTON_0  	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)
#define BUTTON_H 	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)
#define BUTTON_M 	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

void BUTTON()
{
	GPIO_InitTypeDef definisi_GPIO;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	definisi_GPIO.GPIO_Pin =
	GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15;
	definisi_GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	definisi_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &definisi_GPIO);

	definisi_GPIO.GPIO_Pin =
	GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7 |GPIO_Pin_8 |
	GPIO_Pin_9 ;
	definisi_GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	definisi_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &definisi_GPIO);
}
double Konv_azim(float sudut)
{
	float hasil;
	hasil= sudut- 45;
	if(hasil < 0){hasil = 359.99 + hasil;}
	if(hasil >= 360){hasil = hasil - 360;}
	return(hasil);
}

int main(void)
{
	USART_MASTER(9600);
	USART_ARDUINO(115200);
	ENCODER();
	BUTTON();
	delay_ms(700);
	USART_TO_ARDUINO("1");

    while(1)
    {
    	char *MASTER[10];
    	MASTER[0]=strtok(Received_MASTER,	" ");
    	MASTER[1]=strtok(NULL,				" ");
    	MASTER[2]=strtok(NULL,				" ");
    	MASTER[3]=strtok(NULL,				" ");
    	MASTER[4]=strtok(NULL,				" ");
    	MASTER[5]=strtok(NULL,				" ");

    	char *ARDUINO[5];
		ARDUINO[0]=strtok(Received_ARDUINO,	" ");
		ARDUINO[1]=strtok(NULL,				" ");
		ARDUINO[2]=strtok(NULL,				" ");
		ARDUINO[3]=strtok(NULL,				" ");

		if(atoi(MASTER[1]) == 1)
		{
			data 	= atoi(MASTER[0]);
			Kalib_R = atof(MASTER[2]);
			Kalib_L = atof(MASTER[3]);
			Kalib_B = atof(MASTER[4]);
			ToArdu	= atoi(MASTER[5]);
		}
		if(atoi(ARDUINO[1]) == 1)
		{
			ArduST 	 = atoi(ARDUINO[0]);
			GYRO 	 = atof(ARDUINO[2])/100;
			HMC8553L = atof(ARDUINO[3]);
		}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if(GYRO >= GYRO_SAVE)  {Temp = GYRO - GYRO_SAVE;}
		else {Temp = GYRO - GYRO_SAVE + 360.00;}
		Sudut = Temp + arah;
		if(Sudut > 360.00){Sudut = arah - (360.00 - Temp);}
		if(Sudut < 0){Sudut = 360.00 - (Temp + arah);}

		if(HMC8553L >= HMC_SAVE)  {Temp1 = HMC8553L - HMC_SAVE;}
		else {Temp1 = HMC8553L - HMC_SAVE + 360.00;}
		Sudut1 = Temp1 + arah;
		if(Sudut1 > 360.00){Sudut1 = arah - (360.00 - Temp1);}
		if(Sudut1 < 0){Sudut1 = 360.00 - (Temp + arah);}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		float conv_sdt = Sudut-45;
		if(conv_sdt < 0){conv_sdt = 360+conv_sdt;}

		float Sin = sin(conv_sdt * 3.14159265 / 180.00);
		float Cos = cos(conv_sdt * 3.14159265 / 180.00);

		Encoder_Y_Sin = Sin;
		Encoder_X_Sin = Sin;
		Encoder_Y_Cos = Cos;
		Encoder_X_Cos = Cos;

		KOY = abs(keliling_roda * ENCODER_Y / 400.00);
		KOX = abs(keliling_roda * ENCODER_X / 400.00);

		int kode;
		if(ENCODER_X< 0 && ENCODER_Y< 0) {kode = 22;}
		if(ENCODER_X< 0 && ENCODER_Y>=0) {kode = 21;}
		if(ENCODER_X>=0 && ENCODER_Y< 0) {kode = 12;}
		if(ENCODER_X>=0 && ENCODER_Y>=0) {kode = 11;}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    	if(BUTTON_1 == RESET) {Pbutton= 11;} else
    	if(BUTTON_2 == RESET) {Pbutton= 22;} else
    	if(BUTTON_3 == RESET) {Pbutton= 33;} else
    	if(BUTTON_4 == RESET) {Pbutton= 44;} else
		if(BUTTON_5 == RESET) {Pbutton= 55;} else
		if(BUTTON_6 == RESET) {Pbutton= 66;} else
		if(BUTTON_7 == RESET) {Pbutton= 77;} else
		if(BUTTON_8 == RESET) {Pbutton= 88;} else
		if(BUTTON_9 == RESET) {Pbutton= 99;} else
		if(BUTTON_0 == RESET) {Pbutton= 34;} else
		if(BUTTON_H == RESET) {Pbutton= 56;} else
		if(BUTTON_M == RESET) {Pbutton= 12;} else
		{Pbutton = 0;}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    	if(data == 0){USART_TO_ARDUINO("0");}
    	else if(data == 1) 	{USART_TO_ARDUINO("1");}
    	else if(data == 22) {USART_TO_ARDUINO("2");}
    	else if(data == 33) {USART_TO_ARDUINO("3");}

    	else if(data == 2){ENCODER_X = ENCODER_Y = 0.00;} // reset tengah
    	else if(data == 3) // reset home
    	{
    		arah = 90.00;
    		GYRO_SAVE = GYRO;
    		HMC_SAVE  = HMC8553L;
    		ENCODER_X = -220 / keliling_roda * 400.00;
			ENCODER_Y = -630 / keliling_roda * 400.00;
    	}
    	else if(data == 4) //reset hadap gawang
    	{
    		arah = 90.00;
    		GYRO_SAVE = GYRO;
    		HMC_SAVE  = HMC8553L;
    	}
    	else if(data == 5) // kalibrasi home angel & X (line white)
		{
    		ENCODER_Y = (-400.00 + Kalib_R) / keliling_roda * 400.00 ;
    		arah = 90.00 + (Kalib_L*0.75);
    		if(arah < 0){arah += 360;}
    		GYRO_SAVE = GYRO;
			HMC_SAVE  = HMC8553L;
		}

    	else if(data == 6) //kalibrasi run angel & Y (Center line)
		{
    		ENCODER_X = (250.00 - Kalib_R) / keliling_roda * 400.00 ;
		}
    	else if(data == 7) //kalibrasi run angel & Y (penalty line)
		{
    		ENCODER_Y = (350.00 + Kalib_R) / keliling_roda * 400.00;
    		arah = 0.00 - Kalib_L;
    		if(arah < 0){arah = 360.00 + arah;}
    		GYRO_SAVE = GYRO;
			HMC_SAVE  = HMC8553L;
		}
    	else if(data == 8) //kalibrasi run X (white corner)
		{
			ENCODER_X = (-270.00 + Kalib_R) / keliling_roda * 400.00;
		}

		else if(data == 88) //kalibrasi run X (white corner)
		{
			ENCODER_X = -270.00 / keliling_roda * 400.00;
			ENCODER_Y = 415.00 / keliling_roda * 400.00;
		}

		else if(data == 9) //kalibrasi home XY (blue box)
		{
			ENCODER_X = (270.00 - Kalib_R) / keliling_roda * 400.00;
			ENCODER_Y = (200.00 - Kalib_B) / keliling_roda * 400.00;
		}

		else if(data == 99) //kalibrasi home XY (blue box)
		{
			ENCODER_X = (-270.00 + Kalib_R) / keliling_roda * 400.00;
			ENCODER_Y = (415.00 + Kalib_B) / keliling_roda * 400.00;
		}

    	char buff_to_master[50];
    	sprintf(buff_to_master,"%i,1,%i,%i,%i,%i,%i,%i,\n",
    			Pbutton,ArduST,(int)Sudut,(int)Sudut1,kode,(int)KOX,(int)KOY);
    	USART_TO_MASTER(buff_to_master);
    }
}
