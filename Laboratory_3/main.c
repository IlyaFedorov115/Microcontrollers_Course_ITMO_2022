/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32f446xx.h"
#include "led.h"
#include "console.h"


enum LimitValues {
	APB1_MAX = 45,
	APB2_MAX = 90,
	APB1_TIMER_MAX = 90,
	APB2_TIMER_MAX = 180,
	AHP_MAX = 180,
	PLLP_OUT_MAX = 180,
	PLLN_OUT_MAX = 432
};

typedef struct  {
	int F_cpu;
	int F_in;
	int PLLM;
	int PLLN ;
	int PLLP ;
	int AHB_prescaler ;
	int HCKL_ ;
	int APB1_prescaler;
	int APB2_prescaler;
	int APB1_prescaler_T;
	int APB2_prescaler_T;
	int FLASH_ ;
} Coef_Nums;

Coef_Nums Coef_Nums_Start_Init()
{
	Coef_Nums res;
	res.F_cpu = 130;
	res.F_in = 8;
	res.PLLM = 0;
	res.PLLN = 0;
	res.PLLP = 2;
	res.AHB_prescaler = 1;
	res.HCKL_ = 130;
	res.APB1_prescaler = 1;
	res.APB2_prescaler = 1;
	res.APB1_prescaler_T = 1;
	res.APB2_prescaler_T = 1;
	res.FLASH_ = 2;

	return res;
}


Coef_Nums init_coef(Coef_Nums coefs)
{
	/* init PLL Block */
	if (coefs.F_in % 2 != 0){
		coefs.PLLM = coefs.F_in;
		coefs.PLLN = coefs.F_cpu * 2;
	}
	else{
		coefs.PLLM = coefs.F_in / 2;
		coefs.PLLN = coefs.F_cpu;
	}

	/* APB-PCLK init */

	for (int apb1 = 1; apb1 <= 16; apb1 *= 2){
		if ( coefs.HCKL_/apb1 <= APB1_MAX){
			coefs.APB1_prescaler = apb1;
			break;
		}
	}

	for (int apb2 = 1; apb2 <= 16; apb2 *= 2){
		if ( coefs.HCKL_ / apb2 <= APB2_MAX){
			coefs.APB2_prescaler = apb2;
			break;
		}
	}

	if (coefs.APB1_prescaler == 1)
		coefs.APB1_prescaler_T = 1;
	else
		coefs.APB1_prescaler_T = 2;


	if (coefs.APB2_prescaler == 1)
		coefs.APB2_prescaler_T = 1;
	else
		coefs.APB2_prescaler_T = 2;

	return coefs;
}

int LIMIT_WAIT_ITERS = 10000;

enum CodeProblemInit{
	ALL_OK = 0,
	HSE_NOT_RDY = 1,
	PLL_NOT_RDY = 2,
	AHB_NOT_SWT_PLL = 3
};


void SetAHBCoefs(Coef_Nums coefs)
{
	/// делитель AHB
	switch (coefs.AHB_prescaler){
	case 2:
		RCC->CFGR |= RCC_CFGR_HPRE_DIV2; //(RCC_CFGR_HPRE_DIV2 << RCC_CFGR_HPRE_Pos);
		break;
	case 4:
		RCC->CFGR |= RCC_CFGR_HPRE_DIV4; //(RCC_CFGR_HPRE_DIV4 << RCC_CFGR_HPRE_Pos);
		break;
	case 8:
		RCC->CFGR |= RCC_CFGR_HPRE_DIV8; // (RCC_CFGR_HPRE_DIV8 << RCC_CFGR_HPRE_Pos);
		break;
	case 16:
		RCC->CFGR |= RCC_CFGR_HPRE_DIV16;// (RCC_CFGR_HPRE_DIV16 << RCC_CFGR_HPRE_Pos);
		break;
	case 64:
		RCC->CFGR |= RCC_CFGR_HPRE_DIV64; //(RCC_CFGR_HPRE_DIV64 << RCC_CFGR_HPRE_Pos);
		break;
	case 128:
		RCC->CFGR |=  RCC_CFGR_HPRE_DIV128;// (RCC_CFGR_HPRE_DIV128 << RCC_CFGR_HPRE_Pos);
		break;
	case 256:
		RCC->CFGR |=  RCC_CFGR_HPRE_DIV256;// (RCC_CFGR_HPRE_DIV256 << RCC_CFGR_HPRE_Pos);
		break;
	case 512:
		RCC->CFGR |=  RCC_CFGR_HPRE_DIV512;// (RCC_CFGR_HPRE_DIV512 << RCC_CFGR_HPRE_Pos);
		break;
	default:
		RCC->CFGR |=  RCC_CFGR_HPRE_DIV1;// (RCC_CFGR_HPRE_DIV1 << RCC_CFGR_HPRE_Pos);
	}


	switch (coefs.APB1_prescaler){
	case 2:
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;//(RCC_CFGR_PPRE1_DIV2 << RCC_CFGR_PPRE1_Pos);
		break;
	case 4:
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;//(RCC_CFGR_PPRE1_DIV4 << RCC_CFGR_PPRE1_Pos);
		break;
	case 8:
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV8;//(RCC_CFGR_PPRE1_DIV8 << RCC_CFGR_PPRE1_Pos);
		break;
	case 16:
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV16;//(RCC_CFGR_PPRE1_DIV16 << RCC_CFGR_PPRE1_Pos);
		break;
	default:
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;//(RCC_CFGR_PPRE1_DIV1 << RCC_CFGR_PPRE1_Pos);
	}

	switch (coefs.APB2_prescaler){
	case 2:
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;//(RCC_CFGR_PPRE2_DIV2 << RCC_CFGR_PPRE2_Pos);
		break;
	case 4:
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;//(RCC_CFGR_PPRE2_DIV4 << RCC_CFGR_PPRE2_Pos);
		break;
	case 8:
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV8;//(RCC_CFGR_PPRE2_DIV8 << RCC_CFGR_PPRE2_Pos);
		break;
	case 16:
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV16;//(RCC_CFGR_PPRE2_DIV16 << RCC_CFGR_PPRE2_Pos);
		break;
	default:
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;//(RCC_CFGR_PPRE2_DIV1 << RCC_CFGR_PPRE2_Pos);
	}

}





int InitClock(Coef_Nums coefs)
{

	/* Значение для FLASH */
	FLASH->ACR |= (coefs.FLASH_ << FLASH_ACR_LATENCY_Pos);

	/* установить бит HSEON. Запустить HSE генератор */
	RCC->CR |= RCC_CR_HSEON_Msk;

	//??? ОТключить PLL
	//RCC->CR &= ~(1<<RCC_CR_PLLON_Pos); //Останавливаем PLL

	/* ждем пока запустится */
	for (int counter = 0; ; counter++)
	{
		// ok, break
		if(RCC->CR & (1<<RCC_CR_HSERDY_Pos))
			break;

		/* не запустился, обнуляем*/
		if (counter >= LIMIT_WAIT_ITERS){
		    RCC->CR &= ~(1<<RCC_CR_HSEON_Pos);
			return HSE_NOT_RDY;
		}
	}



	/* установить коэффициенты PLL */
	RCC->PLLCFGR = 0;
	RCC->PLLCFGR |= (coefs.PLLM << RCC_PLLCFGR_PLLM_Pos);	//0x000100
	RCC->PLLCFGR |= (coefs.PLLN << RCC_PLLCFGR_PLLN_Pos);
	RCC->PLLCFGR |= (0x00 << RCC_PLLCFGR_PLLP_Pos);
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC; //Тактирование PLL от HSE


	/* Включаем PLL и ждем */
	RCC->CR |= RCC_CR_PLLON;
	for (int counter = 0; ; counter++)
	{
	  if(RCC->CR & RCC_CR_PLLRDY)
		  break;
	  if(counter >= LIMIT_WAIT_ITERS){
		RCC->CR &= ~(1<<RCC_CR_HSEON_Pos);
		RCC->CR &= ~RCC_CR_PLLON;
		return PLL_NOT_RDY;
	  }
	}

	/* установить коэффициенты шин */
	SetAHBCoefs(coefs);
	RCC->CFGR |= (0x02 << RCC_CFGR_SW_Pos);// Переключаемся на работу от PLL

	// ждем переключения
	int counterPLL = 0;
	while((RCC->CFGR & RCC_CFGR_SWS_Msk) != (0x02<<RCC_CFGR_SWS_Pos))
	{
		counterPLL++;
		if (counterPLL >= LIMIT_WAIT_ITERS)
			return AHB_NOT_SWT_PLL;
	}

	return ALL_OK;
}




//define main clock value
#define HCLK 130000000

//define System Tick Timer frequency
#define SysTicksClk 9000
//define System Tick Timer prescaler
#define SysTicks HCLK/SysTicksClk



void mig_led(){
	GPIOC->ODR ^= (1<< RED_LED_PIN);
}

volatile uint32_t timestamp = 0;
 void SysTick_Handler (void)
{
   timestamp++;
   if (timestamp >= SysTicksClk-1){
	   timestamp = 0;
	   mig_led();
   }
}


/*
 * lab 3 part
 */

 /** 3 Lab part */
  void enableTIR(void)
  {
  	USART2->CR1 |= USART_CR1_TXEIE; // . Содержимое TDR передано в сдвиговый
  }

  void disableTIR(void)
  {
  	USART2->CR1 &= ~USART_CR1_TXEIE;
  }



 float lnFunction(int num)
 {
     if (num <= 0) return 99.99;

     float x = (num - 1.0) / (num + 1.0);
     float x_3 = x * x * x;
     float x_5 = x_3 * x * x;
     float x_7 = x_5 * x * x;
     float x_9 = x_7 * x * x;

     return 2 * (x + x_3 / 3.0 + x_5 * 2 / 10.0 + x_7 / 7.0 + x_9 / 9.0);
 }



 struct USART_DATA {
 	char Buffer_Write[128];
 	char Buffer_Read[128];
 	int Counter_Read; 		// счетчик приходящих
 	int Len_Read; 			// число полученных
 	int Write_Counter; 		// число для передачи
 };

 struct USART_DATA Message_Data_Struct;


 void InitUSART_Data()
 {
	 SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

	//Message_Data_Struct.Buffer_Write = "hello\0";
 	Message_Data_Struct.Counter_Read = 0;
 	Message_Data_Struct.Len_Read = 0;
 	Message_Data_Struct.Write_Counter = 0;
 }

 // ------------------------------------------------




// ------------------------------------------------------

 /** Конец ввода- отправляем ответ */

 void float2str(float num, char* buffer)
 {
	    int intpart = (int)num;
	    float decpart = num - intpart;
	    if (num / 10 >= 1)
	           buffer[0] = '0' + (intpart/10) % 10;
	     else
	           buffer[0] = '0';

	     buffer[1] = '0' +  intpart % 10;
	     buffer[2] = '.';
	     int lastpart = (int)(1000 * decpart);
	     buffer[3] = '0' + (lastpart / 100) % 10;
	     buffer[4] = '0' + (lastpart / 10) % 10;
	     buffer[5] = '0' + (lastpart / 1) % 10;
	     return;
 }



 void USART_Response()
 {
 	Message_Write_2_Buffer(Message_Data_Struct.Buffer_Write, lnFunction(Message_Get_Num(Message_Data_Struct.Buffer_Read)));
 	enableTIR();
    /* отослать данное назад
		//sprintf(Message_Data_Struct.Buffer_Write, "\nln(x)=%06.3f\r", 84.1);

	 for (int i = 0; i < 10; i++)
		 Message_Data_Struct.Buffer_Write[i] = 'a' + i;

	 int ind = 0;

	Message_Write_2_Buffer(Message_Data_Struct.Buffer_Write, 14.1);

	 fun(Message_Data_Struct.Buffer_Write);
*/
/*
 	for (ind = 0; ind < 10; ind++){
 		while ((USART2->SR & USART_SR_TXE) == 0){}
 		//while((USART2->SR & USART_CR1_TXEIE) == 0);
 		USART2->DR = Message_Data_Struct.Buffer_Write[ind];
 	}
*/
	//USART2->DR = 65;
 }


 void USART2_IRQHandler(void)
 {
 	/** чисто прием */
 	if (USART2->SR & USART_SR_RXNE){
 		Message_Data_Struct.Buffer_Read[Message_Data_Struct.Counter_Read++] = USART2->DR;

 		if (Message_Data_Struct.Buffer_Read[Message_Data_Struct.Counter_Read-1] == 'E'){
 			//USART2->DR;
 			Message_Data_Struct.Len_Read = Message_Data_Struct.Counter_Read;
 			Message_Data_Struct.Counter_Read = 0;
 			USART_Response();
 		}

 	}
/*
 	if (USART2->SR & USART_SR_IDLE){
 		// сбросим влаг IDLE, типо конец приема
 		USART2->DR;
 		Message_Data_Struct.Len_Read = Message_Data_Struct.Counter_Read;
 		Message_Data_Struct.Counter_Read = 0;
 		USART_Response();
 	}
*/

 	/** передача */
	if (USART2->SR & USART_SR_TXE) {
 		if (Message_Data_Struct.Write_Counter == LEN_TEMPLATE_GET){
 			Message_Data_Struct.Write_Counter = 0;
 			disableTIR();
 		} else{
 			USART2->DR = Message_Data_Struct.Buffer_Write[Message_Data_Struct.Write_Counter++];
 		}
 	}

 }

void testWriteFunction(char buf[], int len)
 {
 	for (int i = 0; i < len; i++){
 		while(!(USART2->SR & USART_SR_TXE)){} //USART2->SR&0x0080
 		USART2->DR = buf[i];
 	}
 }


#define BAUDRATE 57600
#define FREQ_COEF 1000000
#define APB1X 32.5 //65

//#define APBx_FREQ 			32500000
#define APBx_FREQ 			32500000

int main()
{

	InitUSART_Data();
	// Открыли переферию
	RCC->AHB1ENR |= 1 | 4;

	GPIOC->MODER |= (1 << (BLUE_LED_PIN*2));
	GPIOC->MODER |= (1 << (RED_LED_PIN*2));

	Coef_Nums coefs = Coef_Nums_Start_Init();
	coefs.F_cpu = 130;
	coefs.F_in = 20;
	coefs.HCKL_ = 130;
	coefs = init_coef(coefs);
	coefs.FLASH_ = 5;

	int res = InitClock(coefs);

	if (res != ALL_OK){
		updateLed(res);
	} else{
		SysTick_Config(SysTicks);
	}

	/** lab 3 */
	uint32_t PA2_NUM = 2;
	uint32_t PA3_NUM = 3;

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// __disable_irq(); __enable_irq

		/** вкл. альтернативную фунцию на пине  */
	GPIOA->MODER |= (1 << (PA2_NUM * 2 + 1));
	GPIOA->MODER &= ~(1 << (PA2_NUM * 2));
	GPIOA->MODER |= (1 << (PA3_NUM * 2 + 1));
	GPIOA->MODER &= ~(1 << (PA3_NUM * 2));
	GPIOA->AFR[0] |= 0x07700; //0x700; //Alt 7


	//USART2->BRR = (((float)coefs.HCKL_/coefs.APB1_prescaler) * FREQ_COEF )/ BAUDRATE;
	//32500000
	USART2->BRR = APBx_FREQ / BAUDRATE;

	USART2->CR1  |= USART_CR1_UE
					 | USART_CR1_TE
					 | USART_CR1_RXNEIE
					 | USART_CR1_RE
					 //| USART_CR1_IDLEIE; ///?????
					 | USART_CR1_TXEIE;

	disableTIR();
	NVIC_EnableIRQ(USART2_IRQn);


	//USART2->DR = 61;
	/* Loop forever */
	for(;;)
	{
		//test1Function();
		//USART2->DR = 65;
		//my_delay(5000000);
	}



	return 0;
}


void test1Function()
{
	char test[10] = "Testmy";
	enableTIR();
	testWriteFunction(test, 4);
	disableTIR();
	my_delay(400000);
}


void test2Function()
{
	// http://easyelectronics.ru/arm-uchebnyj-kurs-usart.html
	//RCC->APB2ENR  	|= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;	// GPIOA Clock ON. Alter function clock ON

	//before while just put to data registr, without interrupt
	USART2->DR = 10;

}


void test3Function()
{
	 while (1){
      // получить данное
      while ((USART2->SR & USART_SR_RXNE) == 0) {}
      uint8_t d = USART2->DR;

      // отослать данное назад
      while ((USART2->SR & USART_SR_TXE) == 0) {}
      USART2->DR = d;
  }
}


void test4Function()
{
	while((USART2->SR & USART_CR1_TXEIE) == 0) ;
	USART2->DR = 5;
	my_delay(50000);
}

/*
void USART2_IRQHandler(void)
{
	if (USART2->SR & USART_SR_RXNE)
	{
	USART2->DR = (USART2->DR)+10;
	}
}
*/






void testLed()
{
	BIT(&GPIOC->ODR, RED_LED_PIN) = 1;
	my_delay(500000);
	BIT(&GPIOC->ODR, RED_LED_PIN) = 0;
	my_delay(500000);
}


/*
http://mypractic.ru/urok-20-interfejs-uart-v-stm32-rabota-s-nim-cherez-registry-cmsis-ispolzovanie-preryvaniya-uart.html
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // разрешаем тактирование порта GPIOA

// настройка вывода PA9 (TX1) на режим альтернативной функции с активным выходом
// Биты CNF = 10, ,биты MODE = X1
GPIOA->CRH &= (~GPIO_CRH_CNF9_0);
GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);

// настройка вывода PA10 (RX1) на режим входа с подтягивающим резистором
// Биты CNF = 10, ,биты MODE = 00, ODR = 1
GPIOA->CRH &= (~GPIO_CRH_CNF10_0);
GPIOA->CRH |= GPIO_CRH_CNF10_1;
GPIOA->CRH &= (~(GPIO_CRH_MODE10));
GPIOA->BSRR |= GPIO_ODR_ODR10;

Теперь конфигурация самого UART.

*/



/**
Альтернативные функции AF7
http://microsin.net/programming/arm/stm32f407-gpio-pins-alternate-function.html
0111: AF7
077 - 0111 0111 0000 0000
70 -  0000 0111 0000 0000
p. 192
https://niuitmo-my.sharepoint.com/personal/145573_niuitmo_ru/Documents/Work/Education/Microprocessors%20systems/course/offline/docs/stm32f446re_ref.pdf?CT=1671181660205&OR=ItemsView




Steps https://www.rotr.info/electronics/mcu/arm_usart_2.htm


Примеры кода
https://embeddedexpert.io/?p=347
http://easyelectronics.ru/arm-uchebnyj-kurs-usart.html

Теория
http://dimoon.ru/obuchalka/stm32f1/programmirovanie-stm32-chast-5-portyi-vvoda-vyivoda-gpio.html

*/




/**
 * https://niuitmo-my.sharepoint.com/:f:/g/personal/145573_niuitmo_ru/EhM7EnYmthtJgWxcoqatJGoBVM_2_JdgYOHVxce_igAHPQ
 * Источники и указатели для поиска.
 * 1) USART2 к шине APB1 (45MHz) - datasheet_re, p. 16 block diagramm
 * 2) Пины: datasheet_re p. 48 table 10 (PA2 - TX, PA3 - RX). SCHEMATIC (PA2 - RX, PA3 - TX)
 * 3) Настройка пинов GPIOA: Reference manual, p. 187 port mode
 * 4) Т.к. выбрали альтернативный вывод, то в AFR записать. Selecting an alternate. Reference p. 180 Figure 19   ????
 *
*/

/**
 * 2 буфера
 * 2 счетчика считывания и отправки
 * все в одной
 * приемник работает всегда
 * отправляем - отключаем
 */



/** Links
Ln:
https://www.wikiznanie.ru/wp/index.php/%D0%9D%D0%B0%D1%82%D1%83%D1%80%D0%B0%D0%BB%D1%8C%D0%BD%D1%8B%D0%B9_%D0%BB%D0%BE%D0%B3%D0%B0%D1%80%D0%B8%D1%84%D0%BC
http://mathemlib.ru/books/item/f00/s00/z0000027/st226.shtml

*/