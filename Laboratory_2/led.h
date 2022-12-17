/*
 * led.h
 *
 *  Created on: Nov 22, 2022
 *      Author: Студент
 */

#ifndef LED_H_
#define LED_H_

#define PERIPH_BASE 0x40000000
#define PERIPH_BIT_BAND 0x42000000			/* allias */
#define AHB1_BASE PERIPH_BASE + 0x00020000


typedef struct
{
   uint32_t MODER;
   uint32_t OTYPER;
   uint32_t OSPEEDR;
   uint32_t PUPDR;
   uint32_t IDR;
   uint32_t ODR;
   uint32_t BSRR;
   uint32_t LCKR;
   uint32_t AFR[2];
} GPIO_STRUCT;

#define GPIOC_BASE (AHB1_BASE + 0x0800)
#define GPIOC ((GPIO_STRUCT *) GPIOC_BASE)


/* Bit-band */
#define BIT(address, bit) *((uint32_t *) (PERIPH_BIT_BAND + ( (uint32_t)address - PERIPH_BASE)*32 + bit*4))

/* Define buttons and Leds */
#define RED_BUTTON_PIN 5
#define BLUE_BUTTON_PIN 6
#define RED_LED_PIN 9
#define BLUE_LED_PIN 8




void my_delay(long int t){
    for (int i = 0; i < t; i++);
}


void updateLed(int value)
{

	 //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
     //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

	//     LED(RED, ON); //Включаем красный светодиод
    // HAL_Delay(500); //Ждем 0.5 секунды

	BIT(&GPIOC->ODR, RED_LED_PIN) = (value >> 0) & 1; // Red for 0-bit of number
	BIT(&GPIOC->ODR, BLUE_LED_PIN) = (value >> 1) & 1; // Blue for 1-bit of number
}




/*

*((uint32_t *) 0x40023830) |= 1 | 4;

updateLed(CURR_VALUE);
			delay(500);
*/


#endif /* LED_H_ */
