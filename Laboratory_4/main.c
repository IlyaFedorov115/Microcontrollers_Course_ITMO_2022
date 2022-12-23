#include <stdint.h>

#include "stm32f446xx.h"
#include "led.h"


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
	//BIT(&GPIOC->ODR, RED_LED_PIN) = 1;

}

volatile uint32_t timestamp = 0;
 void SysTick_Handler (void)
{
   timestamp++;
   if (timestamp >= SysTicksClk-1){
	   timestamp = 0;
	   //mig_led();
   }
}


#define BAUDRATE 57600
#define FREQ_COEF 1000000
#define APB1X 32.5 //65

//#define APBx_FREQ 			32500000
#define APBx_FREQ 			32500000

uint8_t tx_data[5];
uint8_t ret_data = 5;




void TIM6_DAC_IRQHandler()
{
    // Проверяем, было ли прерывание от таймера...
    if(TIM6->SR&TIM_SR_UIF)
    {
        // Сбрасываем бит TIM_SR_UIF.
        TIM6->SR&=~TIM_SR_UIF;
        mig_led();

        // Выполняем свой код по обработке прерывания.
        // .....
    }

    // Проверяем было ли прерывание от DAC и обрабатываем его
    // (если генерация этого прерывания разрешена настройками DAC).
    // .....
}

void TIM1_UP_TIM10_IRQHandler(void){
	if (TIM1->SR & TIM_SR_UIF){
		TIM1->SR &= ~TIM_SR_UIF;
		mig_led();
	}
}


#define TIMER_FREQ_I 1
#define TIMER_FREQ_OUT 10000 // Hz
#define MHZ_CONST 1000000


void initTimer(Coef_Nums coefs)
{
	//coefs.HCKL_ / coefs.APB1_prescaler_T; // Input freq = 130 Mhz

	// Включить тактовый сигнал для TIM6.
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	//TIM1->PSC = ( (coefs.HCKL_ * MHZ_CONST / coefs.APB2_prescaler_T) / TIMER_FREQ_OUT - 1)
	TIM1->PSC = (coefs.HCKL_ * MHZ_CONST * coefs.APB2_prescaler_T / coefs.APB2_prescaler) / TIMER_FREQ_OUT - 1;
	TIM1->ARR = TIMER_FREQ_OUT / TIMER_FREQ_I; // -1 maybe. 100

	TIM1->SR = 0; // clear flags

	/** разрешить прерывания */
	TIM1->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); // TIM1_DAC_IRQn

	TIM1->CR1 |= TIM_CR1_CEN;


	/*
	RCC->APB1ENR|=RCC_APB1ENR_TIM6EN;
	TIM6->PSC =  ( (coefs.HCKL_ * MHZ_CONST / coefs.APB1_prescaler_T) / TIMER_FREQ_OUT - 1);
	TIM6->CNT = TIMER_FREQ_OUT / TIMER_FREQ_I;
	TIM6->CR1 |= (TIM_CR1_CEN << 1);
*/

	/*
	TIM6->PSC = 0; // делитель

	TIM6->CR1 = 0;
	TIM6->DIER = 0;
	TIM6->SR = 0;
	TIM6->CNT = 0;

	TIM6->ARR = 0;
*/
}


void prepareDMA1Stream6(int dataNum)
{
	DMA1_Stream6->NDTR = dataNum;
	DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6; // half and transfer complete

	DMA1_Stream6->CR |= DMA_SxCR_EN;		  // enable stream
}



void prepareDMA1Stream5(int dataNum)
{
	DMA1_Stream6->NDTR = dataNum;
	DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6; // half and transfer complete

	DMA1_Stream6->CR |= DMA_SxCR_EN;		  // enable stream
}


void initPinsUSART2()
{
	/** пины */
	uint32_t PA2_NUM = 2;
	uint32_t PA3_NUM = 3;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= (1 << (PA2_NUM * 2 + 1));
	GPIOA->MODER &= ~(1 << (PA2_NUM * 2));
	GPIOA->MODER |= (1 << (PA3_NUM * 2 + 1));
	GPIOA->MODER &= ~(1 << (PA3_NUM * 2));
	GPIOA->AFR[0] |= 0x07700; //0x700; //Alt 7
}


int main()
{
	/*
	 * Var 4 CPU = 130 V timer = 9
	 */

	/** Laboratory 2 */
	RCC->AHB1ENR |= 1 | 4;

	GPIOC->MODER |= (1 << (BLUE_LED_PIN*2));
	GPIOC->MODER |= (1 << (RED_LED_PIN*2));

	Coef_Nums coefs = Coef_Nums_Start_Init();
	coefs.F_cpu = 130;
	coefs.F_in = 24;
	coefs.HCKL_ = 130;
	coefs = init_coef(coefs);
	coefs.FLASH_ = 5;

	int res = InitClock(coefs);

	if (res != ALL_OK){
		updateLed(res);
	} else{
		SysTick_Config(SysTicks);
	}
	//////////////////////////////////////

	/** Laboratory 4 */
	/** Настройка таймера */
	initTimer(coefs);

	// включаем переферию
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	initPinsUSART2();

	/** настройка USART */
	USART2->BRR = APBx_FREQ / BAUDRATE;
	USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	USART2->CR3 = USART_CR3_DMAT;

	/** настройка DMA */ // DMA1 channel 4 USART2_RX - stream5 TX - stream6
	/**  Stream6 TX ->  память -> переферия DIR=01 */
	DMA1_Stream6->CR = DMA_SxCR_DIR_0 |  DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC;
	//DMA1_Stream6->CR |= DMA_SxCR_MSIZE_0; // 8bit

	DMA1_Stream6->PAR = (uint32_t) &USART2->DR;
	DMA1_Stream6->M0AR = (uint32_t) &ret_data;


	DMA1_Stream5->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_PINC;
	//DMA1_Stream5->CR |= DMA_SxCR_MSIZE_0; // 8bit

	DMA1_Stream5->PAR = (uint32_t) &USART2->DR;
	DMA1_Stream5->M0AR = (uint32_t) &tx_data;

	// DMA1_Stream5_IRQn

	for(;;){
		prepareDMA1Stream6(1);
		my_delay(500000);
		DMA1_Stream6->CR &= ~DMA_SxCR_EN;
		ret_data += 1;
	};

	/** error code on leds **/

	return 0;
}
