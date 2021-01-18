
// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This code was used for ECE 355 final lab project of measuring the frequency of signal 
// using timers. The signal was measured off a potentiometer to a ADC than DAC which
// was connected to a optical coupler that feed into a 555 timer to generate a PMW signal.
// This signal was than measued using timer in the onboard processer.
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
// ----------------------------------------------------------------------------

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

#define SET_E() (GPIOB->BSRR = GPIO_BSRR_BS_4)
#define SET_RS() (GPIOB->BSRR = GPIO_BSRR_BS_5)
#define SET_RW() (GPIOB->BSRR = GPIO_BSRR_BS_6)

#define CLEAR_E() (GPIOB->BSRR = GPIO_BSRR_BR_4)
#define CLEAR_RS() (GPIOB->BSRR = GPIO_BSRR_BR_5)
#define CLEAR_RW() (GPIOB->BSRR = GPIO_BSRR_BR_6)

/* Base address for LCD line 1*/
#define LINE_1 0x80
/* Base address for LCD line 2*/
#define LINE_2 0xC0

/**
 * @brief Configure GPIO for Port A DAC & 555 timer:
 *  PA1 -> 555timer input
 *  PA4 -> DAC analog output
 */
void myGPIOA_Init(void);
/**
 * @brief Configure GPIO for Port B LCD:
 * 	PB4 	-> ENB
 *	PB5 	-> RS
 *	PB6 	-> R/W
 *	PB7 	-> DONE
 *	PB8-15 - > DATA
 */
void myGPIOB_Init(void);
/**
 * @brief Configure GPIO for Port C ADC:
 *  PC1 -> ADC
 *
 */
void myGPIOC_Init(void);
/**
 * @brief configure TIM 2 for frequency measurement
 *
 */
void myTIM2_Init(void);
/**
 * @brief Configure external interrupts for TIM2
 *
 */
void myEXTI_Init(void);
/**
 * @brief Configure ADC control and system configuration registers
 *
 */
void myADC_Init(void);
/**
 * @brief Configure DAC control control register
 *
 */
void myDAC_Init(void);
/**
 * @brief Configure and initialize LCD screen
 *
 */
void myLCD_Init(void);
/**
 * @brief LCD helper method for sending cmd bytes to LCD.
 *
 */
void LCDcmd(unsigned char c);
/**
 * @brief LCD helper method for sending data bytes to LCD.
 *
 */
void LCDdata(unsigned char  d);
/**
 * @brief LCD helper method for clearing LCD screen
 *
 */
void clearLCD(void);
/**
 * @brief Helper method for parsing string into single digits to send to LCD screen
 *
 */
void digits(unsigned int num);
/**
 * @brief Helper method for updating LCD screen 
 *
 */
void updateLCD();

/*-----------------------GLOBAL VARIBLES-------------------------*/
volatile int timerTriggered = 0;
volatile unsigned int frequency;
volatile unsigned int ADC_result = 0;
volatile unsigned int pot_value;

/**
 * LCD initalization commands
 * 0x38 - initilizes LCD as 2x16 display
 * 0x0E - makes display on & curser blink
 * 0x06 - Shift curser right
 * 0x01 - clear the curser
 * 0x80 - initialize curser position as line1,position1
 */
unsigned char cmd1[5] = {0x38,0x0C,0x06,0x01,0x80};	// LCD commands
unsigned char msg1[2] = {0x46,0x3A};			// commands to be displayed
unsigned int msg2[2] = {0x52,0x3A};			// commands to be displayed
unsigned int temp[4];

int main(int argc, char* argv[]){

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		// Initialize I/O port PB LCD
	myGPIOC_Init();		/* Initialize I/O port PC analog Input */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myADC_Init();		// Initialize ADC
	myDAC_Init();		// Initialize DAC
	myLCD_Init();		// Initialize LCD

	while (1)
	{
		while((ADC1->ISR & ADC_ISR_EOC) == 0);
		pot_value = (ADC1->DR*5000)/4095;
		DAC->DHR12R1 = ADC1->DR;
	}

return 0;

}
/*<-----------------------------------HELPER FUNCTIONS------------------------->  */
void updateLCD(){
		clearLCD();
		LCDcmd(LINE_1);
		unsigned int i;
		for(i = 0; i < 2 ; i++){
			LCDdata(msg1[i]);
		}
		digits(frequency);
		for(i = 0; i < 4 ;i++){
			LCDdata(temp[i]+0x30);
		}
		LCDdata(0x48);
		LCDdata(0x7A);

		LCDcmd(LINE_2);

		for(i = 0; i < 2 ; i++){
			LCDdata(msg2[i]);
		}
		digits(pot_value);
		for(i = 0;i<4;i++){
			LCDdata(temp[i]+0x30);
			}
		LCDdata(0x4F);
		LCDdata(0x68);
		LCDdata(0x6D);
}
void digits(unsigned int num){
		temp[0] = (num/1000);
		temp[1] = (num-(temp[0]*1000))/100;
		temp[2] = (num-(temp[0]*1000+temp[1]*100))/10;
		temp[3] = (num-(temp[0]*1000+temp[1]*100+temp[2]*10));
}

void LCDcmd(unsigned char c){
	GPIOB->ODR = (c << 8);
    CLEAR_RS();
	CLEAR_RW();
	SET_E();
	while((GPIOB->IDR & GPIO_IDR_7)!=GPIO_IDR_7); //wait for pin7 == 1
	CLEAR_E();
	while((GPIOB->IDR & GPIO_IDR_7)!=0); //wait for pin7 == 0
}

void LCDdata(unsigned char d){
	GPIOB->ODR = (d << 8);
    SET_RS();
	CLEAR_RW();
	SET_E();
	while((GPIOB->IDR & GPIO_IDR_7)!=GPIO_IDR_7); //wait for pin7 == 1
	CLEAR_E();
	while((GPIOB->IDR & GPIO_IDR_7)!=0); //wait for pin7 == 0
}

void clearLCD(){
	GPIOB->BSRR = GPIO_BSRR_BS_8; // clear display
	CLEAR_RS();
	CLEAR_RW();
	SET_E();
	while((GPIOB->IDR & GPIO_IDR_7)!=GPIO_IDR_7); //wait for pin7 == 1
	CLEAR_E();
	while((GPIOB->IDR & GPIO_IDR_7)!=0); //wait for pin7 == 0
	GPIOB->BSRR = GPIO_BSRR_BR_8; // DL
}
/*<---------------------------------INITALIZATION FUNCTIONS------------------------->  */
void myADC_Init(){
	RCC -> APB2ENR |= RCC_APB2ENR_ADCEN; // enable ADC clock pin 9
	ADC1->CFGR1 |= (ADC_CFGR1_CONT|ADC_CFGR1_OVRMOD); // Enable continuous conversion mode, 12-bit res, align right, and overrun management
	ADC1->CHSELR = ADC_CHSELR_CHSEL11;// select ADC channel 11-> PC1
	ADC1->SMPR |= ADC_SMPR_SMP; // ADC clock cycles 239.5
	ADC1->ISR |= ADC_ISR_ADRDY;// clear ADC rdy flag
	ADC1->CR |= ADC_CR_ADEN; // Enable ADC in ADC control register
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0); // wait for ADC set up time
	ADC1->CR|= ADC_CR_ADSTART; // Start ADC

}

void myDAC_Init(){
	RCC -> APB1ENR |= RCC_APB1ENR_DACEN; // enable DAC clock pin 29
	DAC->CR |= DAC_CR_EN1; // enable DAC in DAC control register
}

void myLCD_Init(){
	unsigned i;
	for(i = 0; i < 5 ; i++){
		LCDcmd(cmd1[i]);
	}
}


void myGPIOA_Init(){// PA1 -> 555timer input PA4 -> DAC out to optocoupler
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	/* Configure PA4 as analog output*/
	GPIOA->MODER |= (GPIO_MODER_MODER4);
	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
	/* Ensure no pull-up/pull-down for PA4 and PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
}
void myGPIOB_Init(){
	/* Enable clock for GPIOB peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	/* Configure PB4,5,6,8,9,10,11,12,13,14,15 as output */
	// Relevant register: GPIOB->MODER
	GPIOB->MODER |= (GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_0|GPIO_MODER_MODER6_0|GPIO_MODER_MODER8_0);
	GPIOB->MODER |= (GPIO_MODER_MODER9_0|GPIO_MODER_MODER10_0|GPIO_MODER_MODER11_0|GPIO_MODER_MODER12_0);
	GPIOB->MODER |= (GPIO_MODER_MODER13_0|GPIO_MODER_MODER14_0|GPIO_MODER_MODER15_0);
	GPIOB->MODER &= ~(GPIO_MODER_MODER7);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
}
void myGPIOC_Init(){// Set port C for ADC PC1 analog input
	/* Enable clock for GPIOC peripheral */
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	/* Configure PC1 as analog input */
	// Relevant register: GPIOC->MODER
	GPIOC->MODER |= (GPIO_MODER_MODER1);
	/* Ensure no pull-up/pull-down for PC1 */
	// Relevant register: GPIOA->PUPDR
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}

void myTIM2_Init(){
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 |= ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR |= ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn,0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}


void myEXTI_Init(){
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	//SYSCFG->EXTICR[0] = (SYSCFG_EXTICR1_EXTI1_PA);
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;
	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;  ;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn,0);
	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/*<-------------------------INTERRUPT HANDLERS FUNCTIONS------------------------->  */
/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler(){
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);
		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}
/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler(){
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		if(timerTriggered == 0 ){			// 1. If this is the first edge:
			TIM2->CNT = 0; 					//	- Clear count register (TIM2->CNT).
			TIM2->CR1 |= TIM_CR1_CEN; 		//	- Start timer (TIM2->CR1).
			timerTriggered = 1;
		}else{
			TIM2->CR1 &= ~(TIM_CR1_CEN);  	//	- Stop timer (TIM2->CR1).
			timerTriggered = 0;
			frequency = SystemCoreClock/TIM2->CNT;
			updateLCD();
		}
	EXTI->PR |= EXTI_PR_PR1; 		//	Clear EXTI2 interrupt pending flag (EXTI->PR).

	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
