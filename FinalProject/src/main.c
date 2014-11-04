//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

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

#define DAC_MAX 3000.0
#define DAC_MIN 1000.0
#define ADC_MAX 4095.0

void initADC();
void initDAC();
void initTIM2();
void initEXTI();
void TIM2_IRQHandler();
void EXTI0_1_IRQHandler();
void configurePA();
unsigned int pollADC();
void writeDAC(unsigned int);

//for determining frequency with EXTI and TIM2
int second_edge;

int main(int argc, char* argv[]) {
  // At this stage the system clock should have already been configured
  // at high speed.

	configurePA();
	initADC();
	initDAC();
	initTIM2();
	initEXTI();

	second_edge = 0;

  // Infinite loop
	while (1) {

		unsigned int adc = pollADC();
	//	trace_printf("ADC Value: %d\n", adc);
		float out = (((float)adc) * ((DAC_MAX-DAC_MIN)/ADC_MAX)) + DAC_MIN;
	//	trace_printf("DAC Value: %d\n\n", (int)out);
		writeDAC(out);


	   // Add your code here.
	}
}



void configurePA() {
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/****** PA0 *********/

	/* Configure PA0 as analog */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER |= GPIO_MODER_MODER0;

	/* Ensure no pull-up/pull-down for PA0 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	/****** PA1 **********/

	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);


	/****** PA4 *********/

	/* Configure PA4 as analog */
	GPIOA->MODER |= GPIO_MODER_MODER4;

	/* Ensure no pull-up/pull-down for PA4 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

}


void initEXTI()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] = (SYSCFG_EXTICR1_EXTI1_PA);

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
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
void EXTI0_1_IRQHandler()
{
	// Your local variables...

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{

		if (!second_edge) {
			second_edge = 1;

			TIM2->CNT = (uint32_t)0x0; //reset timer to 0
			TIM2->CR1 |= TIM_CR1_CEN; //enable TIM2
		} else {
			second_edge = 0;

			TIM2->CR1  |= TIM_CR1_UDIS; //disable TIM2
			int delta = TIM2->CNT;

			//calculate frequency here
			double period = ((double)(delta)/48000000.0);
			double frequency = 1.0/period;
			if (period < 0.00001) {
				trace_printf("period: %d nsec\n", (uint)(period*1000000000));
			} else if (period < 0.001) {
				trace_printf("period: %d usec\n", (uint)(period*1000000));
			} else {
				trace_printf("period: %d msec\n", (uint)(period*1000));
			}

			if (frequency < 100.0) {
				trace_printf("frequency: %d mHz\n\n",  (uint)(frequency*1000));
			} else if (frequency < 1000000) {
				trace_printf("frequency: %d Hz\n\n",  (uint)(frequency));
			} else {
				trace_printf("frequency: %d KHz\n\n",  (uint)(frequency/1000));
			}




			//print values

		}

		EXTI->PR |= EXTI_PR_PR1;
		//
		// 1. If this is the first edge:
		//	- Clear count register (TIM2->CNT).
		//	- Start timer (TIM2->CR1).
		//    Else (this is the second edge):
		//	- Stop timer (TIM2->CR1).
		//	- Read out count register (TIM2->CNT).
		//	- Calculate signal period and frequency.
		//	- Print calculated values to the console.
		//	  NOTE: Function trace_printf does not work
		//	  with floating-point numbers: you must use
		//	  "unsigned int" type to print your signal
		//	  period and frequency.
		//
		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		//
	}
}

void initTIM2()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = (uint16_t)0x0001;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}

void initDAC() {
	//enable DAC on APB
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	//enable DAC
	DAC->CR |= DAC_CR_EN1;


}

void writeDAC(unsigned int dac_value) {
	DAC->DHR12R1 = dac_value;
}

void initADC() {

	//enable ADC on APB
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	//set ADC clock source to asynchrous
	//ADC1->CFGR2 &= ~(0x3);

	//Cal ADC
	ADC1->CR = ADC_CR_ADCAL;
	while (ADC1->CR == ADC_CR_ADCAL);
	trace_printf("ADC is calibrated\n");


	ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);

	//Select Channel 0 for ADC
	ADC1->CHSELR = ADC_CHSELR_CHSEL0;


	//Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & 0x1) != 0x1);
	trace_printf("ADC is enabled\n");


}

unsigned int pollADC() {

	ADC1->CR |= ADC_CR_ADSTART;
	while ((ADC1->ISR & ADC_ISR_EOC) == 0);
	ADC1->ISR &= ~(ADC_ISR_EOC);
	unsigned int val = ADC1->DR;
	return val;

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
