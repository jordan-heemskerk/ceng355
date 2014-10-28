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

void initADC();
void initDAC();
void configurePA0();
unsigned int pollADC();

int
main(int argc, char* argv[]) {
  // At this stage the system clock should have already been configured
  // at high speed.

	configurePA0();
	initADC();


  // Infinite loop
	while (1) {
	  trace_printf("ADC Value: %d\n", pollADC());
	   // Add your code here.
	}
}

void configurePA0() {
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA0 as analog */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~GPIO_MODER_MODER0;

	/* Ensure no pull-up/pull-down for PA0 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
}

void initDAC() {
RCC->APB1ENR |= RCC_APB1ENR_DACEN;




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
