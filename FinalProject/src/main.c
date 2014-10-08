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
void configurePA1();

int
main(int argc, char* argv[])
{
  // At this stage the system clock should have already been configured
  // at high speed.

  configurePA1();
  initADC();

  // Infinite loop
  while (1)
    {
       // Add your code here.
    }
}

void configurePA1() {
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA1 as analog */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER |= GPIO_MODER_MODER1;

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}

void initADC() {

	//enable ADC on APB
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	//set ADC clock source to asynchrous
	ADC1->CFGR2 &= ~(0x3);

	//Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	//ADC1->CR |= ADC_CR_ADCAL;

	//Wait until ADRDY=1 so that ADC is ready
	while(1) {
		trace_printf("%d\n", ADC1->DR) ;
	}

	trace_printf("ADC is enabled\n");
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
