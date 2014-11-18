//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_spi.h"


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
#define RES_CHANGE 5

void configurePA();
void configurePB();
void configurePD();
void configureSPI1();
void configureLCD();

void initADC();
void initDAC();
void initTIM2();
void initEXTI();

void TIM2_IRQHandler();
void EXTI0_1_IRQHandler();
unsigned int pollADC();

void writeLCD();
void writeCmd(int);
void changeAddr(int);
void writeChar(int);
void sendCommand(uint8_t);
void sendData(uint8_t);
void writeData(uint8_t);
void writeDAC(unsigned int);

//for determining frequency with EXTI and TIM2
int second_edge;

int f, r, last_r, first;

/*
 * Configures IO, initializes registers, and runs main loop.
 */
int main(int argc, char* argv[]) {
    // At this stage the system clock should have already been configured
    // at high speed.

	configurePA();
	configurePB();
	configurePD();
	configureSPI1();
	configureLCD();

	initADC();
	initDAC();
	initTIM2();

	initEXTI();

	second_edge = 0;
	last_r = 0;
	first = 1;

    // Infinite loop
	while (1) {

		unsigned int adc = pollADC();
		//trace_printf("ADC Value: %d\n", adc);

		/* Calculate resistance */
		r = (int)(adc * (5000.0/4095.0));

		float out = (((float)adc) * ((DAC_MAX-DAC_MIN)/ADC_MAX)) + DAC_MIN;
		//trace_printf("DAC Value: %d\n\n", (int)out);
		writeDAC(out);
	}
}

/*
 * Configure the PA registers.
 */
void configurePA() {
	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/****** PA0 *********/

	/* Configure PA0 as analog */
	GPIOA->MODER |= GPIO_MODER_MODER0;
	/* Ensure no pull-up/pull-down for PA0 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	/****** PA1 **********/

	/* Configure PA1 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	/* Ensure no pull-up/pull-down for PA1 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);


	/****** PA4 *********/

	/* Configure PA4 as analog */
	GPIOA->MODER |= GPIO_MODER_MODER4;
	/* Ensure no pull-up/pull-down for PA4 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

}

/*
 * Configure the PB registers.
 */
void configurePB() {
	/* Enable clock for GPIOB peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Configure PB3 as alternate function */
	GPIOB->MODER |= GPIO_MODER_MODER3_1;

	/* No pull-up/down for PB3 */
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);

	/* Configure PB5 as alternate function */
	GPIOB->MODER |= GPIO_MODER_MODER5_1;

	/* No pull-up/down for PB5 */
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

}

/*
 * Configure the PD registers.
 */
void configurePD() {
	/* Enable clock for GPIOD peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIODEN;

	/* Configure PD2 as output */
	GPIOD->MODER |= GPIO_MODER_MODER2_0;

	/* No pull-up/down for PD2 */
	GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
}

/*
 * Configure the SPI registers.
 */
void configureSPI1() {
	/* Enable SPI1 on APB Bus */
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	/* Use CMSIS function calls here to init */
	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256 ;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
}

/*
 * Set up LCD for desired display settings.
 */
void configureLCD() {
	/* Reconfigure and setup in 4 bit mode */
	writeCmd(0x33); //special function set command
	writeCmd(0x32); //special function set command and 4 bit mode

	// LCD options

	/* DDRAM using 4-bit interface; two lines of eight characters */
	writeCmd(0x28); //DL= 0 N=1 F=0
	/* Display on, cursor off and not blinking */
	writeCmd(0x0C); //D = 1, C=0 B =0
	/* Auto-increment DDRAM address; don't shift display */
	writeCmd(0x06); // I/D=1 S=0

	writeCmd(0x1); //clear display
}

/*
 * Initializes ADC for converting analog input to digital.
 */
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

/*
 * Initializes DAC for converting digital input to analog.
 */
void initDAC() {
	//enable DAC on APB
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	//enable DAC
	DAC->CR |= DAC_CR_EN1;
}

/*
 * Initialize incrementing timer for determining frequency period.
 */
void initTIM2()
{
	/* Enable clock for TIM2 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	TIM2->EGR = (uint16_t)0x0001;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;
}

/*
 * Initialize EXTI for interrupt on rising-edge.
 */
void initEXTI()
{
	/* Map EXTI1 line to PA1 */
	SYSCFG->EXTICR[0] = (SYSCFG_EXTICR1_EXTI1_PA);

	/* EXTI1 line interrupts: set rising-edge trigger */
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/* 
 * This handler is declared in system/src/cmsis/vectors_stm32f0xx.c 
 *
 * Resets timer if it over flows.
 */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}


/* 
 * This handler is declared in system/src/cmsis/vectors_stm32f0xx.c 
 *
 * Calculates frequency when rising edge event occurs. Writes to LCD.
 */
void EXTI0_1_IRQHandler()
{
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

			f = (int)(frequency*10);
			writeLCD();
		}

		EXTI->PR |= EXTI_PR_PR1; //clear EXTI interrupt pending flag
	}
}

/*
 * Check ADC for converted analog information.
 */
unsigned int pollADC() {

	ADC1->CR |= ADC_CR_ADSTART; //start conversion

	while ((ADC1->ISR & ADC_ISR_EOC) == 0); //wait for conversion

	ADC1->ISR &= ~(ADC_ISR_EOC); //clear done conversion flag

	unsigned int val = ADC1->DR; //get value

	return val;

}

/*
 * Displays current frequency and resistance on LCD.
 */
void writeLCD() {

	int frequency = f;
	changeAddr(0x0);
	writeChar('F');
	writeChar(':');
	writeChar((frequency/100) + 48);
	writeChar((frequency%100)/10 + 48);
	writeChar('.');
	writeChar((frequency%10) + 48);
	writeChar('H');
	writeChar('z');

	/* Resistance is very touchy, so only display changes over 5 Ohms */
	int resistance;
	int r_diff = r - last_r;
	if (first) {
		last_r = r;
		first = 0;
	}

	if (r_diff < RES_CHANGE && r_diff > -RES_CHANGE) {
		resistance = last_r;
	} else {
		resistance = r;
		last_r = resistance;
	}

	changeAddr(0x40);
	writeChar('R');
	writeChar(':');
	writeChar((resistance/1000) + 48);
	writeChar((resistance%1000)/100 + 48);
	writeChar((resistance%100)/10 + 48);
	writeChar((resistance%10) + 48);
	writeChar('O');
	writeChar('h');
}

/*
 * Sends data in the format of a command to LCD. 
 */
void writeCmd(int c) {
	int high = (c & 0xF0) >> 4;
	int low = c & 0x0F;
	sendCommand(high);
	sendCommand(low);
}

/*
 * Sends data in the format of an address to LCD.
 */
void changeAddr(int p) {
	int high = (p & 0xF0) >> 4;
	high |= 0x8; // Set address bit
	int low = p & 0x0F;
	sendCommand(high);
	sendCommand(low);
}

/*
 * Sends data in ASCII character format to LCD.
 */
void writeChar(int c) {
	int high = (c & 0xF0) >> 4;
	int low = c & 0x0F;
	sendData(high);
	sendData(low);
}

/*
 * Writes data to LCD in command format (RS = 0).
 */
void sendCommand(uint8_t d) {
	writeData(d);
	writeData(d | 0x80);
	writeData(d);
}

/*
 * Writes data to LCD in ASCII format (RS = 1).
 */
void sendData(uint8_t d) {
	writeData(d | 0x40);
	writeData(d | 0x80 | 0x40);
	writeData(d | 0x40);
}

/*
 * Writes data to LCD when SPI is ready.
 */
void writeData(uint8_t data) {

	/* reset LCK signal to 0 by forcing PD2 to 0 */
	GPIOD->BSRR = GPIO_BSRR_BR_2;

	/* Wait until SPI1 is ready */
	while (SPI1->SR & SPI_SR_BSY);

	SPI_SendData8(SPI1, data);

	/* Wait until SPI1 is not busy */
	while (SPI1->SR & SPI_SR_BSY);

	/* Force LCK signal to 1 by forcing PD2 to 1 */
	GPIOD->BSRR = GPIO_BSRR_BS_2;
}

/*
 * Writes data to DAC.
 */
void writeDAC(unsigned int dac_value) {
	DAC->DHR12R1 = dac_value;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
