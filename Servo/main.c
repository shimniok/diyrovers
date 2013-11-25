/*
 * main.c
 *
 * Demonstrate servo (PWM) functionality on LPCXpresso
 *
 *  Created on: Nov 24, 2013
 *      Author: mes
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#include "system_LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <NXP/crp.h>

#define PCPWM1 (1 << 6)
#define PCLK_PWM1_BY8 ((1 << 13)|(1 << 12))
#define PCLK_PWM1_BY4 ~(PCLK_PWM1_BY8)
#define PCLK_PWM1 ((0 << 13)|(1 << 12))
#define PCLK_PWM1_BY2 ((1 << 13)|(0 << 12))
#define LER0_EN 1 << 0
#define LER1_EN 1 << 1
#define LER2_EN 1 << 2
#define LER3_EN 1 << 3
#define LER4_EN 1 << 4
#define LER5_EN 1 << 5
#define LER6_EN 1 << 6
#define PWMENA1 1 << 9
#define PWMENA2 1 << 10
#define PWMENA3 1 << 11
#define PWMENA4 1 << 12
#define PWMENA5 1 << 13
#define PWMENA6 1 << 14
#define TCR_CNT_EN 0x00000001
#define TCR_RESET 0x00000002
#define TCR_PWM_EN 0x00000008

#define INITPWMVAL 0
#define ENDPWMVAL 65500
#define STEP 400

volatile uint32_t msTicks;                       /* timeTicks counter */

__INLINE static void delay_ms(uint32_t dlyTicks) {
	uint32_t future = msTicks + dlyTicks;

	while (msTicks != future);
}

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

void InitPWM();
void SetPWM(int pwmval);

int main(void) {

	SystemClockUpdate();
	InitPWM();

    if (SysTick_Config(SystemFrequency/1000)) { /* Setup SysTick for 1 msec interrupts */
      ;                                         /* Handle Error */
      while (1);
    }

	// Set P0_22 to 00 - GPIO
    LPC_PINCON->PINSEL1 &= (~(3 << 12));
    // Set GPIO - P0_22 - to be output
    LPC_GPIO0->FIODIR |= (1 << 22);

    volatile static int32_t i;
    while (1) {
        LPC_GPIO0->FIOSET = (1 << 22); // Turn LED2 on
    	for(i = INITPWMVAL; i <= ENDPWMVAL; i += STEP) {
    		SetPWM(i);
    		delay_ms(10);
    	}

    	LPC_GPIO0->FIOCLR = (1 << 22); // Turn LED2 off
        for(i = ENDPWMVAL; i > INITPWMVAL; i -= STEP) {
    		SetPWM(i);
    		delay_ms(10);
    	}
    }
    return 0;
}


void InitPWM() {
	//enable PWM1 Power
	LPC_SC ->PCON |= PCPWM1;

	//PWM peripheral clk = PCLK
	LPC_SC ->PCLKSEL0 &= (PCLK_PWM1_BY4);

	//Put P0.26 in Hi-Z so it can be shorted to PWM1.2 (P2.2)
	// LPC_PINCON->PINMODE1 &= ~(0x3<<20);
	// LPC_PINCON->PINMODE1 |= 0x2<<20;
	// LPC_GPIO0->FIODIR &= ~(1<<26);

	//Pin select
	LPC_PINCON->PINSEL4 = (1<<0) | (1<<2) | (1<<4);

	// count frequency:Fpclk
	LPC_PWM1->PR = 0x00;

	//reset on MR0
	LPC_PWM1->MCR = 1 << 1;

	// set PWM cycle
	LPC_PWM1->MR0 = ENDPWMVAL;
	//LEDs default to OFF
	LPC_PWM1->MR1 = 0;

	//Load Shadow register content
	LPC_PWM1->LER = LER0_EN | LER1_EN | LER2_EN| LER3_EN;

	//Enable PWM outputs
	LPC_PWM1->PCR = PWMENA1 | PWMENA2 | PWMENA3 | PWMENA4;

	//Enable PWM Timer
	LPC_PWM1->TCR = TCR_CNT_EN | TCR_PWM_EN;
}


void SetPWM(int pwmval)
{
	LPC_PWM1->MR1 = pwmval;
	LPC_PWM1->MR2 = pwmval;
	LPC_PWM1->MR3 = pwmval;

	LPC_PWM1->LER = LER0_EN | LER1_EN | LER2_EN| LER3_EN;
}


void SysTick_Handler(void) {
	msTicks++;                                     /* increment timeTicks counter */
}


