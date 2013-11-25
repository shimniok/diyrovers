/** Demonstration of LED Blinky functionality on LPCXpresso */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#include "system_LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <NXP/crp.h>

volatile uint32_t msTicks;                       /* timeTicks counter */

__INLINE static void delay_ms(uint32_t dlyTicks) {
  uint32_t curTicks = msTicks;

  while ((msTicks - curTicks) < dlyTicks);
}

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

int main(void) {
	SystemInit();
	SystemClockUpdate();

	// Set P0_22 to 00 - GPIO
    LPC_PINCON->PINSEL1 &= (~(3 << 12));
    // Set GPIO - P0_22 - to be output
    LPC_GPIO0->FIODIR |= (1 << 22);

    if (SysTick_Config(SystemFrequency/1000)) { /* Setup SysTick for 1 msec interrupts */
      ;                                         /* Handle Error */
      while (1);
    }


    while (1) {
        LPC_GPIO0->FIOSET = (1 << 22); // Turn LED2 on
        delay_ms(300);
        LPC_GPIO0->FIOCLR = (1 << 22); // Turn LED2 off
        delay_ms(300);
    }
    return 0;
}


void SysTick_Handler(void) {
  msTicks++;                                     /* increment timeTicks counter */
}
