#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <NXP/crp.h>

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

int main(void) {
	// Set P0_22 to 00 - GPIO
    LPC_PINCON->PINSEL1 &= (~(3 << 12));
    // Set GPIO - P0_22 - to be output
    LPC_GPIO0->FIODIR |= (1 << 22);

    while (1) {
        int i;

        LPC_GPIO0->FIOSET = (1 << 22); // Turn LED2 on
        for (i = 0; i < 1000000; i++);
        LPC_GPIO0->FIOCLR = (1 << 22); // Turn LED2 off
        for (i = 0; i < 1000000; i++);
    }
    return 0;
}

