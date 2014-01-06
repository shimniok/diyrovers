/*
 * faulthandlers.cpp
 *
 * Description: Implement fault handlers to aid in debugging issues.
 *  Created on: Dec 29, 2013
 *      Author: mes
 */

#include "mbed.h"

extern "C" {

// PSR flags

// EPSR flags
#define TFLG (1<<24)

// APSR flags
#define NFLG (1<<31)
#define ZFLG (1<<30)
#define CFLG (1<<29)
#define VFLG (1<<28)
#define QFLG (1<<27)

// IPSR flags
#define ISR_THREADMODE 	0
#define ISR_NMI 		2
#define ISR_HARDFAULT 	3
#define ISR_MEMMANAGE 	4
#define ISR_BUSFAULT 	5
#define ISR_USAGEFAULT 	6
#define ISR_SVCALL 		11
#define ISR_PENDSV 		14
#define ISR_SYSTICK 	15
#define ISR_IRQ0 		16

// HFSR flags
#define VECTTBL 	(1<<1)
#define FORCED 		(1<<30)
#define DEBUGEVT 	(1<<31)

// CFSR flags

// BFSR flags
#define IBUSERR		(1<<0)
#define PRECISERR	(1<<1)
#define IMPRECISERR	(1<<2)
#define UNSTKERR 	(1<<3)
#define STKERR		(1<<4)
#define BFARVALID	(1<<7)

// UFSR flags
#define UNDEFINSTR 	(1<<0)
#define INVSTATE 	(1<<1)
#define INVPC 		(1<<2)
#define NOCP 		(1<<3)
#define UNALIGNED 	(1<<8)
#define DIVBYZERO 	(1<<9)

// MMFSR flags
#define IACCVIOL	(1<<0)
#define DACCVIOL	(1<<1)
#define MUNSTKERR	(1<<3)
#define MSTKERR		(1<<4)
#define MMARVALID	(1<<7)

// DFSR
#define EXTERNAL	(1<<4)
#define VCATCH		(1<<3)
#define DWTTRAP		(1<<2)
#define BKPT		(1<<1)
#define HALTED		(1<<0)

/** Hard Fault Handler code comes from these spots:
 *
 * http://blog.frankvh.com/2011/12/07/cortex-m3-m4-hard-fault-handler/
 * http://blog.feabhas.com/2013/02/developing-a-generic-hard-fault-handler-for-arm-cortex-m3cortex-m4/
 */

/**
 * This is the actual handler, which sets up the data to be used by the C function, then calls it.
 */
void __attribute__((naked)) HardFault_Handler(void)
{
	__asm__(
			".thumb									\n"
    		"   tst     lr, #4                      \n" // for priv/non-priv, test for msp or psp in return (thread or handler mode)
    		"   ite     eq                          \n"
    		"   mrseq   r0, MSP                     \n" // move main stack pointer into r0
    		"   mrsne   r0, PSP                     \n" // move process stack pointer into r0
			"	b		hard_fault_handler			\n" // jump to the c function
			:
			:
			:
			);
}

/**
 * Here we print out the junk in the stack and some special registers to help with debugging.
 */
void hard_fault_handler(unsigned int *hardfault_args)
{
	  unsigned int stacked_r0;
	  unsigned int stacked_r1;
	  unsigned int stacked_r2;
	  unsigned int stacked_r3;
	  unsigned int stacked_r12;
	  unsigned int stacked_lr;
	  unsigned int stacked_pc;
	  unsigned int stacked_psr;
	  unsigned int cfsr, bfsr, ufsr, mmfsr;
	  //unsigned int control;

	  stacked_r0 = ((unsigned long) hardfault_args[0]);
	  stacked_r1 = ((unsigned long) hardfault_args[1]);
	  stacked_r2 = ((unsigned long) hardfault_args[2]);
	  stacked_r3 = ((unsigned long) hardfault_args[3]);

	  stacked_r12 = ((unsigned long) hardfault_args[4]);
	  stacked_lr = ((unsigned long) hardfault_args[5]);
	  stacked_pc = ((unsigned long) hardfault_args[6]);
	  stacked_psr = ((unsigned long) hardfault_args[7]);
	  //control = __get_CONTROL();

	  printf("\n\n[Hard fault]\n");
	  printf("R0 = 0x%08x\n", stacked_r0);
	  printf("R1 = 0x%08x\n", stacked_r1);
	  printf("R2 = 0x%08x\n", stacked_r2);
	  printf("R3 = 0x%08x\n", stacked_r3);
	  printf("R12 = 0x%08x\n", stacked_r12);
	  printf("LR [R14] = 0x%08x  subroutine call return address.\n", stacked_lr);
	  printf("PC [R15] = 0x%08x  program counter\n", stacked_pc);

	  // PSR
	  printf("PSR = 0x%04x ", stacked_psr);
	  if (stacked_psr & NFLG) printf("N");
	  if (stacked_psr & ZFLG) printf("Z");
	  if (stacked_psr & CFLG) printf("C");
	  if (stacked_psr & VFLG) printf("V");
	  if (stacked_psr & QFLG) printf("Q");
	  printf(" ");
	  unsigned int isrnum = (stacked_psr & 0xff);
	  switch (isrnum) {
	  case ISR_THREADMODE:
		  printf("Thread mode ");
		  break;
	  case ISR_NMI:
		  printf("NMI ");
		  break;
	  case ISR_HARDFAULT:
		  printf("HardFault ");
		  break;
	  case ISR_MEMMANAGE:
		  printf("MemManage ");
		  break;
	  case ISR_BUSFAULT:
		  printf("BusFault ");
		  break;
	  case ISR_USAGEFAULT:
		  printf("UsageFault ");
		  break;
	  case ISR_SVCALL:
		  printf("SVCall ");
		  break;
	  case ISR_PENDSV:
		  printf("PendSV ");
		  break;
	  case ISR_SYSTICK:
		  printf("SysTick ");
		  break;
	  case ISR_IRQ0:
		  printf("IRQ0 ");
		  break;
	  }
	  printf(" ");
	  if (stacked_psr & TFLG)
		  printf("thumb");
	  else
		  printf("non-thumb");
	  printf("\n");

	  // CONTROL (not sure this works...)
	  //printf("CONTROL = 0x%04x ", control);
	  //printf("\n");

	  // HFSR
	  printf("HFSR = 0x%08lx ", SCB->HFSR);
	  if (SCB->HFSR & DEBUGEVT) printf("DEBUGEVT ");
	  if (SCB->HFSR & FORCED) printf("FORCED ");
	  if (SCB->HFSR & VECTTBL) printf("VECTTBL ");
	  printf("\n");
	  // CFSR
	  printf("CFSR = 0x%08lx\n", SCB->CFSR);
	  cfsr = SCB->CFSR;
	  ufsr = (cfsr>>16);
	  printf("UFSR = 0x%04x ", ufsr);
	  if (ufsr & DIVBYZERO) printf("Divide by zero UsageFault ");
	  if (ufsr & UNALIGNED) printf("Unaligned access UsageFault ");
	  if (ufsr & NOCP) printf("No coprocessor UsageFault ");
	  if (ufsr & INVPC) printf("Invalid PC load UsageFault ");
	  if (ufsr & INVSTATE) printf("Invalid state UsageFault ");
	  if (ufsr & UNDEFINSTR) printf("Undefined instruction UsageFault ");
	  printf("\n");

	  // BFSR
	  bfsr = ((cfsr >> 8) & 0xff);
	  printf("BFSR = 0x%02x ", bfsr);
	  if ((bfsr & IBUSERR) != 0) printf("IBUSERR ");
	  if ((bfsr & PRECISERR) != 0) printf("PRECISERR ");
	  if ((bfsr & IMPRECISERR) != 0) printf("IMPRECISERR ");
	  if (bfsr & UNSTKERR) printf("UNSTKERR ");
	  if (bfsr & STKERR) printf("STKERR ");
	  if (bfsr & BFARVALID) printf("BFARVALID ");
	  printf("\n");
	  // BFAR
	  //The value of SCB->BFAR indicates the memory address that caused a Bus Fault and is valid if the bit BFARVALID in the
	  //SCB->CFSR register is set.
	  printf("BFAR = ");
	  if (bfsr & BFARVALID) {
		  printf("0x%08lx\n", SCB->BFAR);
	  } else {
		  printf("invalid\n");
	  }

	  // MMFSR
	  mmfsr = (cfsr & 0xff);
	  printf("MMFSR = 0x%02x ", mmfsr);
	  if (mmfsr & IACCVIOL) printf("IACCVIOL ");
	  if (mmfsr & DACCVIOL) printf("DACCVIOL ");
	  if (mmfsr & MUNSTKERR) printf("MUNSTKERR ");
	  if (mmfsr & MSTKERR) printf("MSTKERR ");
	  if (mmfsr & MMARVALID) printf("MMARVALID ");
	  printf("\n");
	  // MMFAR
	  // The value of SCB->MMFAR indicates the memory address that caused a Memory Management Fault and is valid if the bit
	  // MMARVALID in the SCB->CFSR register is set.
	  printf("MMFAR = ");
	  if (mmfsr & MMARVALID) {
		  printf("0x%08lx ", SCB->MMFAR);
	  } else {
		  printf("invalid\n");
	  }

	  // DFSR
	  printf("DFSR = 0x%08lx ", SCB->DFSR);
	  if (SCB->DFSR & EXTERNAL) printf("EXTERNAL ");
	  if (SCB->DFSR & VCATCH) printf("VCATCH ");
	  if (SCB->DFSR & DWTTRAP) printf("DWTTRAP ");
	  if (SCB->DFSR & BKPT) printf("BKPT ");
	  if (SCB->DFSR & HALTED) printf("HALTED ");
	  printf("\n");

	  printf("AFSR = 0x%08lx\n", SCB->AFSR);
	  printf("SHCSR = 0x%08lx\n", SCB->SHCSR);

	  while (1);
}

/*
void HardFault_Handler(void) {

	while(1);
	error("\n\n%% Hard Fault %%\n");
}
*/

void UsageFault_Handler(void) {
	error("\n\n%% Usage Fault %%\n");
}

void BusFault_Handler() {
	error("\n\n%% Bus Fault %%\n");
}

void MemMang_Handler() {
	error("\n\n%% MemMang Fault %%\n");
}

}
