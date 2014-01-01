/*
 * faulthandlers.cpp
 *
 * Description: Implement fault handlers to aid in debugging issues.
 *  Created on: Dec 29, 2013
 *      Author: mes
 */

#include "mbed.h"


extern "C" void HardFault_Handler(void) {
	error("\n\n%% Hard Fault %%\n");
}

extern "C" void UsageFault_Handler(void) {
	error("\n\n%% Usage Fault %%\n");
}

extern "C" void BusFault_Handler() {
	error("\n\n%% Bus Fault %%\n");
}

extern "C" void MemMang_Handler() {
	error("\n\n%% MemMang Fault %%\n");
}
