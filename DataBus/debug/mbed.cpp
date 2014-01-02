/*
 * io.cpp
 *
 * Description: Serial I/O primitives for mbed
 *
 *  Created on: Dec 30, 2013
 *      Author: mes
 */
#include "mbed.h"

Serial dbg(p27, p28);

extern "C" void gdb_serial_interrupt(int data);

extern "C" {

void dbg_irq() {
	while (dbg.readable()) {
		gdb_serial_interrupt(dbg.getc());
	}
}

int gdb_serial_read()
{
	return dbg.getc();
}

void gdb_serial_write(int c)
{
	dbg.putc(c);
	return;
}

void gdb_serial_initialize(void)
{
	// assume already initialized via mbed code
}

}
