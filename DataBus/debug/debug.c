/*
 * Add capability to have "C" and "S", whereby the continue_reason is the same as reason.
 * This is done via "exc_continue", which is normally "exc_return" (LR). If the signal is passed
 * throu, then "exc_continue" points to the target handler.
 *
 * Add "monitor reset".
 *
 * Add checks for Hc-1 and Hc0 (error all other variants)
 *
 * Hardfault needs to check for svc in user space (send E_CTX back if no debugger is active).
 * Otherwise it should signal a ILL reason (for debugging purposes).
 *
 * Use MON_REQ to step busfaults in Hardfault.
 *
 * All other conditions in Hardfault pass throu.
 */

/*
 * Copyright (c) 2013 Thomas Roell
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/******************************************************************************/

#define _GDB_CONFIG_CONSOLE                 1
#define _GDB_CONFIG_TARGET_DESCRIPTION      1
#define _GDB_CONFIG_KERNEL_LITHIUM          0

typedef int (*gdb_serial_read_t)(void);
typedef void (*gdb_serial_write_t)(int);

//extern void gdb_initialize(gdb_serial_read_t, gdb_serial_write_t);
#if (_GDB_CONFIG_CONSOLE == 1)
extern void gdb_console(const char *s);
#endif /* _GDB_CONFIG_CONSOLE */

extern void gdb_debug_exception(void);
extern void gdb_serial_interrupt(int data);

#define __MPU_PRESENT             1
#define __NVIC_PRIO_BITS          3
#define __FPU_PRESENT             0

/******************************************************************************/

#if (_GDB_CONFIG_KERNEL_LITHIUM == 1)

#include <itron.h>
#include <kernel.h>
#include <itron/kernel_config.h>
#include <itron/kernel_types.h>

#define _KERNEL_IDLE_THREAD 256

#endif /* _GDB_CONFIG_KERNEL_LITHIUM */

#define SCRATCH_DATA_SIZE       512

#define REASON_NONE             0
#define REASON_INT              2
#define REASON_ILL              4
#define REASON_TRAP             5
#define REASON_FPE              8
#define REASON_BUS              10
#define REASON_SEGV             11

#define STEPPING_NONE           0
#define STEPPING_INSTRUCTION    1
#define STEPPING_SVCALL         2

#define BREAKPOINT_TYPE_NONE    0
#define BREAKPOINT_TYPE_16BIT   1
#define BREAKPOINT_TYPE_32BIT   2

#define WATCHPOINT_TYPE_NONE    0
#define WATCHPOINT_TYPE_WRITE   1
#define WATCHPOINT_TYPE_READ    2
#define WATCHPOINT_TYPE_ACCESS  3

typedef struct {
	unsigned char stepping;
	volatile unsigned char attached;
	volatile unsigned char serial_interrupted;
	volatile unsigned char serial_data_pending;
	gdb_serial_read_t serial_read;
	gdb_serial_write_t serial_write;
	unsigned short interrupt_count;
	unsigned char breakpoint_count;
	unsigned char watchpoint_count;
	unsigned int watchpoint_max_size;
	unsigned int scratch_data[SCRATCH_DATA_SIZE / sizeof(uint32_t)];
} gdb_target_t;

static gdb_target_t gdb_target;

#define REGISTER_INDEX_R0         0
#define REGISTER_INDEX_R1         1
#define REGISTER_INDEX_R2         2
#define REGISTER_INDEX_R3         3
#define REGISTER_INDEX_R4         4
#define REGISTER_INDEX_R5         5
#define REGISTER_INDEX_R6         6
#define REGISTER_INDEX_R7         7
#define REGISTER_INDEX_R8         8
#define REGISTER_INDEX_R9         9
#define REGISTER_INDEX_R10        10
#define REGISTER_INDEX_R11        11
#define REGISTER_INDEX_R12        12
#define REGISTER_INDEX_SP         13
#define REGISTER_INDEX_LR         14
#define REGISTER_INDEX_PC         15
#define REGISTER_INDEX_XPSR       25

#if (__FPU_PRESENT == 1)

#define REGISTER_INDEX_D0         26
#define REGISTER_INDEX_D1         27
#define REGISTER_INDEX_D2         28
#define REGISTER_INDEX_D3         29
#define REGISTER_INDEX_D4         30
#define REGISTER_INDEX_D5         31
#define REGISTER_INDEX_D6         32
#define REGISTER_INDEX_D7         33
#define REGISTER_INDEX_D8         34
#define REGISTER_INDEX_D9         35
#define REGISTER_INDEX_D10        36
#define REGISTER_INDEX_D11        37
#define REGISTER_INDEX_D12        38
#define REGISTER_INDEX_D13        39
#define REGISTER_INDEX_D14        40
#define REGISTER_INDEX_D15        41
#define REGISTER_INDEX_FPSCR      42

#define REGISTER_INDEX_COUNT      43

#define REGISTER_INDEX_MSP        43
#define REGISTER_INDEX_PSP        44
#define REGISTER_INDEX_PRIMASK    45
#define REGISTER_INDEX_BASEPRI    46
#define REGISTER_INDEX_FAULTMASK  47
#define REGISTER_INDEX_CONTROL    48

#else /* __FPU_PRESENT */

#define REGISTER_INDEX_COUNT      26

#define REGISTER_INDEX_MSP        48
#define REGISTER_INDEX_PSP        49
#define REGISTER_INDEX_PRIMASK    50
#define REGISTER_INDEX_BASEPRI    51
#define REGISTER_INDEX_FAULTMASK  52
#define REGISTER_INDEX_CONTROL    53

#endif /* __FPU_PRESENT */

typedef struct {
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t xpsr;
} gdb_exception_t;

typedef struct {
	uint32_t r4;
	uint32_t r5;
	uint32_t r6;
	uint32_t r7;
	uint32_t r8;
	uint32_t r9;
	uint32_t r10;
	uint32_t r11;
	uint32_t exc_return;
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t xpsr;
} gdb_context_t;

typedef struct {
	uint32_t sp;
	uint32_t exc_continue;
	uint32_t msp;
	uint32_t psp;
	uint32_t primask;
	uint32_t basepri;
	uint32_t faultmask;
	uint32_t control;
} gdb_system_t;

#if (__FPU_PRESENT == 1)

typedef struct {
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t xpsr;
	uint64_t d0;
	uint64_t d1;
	uint64_t d2;
	uint64_t d3;
	uint64_t d4;
	uint64_t d5;
	uint64_t d6;
	uint64_t d7;
	uint32_t fpscr;
	uint32_t reserved;
}gdb_exception_fpu_t;

typedef struct {
	uint32_t r4;
	uint32_t r5;
	uint32_t r6;
	uint32_t r7;
	uint32_t r8;
	uint32_t r9;
	uint32_t r10;
	uint32_t r11;
	uint32_t exc_return;
	uint64_t d8;
	uint64_t d9;
	uint64_t d10;
	uint64_t d11;
	uint64_t d12;
	uint64_t d13;
	uint64_t d14;
	uint64_t d15;
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t xpsr;
	uint64_t d0;
	uint64_t d1;
	uint64_t d2;
	uint64_t d3;
	uint64_t d4;
	uint64_t d5;
	uint64_t d6;
	uint64_t d7;
	uint32_t fpscr;
	uint32_t reserved;
}gdb_context_fpu_t;

#endif /* __FPU_PRESENT */

/******************************************************************************/

static inline unsigned int gdb_atomic_read_b(const void *mem) {
	unsigned int data;

	__asm__ __volatile__("ldrb %0, [%1];" : "=r" (data) : "r" (mem));

	return data;
}

static inline unsigned int gdb_atomic_read_h(const void *mem) {
	unsigned int data;

	__asm__ __volatile__("ldrh %0, [%1];" : "=r" (data) : "r" (mem));

	return data;
}

static inline unsigned int gdb_atomic_read_w(const void *mem) {
	unsigned int data;

	__asm__ __volatile__("ldr %0, [%1];" : "=r" (data) : "r" (mem));

	return data;
}

static inline void gdb_atomic_write_b(void *mem, unsigned int data) {
	__asm__ __volatile__("strb %0, [%1];" : : "r" (data), "r" (mem));
}

static inline void gdb_atomic_write_h(void *mem, unsigned int data) {
	__asm__ __volatile__("strh %0, [%1];" : : "r" (data), "r" (mem));
}

static inline void gdb_atomic_write_w(void *mem, unsigned int data) {
	__asm__ __volatile__("str %0, [%1];" : : "r" (data), "r" (mem));
}

static unsigned int gdb_unpack_nibble(char data) {
	if ((data >= 'a') && (data <= 'f')) {
		return (data - 'a' + 10);
	} else if ((data >= '0') && (data <= '9')) {
		return (data - '0');
	} else if ((data >= 'A') && (data <= 'F')) {
		return (data - 'A' + 10);
	} else {
		return (-1);
	}
}

static inline char gdb_pack_nibble(unsigned int data) {
	static const char xlate[] = "0123456789abcdef";

	return xlate[data & 15];
}

/* GDB does not tell us whether to use 8, 16 or 32 bit accesses. Reading/Writing
 * real HW will trigger faults if read with the wrong size. So the best thing
 * that can be done is to break up read/write transactions into aligned pieces.
 */

static char * gdb_unpack_data(char *cp, unsigned char *data, unsigned int count) {
	unsigned int temp;

	if (((unsigned int) data & 1) && (count >= 1)) {
		temp = (gdb_unpack_nibble(*cp++) << 4);
		temp |= (gdb_unpack_nibble(*cp++) << 0);

		gdb_atomic_write_b(data, temp);

		data += 1;
		count -= 1;
	}

	if (((unsigned int) data & 2) && (count >= 2)) {
		temp = (gdb_unpack_nibble(*cp++) << 4);
		temp |= (gdb_unpack_nibble(*cp++) << 0);
		temp |= (gdb_unpack_nibble(*cp++) << 12);
		temp |= (gdb_unpack_nibble(*cp++) << 8);

		gdb_atomic_write_h(data, temp);

		data += 2;
		count -= 2;
	}

	while (count >= 4) {
		temp = (gdb_unpack_nibble(*cp++) << 4);
		temp |= (gdb_unpack_nibble(*cp++) << 0);
		temp |= (gdb_unpack_nibble(*cp++) << 12);
		temp |= (gdb_unpack_nibble(*cp++) << 8);
		temp |= (gdb_unpack_nibble(*cp++) << 20);
		temp |= (gdb_unpack_nibble(*cp++) << 16);
		temp |= (gdb_unpack_nibble(*cp++) << 28);
		temp |= (gdb_unpack_nibble(*cp++) << 24);

		gdb_atomic_write_w(data, temp);

		data += 4;
		count -= 4;
	}

	if (count >= 2) {
		temp = (gdb_unpack_nibble(*cp++) << 4);
		temp |= (gdb_unpack_nibble(*cp++) << 0);
		temp |= (gdb_unpack_nibble(*cp++) << 12);
		temp |= (gdb_unpack_nibble(*cp++) << 8);

		gdb_atomic_write_h(data, temp);

		data += 2;
		count -= 2;
	}

	if (count >= 1) {
		temp = (gdb_unpack_nibble(*cp++) << 4);
		temp |= (gdb_unpack_nibble(*cp++) << 0);

		gdb_atomic_write_b(data, temp);

		data += 1;
		count -= 1;
	}

	return cp;
}

static char * gdb_pack_data(char *cp, const unsigned char *data,
		unsigned int count) {
	unsigned int temp;

	if (((unsigned int) data & 1) && (count >= 1)) {
		temp = gdb_atomic_read_b(data);

		*cp++ = gdb_pack_nibble((temp >> 4) & 15);
		*cp++ = gdb_pack_nibble((temp >> 0) & 15);

		data += 1;
		count -= 1;
	}

	if (((unsigned int) data & 2) && (count >= 2)) {
		temp = gdb_atomic_read_h(data);

		*cp++ = gdb_pack_nibble((temp >> 4) & 15);
		*cp++ = gdb_pack_nibble((temp >> 0) & 15);
		*cp++ = gdb_pack_nibble((temp >> 12) & 15);
		*cp++ = gdb_pack_nibble((temp >> 8) & 15);

		data += 2;
		count -= 2;
	}

	while (count >= 4) {
		temp = gdb_atomic_read_w(data);

		*cp++ = gdb_pack_nibble((temp >> 4) & 15);
		*cp++ = gdb_pack_nibble((temp >> 0) & 15);
		*cp++ = gdb_pack_nibble((temp >> 12) & 15);
		*cp++ = gdb_pack_nibble((temp >> 8) & 15);
		*cp++ = gdb_pack_nibble((temp >> 20) & 15);
		*cp++ = gdb_pack_nibble((temp >> 16) & 15);
		*cp++ = gdb_pack_nibble((temp >> 28) & 15);
		*cp++ = gdb_pack_nibble((temp >> 24) & 15);

		data += 4;
		count -= 4;
	}

	if (count >= 2) {
		temp = gdb_atomic_read_h(data);

		*cp++ = gdb_pack_nibble((temp >> 4) & 15);
		*cp++ = gdb_pack_nibble((temp >> 0) & 15);
		*cp++ = gdb_pack_nibble((temp >> 12) & 15);
		*cp++ = gdb_pack_nibble((temp >> 8) & 15);

		data += 2;
		count -= 2;
	}

	if (count >= 1) {
		temp = gdb_atomic_read_b(data);

		*cp++ = gdb_pack_nibble((temp >> 4) & 15);
		*cp++ = gdb_pack_nibble((temp >> 0) & 15);

		data += 1;
		count -= 1;
	}

	*cp = '\0';

	return cp;
}

static char * gdb_pack_string(char *cp, const char *s) {
	unsigned char temp;

	while (*s != '\0') {
		temp = *s++;
		*cp++ = gdb_pack_nibble(temp >> 4);
		*cp++ = gdb_pack_nibble(temp & 15);
	}

	*cp = '\0';

	return cp;
}

static char * gdb_unpack_unsigned(char *cp, unsigned int *data) {
	unsigned int temp;
	int nibble;

	if (*cp == '\0') {
		return NULL;
	}

	temp = 0;

	while ((nibble = gdb_unpack_nibble(*cp)) >= 0) {
		temp = (temp << 4) | nibble;
		cp++;
	}

	if ((*cp == ',') || (*cp == ':') || (*cp == '\0')) {
		*data = temp;

		return cp;
	} else {
		return NULL;
	}
}

static char * gdb_pack_unsigned(char *cp, unsigned int data) {
	unsigned int count;

	if (data == 0) {
		*cp++ = '0';
	} else {
		count = 8;

		while ((data & 0xf0000000) == 0) {
			data <<= 4;
			count--;
		}

		while (count) {
			*cp++ = gdb_pack_nibble(data >> 28);

			data <<= 4;
			count--;
		}
	}

	*cp = '\0';

	return cp;
}

static bool gdb_scan_token(char **packet, const char *token) {
	char *cp;

	cp = *packet;

	if (*cp == '\0') {
		return false;
	}

	while ((*cp++ == *token++)) {
		if (*token == '\0') {
			if ((*cp == ',') || (*cp == ':') || (*cp == '\0')) {
				*packet = cp;

				return true;
			} else {
				return false;
			}
		}
	}

	return false;
}

static char * gdb_reply_string(gdb_target_t *target, const char *string,
		unsigned int address, unsigned int length) {
	char *reply, *cp;

	reply = cp = (char*) &target->scratch_data[0];
	*cp++ = 'm';

	do {
		if (string[address] == '\0') {
			reply[0] = 'l';
			break;
		}

		*cp++ = string[address++];
	} while (--length);

	*cp++ = '\0';

	return reply;
}

static char *gdb_packet_receive(gdb_target_t *target, unsigned int *length) {
	char *packet, *cp;
	unsigned char checksum;
	unsigned char xmitcsum;
	int data;

	while (1) {
		/* If we arrived here with a serial interrupt, then we may
		 * have seen the leading '$' already, otherwise wait for it.
		 */
		if (target->serial_data_pending == '$') {
			target->serial_data_pending = '\0';
		} else {
			while ((data = target->serial_read()) != '$') {
				continue;
			}
		}

		retry: packet = cp = (char*) &target->scratch_data[0];
		checksum = 0;
		xmitcsum = -1;

		/* now, read until a # or end of buffer is found */
		while ((cp - packet) < (SCRATCH_DATA_SIZE - 1)) {
			data = target->serial_read();

			if (data == '$') {
				goto retry;
			} else if (data == '#') {
				*length = cp - packet;

				*cp++ = '\0';

				data = target->serial_read();
				xmitcsum = gdb_unpack_nibble(data) << 4;
				data = target->serial_read();
				xmitcsum += gdb_unpack_nibble(data);

				if (checksum != xmitcsum) {
					target->serial_write('-'); /* failed checksum */

					break;
				} else {
					target->serial_write('+'); /* successful checksum */

					return packet;
				}
			}

			checksum += data;
			*cp++ = data;
		}
	}
}

static void gdb_packet_send(gdb_target_t *target, const char *packet,
		bool exception) {
	const char *cp;
	unsigned char checksum;
	int data;

	/*  $<packet info>#<checksum>. */
	do {
		target->serial_write('$');

		checksum = 0;
		cp = packet;

		while ((data = *cp++)) {
			target->serial_write(data);
			checksum += data;
		}

		target->serial_write('#');
		target->serial_write(gdb_pack_nibble(checksum >> 4));
		target->serial_write(gdb_pack_nibble(checksum & 15));

#if (_GDB_CONFIG_CONSOLE == 1)
		if (!exception) {
			/*
			 * If we end up here from a non-exception callee, data is
			 * passed from the serial interrupt via gdb_serial_data_pending.
			 * Hence a busy waiting loop is in use that also checks for
			 * gdb_serial_interrupted, which signal that a ^C or a "$"
			 * was eceived/consumed.
			 */
			do {
				if (target->serial_interrupted) {
					return;
				}

				data = target->serial_data_pending;
			} while (data == '\0');

			target->serial_data_pending = '\0';
		} else
#endif /* _GDB_CONFIG_CONSOLE */
		{
			data = target->serial_read();
		}
	} while (data != '+');
}

/******************************************************************************/

#define DWT_CTRL                          (*((volatile uint32_t*)0xe0001000))
#define DWT_CYCCNT                        (*((volatile uint32_t*)0xe0001004))
#define DWT_CPICNT                        (*((volatile uint32_t*)0xe0001008))
#define DWT_EXCCNT                        (*((volatile uint32_t*)0xe000100c))
#define DWT_SLEEPCNT                      (*((volatile uint32_t*)0xe0001010))
#define DWT_LSUCNT                        (*((volatile uint32_t*)0xe0001014))
#define DWT_FOLDCNT                       (*((volatile uint32_t*)0xe0001018))
#define DWT_PCSR                          (*((volatile uint32_t*)0xe000101c))
#define DWT_COMP(_n)                      (((volatile uint32_t*)0xe0001020)[(_n)*4])
#define DWT_MASK(_n)                      (((volatile uint32_t*)0xe0001024)[(_n)*4])
#define DWT_FUNCTION(_n)                  (((volatile uint32_t*)0xe0001028)[(_n)*4])
#define   DWT_FUNCTION_MATCHED            0x01000000
#define   DWT_FUNCTION_WATCHPOINT_DISABLE 0x00000000
#define   DWT_FUNCTION_WATCHPOINT_READ    0x00000005
#define   DWT_FUNCTION_WATCHPOINT_WRITE   0x00000006
#define   DWT_FUNCTION_WATCHPOINT_ACCESS  0x00000007
#define   DWT_FUNCTION_WATCHPOINT_MASK    0x0000000f

#define FP_CTRL                           (*((volatile uint32_t*)0xe0002000))
#define   FP_CTRL_KEY                     0x00000002
#define   FP_CTRL_ENABLE                  0x00000001
#define FP_REMAP                          (*((volatile uint32_t*)0xe0002004))
#define FP_COMP(_n)                       (((volatile uint32_t*)0xe0002008)[(_n)])
#define   FP_COMP_DISABLE                 0x00000000
#define   FP_COMP_ENABLE                  0x00000001
#define   FP_COMP_REPLACE_ADDRESS         0x00000000
#define   FP_COMP_REPLACE_16BIT_LO        0x40000000
#define   FP_COMP_REPLACE_16BIT_HI        0x80000000
#define   FP_COMP_REPLACE_32BIT           0xc0000000

#define SCB_ICTR		          (*((volatile uint32_t*)0xe000e004))
#define SYST_CSR		          (*((volatile uint32_t*)0xe000e010))
#define   SYST_CSR_COUNTFLAG	          0x00010000
#define   SYST_CSR_CLKSOURCE	          0x00000004
#define   SYST_CSR_TICKINT	          0x00000002
#define   SYST_CSR_ENABLE	          0x00000001
#define SYST_RVR		          (*((volatile uint32_t*)0xe000e014))
#define SYST_CVR		          (*((volatile uint32_t*)0xe000e018))
#define SYST_CALIB		          (*((volatile uint32_t*)0xe000e01c))
#define NVIC_ISER(_n)		          (((volatile uint32_t*)0xe000e100)[(_n)])
#define NVIC_ICER(_n)		          (((volatile uint32_t*)0xe000e180)[(_n)])
#define NVIC_ISPR(_n)		          (((volatile uint32_t*)0xe000e200)[(_n)])
#define NVIC_ICPR(_n)		          (((volatile uint32_t*)0xe000e280)[(_n)])
#define NVIC_IABR(_n)		          (((volatile uint32_t*)0xe000e300)[(_n)])
#define NVIC_IPR(_n)		          (((volatile uint8_t*)0xe000e400)[(_n)])

#define SCB_CPUID		          (*((volatile uint32_t*)0xe000ed00))
#define SCB_ICSR		          (*((volatile uint32_t*)0xe000ed04))
#define   SCB_ICSR_NMIPENDSET	          0x80000000
#define   SCB_ICSR_PENDSVSET	          0x10000000
#define   SCB_ICSR_PENDSVCLR	          0x08000000
#define   SCB_ICSR_PENDSTSET	          0x04000000
#define   SCB_ICSR_PENDSTCLR	          0x02000000
#define   SCB_ICSR_ISRPREEMPT	          0x00800000
#define   SCB_ICSR_ISRPENDING	          0x00400000
#define   SCB_ICSR_VECTPENDING	          0x001ff000
#define   SCB_ICSR_RETTOBASE	          0x000001ff
#define   SCB_ICSR_VECTACTIVE	          0x000001ff
#define SCB_VTOR		          (*((volatile uint32_t*)0xe000ed08))
#define SCB_AIRCR		          (*((volatile uint32_t*)0xe000ed0c))
#define   SCB_AIRCR_VECTKEY	          0x05fa0000
#define   SCB_AIRCR_ENDIANESS	          0x00008000
#define   SCB_AIRCR_PRIGROUP_	          0x00000070
#define   SCB_AIRCR_SYSRESETREQ	          0x00000004
#define   SCB_AIRCR_VECTCLRACTIVE         0x00000002
#define   SCB_AIRCR_VECTRESET             0x00000001
#define SCB_SCR			          (*((volatile uint32_t*)0xe000ed10))
#define   SCB_SCR_SEVONPEND	          0x00000010
#define   SCB_SCR_SLEEPDEEP	          0x00000004
#define   SCB_SCR_SLEEPONEXIT 	          0x00000002
#define SCB_CCR			          (*((volatile uint32_t*)0xe000ed14))
#define   SCB_CCR_STKALIGN	          0x00000200
#define   SCB_CCR_BFHFNMIGN               0x00000100
#define   SCB_SCR_DIV_0_TRP	          0x00000010
#define   SCB_CCR_UNALIGN_TRP	          0x00000008
#define   SCB_CCR_USERSETMPEND            0x00000002
#define   SCB_CCR_NONBASETHRDENA          0x00000001
#define SCB_SHPR(_n)		          (((volatile uint8_t*)0xe000ed18)[(_n)-4])
#define SCB_SHPR1		          (*((volatile uint32_t*)0xe000ed18))
#define SCB_SHPR2		          (*((volatile uint32_t*)0xe000ed1c))
#define SCB_SHPR3		          (*((volatile uint32_t*)0xe000ed20))
#define SCB_SHCSR		          (*((volatile uint32_t*)0xe000ed24))
#define   SCB_SHCSR_USGFAULTENA           0x00040000
#define   SCB_SHCSR_BUSFAULTENA           0x00020000
#define   SCB_SHCSR_MEMFAULTENA           0x00010000
#define   SCB_SHCSR_SVCALLPENDED          0x00008000
#define   SCB_SHCSR_BUSFAULTPENDED        0x00004000
#define   SCB_SHCSR_MEMFAULTPENDED        0x00002000
#define   SCB_SHCSR_USGFAULTPENDED        0x00001000
#define   SCB_SHCSR_SYSTICKACT            0x00000800
#define   SCB_SHCSR_PENDSVACT             0x00000400
#define   SCB_SHCSR_MONITORACT            0x00000100
#define   SCB_SHCSR_SVCALLACT             0x00000080
#define   SCB_SHCSR_USGFAULTACT           0x00000004
#define   SCB_SHCSR_BUSFAULTACT           0x00000002
#define   SCB_SHCSR_MEMFAULTACT           0x00000001
#define SCB_CFSR                          (*((volatile uint32_t*)0xe000ed28))
#define   SCB_CFSR_UFSR_MASK              0xffff0000   /* UsageFault */
#define   SCB_CFSR_DIVBYZERO              0x02000000   /* UsageFault */
#define   SCB_CFSR_UNALIGNED              0x01000000   /* UsageFault */
#define   SCB_CFSR_NOCP                   0x00080000   /* UsageFault */
#define   SCB_CFSR_INVPC                  0x00040000   /* UsageFault */
#define   SCB_CFSR_INVSTATE               0x00020000   /* UsageFault */
#define   SCB_CFSR_UNDEFINSTR             0x00010000   /* UsageFault */
#define   SCB_CFSR_BFSR_MASK              0x0000ff00   /* BusFault   */
#define   SCB_CFSR_BFARVALID              0x00008000   /* BusFault   */
#define   SCB_CFSR_LSPERR                 0x00002000   /* BusFault   */
#define   SCB_CFSR_STKERR                 0x00001000   /* BusFault   */
#define   SCB_CFSR_UNSTKERR               0x00000800   /* BusFault   */
#define   SCB_CFSR_IMPRESCISERR           0x00000400   /* BusFault   */
#define   SCB_CFSR_PRESCISERR             0x00000200   /* BusFault   */
#define   SCB_CFSR_IBUSERR                0x00000100   /* BusFault   */
#define   SCB_CFSR_MMFSR_MASK             0x000000ff   /* MemManage  */
#define   SCB_CFSR_MMARVALID              0x00000080   /* MemManage  */
#define   SCB_CFSR_MLSPERR                0x00000020   /* MemManage  */
#define   SCB_CFSR_MSTKERR                0x00000010   /* MemManage  */
#define   SCB_CFSR_MUNSTKERR              0x00000008   /* MemManage  */
#define   SCB_CFSR_DACCVOIL               0x00000002   /* MemManage  */
#define   SCB_CFSR_IACCVOIL               0x00000001   /* MemManage  */
#define SCB_HFSR                          (*((volatile uint32_t*)0xe000ed2c))
#define   SCB_HFSR_DEBUGEVT               0x80000000
#define   SCB_HFSR_FORCED                 0x40000000
#define   SCB_HFSR_VECTABLE               0x00000002
#define SCB_DFSR                          (*((volatile uint32_t*)0xe000ed30))
#define   SCB_DFSR_EXTERNAL               0x00000010
#define   SCB_DFSR_VCATCH                 0x00000008
#define   SCB_DFSR_DWTTRAP                0x00000004
#define   SCB_DFSR_BKPT                   0x00000002
#define   SCB_DFSR_HALTED                 0x00000001
#define SCB_MMAFR                         (*((volatile uint32_t*)0xe000ed34))
#define SCB_BAFR                          (*((volatile uint32_t*)0xe000ed38))
#define SCB_DHCSR                         (*((volatile uint32_t*)0xe000edf0))
#define SCB_DCRSR                         (*((volatile uint32_t*)0xe000edf4))
#define SCB_DCRDR                         (*((volatile uint32_t*)0xe000edf8))
#define SCB_DEMCR                         (*((volatile uint32_t*)0xe000edfc))
#define   SCB_DEMCR_TRCENA                0x01000000
#define   SCB_DEMCR_MON_REQ               0x00080000
#define   SCB_DEMCR_MON_STEP              0x00040000
#define   SCB_DEMCR_MON_PEND              0x00020000
#define   SCB_DEMCR_MON_EN                0x00010000
#define   SCB_DEMCR_VC_HARDERR            0x00000400
#define   SCB_DEMCR_VC_INTERR             0x00000200
#define   SCB_DEMCR_VC_BUSERR             0x00000100
#define   SCB_DEMCR_VC_STATERR            0x00000080
#define   SCB_DEMCR_VC_CHKERR             0x00000040
#define   SCB_DEMCR_VC_NOCPERR            0x00000020
#define   SCB_DEMCR_VC_MMERR              0x00000010
#define   SCB_DEMCR_VC_CORERESET          0x00000001

#define SCB_FPCCR		          (*((volatile uint32_t*)0xe000ef34))
#define   SCB_FPCCR_ASPEN                 0x80000000
#define   SCB_FPCCR_LSPEN                 0x40000000
#define   SCB_FPCCR_MONRDY                0x00000100
#define   SCB_FPCCR_BFRDY                 0x00000040
#define   SCB_FPCCR_MMRDY                 0x00000020
#define   SCB_FPCCR_HFRDY                 0x00000010
#define   SCB_FPCCR_THREAD                0x00000008
#define   SCB_FPCCR_USER                  0x00000002
#define   SCB_FPCCR_LSPACT                0x00000001
#define SCB_FPCAR		          (*((volatile uint32_t*)0xe000ef38))
#define SCB_FPDSCR		          (*((volatile uint32_t*)0xe000ef3c))
#define SCB_MVFR0		          (*((volatile uint32_t*)0xe000ef40))
#define SCB_MVFR1		          (*((volatile uint32_t*)0xe000ef44))

/******************************************************************************/

static const char* const gdb_memory_map = "<memory-map>"
		"<memory type=\"rom\" start=\"0x00000000\" length=\"0x20000000\"/>"
		"<memory type=\"ram\" start=\"0x20000000\" length=\"0xe0000000\"/>"
		"</memory-map>";

static void gdb_breakpoint_initialize(gdb_target_t *target) {
	unsigned int i;

	target->breakpoint_count = ((FP_CTRL >> 4) & 0x0f)
			| ((FP_CTRL >> 8) & 0x70);

	for (i = 0; i < target->breakpoint_count; i++) {
		FP_COMP(i) = FP_COMP_DISABLE;
	}

	FP_CTRL = (FP_CTRL_KEY | FP_CTRL_ENABLE);
}

static bool gdb_breakpoint_change(gdb_target_t *target, unsigned int address,
		unsigned int type, bool insert) {
	unsigned int i;
	uint32_t fp_comp;

	if (type == BREAKPOINT_TYPE_16BIT) {
		if (address & 1) {
			return false;
		}
	} else {
		if (address & 3) {
			return false;
		}
	}

	if (address >= 0x20000000) {
		return false;
	}

	fp_comp =
			(((type == BREAKPOINT_TYPE_32BIT) ?
					FP_COMP_REPLACE_32BIT :
					((address & 2) ?
							FP_COMP_REPLACE_16BIT_HI : FP_COMP_REPLACE_16BIT_LO))
					| (address & ~3) | FP_COMP_ENABLE);

	if (insert) {
		for (i = 0; i < target->breakpoint_count; i++) {
			if (FP_COMP(i) == FP_COMP_DISABLE)
			{
				FP_COMP(i) = fp_comp;

				return true;
			}
		}
	}
	else
	{
		for (i = 0; i < target->breakpoint_count; i++)
		{
			if (FP_COMP(i) == fp_comp)
			{
				FP_COMP(i) = FP_COMP_DISABLE;

				return true;
			}
		}
	}

	return false;
}

/******************************************************************************/

static void gdb_watchpoint_initialize(gdb_target_t *target) {
	unsigned int i;

	SCB_DEMCR |= SCB_DEMCR_TRCENA;

	target->watchpoint_count = (DWT_CTRL >> 28) & 0x0f;

	DWT_MASK(0) = 0xffffffff;
	target->watchpoint_max_size = DWT_MASK(0) + 1;

	for (i = 0; i < target->watchpoint_count; i++) {
		DWT_FUNCTION(i) = DWT_FUNCTION_WATCHPOINT_DISABLE;

		/* Read back DWT_FUNCTION, so that all DWT_FUNCTION_MATCHED are cleared. */
		DWT_FUNCTION(i);
	}
}

static bool gdb_watchpoint_check(gdb_target_t *target, unsigned int *address) {
	unsigned int i;
	bool matched;

	matched = false;

	for (i = 0; i < target->watchpoint_count; i++) {
		if (DWT_FUNCTION(i) & DWT_FUNCTION_MATCHED)
		{
			matched = true;

			*address = DWT_COMP(i);
		}
	}

	return matched;
}

static bool gdb_watchpoint_change(gdb_target_t *target, unsigned int address,
		unsigned int size, unsigned int type, bool insert) {
	unsigned int i;
	uint32_t dwt_comp, dwt_mask, dwt_function;

	if (size == 0) {
		return false;
	}

	if ((size & (size - 1)) != 0) {
		return false;
	}

	if (size > target->watchpoint_max_size) {
		return false;
	}

	if (address & (size - 1)) {
		return false;
	}

	dwt_comp = address;
	dwt_mask = (size - 1);

	switch (type) {
	case WATCHPOINT_TYPE_WRITE:
		dwt_function = DWT_FUNCTION_WATCHPOINT_WRITE;
		break;
	case WATCHPOINT_TYPE_READ:
		dwt_function = DWT_FUNCTION_WATCHPOINT_READ;
		break;
	case WATCHPOINT_TYPE_ACCESS:
		dwt_function = DWT_FUNCTION_WATCHPOINT_ACCESS;
		break;
	default:
		dwt_function = DWT_FUNCTION_WATCHPOINT_DISABLE;
		break;
	}

	if (insert) {
		for (i = 0; i < target->watchpoint_count; i++) {
			if ((DWT_FUNCTION(i) & DWT_FUNCTION_WATCHPOINT_MASK) == DWT_FUNCTION_WATCHPOINT_DISABLE)
			{
				DWT_COMP(i) = dwt_comp;
				DWT_MASK(i) = dwt_mask;
				DWT_FUNCTION(i) = dwt_function;

				return true;
			}
		}
	}
	else
	{
		for (i = 0; i < target->watchpoint_count; i++)
		{
			if ((DWT_COMP(i) == dwt_comp) &&
					(DWT_MASK(i) == dwt_mask) &&
					((DWT_FUNCTION(i) & DWT_FUNCTION_WATCHPOINT_MASK) == dwt_function))
			{
				DWT_FUNCTION(i) = DWT_FUNCTION_WATCHPOINT_DISABLE;
			}

			return true;
		}
	}

	return false;
}

/******************************************************************************/

#if (_GDB_CONFIG_TARGET_DESCRIPTION == 1)

static const char* const gdb_target_description =
		"<target>"
				"<architecture>arm</architecture>"
				"<feature name=\"org.gnu.gdb.arm.m-profile\">"
				"<reg name=\"r0\" bitsize=\"32\"/>"
				"<reg name=\"r1\" bitsize=\"32\"/>"
				"<reg name=\"r2\" bitsize=\"32\"/>"
				"<reg name=\"r3\" bitsize=\"32\"/>"
				"<reg name=\"r4\" bitsize=\"32\"/>"
				"<reg name=\"r5\" bitsize=\"32\"/>"
				"<reg name=\"r6\" bitsize=\"32\"/>"
				"<reg name=\"r7\" bitsize=\"32\"/>"
				"<reg name=\"r8\" bitsize=\"32\"/>"
				"<reg name=\"r9\" bitsize=\"32\"/>"
				"<reg name=\"r10\" bitsize=\"32\"/>"
				"<reg name=\"r11\" bitsize=\"32\"/>"
				"<reg name=\"r12\" bitsize=\"32\"/>"
				"<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>"
				"<reg name=\"lr\" bitsize=\"32\"/>"
				"<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>"
				"<reg name=\"xpsr\" bitsize=\"32\" regnum=\"25\"/>"
				"</feature>"
#if (__FPU_PRESENT == 1)
		"<feature name=\"org.gnu.gdb.arm.vfp\">"
		"<reg name=\"d0\" bitsize=\"64\" type=\"ieee_double\" regnum=\"26\"/>"
		"<reg name=\"d1\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d2\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d3\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d4\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d5\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d6\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d7\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d8\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d9\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d10\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d11\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d12\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d13\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d14\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"d15\" bitsize=\"64\" type=\"ieee_double\"/>"
		"<reg name=\"fpscr\" bitsize=\"32\" type=\"int\" group=\"float\"/>"
		"</feature>"
#endif /* __FPU_PRESENT */
		"<feature name=\"org.gnu.gdb.arm.system\">"
		"<reg name=\"msp\" bitsize=\"32\" type=\"data_ptr\" group=\"system\" regnum=\"48\"/>"
		"<reg name=\"psp\" bitsize=\"32\" type=\"data_ptr\" group=\"system\"/>"
		"<reg name=\"primask\" bitsize=\"8\" group=\"system\"/>"
		"<reg name=\"basepri\" bitsize=\"8\" group=\"system\"/>"
		"<reg name=\"faultmask\" bitsize=\"8\" group=\"system\"/>"
		"<reg name=\"control\" bitsize=\"8\" group=\"system\"/>"
		"</feature>"
		"</target>";

#endif /* _GDB_CONFIG_TARGET_DESCRIPTION */

#define REGISTER_OFFSET_UNDEFINED 0xfc
#define REGISTER_OFFSET_MASK      0xfc
#define REGISTER_SIZE_4           0x01
#define REGISTER_SIZE_8           0x02

static const uint8_t gdb_register_table[REGISTER_INDEX_COUNT] = {
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r0)), /* REGISTER_INDEX_R0    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r1)), /* REGISTER_INDEX_R1    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r2)), /* REGISTER_INDEX_R2    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r3)), /* REGISTER_INDEX_R3    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r4)), /* REGISTER_INDEX_R4    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r5)), /* REGISTER_INDEX_R5    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r6)), /* REGISTER_INDEX_R6    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r7)), /* REGISTER_INDEX_R7    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r8)), /* REGISTER_INDEX_R8    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r9)), /* REGISTER_INDEX_R9    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r10)), /* REGISTER_INDEX_R10   */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r11)), /* REGISTER_INDEX_R11   */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, r12)), /* REGISTER_INDEX_R12   */
		0, /* REGISTER_INDEX_SP    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, lr)), /* REGISTER_INDEX_LR    */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, pc)), /* REGISTER_INDEX_PC    */
		0, /* 16                   */
		0, /* 17                   */
		0, /* 18                   */
		0, /* 19                   */
		0, /* 20                   */
		0, /* 21                   */
		0, /* 22                   */
		0, /* 23                   */
		0, /* 24                   */
		(REGISTER_SIZE_4 | offsetof(gdb_context_t, xpsr)), /* REGISTER_INDEX_XPSR  */
#if (__FPU_PRESENT == 1)
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D0    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D1    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D2    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D3    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D4    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D5    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D6    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D7    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D8    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D9    */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D10   */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D11   */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D12   */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D13   */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D14   */
		(REGISTER_SIZE_8 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_D15   */
		(REGISTER_SIZE_4 | REGISTER_OFFSET_UNDEFINED), /* REGISTER_INDEX_FPSCR */
#endif /* __FPU_PRESENT */
	};

#if (__FPU_PRESENT == 1)

static const uint8_t gdb_register_table_fpu[REGISTER_INDEX_COUNT] = {
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r0)), /* REGISTER_INDEX_R0    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r1)), /* REGISTER_INDEX_R1    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r2)), /* REGISTER_INDEX_R2    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r3)), /* REGISTER_INDEX_R3    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r4)), /* REGISTER_INDEX_R4    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r5)), /* REGISTER_INDEX_R5    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r6)), /* REGISTER_INDEX_R6    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r7)), /* REGISTER_INDEX_R7    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r8)), /* REGISTER_INDEX_R8    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r9)), /* REGISTER_INDEX_R9    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r10)), /* REGISTER_INDEX_R10   */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r11)), /* REGISTER_INDEX_R11   */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, r12)), /* REGISTER_INDEX_R12   */
	0, /* REGISTER_INDEX_SP    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, lr)), /* REGISTER_INDEX_LR    */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, pc)), /* REGISTER_INDEX_PC    */
	0, /* 16                   */
	0, /* 17                   */
	0, /* 18                   */
	0, /* 19                   */
	0, /* 20                   */
	0, /* 21                   */
	0, /* 22                   */
	0, /* 23                   */
	0, /* 24                   */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, xpsr)), /* REGISTER_INDEX_XPSR  */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d0)), /* REGISTER_INDEX_D0    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d1)), /* REGISTER_INDEX_D1    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d2)), /* REGISTER_INDEX_D2    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d3)), /* REGISTER_INDEX_D3    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d4)), /* REGISTER_INDEX_D4    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d5)), /* REGISTER_INDEX_D5    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d6)), /* REGISTER_INDEX_D6    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d7)), /* REGISTER_INDEX_D7    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d8)), /* REGISTER_INDEX_D8    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d9)), /* REGISTER_INDEX_D9    */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d10)), /* REGISTER_INDEX_D10   */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d11)), /* REGISTER_INDEX_D11   */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d12)), /* REGISTER_INDEX_D12   */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d13)), /* REGISTER_INDEX_D13   */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d14)), /* REGISTER_INDEX_D14   */
	(REGISTER_SIZE_8 | offsetof(gdb_context_fpu_t, d15)), /* REGISTER_INDEX_D15   */
	(REGISTER_SIZE_4 | offsetof(gdb_context_fpu_t, fpscr)), /* REGISTER_INDEX_FPSCR */
};

#endif /* __FPU_PRESENT */

static unsigned char * gdb_register_location(gdb_target_t *target,
		gdb_context_t *context, uint32_t *sp, gdb_system_t *system,
		unsigned int index, unsigned int *size) {
	const uint8_t *register_table;
	unsigned int offset;

	if (index == REGISTER_INDEX_SP) {
		*size = 4;

		return (unsigned char*) sp;
	} else if (index < REGISTER_INDEX_COUNT) {
#if (__FPU_PRESENT == 1)
		if (!(context->exc_return & 0x00000010))
		{
			register_table = &gdb_register_table_fpu[0];
		}
		else
#endif /* __FPU_PRESENT */
		{
			register_table = &gdb_register_table[0];
		}

		if (register_table[index] & (REGISTER_SIZE_4 | REGISTER_SIZE_8)) {
#if (__FPU_PRESENT == 1)
			if (register_table[index] & REGISTER_SIZE_8)
			{
				*size = 8;
			}
			else
#endif /* __FPU_PRESENT */
			{
				*size = 4;
			}

			offset = register_table[index] & REGISTER_OFFSET_MASK;

			if (offset == REGISTER_OFFSET_UNDEFINED) {
				return NULL;
			} else {
				return (unsigned char*) context + offset;
			}
		}
	}
#if (_GDB_CONFIG_TARGET_DESCRIPTION == 1)
	else if ((index >= REGISTER_INDEX_MSP) && (index <= REGISTER_INDEX_PSP)) {
		*size = 4;

		if (system) {
			return (unsigned char*) ((unsigned int*) &system->msp
					+ (index - REGISTER_INDEX_MSP));
		} else {
			return NULL;
		}
	} else if ((index >= REGISTER_INDEX_PRIMASK)
			&& (index <= REGISTER_INDEX_CONTROL)) {
		*size = 1;

		if (system) {
			return (unsigned char*) ((unsigned int*) &system->primask
					+ (index - REGISTER_INDEX_PRIMASK));
		} else {
			return NULL;
		}
	}
#endif /* _GDB_CONFIG_TARGET_DESCRIPTION */

	*size = 0;

	return NULL;
}

/******************************************************************************/

static void gdb_step(gdb_target_t *target, gdb_exception_t *exception,
		unsigned int size) {
	unsigned int itstate;

	exception->pc += size;

	if (exception->xpsr & 0x0000f000) {
		/* unpack */
		itstate = ((exception->xpsr & 0x06000000) >> 25)
				| ((exception->xpsr & 0x0000fc00) >> 8);

		/* advance */
		itstate = (
				!(itstate & 0x07) ?
						0x00 : ((itstate & 0xe0) | ((itstate & 0x0f) << 1)));

		/* pack */
		exception->xpsr = (exception->xpsr & ~0x0600fc00)
				| ((itstate & 0x03) << 25) | ((itstate & 0xfc) << 8);
	}
}

static bool gdb_continue(gdb_target_t *target, gdb_exception_t *exception,
		gdb_context_t *context, gdb_system_t *system, unsigned int exception_pc,
		bool stepping) {
	unsigned int insn0, insn1, index, size, basepri, prio_bits, i;
	unsigned int address, syst_csr, scb_icsr;
	unsigned char *location;
	unsigned int *dp;
	bool resume, executing, priviledged;

	/* The code below is complex and messy, so some explanations are in order. First off,
	 * an instruction is only executed if there is either no IT-block, or if the condition
	 * is meet within the IT-block. If we start executing/stepping from a new location,
	 * the ITSTATE is unknown, hence reset to 0.
	 *
	 * For stepping there are a few special cases to attend. To avoid stepping into a
	 * pending interrupt, BASEPRI gets manipulated, so all interrupts stay pending. That
	 * means that some instructions will need special handling to hide this manipulation.
	 * If an exception other than PENDSV and SYSTICK are pending, those need to be executed,
	 * which means all interrupts get disable/unpended at the NVIC level.
	 *
	 * In general if an instruction is skipped or emulated, the ITSTATE needs to be
	 * advanced, if inside an IT-block ...
	 *
	 * (1) BKPT cannot be stepped. So just skip it.
	 * (2) Reads from BASEPRI need to be emulated.
	 * (3) CPSID will always hang the system, hence ignore the step request.
	 * (4) Setting PRIMASK or FAULTMASK will always hang the system, hence ignore the step request.
	 * (5) Writes to BASEPRI/BASEPRI_MAX need to be emulated
	 */

	resume = true;
	executing = true;

	if (exception->pc != exception_pc) {
		exception->xpsr &= ~0x0600fc00;
	} else if (exception->xpsr & 0x0000f000) {
		static const unsigned short cc_cond_table[16] = { 0x54aa, 0x686a,
				0x55a6, 0x6966, 0x66a9, 0x6a69, 0x64a5, 0x6865, 0x689a, 0x545a,
				0x6996, 0x5556, 0x6a99, 0x6659, 0x6895, 0x6455, };

		executing = !!(cc_cond_table[(exception->xpsr >> 28) & 15]
				& (1u << ((exception->xpsr & 0x0000f000) >> 12)));
	}

	insn0 = ((unsigned short*) exception->pc)[0];
	insn1 = ((unsigned short*) exception->pc)[1];

	if (stepping) {
		target->stepping = STEPPING_INSTRUCTION;

		if (executing) {
			if ((insn0 & 0xff00) == 0xbe00) {
				/* BKPT, skip always */
				gdb_step(target, exception, 2);

				resume = false;
			} else if ((insn0 & 0xff00) == 0xdf00) {
				/* SVC, handle special */
				target->stepping = STEPPING_SVCALL;
			} else if ((insn0 == 0xf3ef)
					&& (((insn1 & 0xf0ff) == 0x8011)
							|| ((insn1 & 0xf0ff) == 0x8012))) {
				/* "MRS r<d>, BASEPRI/BASEPRI_MAX" needs to be emulated. */

				index = (insn1 & 0x0f00) >> 8;

				location = gdb_register_location(target, context, &system->sp,
						system, index, &size);

				*((unsigned int*) location) = system->basepri;

				gdb_step(target, exception, 4);

				resume = false;
			} else {
				priviledged = !(context->exc_return & 0x00000008)
						|| !(system->control & 0x00000001);

				if ((insn0 & 0xfffc) == 0xb670) {
					/* CPSID writes 1 to PRIMASK/FAULTMASK, step in place */
					if (priviledged) {
						resume = false;
					}
				}

				else if (((insn0 & 0xfff0) == 0xf380)
						&& ((insn1 & 0xf0fc) == 0x8010)) {
					if (priviledged) {
						index = insn0 & 0x000f;

						location = gdb_register_location(target, context,
								&system->sp, system, index, &size);

						switch (insn1 & 0x03) {

						case 0x00:
							/* "MSR PRIMASK, r<n>" when r<n> bit 0 is 1, step in place */
							if (*((unsigned int*) location) & 1) {
								resume = false;
							}
							break;

						case 0x01:
							/* "MSR BASEPRI, r<n>", emulated */
							system->basepri = *((unsigned int*) location)
									& 0xff;

							gdb_step(target, exception, 4);

							resume = false;
							break;

						case 0x02:
							/* "MSR BASEPRI_MAX, r<n>", emulated */
							basepri = *((unsigned int*) location) & 0xff;

							if ((basepri != 0x00)
									&& ((basepri < system->basepri)
											|| (system->basepri == 0x00))) {
								system->basepri = basepri;
							}

							gdb_step(target, exception, 4);

							resume = false;
							break;

						case 0x03:
							/* "MSR FAULTMASK, r<n>" when r<n> bit 0 is 1, step in place */
							if (*((unsigned int*) location) & 1) {
								resume = false;
							}
							break;
						}
					}
				}
			}
		}

		if (resume) {
			/* Use target->scratch_data for data saved/restored across gdb_main invocations.
			 * Make sure though that the context restore logic at the tail of gad_main
			 * does not overwrite this data.
			 */
#if (__FPU_PRESENT == 1)
			dp = (unsigned int*)((unsigned char*)&target->scratch_data[0] + sizeof(gdb_context_fpu_t));
#else /* __FPU_PRESENT */
			dp = (unsigned int*) ((unsigned char*) &target->scratch_data[0]
					+ sizeof(gdb_context_t));
#endif /* __FPU_PRESENT */

			if (target->stepping == STEPPING_INSTRUCTION) {
				*dp++ = system->basepri;

				prio_bits = 7 - ((SCB_AIRCR & 0x00000700) >> 8);

				if (prio_bits > __NVIC_PRIO_BITS) {
					prio_bits = __NVIC_PRIO_BITS;
				}

				system->basepri = 1u << (8 - prio_bits);

				SCB_DEMCR |= SCB_DEMCR_MON_STEP;
			} else {
				/* Save/Disable all NVIC interrupts */
				for (i = 0; i < (target->interrupt_count / 32); i++) {
					*dp++ = NVIC_ISER(i) ;

					NVIC_ICER(i) = ~0u;
				}

				/* Save/Disable SYSTICK interrupts */
				syst_csr = SYST_CSR;

				*dp++ = syst_csr;

				SYST_CSR = syst_csr & ~SYST_CSR_TICKINT;

				/* Save/Clear pending PENDSV & SYSTICK interrupts */
				scb_icsr = SCB_ICSR;

				*dp++ = scb_icsr;

				SCB_ICSR = ((scb_icsr & ~(SCB_ICSR_PENDSVSET |SCB_ICSR_PENDSTSET)) |
			    ((scb_icsr & (SCB_ICSR_PENDSVSET | SCB_ICSR_PENDSTSET)) >> 1));

		/* SVCALL needs to be executed and a breakpoint set at the target address */

		address = ((uint32_t*)SCB_VTOR)[11];

		*dp++ = FP_COMP(0);

		FP_COMP(0) = (((address & 2) ? FP_COMP_REPLACE_16BIT_HI : FP_COMP_REPLACE_16BIT_LO) |
			      (address & ~3) |
			      FP_COMP_ENABLE);
	    }
				}
				else
				{
					target->stepping = STEPPING_NONE;
				}
			}
			else
			{
				target->stepping = STEPPING_NONE;

				if ((insn0 & 0xff00) == 0xbe00)
				{
					/* BKPT, skip always */
					gdb_step(target, exception, 2);
				}
			}

	return resume;
}

/******************************************************************************/

#if (_GDB_CONFIG_KERNEL_LITHIUM == 1)

static char * gdb_query_thread_info(gdb_target_t *target, unsigned int *query_thread)
{
	char *reply, *cp;
	unsigned int thread;

	thread = *query_thread;

	reply = cp = (char*)&target->scratch_data[0];

	*cp++ = 'm';

	while (thread <= _kernel_info.max_tsk)
	{
		if (_kernel_tsk_info.tsk_state[thread] & (_KERNEL_TSK_STATE_ACTIVATED | _KERNEL_TSK_STATE_TEX_ENABLED))
		{
			if (cp != &reply[1])
			{
				*cp++ = ',';
			}

			cp = gdb_pack_unsigned(cp, thread);

			if ((cp - reply) > (SCRATCH_DATA_SIZE - 10))
			{
				break;
			}
		}

		thread++;
	}

	/* LITHIUM has no idle thread, but an idle routine. Fake it out as thread if it's active */
	if ((thread == (_kernel_info.max_tsk + 1)) && ((cp - reply) <= (SCRATCH_DATA_SIZE - 10)))
	{
		thread = _KERNEL_IDLE_THREAD;

		if (_kernel_control.tsk_self == 0)
		{
			if (cp != &reply[1])
			{
				*cp++ = ',';
			}

			cp = gdb_pack_unsigned(cp, thread);
		}
	}

	if (cp == &reply[1])
	{
		reply[0] = 'l';
		reply[1] = '\0';
	}

	*query_thread = thread;

	return reply;
}

#endif /* _GDB_CONFIG_KERNEL_LITHIUM */

/******************************************************************************/

unsigned int gdb_main(unsigned int exception_code, gdb_context_t *context,
		gdb_system_t *system, gdb_target_t *target) {
	gdb_system_t *general_system;
	gdb_context_t *general_context;
	gdb_exception_t *exception, *general_exception;
	unsigned int context_size, exception_sp, exception_pc, exception_xpsr,
			register_xpsr;
	unsigned int reason, address, length, index, size, i;
	unsigned char *location;
	unsigned int *dp;
	char command;
	char *packet, *reply, *cp;
	bool resume, success;
	char type;
	uint32_t *general_sp_read, *general_sp_write;
#if (_GDB_CONFIG_KERNEL_LITHIUM == 1)
	unsigned int thread, exception_thread, query_thread;
#endif /* _GDB_CONFIG_KERNEL_LITHIUM */

	/* Unwind MSP/PSP and exception_sp, which is the sp before the debug exception */

#if (__FPU_PRESENT == 1)
	if (!(context->exc_return & 0x00000010))
	{
		exception = (gdb_exception_t*)((unsigned char*)context + (sizeof(gdb_context_fpu_t) - sizeof(gdb_exception_fpu_t)));

		context_size = sizeof(gdb_context_fpu_t);
	}
	else
#endif /* __FPU_PRESENT */
	{
		exception = (gdb_exception_t*) ((unsigned char*) context
				+ (sizeof(gdb_context_t) - sizeof(gdb_exception_t)));

		context_size = sizeof(gdb_context_t);
	}

	exception_pc = exception->pc;
	exception_xpsr = exception->xpsr;

	exception_sp = system->sp;
	exception_sp += context_size;

	if (exception_xpsr & 0x00000200) {
		exception_sp += 0x04;
	}

	system->sp = exception_sp;
	system->exc_continue = context->exc_return;

	if (context->exc_return & 0x00000004) {
		system->psp = exception_sp;
	} else {
		system->msp = exception_sp;
	}

#if (_GDB_CONFIG_KERNEL_LITHIUM == 1)
	query_thread = _KERNEL_IDLE_THREAD;

	exception_thread = (_kernel_control.tsk_self == TSK_NONE) ? _KERNEL_IDLE_THREAD : _kernel_control.tsk_self;
#endif /* _GDB_CONFIG_KERNEL_LITHIUM */

	general_context = context;
	general_exception = exception;
	general_system = system;
	general_sp_read = &system->sp;
	general_sp_write = &system->sp;

	if (!target->attached) {
		gdb_breakpoint_initialize(target);
		gdb_watchpoint_initialize(target);
	}

#if (_GDB_CONFIG_CONSOLE == 1)
	target->serial_interrupted = false;
#endif /* _GDB_CONFIG_CONSOLE */

	switch (exception_code) {
	case 4:
		/* MemManage */
		reason = REASON_SEGV;
		break;

	case 5:
		/* BusFault */
		reason = REASON_BUS;
		break;

	case 6:
		/* UsageFault */
		if (SCB_CFSR & SCB_CFSR_DIVBYZERO) {
			reason = REASON_FPE;
		} else if (SCB_CFSR & SCB_CFSR_UNALIGNED) {
			reason = REASON_BUS;
		} else {
			/* NOCP, INVPC, INVSTATE, UNDEFINSTR */
			reason = REASON_ILL;
		}
		break;

	default:
		/* DebugMonitor */
		if (target->serial_data_pending == '$') {
			reason = REASON_NONE;
		} else {
			if (target->serial_data_pending == '\003') {
				reason = REASON_INT;
			} else {
				reason = REASON_TRAP;
			}
		}
		break;
	}

	/* Here we get either throu the serial interrupt, or and explicite
	 * debug interrupt, or a retry from an emulated step.
	 */
	retry: if (reason != REASON_NONE) {
		/* Send a 'T' reply with all registers listed as "expedite" in the register description */
		reply = cp = (char*) &target->scratch_data[0];
		*cp++ = 'T';
		*cp++ = gdb_pack_nibble(reason >> 4);
		*cp++ = gdb_pack_nibble(reason & 15);

		cp = gdb_pack_unsigned(cp, REGISTER_INDEX_R11);
		*cp++ = ':';
		cp = gdb_pack_data(cp, (unsigned char*) &context->r11, 4);
		*cp++ = ';';

		cp = gdb_pack_unsigned(cp, REGISTER_INDEX_SP);
		*cp++ = ':';
		cp = gdb_pack_data(cp, (unsigned char*) &exception_sp, 4);
		*cp++ = ';';

		cp = gdb_pack_unsigned(cp, REGISTER_INDEX_PC);
		*cp++ = ':';
		cp = gdb_pack_data(cp, (unsigned char*) &exception_pc, 4);
		*cp++ = ';';

#if (_GDB_CONFIG_KERNEL_LITHIUM == 1)
		*cp++ = 't';
		*cp++ = 'h';
		*cp++ = 'r';
		*cp++ = 'e';
		*cp++ = 'a';
		*cp++ = 'd';
		*cp++ = ':';
		cp = gdb_pack_unsigned(cp, exception_thread);
		*cp++ = ';';
#endif /* _GDB_CONFIG_KERNEL_LITHIUM */

		if ((reason == REASON_TRAP) && gdb_watchpoint_check(target, &address)) {
			*cp++ = 'w';
			*cp++ = 'a';
			*cp++ = 't';
			*cp++ = 'c';
			*cp++ = 'h';
			*cp++ = ':';
			cp = gdb_pack_data(cp, (unsigned char*) &address, 4);
			*cp++ = ';';
		}

		*cp++ = '\0';

		target->serial_data_pending = '\0';

		gdb_packet_send(target, reply, true);
	} else {
		/* Leave gdb_serial_data_pending in place, so the gdb_packet_receive() can
		 * see it and do the right thing.
		 */
	}

	/*
	 * If we get here while single stepping, the saved NVIC/BASEPRI values
	 * need to be restored.
	 */

	if (target->stepping != STEPPING_NONE) {
		/* Use target->scratch_data for data saved/restored across gdb_main invocations.
		 * Make sure though that the context restore logic at the tail of gad_main
		 * does not overwrite this data.
		 */
#if (__FPU_PRESENT == 1)
		dp = (unsigned int*)((unsigned char*)&target->scratch_data[0] + sizeof(gdb_context_fpu_t));
#else /* __FPU_PRESENT */
		dp = (unsigned int*) ((unsigned char*) &target->scratch_data[0]
				+ sizeof(gdb_context_t));
#endif /* __FPU_PRESENT */

		if (target->stepping == STEPPING_INSTRUCTION) {
			system->basepri = *dp++;
		} else {
			for (i = 0; i < (target->interrupt_count / 32); i++) {
				NVIC_ISER(i) = *dp++;
			}

			SYST_CSR = *dp++;

			SCB_ICSR |= (*dp++ & (SCB_ICSR_PENDSVSET | SCB_ICSR_PENDSTSET));

			if (target->stepping == STEPPING_SVCALL)
			{
				FP_COMP(0) = *dp++;
			}
		}

		target->stepping = STEPPING_NONE;
	}

	SCB_DEMCR &= ~(SCB_DEMCR_MON_PEND | SCB_DEMCR_MON_STEP);

	do {
		target->attached = true;

		packet = gdb_packet_receive(target, &length);
		command = *packet++;

		resume = false;

		success = false;
		reply = NULL;

		switch (command) {

		/* MANDATORY */
		case '?':
			reply = cp = (char*) &target->scratch_data[0];
			*cp++ = 'S';
			*cp++ = gdb_pack_nibble(reason >> 4);
			*cp++ = gdb_pack_nibble(reason & 15);
			*cp++ = '\0';
			break;

		case 'c':
		case 's':
			/* cAA..AA    Continue at address AA..AA (optional) */
			/* sAA..AA    Continue at address AA..AA (optional) */
			if ((packet = gdb_unpack_unsigned(packet, &address))) {
				exception->pc = address;
			}

			if (!gdb_continue(target, exception, context, system, exception_pc,
					(command == 's'))) {
				reason = REASON_TRAP;

				exception_pc = exception->pc;

				goto retry;
			}

			resume = true;
			break;

		case 'g':
			reply = cp = (char*) &target->scratch_data[0];

			for (index = 0; index < REGISTER_INDEX_COUNT; index++) {
				location = gdb_register_location(target, general_context,
						general_sp_read, general_system, index, &size);

				if (location) {
					cp = gdb_pack_data(cp, (unsigned char*) location, size);
				} else if (size) {
					while (size--) {
						*cp++ = 'x';
						*cp++ = 'x';
					}

				}
			}
			break;

		case 'G':
			for (index = 0; index < REGISTER_INDEX_COUNT; index++) {
				if (index == REGISTER_INDEX_XPSR) {
					packet = gdb_unpack_data(packet,
							(unsigned char*) &register_xpsr, 4);

					general_exception->xpsr = (general_exception->xpsr
							& ~0xf80f0000) | (register_xpsr & 0xf80f0000);
				} else {
					location = gdb_register_location(target, general_context,
							general_sp_write, general_system, index, &size);

					if (location) {
						packet = gdb_unpack_data(packet, location, size);
					} else {
						packet += (size * 2);
					}
				}
			}

			success = true;
			break;

		case 'm':
			/* mAA..AA,LLLL  Read LLLL bytes at address AA..AA */
			if ((packet = gdb_unpack_unsigned(packet, &address))
					&& (*packet++ == ',')
					&& (packet = gdb_unpack_unsigned(packet, &length))) {
				SCB_CFSR = SCB_CFSR_BFSR_MASK;

				reply = (char*) &target->scratch_data[0];

				gdb_pack_data(reply, (unsigned char*) address, length);

				if (SCB_CFSR & SCB_CFSR_BFSR_MASK) {
					reply = NULL;
				}
			}
			break;

		case 'M':
			/* MAA..AA,LLLL: Write LLLL bytes at address AA.AA return OK */
			if ((packet = gdb_unpack_unsigned(packet, &address))
					&& (*packet++ == ',')
					&& (packet = gdb_unpack_unsigned(packet, &length))
					&& (*packet++ == ':')) {
				SCB_CFSR = SCB_CFSR_BFSR_MASK;

				gdb_unpack_data(packet, (unsigned char*) address, length);

				if (!(SCB_CFSR & SCB_CFSR_BFSR_MASK)) {
					success = true;
				}
			}
			break;

		case 'k':
			SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;

			while (1) {
			}
			break;

			/* OPTIONAL */

		case 'D':
			target->attached = false;
			success = true;
			break;

#if (_GDB_CONFIG_KERNEL_LITHIUM == 1)
			case 'H':
			if ((type = *packet++))
			{
				if (type == 'c')
				{
					/* The real value can be ignored. In all-stop mode
					 * GDB only every sends -1 or 0, which is in reality
					 * identical to "continue-with-all-threads".
					 */
					success = true;
				}
				else if (type == 'g')
				{
					if ((packet = gdb_unpack_unsigned(packet, &thread)))
					{
						if (thread == 0)
						{
							thread = (_kernel_control.tsk_self == TSK_NONE) ? _KERNEL_IDLE_THREAD : _kernel_control.tsk_self;
							success = true;
						}
						else if (thread != _KERNEL_IDLE_THREAD)
						{
							success = ((thread <= _kernel_info.max_tsk) &&
									(_kernel_tsk_info.tsk_state[thread] & (_KERNEL_TSK_STATE_ACTIVATED | _KERNEL_TSK_STATE_TEX_ENABLED)));
						}

						if (success)
						{
							if (thread == exception_thread)
							{
								general_context = context;
								general_exception = exception;
								general_system = system;
								general_sp_read = &system->sp;
								general_sp_write = &system->sp;
							}
							else
							{
								general_context = (gdb_context_t*)_kernel_tsk_info.tsk_ctx[thread];
								general_system = NULL;
								general_sp_read = &_kernel_tsk_info.tsk_ctx[thread];
								general_sp_write = NULL;

#if (__FPU_PRESENT == 1)
								if (!(context->exc_return & 0x00000010))
								{
									general_exception = (gdb_exception_t*)&((gdb_context_fpu_t*)general_context)->r0;
								}
								else
#endif /* __FPU_PRESENT */
								{
									general_exception = (gdb_exception_t*)&((gdb_context_t*)general_context)->r0;
								}
							}
						}
					}
				}
				else
				{
					reply = "";
				}
			}
			break;
#endif /* _GDB_CONFIG_KERNEL_LITHIUM */

		case 'p':
			/* pNN..NN  Read register NN..NN */
			if ((packet = gdb_unpack_unsigned(packet, &index))) {
				location = gdb_register_location(target, general_context,
						general_sp_read, general_system, index, &size);

				if (location) {
					reply = cp = (char*) &target->scratch_data[0];

					cp = gdb_pack_data(cp, (unsigned char*) location, size);
				} else if (size) {
					reply = cp = (char*) &target->scratch_data[0];

					while (size--) {
						*cp++ = 'x';
						*cp++ = 'x';
					}
				}
			}
			break;

		case 'P':
			/* PNN..NN:  Write register NN..NN return OK */
			if ((packet = gdb_unpack_unsigned(packet, &index))
					&& (*packet++ == ':')) {
				if (index == REGISTER_INDEX_XPSR) {
					packet = gdb_unpack_data(packet,
							(unsigned char*) &register_xpsr, 4);

					general_exception->xpsr = (general_exception->xpsr
							& ~0xf80f0000) | (register_xpsr & 0xf80f0000);

					success = true;
				} else {
					location = gdb_register_location(target, general_context,
							general_sp_write, general_system, index, &size);

					if (location) {
						gdb_unpack_data(packet, location, size);

						success = true;
					} else if (size) {
						success = true;
					}
				}
			}
			break;

		case 'q':
			if (gdb_scan_token(&packet, "Attached")) {
				reply = "1";
			} else if (gdb_scan_token(&packet, "Supported")) {
				/* Support the packet size query, as per default the gdb remote assumes 400 bytes,
				 * which is not enough to supoort the 'g' and 'G' packets.
				 */
				reply = "PacketSize=01ff"
#if (_GDB_CONFIG_TARGET_DESCRIPTION == 1)
								";qXfer:features:read+"
#endif /* _GDB_CONFIG_TARGET_DESCRIPTION */
						";qXfer:memory-map:read+";
			}
#if (_GDB_CONFIG_TARGET_DESCRIPTION == 1)
			else if (gdb_scan_token(&packet, "Xfer:features:read:target.xml")
					&& (*packet++ == ':')
					&& (packet = gdb_unpack_unsigned(packet, &address))
					&& (*packet++ == ',')
					&& (packet = gdb_unpack_unsigned(packet, &length))) {
				reply = gdb_reply_string(target, gdb_target_description,
						address, length);
			}
#endif /* _GDB_CONFIG_TARGET_DESCRIPTION */
			else if (gdb_scan_token(&packet, "Xfer:memory-map:read:")
					&& (*packet++ == ':')
					&& (packet = gdb_unpack_unsigned(packet, &address))
					&& (*packet++ == ',')
					&& (packet = gdb_unpack_unsigned(packet, &length))) {
				reply = gdb_reply_string(target, gdb_memory_map, address,
						length);
			}
#if (_GDB_CONFIG_KERNEL_LITHIUM == 1)
			else if (gdb_scan_token(&packet, "C"))
			{
				reply = cp = (char*)&target->scratch_data[0];

				*cp++ = 'Q';
				*cp++ = 'C';
				cp = gdb_pack_unsigned(cp, exception_thread);
				*cp++ = '\0';
			}
			else if (gdb_scan_token(&packet, "fThreadInfo"))
			{
				query_thread = 1;

				reply = gdb_query_thread_info(target, &query_thread);
			}
			else if (gdb_scan_token(&packet, "sThreadInfo"))
			{
				reply = gdb_query_thread_info(target, &query_thread);
			}
			else if (gdb_scan_token(&packet, "ThreadExtraInfo") &&
					(*packet++ == ',') &&
					(packet = gdb_unpack_unsigned(packet, &thread)))
			{
				reply = cp = (char*)&target->scratch_data[0];

				if (thread == _KERNEL_IDLE_THREAD)
				{
					cp = gdb_pack_string(cp, "IDLE_ROUTINE");
				}
				else
				{
					cp = gdb_pack_string(cp, (const char*)_kernel_tsk_info.tsk_ro_name[thread]);
				}
			}
#endif /* _GDB_CONFIG_KERNEL_LITHIUM */
			else {
				reply = "";
			}
			break;

#if (_GDB_CONFIG_KERNEL_LITHIUM == 1)
			case 'T':
			/* A thread is alive for GDB is it has a stack frame. For LITHIUM that means that it's
			 * not DORMANT and it has been activated.
			 */
			if ((packet[0] == '-') && (packet[0] == '1'))
			{
				success = (_kernel_control.tsk_self == TSK_NONE);
			}
			else if ((packet = gdb_unpack_unsigned(packet, &thread)))
			{
				if (exception_thread == _KERNEL_IDLE_THREAD)
				{
					success = (_kernel_control.tsk_self == TSK_NONE);
				}
				else
				{
					success = ((thread == 0) ||
							((thread <= _kernel_info.max_tsk) &&
									(_kernel_tsk_info.tsk_state[thread] & (_KERNEL_TSK_STATE_ACTIVATED | _KERNEL_TSK_STATE_TEX_ENABLED))));
				}
			}
			break;
#endif /* _GDB_CONFIG_KERNEL_LITHIUM */

		case 'z':
		case 'Z':
			if ((type = *packet++)) {
				if ((*packet++ == ',')
						&& (packet = gdb_unpack_unsigned(packet, &address))
						&& (*packet++ == ',')
						&& (packet = gdb_unpack_unsigned(packet, &size))) {
					switch (type) {
					case '1':
						if ((size == 2) || (size == 3)) {
							success = gdb_breakpoint_change(target, address,
									((size - 2) + BREAKPOINT_TYPE_16BIT),
									(command == 'Z'));
						}
						break;
					case '2':
					case '3':
					case '4':
						success = gdb_watchpoint_change(target, address, size,
								((type - '2') + WATCHPOINT_TYPE_WRITE),
								(command == 'Z'));
						break;
					default:
						reply = "";
						break;
					}
				}
			}
			break;
		default:
			reply = "";
			break;
		}

		if (!resume) {
			if (reply == NULL) {
				reply = (success ? "OK" : "E01");
			}

			gdb_packet_send(target, reply, true);
		}
	} while (target->attached && !resume);

#if (_GDB_CONFIG_CONSOLE == 1)
	target->serial_interrupted = true;
#endif /* _GDB_CONFIG_CONSOLE */

	/*
	 * If the sp has been changed by gdb, then copy the context over to
	 * target->scratch_data[] and set system->count to context_size.
	 */

	if (exception_sp != system->sp) {
		exception_sp = system->sp & ~3;

		exception->xpsr &= ~0x00000200;

		if ((exception_sp & 4) && (
#if (__FPU_PRESENT == 1)
				!(context->exc_return & 0x00000010) ||
#endif /* __FPU_PRESENT */
				(SCB_CCR & SCB_CCR_STKALIGN))) {
			exception->xpsr |= 0x00000200;
		}

		dp = &target->scratch_data[0];

		for (i = 0; i < (context_size / 4); i++) {
			*dp++ = ((unsigned int*) context)[i];
		}

		size = context_size;
	} else {
		size = 0;
	}

	/* Properly recompute MSP/PSP based upon the return sp */

	if (exception->xpsr & 0x00000200) {
		exception_sp -= 4;
	}

	exception_sp -= context_size;

	if (context->exc_return & 0x00000004) {
		system->psp = exception_sp;
	} else {
		system->msp = exception_sp;
	}

	system->sp = exception_sp;

	return size;
}

void gdb_exception(unsigned int code, gdb_exception_t *exception) {
	unsigned int insn0;

	if (SCB_SHCSR & SCB_SHCSR_MONITORACT) {
		if (SCB_CFSR & SCB_CFSR_PRESCISERR) {
			insn0 = ((unsigned short*) exception->pc)[0];

			if (((insn0 & 0xf800) == 0x7800) || ((insn0 & 0xf800) == 0xf000)
					|| ((insn0 & 0xf800) == 0xf800)) {
				exception->pc += 4;
			} else {
				exception->pc += 2;
			}

			return;
		}

		if (SCB_CFSR & SCB_CFSR_IMPRESCISERR) {
			return;
		}
	}

	while (1) {
	}
}

/******************************************************************************/
extern int gdb_serial_read();
extern void gdb_serial_write(int c);

//void gdb_initialize(gdb_serial_read_t serial_read, gdb_serial_write_t serial_write) {
void gdb_initialize() {
	SCB_SHPR3 &= ~0xff;

	SCB_DEMCR &= ~(SCB_DEMCR_MON_PEND | SCB_DEMCR_MON_STEP);
	SCB_DEMCR |= SCB_DEMCR_MON_EN;

	SCB_SHCSR |= (SCB_SHCSR_USGFAULTENA | SCB_SHCSR_BUSFAULTENA
			| SCB_SHCSR_MEMFAULTENA);

	gdb_target.attached = false;
	gdb_target.interrupt_count = 32 * ((SCB_ICTR & 15) + 1);
	gdb_target.serial_interrupted = false;
	gdb_target.serial_data_pending = '\0';
	gdb_target.serial_read = gdb_serial_read;
	gdb_target.serial_write = gdb_serial_write;
}

#if (_GDB_CONFIG_CONSOLE == 1)

void gdb_console(const char *s) {
	char *reply, *cp;
	unsigned int remaining, data;
	gdb_target_t *target = &gdb_target;

	target->serial_interrupted = false;

	if (target->attached) {
		while (*s != '\0') {
			reply = cp = (char*) &target->scratch_data[0];
			*cp++ = 'O';

			remaining = (SCRATCH_DATA_SIZE / 2) - 1;

			while (remaining && (*s != '\0')) {
				data = *s++;
				*cp++ = gdb_pack_nibble(data >> 4);
				*cp++ = gdb_pack_nibble(data & 15);
				remaining--;
			}

			*cp++ = '\0';

			gdb_packet_send(target, reply, false);

			if (target->serial_interrupted) {
				return;
			}
		}
	} else {
		while (*s != '\0') {
			target->serial_write(*s++);

			if (target->serial_interrupted) {
				return;
			}
		}
	}
}

#endif /* _GDB_CONFIG_CONSOLE */

/* Wrapper to set up the debug_frame properly for gdb_debug_monitor() */
void __attribute__((naked)) DebugMon_Handler(void) {
	__asm__(
			"   tst      lr, #0x00000004             \n"
			"   itte     eq                          \n"
			"   moveq    r1, sp                      \n"
			"   subeq    sp, #0x28                   \n" /* adjust MSP for R4-R11, exc_return, plus 8 byte padding */
			"   mrsne    r1, PSP                     \n"
#if (__FPU_PRESENT == 1)
			"   tst      lr, #0x00000010             \n"
			"   beq.n    1f                          \n"
			"   tst      lr, #0x00000004             \n"
			"   it       eq                          \n"
			"   subeq    sp, #0x40                   \n" /* adjust for D8-D15 */
			"   vstmdb   r1!, { d8-d15 }             \n"
#endif /* __FPU_PRESENT */
			"1: stmdb    r1!, { r4-r11, lr }         \n"
			"   mrs      r3, MSP                     \n"
			"   mrs      r4, PSP                     \n"
			"   mrs      r5, PRIMASK                 \n"
			"   mrs      r6, BASEPRI                 \n"
			"   mrs      r7, FAULTMASK               \n"
			"   mrs      r8, CONTROL                 \n"
			"   push     { r1-r8 }                   \n" /* /R1/R2 are scratch */
			"   mrs      r0, IPSR                    \n"
			"   mov      r2, sp                      \n"
			"   ldr      r3, =gdb_target             \n"
			"   bl       gdb_main                    \n"
			"   pop      { r1-r8 }                   \n" /* R0 is "size", R1 "context", and R2 "exc_continue" */
			"   msr      MSP, r3                     \n" /* MSP/PSP contain non-unwound values   */
			"   msr      PSP, r4                     \n"
			"   msr      PRIMASK, r5                 \n"
			"   msr      BASEPRI, r6                 \n"
			"   msr      FAULTMASK, r7               \n"
			"   msr      CONTROL, r8                 \n"
			"   isb                                  \n"
			"   cbz.n    r0, 3f                      \n"
			"   ldr      r3, =gdb_target             \n"
			"   adds     r3, %0                      \n"
			"   movs     r4, r1                      \n"
			"2: ldr      r5, [r3], #4                \n"
			"   subs     r0, #4                      \n"
			"   str      r5, [r4], #4                \n"
			"   bne.n    2b                          \n"
			"3: ldmia    r1!, { r4-r11, lr }         \n"
#if (__FPU_PRESENT == 1)
			"   tst      lr, #0x00000010             \n"
			"   it       eq                          \n"
			"   vldmiaeq r1!, { d8-d15 }             \n"
#endif /* __FPU_PRESENT */
			"   tst      lr, #00000004               \n"
			"   ite      eq                          \n"
			"   moveq    sp, r1                      \n"
			"   msrne    PSP, r1                     \n"
			"   bx       r2                          \n"
			:
			: "I" (offsetof(gdb_target_t, scratch_data))
			:
	);
}

void __attribute__((naked)) HardFault_Handler(void) {
	__asm__(
			"   mrs     r0, IPSR                    \n"
			"   tst     lr, #00000004               \n"
			"   ite     eq                          \n"
			"   moveq   r1, sp                      \n"
			"   mrsne   r1, PSP                     \n"
			"   b       gdb_exception               \n"
			:
			:
			:
	);
}

void gdb_serial_interrupt(int data) {
	gdb_target_t *target = &gdb_target;

	if ((data != '\003') && (data != '$')) {
#if (_GDB_CONFIG_CONSOLE == 1)
		if (target->serial_data_pending == '\0') {
			target->serial_data_pending = data;
		}
#endif /* _GDB_CONFIG_CONSOLE */
		return;
	}

	target->serial_data_pending = data;

	SCB_DEMCR |= SCB_DEMCR_MON_PEND;
}

/******************************************************************************/
