/*
 * boards.h
 *
 *  Created on: Mar 25, 2014
 *      Author: mes
 */

#ifndef BOARDS_H_
#define BOARDS_H_

/** RoverBaseboard R0.5 */
#define I2CSDA 		p9
#define I2CSCL 		p10
#define UART0TX 	USBTX /** mbed */
#define UART0RX 	USBRX /** mbed */
#define UART1TX		p13
#define UART1RX		p14
#define UART2TX 	p28
#define UART2RX 	p27
#define UART3TX 	p17
#define UART3RX 	p18
#define ENCAL 		p30
#define ENCAR 		p29
#define ENCBL 		/** unavailable for mbed */
#define ENCBL 		/** unavailable for mbed */
#define D0 p11
#define D1 p12
#define D2			/** unavailable for mbed */
#define D7			/** unavailable for mbed */
#define D8			/** unavailable for mbed */
#define D9			/** unavailable for mbed */
#define P1 			p26
#define P2 			p25
#define A0 			p15
#define A1 			p16
#define A2 			p19
#define A3 			p20

#endif /* BOARDS_H_ */
