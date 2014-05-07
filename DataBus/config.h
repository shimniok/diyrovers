/*
 * config.h
 *
 *  Created on: May 7, 2014
 *      Author: mes
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/** Configure device assignments */

#include "boards.h"

/** GPS */
#define GPSTX			UART1TX
#define GPXRX			UART1RX

/** Telemetry */
#define TELEMTX			UART2TX
#define TELEMRX			UART2RX

/** LCD */
#define LCDTX			UART3TX
#define LCDRX			UART3RX

/** UI Buttons */
#define BUTTONLEFT		P1
#define BUTTONRIGHT		D1
#define BUTTONSELECT	D0

#endif /* CONFIG_H_ */
