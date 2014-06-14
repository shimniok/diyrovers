/*
 * config.h
 *
 *  Created on: May 7, 2014
 *      Author: mes
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/** Configure device assignments */

#include "mbed.h"
#include "boards.h"
#include "SerialTextLCD.h"

/** GPS */
#define GPSTX			UART1TX
#define GPSRX			UART1RX

/** Telemetry */
#define TELEMTX			UART2TX
#define TELEMRX			UART2RX

/** LCD */
#define LCDTX			UART3TX
#define LCDRX			UART3RX

/** UI Buttons */
#define BUTTONNEXT		IO0
#define BUTTONPREV		IO1
#define BUTTONSELECT	PW0

/** Horn */
#define HORN			RXSTR
extern DigitalOut horn;

/** Brake */
#define BRAKE			RXSPD
#define BRAKE_DISABLE	1
#define BRAKE_ENABLE	0
extern DigitalOut brake;

#endif /* CONFIG_H_ */
