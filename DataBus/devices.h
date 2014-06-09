/*
 * devices.h
 *
 * Configure device assignments and device objects
 *
 *  Created on: May 7, 2014
 *      Author: mes
 */

#ifndef DEVICES_H_
#define DEVICES_H_

/** Device Objects */
#include "L3G4200D.h"

extern L3G4200D gyro;

#include "mbed.h"
#include "boards.h"

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
#define BUTTONNEXT		_D1
#define BUTTONPREV		_P1
#define BUTTONSELECT	_D0

#endif /* CONFIG_H_ */
