/*
 * Telemetry.h
 *
 *  Created on: May 6, 2014
 *      Author: mes
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "mbed.h"
#include "SystemState.h"

class Telemetry {
public:
	Telemetry(Serial &uart);
	void baud(int baud);
	void sendPacket(SystemState *s);
private:
	Serial *_uart;
};

#endif /* TELEMETRY_H_ */
