/*
 * Telemetry.h
 *
 *  Created on: May 6, 2014
 *      Author: mes
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "mbed.h"

class Telemetry {
public:
	Telemetry(Serial &uart);
	void sendPacket();
private:
	Serial *_uart;
};

#endif /* TELEMETRY_H_ */
