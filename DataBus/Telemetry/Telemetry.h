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
#include "CartPosition.h"

class Telemetry {
public:

	/** Create a new telemetry object
	 *
	 * @param uart is the Serial object used to send data
	 */
	Telemetry(Serial &uart);

	/** Set baud rate for the serial connection
	 *
	 * @param baud is the integer baud rate
	 */
	void baud(int baud);

	/** Send waypoints to the GCS
	 *
	 * @param wpt is the array of CartPosition waypoints
	 */
	void sendPacket(SystemState *s);

	/** Send waypoints to the GCS
	 *
	 * @param wpt is the array of CartPosition waypoints
	 */
	void sendPacket(CartPosition wpt[], int wptCount);

private:
	Serial *_uart;
};

#endif /* TELEMETRY_H_ */
