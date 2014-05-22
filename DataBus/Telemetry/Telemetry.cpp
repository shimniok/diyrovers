/*
 * Telemetry.cpp
 *
 *  Created on: May 6, 2014
 *      Author: mes
 */

#include "mbed.h"
#include "Telemetry.h"
#include "util.h"

/** Create a new telemetry object
 *
 * @param uart is the Serial object used to send data
 */
Telemetry::Telemetry(Serial &uart) {
	_uart = &uart;
}

/** Set baud rate for the serial connection
 *
 * @param baud is the integer baud rate
 */
void Telemetry::baud(int baud) {
    _uart->baud(115200);
}

/** Send waypoints to the GCS
 *
 * @param wpt is the array of CartPosition waypoints
 */
void Telemetry::sendPacket(CartPosition wpt[], int wptCount) {
	if (wpt) {
		_uart->puts("`01, "); // waypoint message
		_uart->puts(cvitos(wptCount));
		_uart->puts(",");
		for (int i=0; i < wptCount; i++) {
			_uart->puts(cvftos(wpt[i].x, 5));
			_uart->puts(",");
			_uart->puts(cvftos(wpt[i].y, 5));
			if (i+1 < wptCount) _uart->puts(",");
		}
		_uart->puts("\n");
	}
}

/** Send system state to the GCS
 *
 * @param s is the SystemState object to send
 */
void Telemetry::sendPacket(SystemState *s) {
	if (s) {
		// Bearing is given as absolute; we want to send relative bearing
		float bearing = s->bearing - s->estHeading;
		while (bearing >= 360.0) {
			bearing -= 360.0;
		}
		while (bearing < 0) {
			bearing += 360.0;
		}

		_uart->puts("`00, "); // standard status message
		_uart->puts(cvntos(s->millis));
		_uart->puts(", ");
		_uart->puts(cvftos(s->voltage, 2));
		_uart->puts(", ");
		_uart->puts(cvftos(s->current, 2));
		_uart->puts(", ");
		_uart->puts(cvftos(s->estHeading, 2));
		_uart->puts(", ");
//		_uart->puts(cvftos(s->gpsLatitude, 7));
		_uart->puts(cvftos(s->estX, 5));
		_uart->puts(", ");
//		_uart->puts(cvftos(s->gpsLongitude, 7));
		_uart->puts(cvftos(s->estY, 5));
		_uart->puts(", ");
		_uart->puts(cvftos(s->gpsHDOP, 1));
		_uart->puts(", ");
		_uart->puts(cvitos(s->gpsSats));
		_uart->puts(", ");
		_uart->puts(cvftos((s->lrEncSpeed + s->rrEncSpeed)/2.0, 1));
		_uart->puts(", ");
		_uart->puts(cvitos(s->nextWaypoint));
		_uart->puts(", ");
		_uart->puts(cvftos(bearing, 2));
		_uart->puts(", ");
		_uart->puts(cvftos(s->distance, 5));
		_uart->puts(", ");
		_uart->puts(cvftos(s->steerAngle, 2));
		_uart->puts(", ");
		_uart->puts(cvftos(s->LABrg, 1));
		_uart->puts(", ");
		_uart->puts(cvftos(s->LAx, 2));
		_uart->puts(", ");
		_uart->puts(cvftos(s->LAy, 2));
		_uart->puts("\n");
	}
}

