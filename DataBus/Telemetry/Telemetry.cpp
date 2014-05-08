/*
 * Telemetry.cpp
 *
 *  Created on: May 6, 2014
 *      Author: mes
 */

#include "mbed.h"
#include "Telemetry.h"
#include "util.h"

Telemetry::Telemetry(Serial &uart) {
	_uart = &uart;
}

void Telemetry::baud(int baud) {
    _uart->baud(115200);
}

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

		// TODO: get rid of printf
		_uart->puts("`");
		_uart->puts(cvntos(s->millis));
		_uart->puts(", ");
		_uart->puts(cvftos(s->voltage, 2));
		_uart->puts(", ");
		_uart->puts(cvftos(s->current, 2));
		_uart->puts(", ");
		_uart->puts(cvftos(s->estHeading, 2));
		_uart->puts(", ");
		_uart->puts(cvftos(s->gpsLatitude, 7));
		_uart->puts(", ");
		_uart->puts(cvftos(s->gpsLongitude, 7));
		_uart->puts(", ");
		_uart->puts(cvftos(s->gpsHDOP, 1));
		_uart->puts(", ");
		_uart->puts(cvitos(s->gpsSats));
		_uart->puts(", ");
		_uart->puts(cvftos((s->lrEncSpeed + s->rrEncSpeed)/2.0, 1));
		_uart->puts(", ");
		_uart->puts(cvftos(bearing, 2));
		_uart->puts(", ");
		_uart->puts(cvftos(s->distance, 5));
		_uart->puts(", ");
		_uart->puts(cvftos(s->steerAngle, 2));
		_uart->puts("\n");
	}
}

