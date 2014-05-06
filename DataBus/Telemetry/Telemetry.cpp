/*
 * Telemetry.cpp
 *
 *  Created on: May 6, 2014
 *      Author: mes
 */

#include "mbed.h"
#include "Telemetry.h"
#include "SystemState.h"
#include "util.h"

Telemetry::Telemetry(Serial &uart) {
	_uart = &uart;
}

void Telemetry::sendPacket() {
	SystemState *s = fifo_pull();

	if (s) {
		float bearing = s->bearing - s->estHeading;
		while (bearing >= 360.0) {
			bearing -= 360.0;
		}
		while (bearing < 0) {
			bearing += 360.0;
		}

		// TODO: get rid of printf
		_uart->puts("^");
		_uart->puts(cvntos(s->millis));
		_uart->puts(",");
		_uart->puts(cvftos(s->voltage, 2));
		_uart->puts(",");
		_uart->puts(cvftos(s->current, 2));
		_uart->puts(",");
		_uart->printf("%.2f, %.7f, %.7f, %.1f, %d, ",
				s->estHeading,
				s->gpsLatitude, s->gpsLongitude,
				s->gpsHDOP, s->gpsSats );
		_uart->printf("%.1f, ", (s->lrEncSpeed + s->rrEncSpeed)/2.0);
		_uart->printf("%.2f, %.5f, %.2f\n", bearing, s->distance, s->steerAngle);
	}
}

