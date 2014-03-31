#!/usr/bin/python2.7

import serial, sys;

ser = serial.Serial('/dev/mbed', 115200, timeout=None)

ser.write("8")

while 1:
	if ser.readable():
		sys.stdout.write( ser.read() )

ser.close()
