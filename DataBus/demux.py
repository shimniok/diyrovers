#!/usr/bin/python2.7

import serial;

def deserialize(x):
	return ord(x)-ord('0')

ser = serial.Serial('/dev/mbed', 115200, timeout=None)
channel = ser.read()
print 'Channel:{0}'.format( deserialize(channel) )
x = deserialize( ser.read() )
	print 'Byte: {0}'.format( x )
len = 0
for i in range(0,len)
ser.close()
