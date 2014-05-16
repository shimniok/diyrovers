mbed lpc1768 pinouts
====================

UART
----

 * UART0 is the USB connection
 * UART1 
    * rx = p14 
    * tx = p13
 * UART2 
    * rx = p27
    * tx = p28
 * UART3
    * rx = p18
    * tx = p17

I2C
---
 * sda = p9 
 * scl = p10

Encoders
--------
 * A (Outermost)
    * ALEFT = p30
    * ARIGHT = p29
 * B (Innermost)
    * LEFT = unavailable
    * RIGHT = unavailable


GPIO
----
 * Digital
    * IO0 = D0 = p11 
    * IO1 = D1 = p12
    * IO2 = D2 = P0.21 unavailable
    * IO3 = D7 = P2.12 unavailable
    * IO4 = D8 = P2.13 unavailable
 * PWM 
    * PW0 P2 = p25
    * PW1 P1 = p26 
 * Analog
    * AN0 = A0 = p15
    * AN1 = A1 = p16
    * AN2 = A2 = p19
    * AN3 = A3 = p20
