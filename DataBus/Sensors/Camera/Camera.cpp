#include "mbed.h"
#include "Camera.h"

Camera::Camera(PinName tx, PinName rx):
    serial(tx, rx)
{
    serial.baud(115200);

    return;
} 

void Camera::start()
{
    serial.attach(this, &Camera::receive, Serial::RxIrq);
    
    // ping
    // should now get ACK\r or NACK\r
    // send colormap here
    // should now get ACK\r or NACK\r
    // disable white balance
    // should now get ACK\r or NACK\r
    serial.printf("ET\r\n");
    // should now get ACK\r or NACK\r
    return;
}


void Camera::receive()
{
    while (serial.readable()) {
        char c = serial.getc();
        fprintf(stdout, "%c\n", c);
    }
    return;
}


/*
 * (Byte 0: 0x0A – Indicating the start of a tracking packet 
 * Byte 1: Number of trac k ed objects (0x00 – 0x08 are valid) 
 * Byte 2: Color of object tracked in bounding box 1 
 * Byte 3: X upper left corner of bounding box 1 
 * Byte 4: Y upper left corner of bouding box 1 
 * Byte 5: X lower right corner of boudning box 1 
 * Byte 6: Y lower right corner of boudning box 1 
 * Byte 7: Color object tracked in bound box 2 
 * ... 
 * Byte x: 0xFF (indicates the end of line, and will be sent after all tracking info  
 * for the current frame has been sent)   
 */
void Camera::parse(char c)
{

    return;
}

