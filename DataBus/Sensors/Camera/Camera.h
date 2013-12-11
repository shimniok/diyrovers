#ifndef __CAMERA_H
#define __CAMERA_H


#define CAM_WAIT_ACK    0x01
#define CAM_PARSE_TRACK 0x02

#define CAM_ACK         0x00
#define CAM_NACK        0x01

class Camera {
public:
    Camera(PinName tx, PinName rx);
    void start(void);
    void receive(void);
    void parse(char c);
    
    int lastStatus;
    Serial serial;
};    
#endif