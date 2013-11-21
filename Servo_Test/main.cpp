#include "mbed.h"
#include "Servo.h"

Servo Servo1(p21);
#define SERVO_MIN 1000
#define SERVO_MID 1500
#define SERVO_MAX 2000

int main() {
    Servo1.Enable(SERVO_MID,20000);
    
    while(1) {
        for (int pos = SERVO_MIN; pos < SERVO_MAX; pos += 25) {
            Servo1.SetPosition(pos);  
            wait_ms(20);
        }
        for (int pos = SERVO_MAX; pos > SERVO_MIN; pos -= 25) {
            Servo1.SetPosition(pos); 
            wait_ms(20); 
        }
    }
}