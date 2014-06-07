#include <Servo.h>

#define INTERVAL     10  // Update interval
#define MOTORRIGHT 1200  // Motor speed control limit (CCW or CW?)
#define MOTORCTR   1500  // Motor stopped speed control
#define MOTORLEFT  1800  // Motor speed control limit (CCW or CW?)
#define POT           0  // Feedback potentiometer pin
#define MOTOR         3  // Motor servo-PWM out pin
#define INPWM         4  // Control servo-PWM in pin
#define LED           5  // LED status pin

#define FULL_RIGHT  367  // POT ADC value at full right lock
#define FULL_LEFT 823    // POT ADC value at full left lock
#define CENTER 560       // POT ADC value at center (approximately)

Servo motor;             // Servo PWM output for motor control
int pwmin=1500;          // Commanded position PWM value in ms
int pos;                 // current steering position
int target;              // target steering position
int next;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  motorInit();
  pwmHandlerInit();
  next = millis() + INTERVAL;
}

void loop() {

  if (millis() > next) {
    target = pulseWidthToPosition(pwmin);

    // Run the motor in the correct direction
    // to move to the target position
    if (target < pos) {
      motorWrite(1500);
    } else if (target > pos) {
      motorWrite(1500);
    }
    next += INTERVAL;
    /*
    Serial.print(" in=");
    Serial.print(pwmin);
    Serial.println();
    Serial.print(" out=");
    Serial.print(i);
    Serial.print(" pos=");
    Serial.print(pulseWidthToPosition(pwmin));
    */
  }    
}





