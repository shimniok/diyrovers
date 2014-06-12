#include <Servo.h>

#define INTERVAL     20  // Update interval
#define MOTORRIGHT 1800  // Motor speed control limit (CCW or CW?)
#define MOTORFAST   200  // fast motor speed, added or subtracted to/from MOTORCTR
#define MOTORSLOW   100  //
#define MOTORCTR   1500  // Motor stopped speed control
#define MOTORLEFT  1200  // Motor speed control limit (CCW or CW?)
#define POT           0  // Feedback potentiometer pin
#define MOTOR         3  // Motor servo-PWM out pin
#define INPWM         4  // Control servo-PWM in pin
#define LED           5  // LED status pin

#define RANGE_WIDE   10  // threshold range for pot value
#define RANGE_NARROW  5

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
    target = 500;
//    target = pulseWidthToPosition(pwmin);
//    

    // For motorVal > MOTORCTR, wheel turns right
    // For motorVal < MOTORCTR, wheel turns left
    // If analogRead(POT) > target, steering is left of target and must be turned right
    // If analogRead(POT) < target, steering is right of target and must be turned left
    int motorVal = MOTORCTR;
    int sensor = analogRead(POT);
    if (sensor > target + RANGE_WIDE) { // Target is quite a bit to the right
      motorVal = (MOTORCTR+200); // turn wheel right
    } else if (sensor < target-10) { // Target is quite a bit to the left
      motorVal = (MOTORCTR-200); // turn wheel left
    } else if (sensor > target) { // Target is a little bit to the right
      motorVal = (MOTORCTR+100); // turn wheel right
    } else if (sensor < target) { // Target is a little bit to the left
      motorVal = (MOTORCTR-100); // turn wheel left
    } else {
      motorVal = (1500);
    }
    motorWrite(motorVal);
    
    //Serial.print("T: "); Serial.println(analogRead(POT));
    //next += INTERVAL;
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







