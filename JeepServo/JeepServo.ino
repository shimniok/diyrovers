#include <Servo.h>

#define INTERVAL     20  // Update interval

// Steering motor speed controller values
// motorSpeed > MOTORCTR, wheel turns right
// motorSpeed < MOTORCTR, wheel turns left
#define MOTORRIGHT 1800  // Motor speed control limit (CCW or CW?)
#define MOTORCTR   1500  // Motor stopped speed control
#define MOTORLEFT  1200  // Motor speed control limit (CCW or CW?)
#define MOTORFAST   200  // fast motor speed, added to or subtracted from MOTORCTR

// Pin definitions
#define POT           0  // Feedback potentiometer pin
#define MOTOR         3  // Motor servo-PWM out pin
#define INPWM         4  // Control servo-PWM in pin
#define LED           5  // LED status pin

// Feedback sensor values
// position < CENTER, position is right
// position > CENTER, position is left
#define FULL_RIGHT  367  // POT ADC value at full right lock
#define CENTER      560  // POT ADC value at center (approximately)
#define FULL_LEFT   823  // POT ADC value at full left lock

Servo motor;             // Servo PWM output for motor control
int pwmin=1500;          // Commanded position PWM value in ms
unsigned long nextUpdate;          // Keep track of the next update time

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  motorInit();
  pwmHandlerInit();
  nextUpdate = millis() + INTERVAL;
}

// TOOD add a watchdog timer

void loop() {
  if (millis() > nextUpdate) { // run at set INTERVAL
    int currentPosition; // This holds the raw ADC value from steering feedback potentiometer
    int motorSpeed;      // This value is added to MOTORCTR to call motorWrite()
    int targetPosition;  // target steering position

    // Convert incoming PWM to position value (raw ADC), and clamp just in case
    //targetPosition = 500; // test
    targetPosition = pulseWidthToPosition(pwmin);
    if (targetPosition > FULL_LEFT) {
      targetPosition = FULL_LEFT;
    } else if (targetPosition < FULL_RIGHT) {
      targetPosition = FULL_RIGHT;
    }      

    // Read steering position feedback and clamp, just in case
    currentPosition = analogRead(POT);
    if (currentPosition > FULL_LEFT) {
      currentPosition = FULL_LEFT;
    } else if (currentPosition < FULL_RIGHT) {
      currentPosition = FULL_RIGHT;
    }

    // Convert current position error to motor speed
    motorSpeed = diffToSteerSpeed(targetPosition-currentPosition);

    // TODO: Can we detect if the motor doesn't run at all and issue a warning?
    // really we'd need an encoder on the motor itself to do this and what if the motor
    // just stalls because we're parked?
    // TODO: Can we detect if the potentiometer fails? Or just solder and hope?
    // We'd need to keep track of change in position and/or change in error... if
    // error is > RANGE_WIDE and isn't reducing over several iterations, we have a problem

    // Is the target left or right of the current position? Move motor accordingly.
    // If analogRead(POT) > target, steering is left of target and must be turned right
    // If analogRead(POT) < target, steering is right of target and must be turned left
    //
    if (currentPosition > targetPosition) {         // Target is to the right
      motorWrite(MOTORCTR + motorSpeed);            // turn wheel right
    } else if (currentPosition < targetPosition) {  // Target is to the left
      motorWrite(MOTORCTR - motorSpeed);            // turn wheel left
    } else {
      motorWrite(MOTORCTR);
    }

    //Serial.print("T: "); Serial.print(targetPosition); Serial.print(" ");
    //Serial.print("P: "); Serial.print(currentPosition); Serial.print(" ");
    //Serial.print("M: "); Serial.print(motorSpeed); 
    //Serial.print("T: "); Serial.println(analogRead(POT));
    nextUpdate += INTERVAL;

    Serial.print(millis());
    Serial.print(" ");
    Serial.print(nextUpdate);   
    Serial.println();

  }    
}
