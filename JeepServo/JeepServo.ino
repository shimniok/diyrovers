#include <Servo.h>

#define MOTORMIN 1200
#define MOTORCTR 1500
#define MOTORMAX 1800
#define INC 10
#define DELAY 50
#define MOTOR 3
#define POT 0
#define LED 5

#define FULL_RIGHT 367
#define FULL_LEFT 823
#define CENTER 570

Servo motor;
int pos=1500;
int target;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  motor.attach(MOTOR);
  motor.writeMicroseconds(1500);

  // Setup pulse read timing thingy
  
  // Convert pulse width to target position
  
}

void loop() {
  for (int i=1000; i < 2000; i+=50) {
    Serial.print("pw=");
    Serial.print(i);
    Serial.print(" pos=");
    Serial.println(pulseWidthToPosition(i));
//    Serial.println(analogRead(POT));
    delay(500);
  }
}

#define M 456L
#define B -101000L
#define M1 386L
#define B1 -19000L
#define M2 526L
#define B2 -229000L

long pulseWidthToPosition(long pw) {
  long pos = CENTER;

  // Dual slope interpolation
  if (pw < 1500) {
    pos = M1*pw + B1;
  } else {
    pos = M2*pw + B2;
  }
  pos /= 1000;

//  if (pos > FULL_LEFT) pos = FULL_LEFT;
//  if (pos < FULL_RIGHT) pos = FULL_RIGHT;
  
  return pos;
}

void servoTest() {
  digitalWrite(LED, HIGH);
  for (int i=MOTORCTR; i >= MOTORMIN; i-=INC) {
    motor.writeMicroseconds(i);
    delay(DELAY);
  }
  for (int i=MOTORMIN; i < MOTORCTR; i+=INC) {
    motor.writeMicroseconds(i);
    delay(DELAY);
  }

  digitalWrite(LED, LOW);
  delay(1000);

  digitalWrite(LED, HIGH);
  for (int i=MOTORCTR; i < MOTORMAX; i+=INC) {
    motor.writeMicroseconds(i);
    delay(DELAY);
  }
  for (int i=MOTORMAX; i >= MOTORCTR; i-=INC) {
    motor.writeMicroseconds(i);
    delay(DELAY);
  }

  digitalWrite(LED, LOW);
  delay(1000);
}


