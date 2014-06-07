

void motorInit() {
  pinMode(MOTOR, OUTPUT);
  motor.attach(MOTOR);
  motor.writeMicroseconds(1500);
}


void motorWrite(int us) {
  motor.writeMicroseconds(us);
}


/*
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
*/
