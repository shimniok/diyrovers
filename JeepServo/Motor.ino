

void motorInit() {
  pinMode(MOTOR, OUTPUT);
  motor.attach(MOTOR);
  motor.writeMicroseconds(1500);
}


void motorWrite(int us) {
  motor.writeMicroseconds(us);
}
