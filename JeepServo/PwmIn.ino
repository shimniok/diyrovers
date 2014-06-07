#include <avr/io.h>
#include <avr/interrupt.h>

volatile unsigned long pwmstart;
volatile unsigned long pwmstop;

void pwmHandlerInit() {
  // Setup pulse read timing thingy
  pinMode(INPWM, INPUT);
  // D4 : PD4 : PCINT20
  PCMSK2 |= (1<<PCINT20); // Any change on PD4(D4)
  PCICR |= (1<<PCIE2);    // interrupt enable
  pwmstart = pwmstop = micros();
}


ISR(PCINT2_vect) {
  if (digitalRead(4) == HIGH) {
    pwmstart = micros();
  } else {
    pwmstop = micros();
    if (pwmstop >= pwmstart) {
      pwmin = pwmstop - pwmstart;
    }
  }
}
