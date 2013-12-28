#include "mbed.h"
#include <CoOS.h>

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
Serial pc(USBTX, USBRX);

#define STK_SIZE 128

OS_STK	a_task_stk[STK_SIZE];
OS_STK	b_task_stk[STK_SIZE];

#define A_TASK_PRIO 10
#define B_TASK_PRIO 10

void taskA(void *args) {
	while (1) {
		fprintf(stdout, "Hello ---\n");
		led1 = !led1;
		CoTickDelay(60);
	}
}

void taskB(void *args) {
	while (1) {
        fprintf(stdout, "Hello -\n");
        led2 = !led2;
        CoTickDelay(200);
	}
}

int main() {
	pc.baud(115200);

	led3 = 1;

	CoInitOS();
	CoCreateTask(taskA, 0, A_TASK_PRIO, &a_task_stk[STK_SIZE-1], STK_SIZE);
	CoCreateTask(taskB, 0, B_TASK_PRIO, &b_task_stk[STK_SIZE-1], STK_SIZE);
	CoStartOS();

	led4 = 1;

	while (1);
}
