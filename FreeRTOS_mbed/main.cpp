#include "FreeRTOS.h"
#include "task.h"
#include "mbed.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
Serial pc(USBTX, USBRX);

void thread1(void *args) {
	while (1) {
		fprintf(stdout, "Hello ---\n");
		led1 = !led1;
		vTaskDelay(300);
	}
}

void thread2(void *args) {
	while (1) {
        fprintf(stdout, "Hello -\n");
        led2 = !led2;
		vTaskDelay(1000);
	}
}


int main() {
	pc.baud(115200);

	/* Start the two tasks as described in the accompanying application	note. */
	xTaskCreate( thread1, ( signed char * ) "Thread1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL );
	xTaskCreate( thread2, ( signed char * ) "Thread2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL );

	led3 = 1;

	/* Start the tasks running. */
	vTaskStartScheduler();

	led4 = 1;

	while (1);
}
