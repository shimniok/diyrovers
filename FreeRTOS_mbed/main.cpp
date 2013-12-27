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

/* Constants required to manipulate the NVIC. */
#define portNVIC_SYSTICK_CTRL		( ( volatile unsigned long *) 0xe000e010 )
#define portNVIC_SYSTICK_LOAD		( ( volatile unsigned long *) 0xe000e014 )
#define portNVIC_INT_CTRL			( ( volatile unsigned long *) 0xe000ed04 )
#define portNVIC_SYSPRI2			( ( volatile unsigned long *) 0xe000ed20 )
#define portNVIC_SYSTICK_CLK		0x00000004
#define portNVIC_SYSTICK_INT		0x00000002
#define portNVIC_SYSTICK_ENABLE		0x00000001
#define portNVIC_PENDSVSET			0x10000000
#define portNVIC_PENDSV_PRI			( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 16 )
#define portNVIC_SYSTICK_PRI		( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 24 )

int main() {
	pc.baud(115200);

	/* Start the two tasks as described in the accompanying application	note. */
	xTaskCreate( thread1, ( signed char * ) "Thread1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL );
	xTaskCreate( thread2, ( signed char * ) "Thread2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL );

	/* Configure SysTick to interrupt at the requested rate. */
	*(portNVIC_SYSTICK_LOAD) = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	*(portNVIC_SYSTICK_CTRL) = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT | portNVIC_SYSTICK_ENABLE;

	/* Start the tasks running. */
	vTaskStartScheduler();

	while (1);
}
