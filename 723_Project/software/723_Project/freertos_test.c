// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <system.h>
#include <sys/alt_irq.h>
#include <io.h>
#include <altera_avalon_pio_regs.h>

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048


// roc value and frequency value
//typedef struct{
//	unsigned int rocValue;
//	unsigned int newFreq;
//}freqValues;

// Definition of Task Priorities
#define PRINT_STATUS_TASK_PRIORITY 14
#define GETSEM_TASK1_PRIORITY 13
#define GETSEM_TASK2_PRIORITY 12
#define RECEIVE_TASK1_PRIORITY 11
#define RECEIVE_TASK2_PRIORITY 10
#define SEND_TASK_PRIORITY 9
#define counter_task_priority 1
#define lcd_task1_priority 7
#define freq_relay_priority 8

// Definition of Message Queue
#define   MSG_QUEUE_SIZE  30
QueueHandle_t msgqueue;

// Definition of Shed Queue
#define SHED_QUEUE_SIZE 100
QueueHandle_t shedqueue;

// used to delete a task
TaskHandle_t xHandle;

// Definition of Semaphore
SemaphoreHandle_t shared_resource_sem;

/* Queue Definitions */
xQueueHandle qNewFreq;
xQueueHandle qThreshold;
xQueueHandle qShed;
xQueueHandle qLoad;
xQueueHandle qTiming;
xQueueHandle qResp;

// globals variables
unsigned int number_of_messages_sent = 0;
unsigned int number_of_messages_received_task1 = 0;
unsigned int number_of_messages_received_task2 = 0;
unsigned int getsem_task1_got_sem = 0;
unsigned int getsem_task2_got_sem = 0;
char sem_owner_task_name[20];

void freq_relay(void);
void NetworkStatusTask();
void LoadManagerTask();

static QueueHandle_t Q_freq_data;

/* Local Function Prototypes */
int initOSDataStructs(void);
int initCreateTasks(void);

xSemaphoreHandle shared_lcd_sem;
FILE *lcd;

#define ESC 27
#define CLEAR_LCD_STRING "[2J"

// The next two task compete for a shared resource via a semaphore.  The name of
// the task that owns the semaphore is copied into the global variable
// sem_owner_task_name[].
void getsem_task1(void *pvParameters)
{
	while (1)
	{
		xSemaphoreTake(shared_resource_sem, portMAX_DELAY);
		// block forever until receive the mutex
		strcpy(&sem_owner_task_name[0], "getsem_task1");
		getsem_task1_got_sem++;
		xSemaphoreGive(shared_resource_sem);
		vTaskDelay(100);
	}
}

void getsem_task2(void *pvParameters)
{
	while (1)
	{
		xSemaphoreTake(shared_resource_sem, portMAX_DELAY);
		// block forever until receive the mutex
		strcpy(&sem_owner_task_name[0], "getsem_task2");
		getsem_task2_got_sem++;
		xSemaphoreGive(shared_resource_sem);
		vTaskDelay(130);
	}
}

// The following task fills up a message queue with incrementing data.  The data
// is not actually used by the application.  If the queue is full the task is
// suspended for 1 second.
void send_task(void *pvParameters)
{
	unsigned int msg = 0;
	while (1)
	{
		if (xQueueSend(msgqueue, (void *)&msg, 0) == pdPASS)
		{
			// in the message queue
			msg++;
			number_of_messages_sent++;
		}
		else
		{
			vTaskDelay(1000);
		}
	}
}

// The next two task pull messages in the queue at different rates.  The number
// of messages received by the task is incremented when a new message is received
void receive_task1(void *pvParameters)
{
	unsigned int *msg;
	while (1)
	{
		xQueueReceive(msgqueue, &msg, portMAX_DELAY);
		number_of_messages_received_task1++;
		vTaskDelay(333);
	}
}

void receive_task2(void *pvParameters)
{
	unsigned int *msg;
	while (1)
	{
		xQueueReceive(msgqueue, &msg, portMAX_DELAY);
		number_of_messages_received_task2++;
		vTaskDelay(1000);
	}
}

void lcd_task1(void* pvParameters)
 {
	printf("STARTING LCD");
	while(1){
	  IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0x55);
	  usleep(1000000);
	  IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0xaa);
	  usleep(1000000);
	}
 }

void freq_relay(){
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0);
	//printf("%f Hz\n", 16000/(double)temp);
	xQueueSendToBackFromISR( Q_freq_data, &temp, pdFALSE );
	return;
}

void NetworkStatusTask(){
	double freq[100], dfreq[100];
	int i = 99, j = 0;

	/* threshold test purpose */
	double rocFreq = 7.0;
	double lowerFreq = 48.2;
	unsigned int false = 0;
	while(1){

		/* receive frequency data from queue */
		while(uxQueueMessagesWaiting( Q_freq_data ) != 0)
		{
			xQueueReceive( Q_freq_data, freq+i, 0 );

			/* calculate frequency RoC */

			if(i==0)
			{

				dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]); //a special case calculation because if you do i-1 on 0
			}
			else
			{
				dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]); //normal rate of change calculation
			}

			if (dfreq[i] > 100.0)
			{
				dfreq[i] = 100.0; /* maximum rate of change is 100 */
			}

			/* compare arbitrary thresholds with RoC and newly calculated frequency to test end to shedQ
			if RoC exceeds frequency value or frequency values is less than the lower bound */
			if ((fabs(rocFreq) < dfreq[i]) || lowerFreq > freq[i]  )
			{
				xQueueSendToBack(shedqueue, (void*)&false, pdFALSE);
			}
			i =	++i%100;  /* point to the next data (oldest) to be overwritten */
		}
	}
}

void LoadManagerTask(){
//xQueueReceive();

	//IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE,  )
}

int main(int argc, char* argv[], char* envp[])
{
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);
	initOSDataStructs();
	initCreateTasks();
	vTaskStartScheduler();
	for (;;);
	return 0;
}

/* This function simply creates a message queue and a semaphore */
int initOSDataStructs(void)
{
	/* Create Queues */
	qNewFreq = xQueueCreate( 100, sizeof( double ) );
	qThreshold = xQueueCreate( 2, sizeof( double ) );
	qShed = xQueueCreate( 10, sizeof( int ) );
	qLoad = xQueueCreate( 10, sizeof( int ) );
	qTiming = xQueueCreate( MSG_QUEUE_SIZE, sizeof( void* ) );
	qResp = xQueueCreate( MSG_QUEUE_SIZE, sizeof( void* ) );

	msgqueue = xQueueCreate( MSG_QUEUE_SIZE, sizeof( void* ) );
	shedqueue= xQueueCreate( SHED_QUEUE_SIZE, sizeof(double));


	/* Create Semaphores */
	shared_resource_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_lcd_sem = xSemaphoreCreateCounting( 9999, 1 );

	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(getsem_task1, "getsem_task1", TASK_STACKSIZE, NULL, GETSEM_TASK1_PRIORITY, NULL);
	xTaskCreate(getsem_task2, "getsem_task2", TASK_STACKSIZE, NULL, GETSEM_TASK2_PRIORITY, NULL);
	xTaskCreate(receive_task1, "receive_task1", TASK_STACKSIZE, NULL, RECEIVE_TASK1_PRIORITY, NULL);
	xTaskCreate(receive_task2, "receive_task2", TASK_STACKSIZE, NULL, RECEIVE_TASK2_PRIORITY, NULL);
	xTaskCreate(send_task, "send_task", TASK_STACKSIZE, NULL, SEND_TASK_PRIORITY, NULL);
	xTaskCreate(lcd_task1, "lcd_task1",	TASK_STACKSIZE,	NULL, lcd_task1_priority, NULL);
	xTaskCreate(freq_relay, "freq_relay",	TASK_STACKSIZE,	NULL, freq_relay_priority, NULL);
	return 0;
}
