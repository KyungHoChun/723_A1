// Standard includes
#include <stdio.h>
#include <string.h>
#include <system.h>
#include <io.h>
#include <math.h>
#include <stddef.h>
#include <unistd.h> // for sleep
#include <sys/alt_irq.h>
#include <altera_avalon_pio_regs.h>

#include "altera_avalon_uart_regs.h"
#include "altera_avalon_uart.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"

// Scheduler includes
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/portmacro.h"
#include "FreeRTOS/timers.h"

// Definition of Task Stacks
#define SAMPLING_FREQ 16000.0

/* RoC value and frequency value */
typedef struct{
	double rocValue;
	double newFreq;
}freqValues;

// Definition of Task Priorities
#define add_priority 5
#define freq_analyser_priority 5
#define network_status_priority 5
#define shed_priority 5
#define load_manager_priority 5
#define switch_con_priority 5
#define PRVGA_priority 5
#define lcd_mode_priority 5
#define Timer500Reset_priority (tskIDLE_PRIORITY+1)

// used to delete a task
TaskHandle_t xHandle;

// Definition of Semaphore
SemaphoreHandle_t add_sem;
SemaphoreHandle_t manager_sem;
SemaphoreHandle_t network_sem;
SemaphoreHandle_t roc_sem;
SemaphoreHandle_t shed_sem;
SemaphoreHandle_t state_sem;
SemaphoreHandle_t switch_sem;

/*
SemaphoreHandle_t callback_sem;
SemaphoreHandle_t reset_sem;*/

/* Queue Definitions */
xQueueHandle Q_add;
xQueueHandle Q_freq_data;
xQueueHandle Q_freq_calc;
xQueueHandle Q_resp;
xQueueHandle Q_network_stat;
xQueueHandle Q_threshold;
xQueueHandle Q_timer_reset;
xQueueHandle Q_tmp;

xQueueHandle Q_switch;
xQueueHandle Q_load_stat;

/* Timers*/
TimerHandle_t timer500;
void vTimer500Callback();

/* Globals variables */
typedef enum
{
	normal = 0,
	maintenance = 1
} eMode;

typedef struct
{
	unsigned int iCurrentLoad;
	unsigned int iSheddedLoad;
	unsigned int iUnSheddedLoad;
} loadStat;

char sem_owner_task_name[20];
static double freqThres = 49; //for test purpose - delete later
static double rocThres = 20; //for test purpose - delete later

int iMode = normal;
int Timer_500_flag = 0;
void vfreq_relay(void);

/* Local Function Prototypes */
void initOSDataStructs(void);
void initCreateTasks(void);
void vNormalMode(void);

/* VGA Variables */
//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 //minimum frequency to draw

#define PRVGADraw_Task_P (tskIDLE_PRIORITY+1)
TaskHandle_t PRVGADraw;

typedef struct
{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

/* LCD Variables */
//xSemaphoreHandle shared_lcd_sem;
FILE *lcd;
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

/* shows what the current mode is through LCD Display on push button press*/
void lcd_set_mode()
{
	unsigned int maintMode = 0;
	int tmp;
	while (1)
	{
		if (xQueueReceive(Q_tmp, &tmp, portMAX_DELAY) == pdTRUE)
		{
			maintMode = !maintMode;
			printf("maint mode is: %d\n", maintMode);
			lcd = fopen(CHARACTER_LCD_NAME, "w");
			if (lcd != NULL)
			{
				fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);

				if (maintMode) {
					printf("maint mode\n");
					fprintf (lcd, "MAINTENANCE MODE\n");
				} else {
					printf("normal mode\n");
					fprintf(lcd, "NORMAL MODE\n");
				}
			}
			fclose(lcd);
		}
		vTaskDelay(10);
	}
}

/* VGA Display */
void PRVGADraw_Task(void *pvParameters ){
	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);

	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);

	double freq[100], dfreq[100];
	int i = 99, j = 0;
	Line line_freq, line_roc;

	while(1){

		//receive frequency data from queue
		while(uxQueueMessagesWaiting( Q_freq_data ) != 0){
			xQueueReceive( Q_freq_data, freq+i, 0 );

			//calculate frequency RoC

			if(i==0){
				dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
			}
			else{

				dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
			}

			if (dfreq[i] > 100.0){
				dfreq[i] = 100.0;
			}


			i =	(++i) % 100; //point to the next data (oldest) to be overwritten

		}

		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

		for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(i+j)%100]) > MIN_FREQ) && ((int)(freq[(i+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X
						+ FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
			}
		}
		vTaskDelay(10);

	}
}

/* push button mode */
void button_mode_isr(void* context, alt_u32 id)
{
	volatile int* button = (volatile int*) context;
	//cast the context
	(*button) = IORD_ALTERA_AVALON_PIO_DATA(PUSH_BUTTON_BASE);

	int btn = (int) *button;
	printf("button number is %d\n", btn);

	xQueueSendToBackFromISR(Q_tmp, &btn, pdFALSE);

	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

/* Keyboard ISR */
void ps2_isr(void* ps2_device, alt_u32 id){
	unsigned char byte;
	alt_up_ps2_read_data_byte_timeout(ps2_device, &byte);
	xQueueSendToBackFromISR( Q_threshold, &byte, pdFALSE );
	return;
}

/* Read incoming frequency from ISR*/
void vfreq_relay()
{
	double new_freq = SAMPLING_FREQ / (double) IORD(FREQUENCY_ANALYSER_BASE, 0);

	// Send calculation tasks
	xQueueSendToBackFromISR(Q_freq_data, &new_freq, pdFALSE);
	xSemaphoreGiveFromISR(network_sem, pdFALSE);
	return;
}

/* Read the frequency from the frequency relay */
void vFrequAnalyser_Task()
{
	double new_freq;
	double freqPrev = 50.0;
	freqValues freqValues;

	while(1)
	{
		if(xSemaphoreTake(network_sem, portMAX_DELAY) == pdTRUE)
		{
			xQueueReceive(Q_freq_data, &new_freq, 0);
			// Do calculations
			freqValues.newFreq = new_freq;
			freqValues.rocValue = ((new_freq-freqPrev)*SAMPLING_FREQ) / IORD(FREQUENCY_ANALYSER_BASE, 0);
			freqPrev = new_freq;

			//Send new Data to the Queue
			if(xQueueSend(Q_freq_calc, &freqValues, 0) == pdFALSE){
				xQueueReset( Q_freq_calc );
			}
			xSemaphoreGive(roc_sem);
		}
		vTaskDelay(10);
	}
}

/* Read the frequency from the*/
void vNetworkStatus_Task(void * pvParameters)
{
	freqValues freqValues;

	while(1)
	{
		if(xSemaphoreTake(roc_sem, portMAX_DELAY))
		{
			xQueueReceive(Q_freq_calc, (void *) &freqValues, 0);
			if(freqValues.newFreq < freqThres || fabs(freqValues.rocValue) > rocThres)
			{
				xQueueSend(Q_network_stat, 1, 0);
			}
			xSemaphoreGive(manager_sem);
		}
		vTaskDelay(10);
	}
}

/* Switch controller */
void vSwitchCon_Task(void *pvParameters)
{
	unsigned int iCurrentLoad;
	while(1){
		xSemaphoreTake(switch_sem, pdFALSE);
		iCurrentLoad = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F; //read slide switches
		xQueueSend(Q_switch, &iCurrentLoad, portMAX_DELAY);
		xSemaphoreGive(manager_sem);
		vTaskDelay(10);
	}
}


/* Load manager */
void vLoadManager_Task(void * pvParameters)
{
	int stability;
	unsigned int iCurrentLoad;
	loadStat loadStat;
	loadStat.iSheddedLoad = 0x00;
	loadStat.iUnSheddedLoad = 0x00;

	while(1)
	{
		if(xSemaphoreTake(manager_sem, portMAX_DELAY))
		{
			xQueueReceive(Q_switch, &iCurrentLoad, 0);
			loadStat.iCurrentLoad = iCurrentLoad;
			if(uxQueueMessagesWaiting(Q_network_stat) != 0 && iMode == normal){
				if(xQueueReceive(Q_network_stat, &stability, 0) == pdTRUE){
					printf("iCurrentLoad is: %d\n ", loadStat.iCurrentLoad);
					printf("iSheddedLoad is: %d\n ", loadStat.iSheddedLoad);
					printf("iUnSheddedLoad is: %d\n ", loadStat.iUnSheddedLoad);
					//printf("unstable\n");
					//START TIMER
					//xQueueSend(Q_load_stat, &loadStat, portMAX_DELAY);
					//printf("unstable1\n");
					//xSemaphoreGive(shed_sem);
					//printf("unstable2\n");
				}
			}

//			xSemaphoreGive(switch_sem);
//
			//xQueueReceive(Q_load, &iSwitchRes, portMAX_DELAY);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, loadStat.iSheddedLoad);
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, loadStat.iUnSheddedLoad);
		}
		vTaskDelay(15);
	}
}

/* Shedding load task */
void vShed_Task(void *pvParameters)
{
	loadStat loadStat;

	while(1)
	{
		printf("IN SHED");
		if(xSemaphoreTake(shed_sem, portMAX_DELAY))
		{
			xQueueReceive(Q_load_stat,  (void *) &loadStat, portMAX_DELAY);

			if((loadStat.iCurrentLoad & 0x01) == 0x01){
				loadStat.iSheddedLoad |= 01;
				loadStat.iUnSheddedLoad &= ~01;
			}
			else if((loadStat.iCurrentLoad & 0x02) == 0x02){
				loadStat.iSheddedLoad |= 0x02;
				loadStat.iUnSheddedLoad &= ~0x02;
			}
			else if((loadStat.iCurrentLoad & 0x04) == 0x04){
				loadStat.iSheddedLoad |= 0x04;
				loadStat.iUnSheddedLoad &= ~0x04;
			}
			else if((loadStat.iCurrentLoad & 0x08) == 0x08){
				loadStat.iSheddedLoad |= 0x08;
				loadStat.iUnSheddedLoad &= ~0x08;
			}
			else if((loadStat.iCurrentLoad & 0x16) == 0x16){
				loadStat.iSheddedLoad |= 0x16;
				loadStat.iUnSheddedLoad &= ~0x16;
			}
			xQueueSend(Q_load_stat, (void*)&loadStat, portMAX_DELAY);
			xSemaphoreGive(manager_sem);
		}
		vTaskDelay(10);
	}
	return;
}

void vAdd_Task(void *pvParameters)
{

	loadStat loadStat;

	while(1)
	{
		printf("IN ADD");
		if(xSemaphoreTake(add_sem, portMAX_DELAY))
		{
			xQueueReceive(Q_load_stat,  (void *) &loadStat, portMAX_DELAY);

			if((loadStat.iCurrentLoad & 0x16) == 0x16){
				loadStat.iSheddedLoad &= ~0x16;
				loadStat.iUnSheddedLoad |= 0x16;
			}
			else if((loadStat.iCurrentLoad & 0x08) == 0x08){
				loadStat.iSheddedLoad |= 0x08;
				loadStat.iUnSheddedLoad &= ~0x08;
			}
			else if((loadStat.iCurrentLoad & 0x04) == 0x04){
				loadStat.iSheddedLoad |= 0x04;
				loadStat.iUnSheddedLoad &= ~0x04;
			}
			else if((loadStat.iCurrentLoad & 0x02) == 0x02){
				loadStat.iSheddedLoad |= 0x02;
				loadStat.iUnSheddedLoad &= ~0x02;
			}
			else if((loadStat.iCurrentLoad & 0x01) == 0x01){
				loadStat.iSheddedLoad |= 0x01;
				loadStat.iUnSheddedLoad &= ~0x01;
			}
			xQueueSend(Q_load_stat, (void*)&loadStat, portMAX_DELAY);
			xSemaphoreGive(manager_sem);
		}
		vTaskDelay(10);
	}
	return;
}

void vTimer500Reset_Task(void *pvParameters){
	int reset;
	while(1){
		if(uxQueueMessagesWaiting(Q_timer_reset)){
			if(xQueueReceive(Q_timer_reset, &reset, 0) == pdTRUE){
				xTimerReset(timer500,0);
			}
		}
		vTaskDelay(10);
	}
	return;
}

/* Timer functions*/
void vTimer500_Callback(xTimerHandle t_timer500)
{
	printf("--\n");
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0x1F^IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE));
}

void main(int argc, char* argv[], char* envp[])
{
	/* Start Frequency ISR */
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, vfreq_relay);

	/* Start Keyboard setup and ISR */
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	if(ps2_device == NULL){
		printf("can't find PS/2 device\n");
		return;
	}
	alt_up_ps2_enable_read_interrupt(ps2_device);
	alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);

	/* Push button setup */
	int buttonValue = 0;
	/* clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.*/
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x07);

	/* Enable interrupts for key1 */
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x01);

	/* Register the ISR */
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, button_mode_isr);

	/* Create Timers */
//	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0x1F);
//	timer500 = xTimerCreate("Timer500", 1000, pdTRUE, NULL, vTimer500_Callback);
//
//	if(xTimerStart(timer500,0) != pdPASS){
//		printf("Cannot start timer");
//	}

	initOSDataStructs();
	initCreateTasks();
	vTaskStartScheduler();

	for(;;);
	return;
}

/* This function simply creates a message queue and a semaphore */
void initOSDataStructs(void)
{
	/* Create Semaphores */
	add_sem = xSemaphoreCreateBinary();
	manager_sem = xSemaphoreCreateBinary();
	network_sem = xSemaphoreCreateBinary();
	roc_sem = xSemaphoreCreateBinary();
	state_sem = xSemaphoreCreateBinary();
	switch_sem = xSemaphoreCreateBinary();

	//callback_sem = xSemaphoreCreateBinary();
	//reset_sem = xSemaphoreCreateBinary()

	/* Create Queues */
	Q_freq_data = xQueueCreate( 100, sizeof( double ) );
	Q_freq_calc = xQueueCreate( 100, sizeof( freqValues ) );
	Q_threshold = xQueueCreate( 2, sizeof( double ) );
	Q_network_stat = xQueueCreate( 10, sizeof( int ) );
	Q_load_stat = xQueueCreate( 10, sizeof( loadStat ) );
	Q_switch = xQueueCreate( 10, sizeof( int ) );
	Q_resp = xQueueCreate( 1, sizeof( int ) );
	Q_tmp = xQueueCreate(100, sizeof(int));

	Q_timer_reset = xQueueCreate( 4, sizeof( int ) );

	return;
}

// This function creates the tasks used in this example
void initCreateTasks(void)
{
	//xTaskCreate( vAdd_Task, "AddTask", configMINIMAL_STACK_SIZE, NULL, add_priority, NULL );
	xTaskCreate( vFrequAnalyser_Task, "FreqAnalyserTask", configMINIMAL_STACK_SIZE, NULL, freq_analyser_priority, NULL );
	xTaskCreate( vNetworkStatus_Task, "NetworkStatusTask", configMINIMAL_STACK_SIZE, NULL, network_status_priority, NULL );
	//xTaskCreate( vShed_Task, "ShedTask", configMINIMAL_STACK_SIZE, NULL, shed_priority, NULL );
	xTaskCreate( vLoadManager_Task, "LoadManagerTask", configMINIMAL_STACK_SIZE, NULL, load_manager_priority, NULL );
	xTaskCreate( vSwitchCon_Task, "SwitchCon", configMINIMAL_STACK_SIZE, NULL, switch_con_priority, NULL );
	xTaskCreate( vTimer500Reset_Task, "Timer500Reset", configMINIMAL_STACK_SIZE, NULL, Timer500Reset_priority, NULL );
	// xTaskCreate( PRVGADraw_Task, "PRVGADraw", configMINIMAL_STACK_SIZE, NULL, PRVGA_priority, NULL );
	xTaskCreate (lcd_set_mode, "lcd_mode", configMINIMAL_STACK_SIZE, NULL, lcd_mode_priority, NULL );

	return;
}
