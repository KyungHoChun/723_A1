// Standard includes
#include <stdio.h>
#include <string.h>
#include <system.h>
#include <io.h>
#include <math.h>
#include <stddef.h>
#include <unistd.h> // for sleep
#include <sys/alt_irq.h>

#include "altera_avalon_pio_regs.h"
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
#define freq_analyser_priority 5
#define network_status_priority 5
#define shed_priority 5
#define load_manager_priority 4
#define switch_con_priority 2
#define Timer500Reset_priority 2
#define PRVGA_priority 1

// used to delete a task
TaskHandle_t xHandle;

// Definition of Semaphore
SemaphoreHandle_t roc_sem;
SemaphoreHandle_t state_sem;
SemaphoreHandle_t network_sem;
SemaphoreHandle_t shed_sem;
SemaphoreHandle_t reset_sem;
SemaphoreHandle_t callback_sem;

/* Queue Definitions */
xQueueHandle Q_freq_data;
xQueueHandle Q_freq_calc;
xQueueHandle Q_threshold;
xQueueHandle Q_shed;
xQueueHandle Q_load;
xQueueHandle Q_timing;
xQueueHandle Q_resp;

/* Timers*/
//alt_u32 firstShed_timer_isr(void* context);
//TimerHandle_t timer_200;
TimerHandle_t timer_500;
void vTimer500Reset_Task();
void vTimer500Callback();

/* Globals variables */
typedef enum
{
	normal = 0,
	maintanence = 1,
} eMode;

char sem_owner_task_name[20];
static double rocThres = 20; //for test purpose - delete later
static double freqThres = 49; //for test purpose - delete later

int iMode = normal;
int Timer_500_flag = 0;
void vfreq_relay(void);

/* Local Function Prototypes */
int initOSDataStructs(void);
int initCreateTasks(void);
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
xSemaphoreHandle shared_lcd_sem;
FILE *lcd;

#define ESC 27
#define CLEAR_LCD_STRING "[2J"

/* VGA Display */
void PRVGADraw_Task(void *pvParameters ){
	/*
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
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
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

	}*/
}

/* Keyboard ISR */
void ps2_isr(void* ps2_device, alt_u32 id){
	unsigned char byte;
	alt_up_ps2_read_data_byte_timeout(ps2_device, &byte);
	xQueueSendToBackFromISR( Q_threshold, &byte, pdFALSE );
}

/* Push button ISR */
void push_button_irq(){
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE));
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); //write 1 to clear all detected falling edges
	return;
}

/* Read incoming frequency from ISR*/
void vfreq_relay()
{
	double new_freq = SAMPLING_FREQ / IORD(FREQUENCY_ANALYSER_BASE, 0);

	// Send calculation tasks
	xQueueSendToBackFromISR( Q_freq_data, &new_freq, pdFALSE );
	xSemaphoreGiveFromISR(roc_sem, NULL);
	return;
}

/* Read the frequency from the frequency relay */
void vFrequAnalyser_Task()
{
	double new_freq;
	freqValues freqValues;
	double freqPrev = 50.0;

	while(1)
	{
		if(xSemaphoreTake(roc_sem, portMAX_DELAY))
		{
			xQueueReceive(Q_freq_data, &new_freq, 0);
			// Do calculations
			freqValues.newFreq = new_freq;
			freqValues.rocValue = ((new_freq-freqPrev)*SAMPLING_FREQ) / IORD(FREQUENCY_ANALYSER_BASE, 0);
			freqPrev = new_freq;

			//Send new Data to the Queue
			if(xQueueSend(Q_freq_calc, &freqValues, 0) == pdTRUE){
				printf("SENT \n");
			}else{
				printf("NOT SENT \n");
			}
			xSemaphoreGive(roc_sem);
		}
		vTaskDelay(333);
	}
}

/* Read the frequency from the*/
void vNetworkStatus_Task(void * pvParameters)
{
	freqValues freqValues;
	//freqThres and rocThres are hardcoded at the moment - delete later

	while(1)
	{
		if(xSemaphoreTake(network_sem, portMAX_DELAY))
		{
			xQueueReceive(Q_freq_calc, (void *) &freqValues, 0);
			printf("freqThres is: %f \n",freqValues.newFreq);
			printf("RoC is: %f \n",freqValues.rocValue);
			if(freqValues.newFreq < freqThres || fabs(freqValues.rocValue) > rocThres)
			{
				xQueueSend(Q_shed, 1, 0);

				//implement timer first - first load shedding needs to happen less than 200ms
				xSemaphoreGive(network_sem);
			}
		}
		vTaskDelay(500);
	}
}

/* Switch controller */
void vSwitchCon_Task(void *pvParameters)
{
	unsigned int iSwitchRes = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x00FF; //read slide switches
	xQueueOverwrite(Q_load, iSwitchRes);
	vTaskDelay(750);
}


/* Load manager */
void vLoadManager_Task(void * pvParameters)
{
	unsigned int iSwitchRes;
	int clearLed = 0x00;

	while(1)
	{
		if(xSemaphoreTake(network_sem, portMAX_DELAY))
		{
			if(iMode == normal){
				vNormalMode();
				// if all loads connected, exit load managing.
			}
			else
			{
				// Maintenance mode
				xQueueReceive(Q_load, &iSwitchRes, 0);
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, iSwitchRes);
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, clearLed);
			}
		}
		vTaskDelay(750);
	}
}

/* Normal Mode */
void vNormalMode(){
	unsigned int iSwitchRes;
	int shedRes;
	int shedFlag = 0;

	// read from shed
	xQueueReceive(Q_shed, &shedRes, 0);
	// if there is no shed required and there has been no shed load, read the switches and light red LEDs
	if (shedRes == pdFALSE && !shedFlag)
	{
		xQueueReceive(Q_load, &iSwitchRes, 0);
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, 0x00);
	}
	else
	{
		vShed_Task();
	}

	// shed load with lowest prio
	//start 500ms timer,
	// if another shed occurs, shed the next load.
	// else if network remains stable, reconnect the highest priority load that has been shedded.
	// if at any time the network status changes from stable to unstable and vice versa before 500ms period ends, reset 500ms timer.

	// if the switches have changed
	// light the appropriate red LED

}

/* Shedding load task */
void vShed_Task(void * pvParameters)
{
	int iSwitchRes;
	int shedFlag = 0;
	// Start 200ms
	// has not shed a load
	if(shedFlag == 0)
	{
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, 0x00);

		// look at switch configuration
		// save slide configuration
		// shed load with lowest prio
		// switch off red led and turn on green led
		// start 500ms timer
		// Timer500_Reset();
	}
	else
	{
		// look at previous slide configuration
		// shed next lowest prio load
		// switch off red led and turn on green led
		// start 500ms timer
		// Timer500_Reset();
	}
}

/* Timer functions */
void vTimer500Reset_Task(){
	xSemaphoreTake(reset_sem, 10);
	Timer_500_flag = 0;
	xSemaphoreGive(reset_sem);

	xTimerReset(timer_500, 10);
}

/* Timer callback */
void vTimer500Callback(xTimerHandle t_timer){
	xSemaphoreTake(callback_sem, 10);
	Timer_500_flag = 1;
	xSemaphoreGive(callback_sem);
}

int main(int argc, char* argv[], char* envp[])
{
	/* Start Frequency ISR */
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, vfreq_relay);

	/* Start Keyboard setup and ISR */
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	if(ps2_device == NULL){
		printf("can't find PS/2 device\n");
		return 1;
	}
	alt_up_ps2_enable_read_interrupt(ps2_device);
	alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);

	/* Push button setup */
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7); //enable interrupt for all three push buttons (Keys 1-3 -> bits 0-2)
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); //write 1 to edge capture to clear pending interrupts
	alt_irq_register(PUSH_BUTTON_IRQ, 0, push_button_irq);  //register ISR for push button interrupt request

	initOSDataStructs();
	initCreateTasks();
	vTaskStartScheduler();

	/* Create Timers */
	timer_500 = xTimerCreate("500_timer", 500, pdTRUE, NULL, vTimer500Callback);
	xTimerStop(timer_500,10);

	for(;;);
	return 0;
}

/* This function simply creates a message queue and a semaphore */
int initOSDataStructs(void)
{
	/* Create Queues */
	Q_freq_data = xQueueCreate( 10, sizeof( double ) );
	Q_freq_calc = xQueueCreate( 100, sizeof( freqValues ) );
	Q_threshold = xQueueCreate( 2, sizeof( double ) );
	Q_shed = xQueueCreate( 1, sizeof( int ) );
	Q_load = xQueueCreate( 10, sizeof( int ) );
	Q_timing = xQueueCreate( 4, sizeof( int ) );
	Q_resp = xQueueCreate( 1, sizeof( int ) );

	/* Create Semaphores */
	roc_sem = xSemaphoreCreateCounting(9999, 1);
	state_sem = xSemaphoreCreateCounting(9999, 1);
	network_sem = xSemaphoreCreateCounting(9999, 1);
	reset_sem = xSemaphoreCreateCounting(9999, 1);
	callback_sem = xSemaphoreCreateCounting(9999, 1);

	return 0;
}
// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate( vFrequAnalyser_Task, "FreqAnalyserTask", configMINIMAL_STACK_SIZE, NULL, freq_analyser_priority, NULL );
	xTaskCreate( vNetworkStatus_Task, "NetworkStatusTask", configMINIMAL_STACK_SIZE, NULL, network_status_priority, NULL );/*
	xTaskCreate( vShed_Task, "ShedTask", configMINIMAL_STACK_SIZE, NULL, shed_priority, NULL );
	xTaskCreate( vLoadManager_Task, "LoadManagerTask", configMINIMAL_STACK_SIZE, NULL, load_manager_priority, NULL );
	xTaskCreate( vSwitchCon_Task, "SwitchCon", configMINIMAL_STACK_SIZE, NULL, switch_con_priority, NULL );
	xTaskCreate( vTimer500Reset_Task, "Timer500Reset", configMINIMAL_STACK_SIZE, NULL, Timer500Reset_priority, NULL );
	xTaskCreate( PRVGADraw_Task, "PRVGADraw", configMINIMAL_STACK_SIZE, NULL, PRVGA_priority, NULL );*/

	return 0;
}
