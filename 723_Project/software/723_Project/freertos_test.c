/* Standard includes */
#include <system.h>
#include <stdio.h>
#include <string.h>
#include <io.h>
#include <math.h>
#include <stddef.h>
#include <unistd.h> // for sleep

/* Scheduler includes*/
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/portmacro.h"
#include "FreeRTOS/timers.h"

/* Altera includes. */
#include <sys/alt_alarm.h>
#include <sys/alt_irq.h>
#include "altera_avalon_pio_regs.h"
#include "altera_avalon_uart_regs.h"
#include "altera_avalon_uart.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"


/* Definition of Task Stacks */
#define SAMPLING_FREQ 16000.0

/* RoC value and frequency value */
typedef struct{
	double rocValue;
	double newFreq;
}freqValues;

/* Definition of Task Priorities */
#define add_priority 5
#define shed_priority 5

#define network_status_priority 6
#define load_manager_priority 6
#define PRVGA_priority 7
#define lcd_mode_priority 4

#define switch_con_priority 4
#define keyboard_priority 4
//#define Timer500Reset_priority (tskIDLE_PRIORITY+1)

// used to delete a task
TaskHandle_t xHandle;

/* Definition of Semaphore */
SemaphoreHandle_t add_sem;
SemaphoreHandle_t manager_sem;
SemaphoreHandle_t roc_sem;
SemaphoreHandle_t timer_sem;
SemaphoreHandle_t switch_sem;

/* Queue Definitions */
xQueueHandle Q_freq_data;
xQueueHandle Q_network_stat;
xQueueHandle Q_timer_reset;
xQueueHandle Q_tmp;
xQueueHandle Q_switch;
xQueueHandle Q_load_stat;
xQueueHandle Q_a_load_stat;
xQueueHandle Q_ms_load_stat;
xQueueHandle Q_ma_load_stat;
xQueueHandle Q_keyboard;

/* Timers*/
TimerHandle_t timer500;
void vTimer500_Callback(xTimerHandle t_timer);

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

typedef struct
{
	unsigned int iSheddedLoad;
	unsigned int iUnSheddedLoad;
} handledLoadStat;

int timer_flag = 0;
int timer_fin = 0;
int stable_network = 1;

char sem_owner_task_name[20];
static double freqThreshold=49; //for test purpose - delete later
static double rocThreshold=20; //for test purpose - delete later

int iMode = normal;
int Timer_500_flag = 0;

/* Local Function Prototypes */
void initOSDataStructs(void);
void initCreateTasks(void);
void vfreq_relay(void);

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
	int tmp;
	lcd = fopen(CHARACTER_LCD_NAME, "w");
	fprintf (lcd, "MAINTENANCE MODE\n");
	fclose(lcd);

	while (1)
	{
		if (xQueueReceive(Q_tmp, &tmp, portMAX_DELAY) == pdTRUE)
		{
			iMode = !iMode;
			printf("iMode: %d\n", iMode);
			lcd = fopen(CHARACTER_LCD_NAME, "w");
			if (lcd != NULL)
			{
				fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);

				if (iMode)
				{
					printf("maint mode\n");
					fprintf (lcd, "MAINTENANCE MODE\n");
				} else
				{
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

void ps2_isr (void* context, alt_u32 id)
{
	char ascii;
	int status = 0;
	unsigned char key = 0;
	KB_CODE_TYPE decode_mode;
	status = decode_scancode (context, &decode_mode , &key , &ascii) ;
	if ( status == 0 ) //success
	{
		if (key == 0x5A){
			ascii = 0x0D;
		}
		if (decode_mode != KB_LONG_BINARY_MAKE_CODE && decode_mode != KB_BREAK_CODE){
			xQueueSendToBackFromISR(Q_keyboard, &ascii, 0);
		}
	}
}

void vkeyboardHandlerTask (void * pvParameters)
{
	char keyInput;
	unsigned int val;
	unsigned int inputs[2] = {0, 0};
	int i = 0;

	while (1){
		while(uxQueueMessagesWaiting(Q_keyboard) != 0){
			xQueueReceive(Q_keyboard, (void*) &keyInput, 0);
			printf("print: %d \n", keyInput-0x30);

			/*check if the input number is valid*/
			if (keyInput >= 0x30 && keyInput <=0x39){
				val = keyInput-0x30;
				inputs[i] = inputs[i]*10 + val;
			}

			/* check the keyboard input is comma*/
			else if (keyInput == 0x2C)
			{
				i++;
				if (i >= 2){
					xSemaphoreTake(roc_sem, portMAX_DELAY);
					freqThreshold = inputs[0];
					rocThreshold = inputs[1];
					inputs[0] = 0;
					inputs[1] = 0;
					i = 0;
					printf("New Value: %f,  %f\n", freqThreshold, rocThreshold);
					xSemaphoreGive(roc_sem);

				}
			}
			/* check if the keyboard input is enter*/
			else if (keyInput == 0x0D){
				xSemaphoreTake(roc_sem, portMAX_DELAY);
				freqThreshold = inputs[0];
				rocThreshold = inputs[1];
				inputs[0] = 0;
				inputs[1] = 0;
				i = 0;
				printf("New Value: %f,  %f\n", freqThreshold, rocThreshold);
				xSemaphoreGive(roc_sem);
			}
		}
		vTaskDelay(10);
	}
}

/* Read incoming frequency from ISR*/
void vfreq_relay()
{
	double new_freq = SAMPLING_FREQ / (double) IORD(FREQUENCY_ANALYSER_BASE, 0);

	/* Send calculation tasks */
	xQueueSendToBackFromISR(Q_freq_data, &new_freq, pdFALSE);
	return;
}

/* Read the frequency from the*/
void vNetworkStatus_Task(void * pvParameters)
{
	freqValues freqValues;
	double new_freq;
	double freqPrev = 50.0;
	double rocValue = 0;
	int stable = 1;

	while(1)
	{
		if(xQueueReceive(Q_freq_data, &new_freq, portMAX_DELAY) == pdTRUE)
			{
			xSemaphoreTake(roc_sem, portMAX_DELAY);
			//Calculate Rate of Change Value
			rocValue = ((new_freq-freqPrev)*SAMPLING_FREQ) / IORD(FREQUENCY_ANALYSER_BASE, 0);
			freqPrev = new_freq;
			xSemaphoreGive(roc_sem);

			if(new_freq < freqThreshold || fabs(rocValue) > rocThreshold)
			{
				stable = 0;
			}
			else{
				stable = 1;
			}
			xQueueSend(Q_network_stat, &stable, 0);
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
	handledLoadStat handledLoadStat;

	loadStat.iSheddedLoad = 0x00;
	loadStat.iUnSheddedLoad = 0x00;

	int shed_flag = 0;
	int add_flag = 0;
	int timer_start = 0;

	while(1)
	{
		if(uxQueueMessagesWaiting(Q_network_stat) == pdTRUE)
		{
			if(xQueueReceive(Q_switch, &iCurrentLoad, 0) == pdTRUE)
			{
				loadStat.iCurrentLoad = iCurrentLoad;
				loadStat.iSheddedLoad &= iCurrentLoad;
				loadStat.iUnSheddedLoad &= iCurrentLoad;
				handledLoadStat.iSheddedLoad &= iCurrentLoad;
				handledLoadStat.iUnSheddedLoad &= iCurrentLoad;

			}

			if(xQueueReceive(Q_network_stat, &stability, 0) == pdTRUE && loadStat.iCurrentLoad != 0 && iMode == normal){

				//SHED
				//// If timer is not active and unstable - start timer - shed load
				//// if timer is active (from stable) and has become unstable - reset timer - shed load

				//// If timer has finished and unstable - reset timer - shed load
				//// If timer has finished (from unstable) and has become stable - reset timer
					//// if a load was shed - add load

				//ADD
				// If timer has finished (from unstable) and has become stable - reset timer
					// If a load was shed - add load

				// if timer is active (from stable) and has become unstable - reset timer - shed load
				if(timer_flag && !timer_fin && !stability){
					xSemaphoreTake(timer_sem, portMAX_DELAY);
					timer_flag = 1;
					shed_flag = 1;
					timer_start = 1;
					xSemaphoreGive(timer_sem);
				}
				// If timer is not active and unstable - start timer - shed load
				else if(!timer_flag && !timer_fin && !stability){
					xSemaphoreTake(timer_sem, portMAX_DELAY);
					timer_flag = 1;
					shed_flag = 1;
					timer_start = 1;
					xSemaphoreGive(timer_sem);
				}
				// If timer has finished and unstable - reset timer - shed load
				else if(!timer_flag && timer_fin && !stability){
					xSemaphoreTake(timer_sem, portMAX_DELAY);
					timer_flag = 1;
					shed_flag = 1;
					timer_fin = 0;
					timer_start = 1;
					xSemaphoreGive(timer_sem);
				}
				// If timer has finished (from unstable) and has become stable - reset timer
				else if(!timer_flag && timer_fin && stability){
					// if a load was shed - add load

					if(loadStat.iSheddedLoad > 0){
						xSemaphoreTake(timer_sem, pdFALSE);
						add_flag = 1;
						timer_fin = 0;
						timer_start = 1;
						xSemaphoreGive(timer_sem);
					}
				}

				if(timer_start){
					timer_start = 0;
					if (xTimerStart(timer500, 10) != pdPASS){
						printf("Cannot start 500ms timer\n");
					}
				}

				if(shed_flag){
					xQueueSend(Q_load_stat, &loadStat, portMAX_DELAY);
				}
				else if(add_flag){
					xQueueSend(Q_a_load_stat, &loadStat, portMAX_DELAY);
				}
			}
			else{
				loadStat.iSheddedLoad = 0;
				loadStat.iUnSheddedLoad = 0;
				handledLoadStat.iSheddedLoad = 0;
				handledLoadStat.iUnSheddedLoad = 0;
			}

			if (xQueueReceive(Q_ms_load_stat, &handledLoadStat, 0) == pdTRUE && shed_flag){
				printf("RECEIVE SHED\n");
				shed_flag = 0;
				loadStat.iSheddedLoad = handledLoadStat.iSheddedLoad;
				loadStat.iUnSheddedLoad = handledLoadStat.iUnSheddedLoad;
			}
			else if (xQueueReceive(Q_ma_load_stat, &handledLoadStat, 0) == pdTRUE && add_flag){
				printf("ADD SHED\n");
				add_flag = 0;
				loadStat.iSheddedLoad = handledLoadStat.iSheddedLoad;
				loadStat.iUnSheddedLoad = handledLoadStat.iUnSheddedLoad;
			}

			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, loadStat.iSheddedLoad & 0x1F);
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, (loadStat.iSheddedLoad ^ loadStat.iCurrentLoad) & 0x1F);
		}
		vTaskDelay(10);
	}
}

/* Shedding load task */
void vShed_Task(void *pvParameters)
{
	loadStat loadStat;
	handledLoadStat handledLoadStat;

	while(1)
	{
		if(uxQueueMessagesWaiting(Q_load_stat) != 0){
			xQueueReceive(Q_load_stat,  (void *) &loadStat, portMAX_DELAY);

			if((loadStat.iCurrentLoad & 0x01) == 0x01 && (loadStat.iSheddedLoad & 0x01) != 0x01){
				loadStat.iSheddedLoad |= 0x01;
				loadStat.iUnSheddedLoad &= ~01;
			}
			else if((loadStat.iCurrentLoad & 0x02) == 0x02 && (loadStat.iSheddedLoad & 0x02) != 0x02){
				loadStat.iSheddedLoad |= 0x02;
				loadStat.iUnSheddedLoad &= ~0x02;
			}
			else if((loadStat.iCurrentLoad & 0x04) == 0x04 && (loadStat.iSheddedLoad & 0x04) != 0x04){
				loadStat.iSheddedLoad |= 0x04;
				loadStat.iUnSheddedLoad &= ~0x04;
			}
			else if((loadStat.iCurrentLoad & 0x08) == 0x08 && (loadStat.iSheddedLoad & 0x08) != 0x08){
				loadStat.iSheddedLoad |= 0x08;
				loadStat.iUnSheddedLoad &= ~0x08;
			}
			else if((loadStat.iCurrentLoad & 0x16) == 0x16 && (loadStat.iSheddedLoad & 0x16) != 0x16){
				loadStat.iSheddedLoad |= 0x16;
				loadStat.iUnSheddedLoad &= ~0x16;
			}

			handledLoadStat.iSheddedLoad = loadStat.iSheddedLoad;
			handledLoadStat.iUnSheddedLoad = loadStat.iUnSheddedLoad;

			xQueueSend(Q_ms_load_stat, (void*)&handledLoadStat, portMAX_DELAY);
		}
		vTaskDelay(10);
	}
	return;
}

void vAdd_Task(void *pvParameters)
{
	loadStat loadStat;
	handledLoadStat handledLoadStat;

	while(1)
	{
		if(uxQueueMessagesWaiting(Q_a_load_stat) != 0){
			xQueueReceive(Q_a_load_stat,  (void *) &loadStat, portMAX_DELAY);
			printf("b Add iSheddedLoad: %d\n",handledLoadStat.iSheddedLoad);
			printf("b Add iUnSheddedLoad: %d\n",handledLoadStat.iUnSheddedLoad);

			if((loadStat.iCurrentLoad & 0x16) == 0x16 && (loadStat.iSheddedLoad & 0x16) == 0x16){
				loadStat.iSheddedLoad &= ~0x16;
				loadStat.iUnSheddedLoad |= 0x16;
			}
			else if((loadStat.iCurrentLoad & 0x08) == 0x08 && (loadStat.iSheddedLoad & 0x08) == 0x08){
				loadStat.iSheddedLoad &= ~0x08;
				loadStat.iUnSheddedLoad |= 0x08;
			}
			else if((loadStat.iCurrentLoad & 0x04) == 0x04 && (loadStat.iSheddedLoad & 0x04) == 0x04){
				loadStat.iSheddedLoad &= ~0x04;
				loadStat.iUnSheddedLoad |= 0x04;
			}
			else if((loadStat.iCurrentLoad & 0x02) == 0x02 && (loadStat.iSheddedLoad & 0x02) == 0x02){
				loadStat.iSheddedLoad &= ~0x02;
				loadStat.iUnSheddedLoad |= 0x02;
			}
			else if((loadStat.iCurrentLoad & 0x01) == 0x01 && (loadStat.iSheddedLoad & 0x01) == 0x01){
				loadStat.iSheddedLoad &= ~0x01;
				loadStat.iUnSheddedLoad |= 0x01;
			}

			handledLoadStat.iSheddedLoad = loadStat.iSheddedLoad;
			handledLoadStat.iUnSheddedLoad = loadStat.iUnSheddedLoad;

			xQueueSend(Q_ma_load_stat, (void*)&handledLoadStat, portMAX_DELAY);
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
void vTimer500_Callback(xTimerHandle t_timer)
{
	//xSemaphoreTake(timer_sem, portMAX_DELAY);
	timer_flag = 0;
	timer_fin = 1;
	xSemaphoreGiveFromISR(timer_sem, pdTRUE);
	return;
}

void main(int argc, char* argv[], char* envp[])
{
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	if(ps2_device == NULL){
		printf("can't find PS/2 device\n");
	}

	alt_up_ps2_clear_fifo (ps2_device) ;
	alt_up_ps2_enable_read_interrupt(ps2_device);
	alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);

	/* Register the PS/2 interrupt */
	IOWR_8DIRECT(PS2_BASE,4,1);

	/* Start Frequency ISR */
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, vfreq_relay);

	/* Push button setup */
	int buttonValue = 0;

	/* clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.*/
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x07);

	/* Enable interrupts for key1 */
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x01);

	/* Register the ISR */
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, button_mode_isr);

	/* Create Timers */
	timer500 = xTimerCreate("Timer500", pdMS_TO_TICKS(500), pdFALSE, NULL, vTimer500_Callback);

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
	manager_sem = xSemaphoreCreateBinary();
	roc_sem = xSemaphoreCreateMutex();
	switch_sem = xSemaphoreCreateBinary();
	timer_sem = xSemaphoreCreateMutex();

	/* Create Queues */
	Q_freq_data = xQueueCreate( 100, sizeof( double ) );
	Q_load_stat = xQueueCreate( 1, sizeof( loadStat ) );
	Q_a_load_stat = xQueueCreate( 1, sizeof( loadStat ) );
	Q_ma_load_stat = xQueueCreate( 1, sizeof( handledLoadStat ) );
	Q_ms_load_stat = xQueueCreate( 1, sizeof( handledLoadStat ) );

	Q_network_stat = xQueueCreate( 10, sizeof( int ) );
	Q_switch = xQueueCreate( 10, sizeof( unsigned int ) );
	Q_tmp = xQueueCreate(100, sizeof(int));

	Q_keyboard = xQueueCreate(10, sizeof (char));

	Q_timer_reset = xQueueCreate( 4, sizeof( int ) );

	return;
}

// This function creates the tasks used in this example
void initCreateTasks(void)
{
	xTaskCreate( vNetworkStatus_Task, "NetworkStatusTask", configMINIMAL_STACK_SIZE, NULL, network_status_priority, NULL );
	xTaskCreate( vAdd_Task, "AddTask", configMINIMAL_STACK_SIZE, NULL, add_priority, NULL );
	xTaskCreate( vShed_Task, "ShedTask", configMINIMAL_STACK_SIZE, NULL, shed_priority, NULL );

	xTaskCreate( vLoadManager_Task, "LoadManagerTask", configMINIMAL_STACK_SIZE, NULL, load_manager_priority, NULL );
	xTaskCreate( vSwitchCon_Task, "SwitchCon", configMINIMAL_STACK_SIZE, NULL, switch_con_priority, NULL );
	//xTaskCreate( vTimer500Reset_Task, "Timer500Reset", configMINIMAL_STACK_SIZE, NULL, Timer500Reset_priority, NULL );
	xTaskCreate( PRVGADraw_Task, "PRVGADraw", configMINIMAL_STACK_SIZE, NULL, PRVGA_priority, NULL );
	xTaskCreate (lcd_set_mode, "lcd_mode", configMINIMAL_STACK_SIZE, NULL, lcd_mode_priority, NULL );
	xTaskCreate (vkeyboardHandlerTask, "keyboardTask", configMINIMAL_STACK_SIZE, NULL, keyboard_priority, NULL);

	return;
}
