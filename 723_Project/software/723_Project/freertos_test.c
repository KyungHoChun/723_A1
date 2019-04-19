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

// used to delete a task
TaskHandle_t xHandle;

/* Definition of Semaphore */
SemaphoreHandle_t semAdd;
SemaphoreHandle_t semManager;
SemaphoreHandle_t semRoc;
SemaphoreHandle_t semTimer;
SemaphoreHandle_t semSwitch;

/* Queue Definitions */
xQueueHandle Q_freq_data;
xQueueHandle Q_network_stat;
xQueueHandle Q_timer_reset;
xQueueHandle QBtn;
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
} sLoadStat;

typedef struct
{
	unsigned int iSheddedLoad;
	unsigned int iUnSheddedLoad;
} sHandledLoadStat;

int iMode = normal;
int iTimerFlag = 0;
int iTimerFin = 0;

static double dThresFreq = 49; //for test purpose - delete later
static double dThresRoc = 20; //for test purpose - delete later

/* Local Function Prototypes */
void initOSDataStructs(void);
void initCreateTasks(void);
void vISRFreqRelay(void);
void vISRButtonMode(void);

/* VGA Variables */
/* For frequency plot */
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
FILE *lcd;
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

/* Shows what the current mode is through LCD Display on push button press*/
void vTaskLCDMode(void *pvParameters)
{
	int iBtn;
	lcd = fopen(CHARACTER_LCD_NAME, "w");
	fprintf (lcd, "MAINTENANCE MODE\n");
	fclose(lcd);

	while (1)
	{
		if (xQueueReceive(QBtn, &iBtn, portMAX_DELAY) == pdTRUE)
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
void PRVGADraw_Task(void *pvParameters){
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

/* ISR Push button mode */
void vISRButtonMode(void* context, alt_u32 id)
{
	volatile int* button = (volatile int*) context;
	/* Cast the context */
	(*button) = IORD_ALTERA_AVALON_PIO_DATA(PUSH_BUTTON_BASE);
	int btn = (int) *button;

	xQueueSendToBackFromISR(QBtn, &btn, pdFALSE);

	/* Clears the edge capture register */
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

/* ISR for keyboard inputs */
void vISRKeyboard(void* context, alt_u32 id)
{
	char cAscii;
	int iStatus = 0;
	unsigned char cKey = 0;
	KB_CODE_TYPE decode_mode;
	iStatus = decode_scancode (context, &decode_mode , &cKey , &cAscii);

	/* If keyboard press has been read */
	if (iStatus)
	{
		if (cKey == 0x5A){
			cAscii = 0x0D;
		}
		if (decode_mode != KB_LONG_BINARY_MAKE_CODE && decode_mode != KB_BREAK_CODE){
			xQueueSendToBackFromISR(Q_keyboard, &cAscii, 0);
		}
	}
}

/* Handles the inputs of the keyboard, changes the thresholds of the roc and frequency */
void vTaskKeyboardHandler (void * pvParameters)
{
	char cKeyInput;
	unsigned int val;
	unsigned int inputs[2] = {0, 0};
	int i = 0;

	while (1){
		while(uxQueueMessagesWaiting(Q_keyboard) != 0){
			xQueueReceive(Q_keyboard, (void*) &cKeyInput, 0);
			printf("print: %d \n", cKeyInput-0x30);

			/*check if the input number is valid*/
			if (cKeyInput >= 0x30 && cKeyInput <=0x39){
				val = cKeyInput-0x30;
				inputs[i] = inputs[i]*10 + val;
			}

			/* check the keyboard input is comma*/
			else if (cKeyInput == 0x2C)
			{
				i++;
				if (i >= 2){
					xSemaphoreTake(semRoc, portMAX_DELAY);
					dThresFreq = inputs[0];
					dThresRoc = inputs[1];
					inputs[0] = 0;
					inputs[1] = 0;
					i = 0;
					printf("New Value: %f,  %f\n", dThresFreq, dThresRoc);
					xSemaphoreGive(semRoc);

				}
			}
			/* check if the keyboard input is enter*/
			else if (cKeyInput == 0x0D){
				xSemaphoreTake(semRoc, portMAX_DELAY);
				dThresFreq = inputs[0];
				dThresRoc = inputs[1];
				inputs[0] = 0;
				inputs[1] = 0;
				i = 0;
				printf("New Value: %f,  %f\n", dThresFreq, dThresRoc);
				xSemaphoreGive(semRoc);
			}
		}
		vTaskDelay(10);
	}
}

/* Reads incoming frequency from ISR*/
void vISRFreqRelay()
{
	double dInFreq = SAMPLING_FREQ / (double) IORD(FREQUENCY_ANALYSER_BASE, 0);

	/* Send calculation tasks */
	xQueueSendToBackFromISR(Q_freq_data, &dInFreq, pdFALSE);
	return;
}

/* Receives frequency from the relay and calculates ROC*/
void vTaskNetworkStat(void * pvParameters)
{
	double dFreqNew;
	double dFreqPrev = 50.0;
	double dRocVal = 0.0;
	int iStable = 1;

	while(1)
	{
		if(xQueueReceive(Q_freq_data, &dFreqNew, portMAX_DELAY) == pdTRUE)
			{
			xSemaphoreTake(semRoc, portMAX_DELAY);
			//Calculate Rate of Change Value
			dRocVal = ((dFreqNew-dFreqPrev)*SAMPLING_FREQ) / IORD(FREQUENCY_ANALYSER_BASE, 0);
			dFreqPrev = dFreqNew;
			xSemaphoreGive(semRoc);

			if(dFreqNew < dThresFreq || fabs(dRocVal) > dThresRoc)
			{
				iStable = 0;
			}
			else{
				iStable = 1;
			}
			xQueueSend(Q_network_stat, &iStable, 0);
		}
		vTaskDelay(10);
	}
}

/* Switch controller task*/
void vTaskSwitchCon(void *pvParameters)
{
	unsigned int iCurrentLoad;
	while(1){
		xSemaphoreTake(semSwitch, pdFALSE);
		iCurrentLoad = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F; //read slide switches
		xQueueSend(Q_switch, &iCurrentLoad, portMAX_DELAY);
		xSemaphoreGive(semManager);
		vTaskDelay(10);
	}
}


/* Load manager task*/
void vTaskLoadManager(void * pvParameters)
{
	int iStable;
	int iFlagShed = 0;
	int iFlagAdd = 0;
	int iTimerActive = 0;
	unsigned int iCurrentLoad;

	sLoadStat sLoadStat;
	sHandledLoadStat sHandledLoadStat;

	sLoadStat.iSheddedLoad = 0x00;
	sLoadStat.iUnSheddedLoad = 0x00;

	while(1)
	{
		if(uxQueueMessagesWaiting(Q_network_stat) == pdTRUE)
		{
			if(xQueueReceive(Q_switch, &iCurrentLoad, 0) == pdTRUE)
			{
				sLoadStat.iCurrentLoad = iCurrentLoad;
				sLoadStat.iSheddedLoad &= iCurrentLoad;
				sLoadStat.iUnSheddedLoad &= iCurrentLoad;
				sHandledLoadStat.iSheddedLoad &= iCurrentLoad;
				sHandledLoadStat.iUnSheddedLoad &= iCurrentLoad;

			}

			if(xQueueReceive(Q_network_stat, &iStable, 0) == pdTRUE && sLoadStat.iCurrentLoad != 0 && iMode == normal){
				/*
				SHED
				If timer is not active and unstable - start timer - shed load
				If timer is active (from stable) and has become unstable - reset timer - shed load

				If timer has finished and unstable - reset timer - shed load
				If timer has finished (from unstable) and has become stable - reset timer
					If a load was shed - add load

				ADD
				If timer has finished (from unstable) and has become stable - reset timer
					 If a load was shed - add load
				 */

				// If timer is active (from stable) and has become unstable - reset timer - shed load
				if(iTimerFlag && !iTimerFin && !iStable){
					xSemaphoreTake(semTimer, portMAX_DELAY);
					iTimerFlag = 1;
					iFlagShed = 1;
					iTimerActive = 1;
					xSemaphoreGive(semTimer);
				}
				// If timer is not active and unstable - start timer - shed load
				else if(!iTimerFlag && !iTimerFin && !iStable){
					xSemaphoreTake(semTimer, portMAX_DELAY);
					iTimerFlag = 1;
					iFlagShed = 1;
					iTimerActive = 1;
					xSemaphoreGive(semTimer);
				}
				// If timer has finished and unstable - reset timer - shed load
				else if(!iTimerFlag && iTimerFin && !iStable){
					xSemaphoreTake(semTimer, portMAX_DELAY);
					iTimerFlag = 1;
					iFlagShed = 1;
					iTimerFin = 0;
					iTimerActive = 1;
					xSemaphoreGive(semTimer);
				}
				// If timer has finished (from unstable) and has become stable - reset timer
				else if(!iTimerFlag && iTimerFin && iStable){
					// if a load was shed - add load
					if(sLoadStat.iSheddedLoad > 0){
						xSemaphoreTake(semTimer, pdFALSE);
						iFlagAdd = 1;
						iTimerFin = 0;
						iTimerActive = 1;
						xSemaphoreGive(semTimer);
					}
				}

				if(iTimerActive){
					iTimerActive = 0;
					if (xTimerStart(timer500, 10) != pdPASS){
						printf("Cannot start 500ms timer\n");
					}
				}

				if(iFlagShed){
					xQueueSend(Q_load_stat, &sLoadStat, portMAX_DELAY);
				}
				else if(iFlagAdd){
					xQueueSend(Q_a_load_stat, &sLoadStat, portMAX_DELAY);
				}
			}
			else{
				sLoadStat.iSheddedLoad = 0;
				sLoadStat.iUnSheddedLoad = 0;
				sHandledLoadStat.iSheddedLoad = 0;
				sHandledLoadStat.iUnSheddedLoad = 0;
			}

			if (xQueueReceive(Q_ms_load_stat, &sHandledLoadStat, 0) == pdTRUE && iFlagShed){
				printf("RECEIVE SHED\n");
				iFlagShed = 0;
				sLoadStat.iSheddedLoad = sHandledLoadStat.iSheddedLoad;
				sLoadStat.iUnSheddedLoad = sHandledLoadStat.iUnSheddedLoad;
			}
			else if (xQueueReceive(Q_ma_load_stat, &sHandledLoadStat, 0) == pdTRUE && iFlagAdd){
				printf("ADD SHED\n");
				iFlagAdd = 0;
				sLoadStat.iSheddedLoad = sHandledLoadStat.iSheddedLoad;
				sLoadStat.iUnSheddedLoad = sHandledLoadStat.iUnSheddedLoad;
			}

			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, sLoadStat.iSheddedLoad & 0x1F);
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, (sLoadStat.iSheddedLoad ^ sLoadStat.iCurrentLoad) & 0x1F);
		}
		vTaskDelay(10);
	}
}

/* Shed load task */
void vTaskShed(void *pvParameters)
{
	sLoadStat sLoadStat;
	sHandledLoadStat sHandledLoadStat;

	while(1)
	{
		if(uxQueueMessagesWaiting(Q_load_stat) != 0){
			xQueueReceive(Q_load_stat,  (void *) &sLoadStat, portMAX_DELAY);

			if((sLoadStat.iCurrentLoad & 0x01) == 0x01 && (sLoadStat.iSheddedLoad & 0x01) != 0x01){
				sLoadStat.iSheddedLoad |= 0x01;
				sLoadStat.iUnSheddedLoad &= ~01;
			}
			else if((sLoadStat.iCurrentLoad & 0x02) == 0x02 && (sLoadStat.iSheddedLoad & 0x02) != 0x02){
				sLoadStat.iSheddedLoad |= 0x02;
				sLoadStat.iUnSheddedLoad &= ~0x02;
			}
			else if((sLoadStat.iCurrentLoad & 0x04) == 0x04 && (sLoadStat.iSheddedLoad & 0x04) != 0x04){
				sLoadStat.iSheddedLoad |= 0x04;
				sLoadStat.iUnSheddedLoad &= ~0x04;
			}
			else if((sLoadStat.iCurrentLoad & 0x08) == 0x08 && (sLoadStat.iSheddedLoad & 0x08) != 0x08){
				sLoadStat.iSheddedLoad |= 0x08;
				sLoadStat.iUnSheddedLoad &= ~0x08;
			}
			else if((sLoadStat.iCurrentLoad & 0x16) == 0x16 && (sLoadStat.iSheddedLoad & 0x16) != 0x16){
				sLoadStat.iSheddedLoad |= 0x16;
				sLoadStat.iUnSheddedLoad &= ~0x16;
			}

			sHandledLoadStat.iSheddedLoad = sLoadStat.iSheddedLoad;
			sHandledLoadStat.iUnSheddedLoad = sLoadStat.iUnSheddedLoad;

			xQueueSend(Q_ms_load_stat, (void*)&sHandledLoadStat, portMAX_DELAY);
		}
		vTaskDelay(10);
	}
	return;
}

/* Add load task */
void vTaskAdd(void *pvParameters)
{
	sLoadStat sLoadStat;
	sHandledLoadStat sHandledLoadStat;

	while(1)
	{
		if(uxQueueMessagesWaiting(Q_a_load_stat) != 0){
			xQueueReceive(Q_a_load_stat,  (void *) &sLoadStat, portMAX_DELAY);
			printf("b Add iSheddedLoad: %d\n",sHandledLoadStat.iSheddedLoad);
			printf("b Add iUnSheddedLoad: %d\n",sHandledLoadStat.iUnSheddedLoad);

			if((sLoadStat.iCurrentLoad & 0x16) == 0x16 && (sLoadStat.iSheddedLoad & 0x16) == 0x16){
				sLoadStat.iSheddedLoad &= ~0x16;
				sLoadStat.iUnSheddedLoad |= 0x16;
			}
			else if((sLoadStat.iCurrentLoad & 0x08) == 0x08 && (sLoadStat.iSheddedLoad & 0x08) == 0x08){
				sLoadStat.iSheddedLoad &= ~0x08;
				sLoadStat.iUnSheddedLoad |= 0x08;
			}
			else if((sLoadStat.iCurrentLoad & 0x04) == 0x04 && (sLoadStat.iSheddedLoad & 0x04) == 0x04){
				sLoadStat.iSheddedLoad &= ~0x04;
				sLoadStat.iUnSheddedLoad |= 0x04;
			}
			else if((sLoadStat.iCurrentLoad & 0x02) == 0x02 && (sLoadStat.iSheddedLoad & 0x02) == 0x02){
				sLoadStat.iSheddedLoad &= ~0x02;
				sLoadStat.iUnSheddedLoad |= 0x02;
			}
			else if((sLoadStat.iCurrentLoad & 0x01) == 0x01 && (sLoadStat.iSheddedLoad & 0x01) == 0x01){
				sLoadStat.iSheddedLoad &= ~0x01;
				sLoadStat.iUnSheddedLoad |= 0x01;
			}

			sHandledLoadStat.iSheddedLoad = sLoadStat.iSheddedLoad;
			sHandledLoadStat.iUnSheddedLoad = sLoadStat.iUnSheddedLoad;

			xQueueSend(Q_ma_load_stat, (void*)&sHandledLoadStat, portMAX_DELAY);
		}
		vTaskDelay(10);
	}
	return;
}

/* Timer functions*/
void vTimer500_Callback(xTimerHandle t_timer)
{
	//xSemaphoreTake(timer_sem, portMAX_DELAY);
	iTimerFlag = 0;
	iTimerFin = 1;
	xSemaphoreGiveFromISR(semTimer, pdTRUE);
	return;
}

void main(int argc, char* argv[], char* envp[])
{
	/* Keyboard set up */
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	if(ps2_device == NULL){
		printf("can't find PS/2 device\n");
	}

	alt_up_ps2_clear_fifo (ps2_device) ;
	alt_up_ps2_enable_read_interrupt(ps2_device);
	alt_irq_register(PS2_IRQ, ps2_device, vISRKeyboard);

	/* Register the PS/2 interrupt */
	IOWR_8DIRECT(PS2_BASE,4,1);

	/* Start Frequency ISR */
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, vISRFreqRelay);

	/* Push button setup */
	int iButtonValue = 0;

	/* Clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.*/
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x07);

	/* Enable interrupts for key1 */
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x01);

	/* Register the ISR for push button */
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&iButtonValue, vISRButtonMode);

	/* Create Timer */
	timer500 = xTimerCreate("Timer500", pdMS_TO_TICKS(500), pdFALSE, NULL, vTimer500_Callback);

	/* Initialize semaphores, queues, tasks and scheduler */
	initOSDataStructs();
	initCreateTasks();
	vTaskStartScheduler();

	for(;;);
	return;
}

/* This function creates queues and semaphores */
void initOSDataStructs(void)
{
	/* Create Semaphores */
	semManager = xSemaphoreCreateBinary();
	semRoc = xSemaphoreCreateMutex();
	semSwitch = xSemaphoreCreateBinary();
	semTimer = xSemaphoreCreateMutex();

	/* Create Queues */
	Q_freq_data = xQueueCreate( 100, sizeof( double ) );
	Q_load_stat = xQueueCreate( 1, sizeof( sLoadStat ) );
	Q_a_load_stat = xQueueCreate( 1, sizeof( sLoadStat ) );
	Q_ma_load_stat = xQueueCreate( 1, sizeof( sHandledLoadStat ) );
	Q_ms_load_stat = xQueueCreate( 1, sizeof( sHandledLoadStat ) );

	Q_network_stat = xQueueCreate( 10, sizeof( int ) );
	Q_switch = xQueueCreate( 10, sizeof( unsigned int ) );
	QBtn = xQueueCreate(100, sizeof(int));

	Q_keyboard = xQueueCreate(10, sizeof (char));
	Q_timer_reset = xQueueCreate( 4, sizeof( int ) );

	return;
}

/* This function creates the tasks */
void initCreateTasks(void)
{
	xTaskCreate(vTaskAdd, "Add Task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(vTaskShed, "Shed Task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(vTaskNetworkStat, "Network Status Task", configMINIMAL_STACK_SIZE, NULL, 6, NULL);
	xTaskCreate(vTaskLoadManager, "Load Manager Task", configMINIMAL_STACK_SIZE, NULL, 6, NULL);

	xTaskCreate(vTaskSwitchCon, "Switch Controller Task", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(PRVGADraw_Task, "PR VGA Draw Task", configMINIMAL_STACK_SIZE, NULL, 7, NULL);
	xTaskCreate(vTaskLCDMode, "LCD Mode Task", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(vTaskKeyboardHandler, "Keyboard Handler Task", configMINIMAL_STACK_SIZE, NULL, 4, NULL);

	return;
}
