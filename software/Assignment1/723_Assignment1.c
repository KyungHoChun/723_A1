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
SemaphoreHandle_t semThres;
SemaphoreHandle_t semTimer;
SemaphoreHandle_t semResp;

/* Queue Definitions */
xQueueHandle QFreqData;
xQueueHandle QFreqDataVGA;
xQueueHandle QNetworkStat;
xQueueHandle QBtn;
xQueueHandle QSwitch;
xQueueHandle QShedLoad;
xQueueHandle QAddLoad;
xQueueHandle QAddedShed;
xQueueHandle QAddedLoad;
xQueueHandle QKeyboard;
xQueueHandle QStability;

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

typedef struct
{
	unsigned int freq;
	unsigned int stability;
} sVGAData;

int iRecentFiveRec[5] = {0,0,0,0,0};
int iMaxTime = 0;
int iMinTime = 10;
int iAvgTime = 0;

int iMode = normal;
int iTimerFlag = 0;
int iTimerFin = 0;

static double dThresFreq = 49; //for test purpose - delete later
static double dThresRoc = 20; //for test purpose - delete later

/* Local Function Prototypes */
void initOSDataStructs(void);
void initCreateTasks(void);

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

/* VGA Display */
void vTaskPRVGADraw(void *pvParameters){
	/* Initialize VGA controllers */
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);

	if(pixel_buf == NULL)
	{
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL)
	{
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);

	/* Set up plot axes */
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

	/* Prints current threshold on VGA display - col, row */
	alt_up_char_buffer_string(char_buf, "723 Assignment 1 - Group 10", 4, 0);
	alt_up_char_buffer_string(char_buf, "Kyung Ho Chun & Yunseung Lee", 4, 2);

	alt_up_char_buffer_string(char_buf, "System Active Time (s):", 42, 2);

	alt_up_char_buffer_string(char_buf, "Frequency Threshold(Hz):", 4, 40);
	alt_up_char_buffer_string(char_buf, "RoC Threshold(Hz):", 4, 43);
	alt_up_char_buffer_string(char_buf, "Stability:", 4, 49);
	alt_up_char_buffer_string(char_buf, "Mode:", 4, 46);

	alt_up_char_buffer_string(char_buf, "Maximum time (ms):", 49, 40);
	alt_up_char_buffer_string(char_buf, "Minimum time (ms):", 49, 43);
	alt_up_char_buffer_string(char_buf, "Average time (ms):", 49, 46);
	alt_up_char_buffer_string(char_buf, "5 Recent response times (Left is most recent) in ms:", 4, 52);

	int iVgaStable;
	double dSystemTime;
	char cFreqThresStr[4] = "";
	char cRocThresStr[4] = "";
	char cTimeStr[10] = "";
	char cMaxStr[4] = "";
	char cMinStr[4] = "";
	char cAvgStr[4] = "";
	char cRecentRecord1[6] = "";
	char cRecentRecord2[6] = "";
	char cRecentRecord3[6] = "";
	char cRecentRecord4[6] = "";
	char cRecentRecord5[6] = "";

	double freq[100], dfreq[100];
	int i = 99, j = 0;
	Line line_freq, line_roc;

	while(1)
	{
		/* Display frequency and ROC thresholds */
		sprintf(cFreqThresStr, "%.2f", dThresFreq);
		alt_up_char_buffer_string(char_buf, cFreqThresStr, 30, 40);
		sprintf(cRocThresStr, "%.2f", dThresRoc);
		alt_up_char_buffer_string(char_buf, cRocThresStr, 25, 43);

		/* System Time */
		dSystemTime = xTaskGetTickCount() / 1000.0;
		sprintf(cTimeStr, "%.2f", dSystemTime);
		alt_up_char_buffer_string(char_buf, cTimeStr, 70, 2);

		/* Display max, average and min response times */
		sprintf(cMaxStr, "%u  ", iMaxTime);
		alt_up_char_buffer_string(char_buf, cMaxStr, 69, 40);
		sprintf(cMinStr, "%u  ", iMinTime);
		alt_up_char_buffer_string(char_buf, cMinStr, 69, 43);
		sprintf(cAvgStr, "%u  ", iAvgTime);
		alt_up_char_buffer_string(char_buf, cAvgStr, 69, 46);

		/* Display the 5 most recent response times */
		sprintf(cRecentRecord5, "%d  ", iRecentFiveRec[4]);
		alt_up_char_buffer_string(char_buf, cRecentRecord5, 4, 54);
		sprintf(cRecentRecord4, "%d  ", iRecentFiveRec[3]);
		alt_up_char_buffer_string(char_buf, cRecentRecord4, 9, 54);
		sprintf(cRecentRecord3, "%d  ", iRecentFiveRec[2]);
		alt_up_char_buffer_string(char_buf, cRecentRecord3, 14, 54);
		sprintf(cRecentRecord2, "%d  ", iRecentFiveRec[1]);
		alt_up_char_buffer_string(char_buf, cRecentRecord2, 19, 54);
		sprintf(cRecentRecord1, "%d  ", iRecentFiveRec[0]);
		alt_up_char_buffer_string(char_buf, cRecentRecord1, 24, 54);


		/* Display the current mode of the system */
		if(iMode == 0)
		{
			alt_up_char_buffer_string(char_buf, "Normal      ", 11, 46);
		}
		else
		{
			alt_up_char_buffer_string(char_buf, "Maintenance", 11, 46);
		}


		/* Receive the stability status of the system */
		if(uxQueueMessagesWaiting(QStability) != 0)
		{
			xQueueReceive(QStability, (void *) &iVgaStable, portMAX_DELAY);

			if (iVgaStable == 0)
			{
				alt_up_char_buffer_string(char_buf, "Unstable", 15, 49);
			}
			else
			{
				alt_up_char_buffer_string(char_buf, "Stable  ", 15, 49);
			}
		}

		/* Receive frequency data from queue */
		while(uxQueueMessagesWaiting(QFreqDataVGA) != 0)
		{
			xQueueReceive(QFreqDataVGA, freq+i, 0);

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
			if (((int)(freq[(i+j)%100]) > MIN_FREQ) && ((int)(freq[(i+j+1)%100]) > MIN_FREQ))
			{
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
		vTaskDelay(5);
	}
}

/* ISR Push button mode */
void vISRButtonMode(void* context, alt_u32 id)
{
	volatile int* button = (volatile int*) context;
	/* Cast the context */
	(*button) = IORD_ALTERA_AVALON_PIO_DATA(PUSH_BUTTON_BASE);
	int btn = (int) *button;

	xQueueSendFromISR(QBtn, &btn, pdFALSE);

	/* Clears the edge capture register */
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	return;
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
	if (!iStatus)
	{
		if (cKey == 0x5A)
		{
			cAscii = 0x0D;
		}
		if (decode_mode != KB_LONG_BINARY_MAKE_CODE && decode_mode != KB_BREAK_CODE)
		{
			xQueueSendFromISR(QKeyboard, &cAscii, 0);
		}
	}
}

/* Shows what the current mode is through LCD Display on push button press*/
void vTaskLCDMode(void *pvParameters)
{
	int iBtn;
	lcd = fopen(CHARACTER_LCD_NAME, "w");
	fprintf (lcd, "NORMAL MODE\n");
	fclose(lcd);

	while (1)
	{
		if (xQueueReceiveFromISR(QBtn, &iBtn, portMAX_DELAY) == pdTRUE)
		{
			iMode = !iMode;

			lcd = fopen(CHARACTER_LCD_NAME, "w");
			if (lcd != NULL)
			{
				fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);

				if (iMode)
				{
					fprintf (lcd, "MAINTENANCE MODE\n");
				}
				else
				{
					fprintf(lcd, "NORMAL MODE\n");
				}
			}
			fclose(lcd);
		}
		vTaskDelay(5);
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
		if(uxQueueMessagesWaitingFromISR(QKeyboard) != 0)
		{
			xQueueReceiveFromISR(QKeyboard, (void*) &cKeyInput, 0);

			/* Check if the input number is valid */
			if (cKeyInput >= 0x30 && cKeyInput <=0x39)
			{
				val = cKeyInput-0x30;
				inputs[i] = inputs[i]*10 + val;
			}

			/* Check the keyboard input is comma */
			else if (cKeyInput == 0x2C)
			{
				i++;
				if (i >= 2)
				{
					xSemaphoreTake(semThres, portMAX_DELAY);
					dThresFreq = inputs[0];
					dThresRoc = inputs[1];
					xSemaphoreGive(semThres);
					inputs[0] = 0;
					inputs[1] = 0;
					i = 0;

				}
			}
			/* Check if the keyboard input is enter */
			else if (cKeyInput == 0x0D)
			{
				xSemaphoreTake(semThres, portMAX_DELAY);
				dThresFreq = inputs[0];
				dThresRoc = inputs[1];
				xSemaphoreGive(semThres);
				inputs[0] = 0;
				inputs[1] = 0;
				i = 0;
			}
		}
		vTaskDelay(2);
	}
	return;
}

/* Reads incoming frequency from ISR*/
void vISRFreqRelay(void* context, alt_u32 id)
{
	double dInFreq = SAMPLING_FREQ / (double) IORD(FREQUENCY_ANALYSER_BASE, 0);
	/* Send calculation tasks */
	xQueueSendToBackFromISR(QFreqData, &dInFreq, pdFALSE);
	return;
}

/* Switch controller task*/
void vTaskSwitchCon(void *pvParameters)
{
	unsigned int iCurrentLoad;
	while(1)
	{
		iCurrentLoad = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F; //read slide switches
		xQueueSend(QSwitch, &iCurrentLoad, portMAX_DELAY);
		vTaskDelay(2);
	}
}

/* Load manager task*/
void vTaskLoadManager(void * pvParameters)
{
	int iStable = 1;
	int iFlagShed = 0;
	int iFlagAdd = 0;
	int iTimerActive = 0;
	int iManagingLoad = 0;
	int iRespTimeFlag = 0;
	int iRespCount = 0;

	unsigned int iCurrentLoad;

	double dInFreq;
	double dFreqPrev = 50.0;
	double dRocVal = 0.0;

	sLoadStat sLoadStat;
	sHandledLoadStat sHandledLoadStat;

	TickType_t tStartTime = 0;
	TickType_t tEndTime = 0;
	TickType_t tTimeDiff = 0;
	TickType_t tSumTime = 0;

	sLoadStat.iSheddedLoad = 0x00;
	sLoadStat.iUnSheddedLoad = 0x00;

	while(1)
	{
		if(xQueueReceiveFromISR(QFreqData, &dInFreq, portMAX_DELAY) == pdTRUE)
		{
			/* Calculate rate of change */
			dRocVal = ((dInFreq-dFreqPrev)* SAMPLING_FREQ) / (double) IORD(FREQUENCY_ANALYSER_BASE, 0);
			dFreqPrev = dInFreq;

			if(dInFreq < dThresFreq || fabs(dRocVal) > dThresRoc)
			{
				iStable = 0;
				if(iRespTimeFlag == 0)
				{
					iRespTimeFlag = 1;
				}
			}
			else
			{
				iStable = 1;
			}

			/* Update switch configuration */
			if(xQueueReceive(QSwitch, &iCurrentLoad, 0) == pdTRUE && iManagingLoad)
			{
				sLoadStat.iCurrentLoad &= iCurrentLoad;
				sLoadStat.iSheddedLoad &= iCurrentLoad;
				sLoadStat.iUnSheddedLoad &= iCurrentLoad;
				sHandledLoadStat.iSheddedLoad &= iCurrentLoad;
				sHandledLoadStat.iUnSheddedLoad &= iCurrentLoad;

			}
			else
			{
				sLoadStat.iCurrentLoad = iCurrentLoad;
				sLoadStat.iUnSheddedLoad = iCurrentLoad;
			}

			xQueueSend(QStability, &iStable, 0);
			xQueueSendToBack(QFreqDataVGA, &dInFreq, 0);

			/* Handle logic in normal mode */
			if(sLoadStat.iCurrentLoad != 0 && iMode == normal)
			{
				/* If timer is not active and unstable - start timer - shed load */
				if(!iTimerFlag && !iTimerFin && !iStable && sLoadStat.iUnSheddedLoad > 0)
				{
					xSemaphoreTake(semTimer, portMAX_DELAY);
					iTimerFlag = 1;
					xSemaphoreGive(semTimer);
					iFlagShed = 1;
					iTimerActive = 1;
					tStartTime = xTaskGetTickCount();
				}
				/* If timer has finished and unstable - reset timer - shed load */
				else if(!iTimerFlag && iTimerFin && !iStable && sLoadStat.iUnSheddedLoad > 0)
				{
					xSemaphoreTake(semTimer, portMAX_DELAY);
					iTimerFlag = 1;
					iTimerFin = 0;
					xSemaphoreGive(semTimer);
					iFlagShed = 1;
					iTimerActive = 1;
					tStartTime = xTaskGetTickCount();
				}
				/* If timer has finished (from unstable) and has become stable - reset timer */
				else if(!iTimerFlag && iTimerFin && iStable)
				{
					/* If a load was shed - add load */
					if(sLoadStat.iSheddedLoad > 0){
						xSemaphoreTake(semTimer, pdFALSE);
						iTimerFin = 0;
						xSemaphoreGive(semTimer);
						iTimerActive = 1;
						iFlagAdd = 1;
					}
				}

				/* Start 500ms timer */
				if(iTimerActive)
				{
					iTimerActive = 0;
					if (xTimerStart(timer500, 10) != pdPASS)
					{
						printf("Cannot start 500ms timer\n");
					}
				}

				/* Send message to shed or add tasks */
				if(iFlagShed == 1)
				{
					xQueueSend(QShedLoad, &sLoadStat, portMAX_DELAY);
					if (!iManagingLoad)
					{
						iManagingLoad = 1;
					}
					iFlagShed = 2;
				}
				else if(iFlagAdd == 1)
				{
					iFlagAdd = 2;
					xQueueSend(QAddLoad, &sLoadStat, portMAX_DELAY);
				}
			}
			else
			{
				/* Reset all flags and switch configuration in maintenance mode */
				iTimerFin = 0;
				iTimerActive = 0;
				iFlagShed = 0;
				iFlagAdd = 0;
				iManagingLoad = 0;
				iRespTimeFlag = 0;

				sLoadStat.iSheddedLoad = 0x00;
				sLoadStat.iUnSheddedLoad = 0x00;
				sHandledLoadStat.iSheddedLoad = 0x00;
				sHandledLoadStat.iUnSheddedLoad = 0x00;
			}

			/* Logic after receiving new configuration after shed */
			if (xQueueReceive(QAddedShed, &sHandledLoadStat, 0) == pdTRUE && iFlagShed == 2)
			{
				iFlagShed = 0;
				sLoadStat.iSheddedLoad = sHandledLoadStat.iSheddedLoad;
				sLoadStat.iUnSheddedLoad = sHandledLoadStat.iUnSheddedLoad;

				/* Record the response time to shed */
				if (iRespTimeFlag == 1)
				{
					tEndTime = xTaskGetTickCount();
					iRespTimeFlag = 0;
					tTimeDiff = tEndTime - tStartTime;

					xSemaphoreTake(semResp,portMAX_DELAY);
					for (int i = 0; i < 4; i++)
					{
						iRecentFiveRec[i] = iRecentFiveRec [i+1];
					}

					iRecentFiveRec[4] = tTimeDiff;

					/* Calculate Max, Min and Average */
					if (tTimeDiff > iMaxTime)
					{
						iMaxTime = tTimeDiff;
					}
					else if (tTimeDiff < iMinTime)
					{
						iMinTime = tTimeDiff;
					}
					xSemaphoreGive(semResp);

					iRespCount += 1;
					tSumTime += tTimeDiff;
					iAvgTime = tSumTime / iRespCount;
				}
			}
			/* Logic after receiving new configuration after add */
			else if (xQueueReceive(QAddedLoad, &sHandledLoadStat, 0) == pdTRUE && iFlagAdd == 2)
			{
				if (sHandledLoadStat.iSheddedLoad == 0)
				{
					iManagingLoad = 0;
				}
				iFlagAdd = 0;
				sLoadStat.iSheddedLoad = sHandledLoadStat.iSheddedLoad;
				sLoadStat.iUnSheddedLoad = sHandledLoadStat.iUnSheddedLoad;
			}

			/* Output to red and green LED */
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, sLoadStat.iSheddedLoad & 0x1F);
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, (sLoadStat.iSheddedLoad ^ sLoadStat.iCurrentLoad) & 0x1F);
		}
		vTaskDelay(2);
	}
}

/* Shed load task */
void vTaskShed(void *pvParameters)
{
	sLoadStat sLoadStat;
	sHandledLoadStat sHandledLoadStat;

	while(1)
	{
		if(uxQueueMessagesWaiting(QShedLoad) != 0)
		{
			xQueueReceive(QShedLoad, (void *) &sLoadStat, portMAX_DELAY);

			if((sLoadStat.iCurrentLoad & 0x01) == 0x01 && (sLoadStat.iSheddedLoad & 0x01) != 0x01)
			{
				sLoadStat.iSheddedLoad |= 0x01;
				sLoadStat.iUnSheddedLoad &= ~01;
			}
			else if((sLoadStat.iCurrentLoad & 0x02) == 0x02 && (sLoadStat.iSheddedLoad & 0x02) != 0x02)
			{
				sLoadStat.iSheddedLoad |= 0x02;
				sLoadStat.iUnSheddedLoad &= ~0x02;
			}
			else if((sLoadStat.iCurrentLoad & 0x04) == 0x04 && (sLoadStat.iSheddedLoad & 0x04) != 0x04)
			{
				sLoadStat.iSheddedLoad |= 0x04;
				sLoadStat.iUnSheddedLoad &= ~0x04;
			}
			else if((sLoadStat.iCurrentLoad & 0x08) == 0x08 && (sLoadStat.iSheddedLoad & 0x08) != 0x08)
			{
				sLoadStat.iSheddedLoad |= 0x08;
				sLoadStat.iUnSheddedLoad &= ~0x08;
			}
			else if((sLoadStat.iCurrentLoad & 0x10) == 0x10 && (sLoadStat.iSheddedLoad & 0x10) != 0x10)
			{
				sLoadStat.iSheddedLoad |= 0x10;
				sLoadStat.iUnSheddedLoad &= ~0x10;
			}

			sHandledLoadStat.iSheddedLoad = sLoadStat.iSheddedLoad;
			sHandledLoadStat.iUnSheddedLoad = sLoadStat.iUnSheddedLoad;

			xQueueSend(QAddedShed, (void*)&sHandledLoadStat, portMAX_DELAY);
		}
		vTaskDelay(2);
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
		if(uxQueueMessagesWaiting(QAddLoad) != 0){
			xQueueReceive(QAddLoad, (void *) &sLoadStat, portMAX_DELAY);

			if((sLoadStat.iCurrentLoad & 0x10) == 0x10 && (sLoadStat.iSheddedLoad & 0x10) == 0x10)
			{
				sLoadStat.iSheddedLoad &= ~0x10;
				sLoadStat.iUnSheddedLoad |= 0x10;

			}
			else if((sLoadStat.iCurrentLoad & 0x08) == 0x08 && (sLoadStat.iSheddedLoad & 0x08) == 0x08)
			{
				sLoadStat.iSheddedLoad &= ~0x08;
				sLoadStat.iUnSheddedLoad |= 0x08;
			}
			else if((sLoadStat.iCurrentLoad & 0x04) == 0x04 && (sLoadStat.iSheddedLoad & 0x04) == 0x04)
			{
				sLoadStat.iSheddedLoad &= ~0x04;
				sLoadStat.iUnSheddedLoad |= 0x04;
			}
			else if((sLoadStat.iCurrentLoad & 0x02) == 0x02 && (sLoadStat.iSheddedLoad & 0x02) == 0x02)
			{
				sLoadStat.iSheddedLoad &= ~0x02;
				sLoadStat.iUnSheddedLoad |= 0x02;
			}
			else if((sLoadStat.iCurrentLoad & 0x01) == 0x01 && (sLoadStat.iSheddedLoad & 0x01) == 0x01)
			{
				sLoadStat.iSheddedLoad &= ~0x01;
				sLoadStat.iUnSheddedLoad |= 0x01;
			}

			sHandledLoadStat.iSheddedLoad = sLoadStat.iSheddedLoad;
			sHandledLoadStat.iUnSheddedLoad = sLoadStat.iUnSheddedLoad;

			xQueueSend(QAddedLoad, (void*)&sHandledLoadStat, portMAX_DELAY);
		}
		vTaskDelay(2);
	}
	return;
}

/* 500ms timer callback function */
void vTimer500_Callback(xTimerHandle t_timer)
{
	xSemaphoreTakeFromISR(semTimer, portMAX_DELAY);
	iTimerFlag = 0;
	iTimerFin = 1;
	xSemaphoreGiveFromISR(semTimer, portMAX_DELAY);
	return;
}

int main(int argc, char* argv[], char* envp[])
{
	/* Keyboard setup */
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
	return 0;
}

/* This function creates queues and semaphores */
void initOSDataStructs(void)
{
	/* Create Semaphores */
	semThres = xSemaphoreCreateMutex();
	semTimer = xSemaphoreCreateMutex();
	semResp = xSemaphoreCreateMutex();

	/* Create Queues */
	QFreqData = xQueueCreate(20, sizeof(double));
	QFreqDataVGA = xQueueCreate(20, sizeof(double));

	QShedLoad = xQueueCreate(1, sizeof(sLoadStat));
	QAddLoad = xQueueCreate(1, sizeof(sLoadStat));
	QAddedLoad = xQueueCreate(1, sizeof(sHandledLoadStat));
	QAddedShed = xQueueCreate(1, sizeof(sHandledLoadStat));

	QNetworkStat = xQueueCreate(10, sizeof(int));
	QSwitch = xQueueCreate(10, sizeof(unsigned int));
	QBtn = xQueueCreate(10, sizeof(int));

	QStability = xQueueCreate (1, sizeof (int));
	QKeyboard = xQueueCreate(10, sizeof (char));

	return;
}

/* This function creates the tasks */
void initCreateTasks(void)
{
	xTaskCreate(vTaskPRVGADraw, "PR VGA Draw Task", configMINIMAL_STACK_SIZE, NULL, 6, NULL);

	xTaskCreate(vTaskLoadManager, "Load Manager Task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(vTaskAdd, "Add Task", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(vTaskShed, "Shed Task", configMINIMAL_STACK_SIZE, NULL, 4, NULL);

	xTaskCreate(vTaskSwitchCon, "Switch Controller Task", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(vTaskLCDMode, "LCD Mode Task", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(vTaskKeyboardHandler, "Keyboard Handler Task", configMINIMAL_STACK_SIZE, NULL, 3, NULL);

	return;
}
