/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_lpuart_freertos.h"
#include "fsl_lpuart.h"
#include "fsl_gpio.h"
#include "fsl_common.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_LPUART LPUART1
#define DEMO_LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()
#define DEMO_LPUART_IRQn LPUART1_IRQn
/* Task priorities. */
#define uart_task_PRIORITY 		(configMAX_PRIORITIES - 1)
#define button_task_PRIORITY 	(configMAX_PRIORITIES - 2)
/* Button definitions */
#define USER_SW_GPIO BOARD_USER_BUTTON_GPIO
#define USER_SW_GPIO_PIN BOARD_USER_BUTTON_GPIO_PIN
#define USER_SW_IRQ BOARD_USER_BUTTON_IRQ
#define USER_GPIO_IRQHandler BOARD_USER_BUTTON_IRQ_HANDLER
#define USER_SW_NAME BOARD_USER_BUTTON_NAME
/* Bit set for group event button */
#define MODE_0 ( 1 << 0 )
#define MODE_1 ( 1 << 1 )
/* Number of mode */
#define MAXMODE 2U
/* Mutex to protect script sent */
SemaphoreHandle_t cmdMutex = NULL;
/* Semaphore to protect count pressed */
SemaphoreHandle_t countSemaphore = NULL;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void uart_task(void *pvParameters);
static void button_task(void *pvParameters);
/*!
 * @brief delay a while.
 */
static void delay(uint32_t time);
/*!
 * @brief copy data in mode to buffer transferred
 */
static unsigned int copyBuffer(char* data, char** des);
static void freeBuffer(void);
static unsigned int allocBufferArray();

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Whether the SW is turned on */
volatile bool g_InputSignal = false;
volatile uint8_t countPressed = 0;
EventGroupHandle_t EventGroup_Button;
/*******************************************************************************
 * Code
 ******************************************************************************/
const char *mode1[]               = {	"Movement with mode 1\r\n",
										"#2P1450S1000\r\n",
										"#0P700S1000\r\n",
										"#4P600S1000#3P750S1000#1P1300S1000\r\n",
										"#4P2000S1000\r\n",
										"#1P1400S1000\r\n",
										"#5P2350S100\r",
										"#5P600S1000#0P1450S1000\r\n",
										"#5P1350S1000\r\n",
										"#4P600S10000\r\n",
										"#3P1450S1000#1P2000S1000#2P2000S1000\r\n"};
const char *mode2[]               = {	"Movement with mode 2\r\n",
										"#2P1450S1000\r\n",
										"#0P700S1000\r\n",
										"#4P600S1000#3P750S1000#1P1300S1000\r\n",
										"#4P2000S1000\r\n",
										"#1P1400S1000\r\n",
										"#5P2350S100\r",
										"#5P600S1000#0P1450S1000\r\n",
										"#5P1350S1000\r\n",
										"#4P600S10000\r\n",
										"#3P1450S1000#1P2000S1000#2P2000S1000\r\n"};
typedef struct scenario_buffer {
	char** data_to_send;
	uint8_t size;
} scenario_buffer_t;
scenario_buffer_t buf_mode;
uint8_t background_buffer[32];
uint8_t recv_buffer[4];

lpuart_rtos_handle_t handle;
struct _lpuart_handle t_handle;

lpuart_rtos_config_t lpuart_config = {
    .baudrate    = 115200,
    .parity      = kLPUART_ParityDisabled,
    .stopbits    = kLPUART_OneStopBit,
    .buffer      = background_buffer,
    .buffer_size = sizeof(background_buffer),
};

gpio_pin_config_t pin_gpio_button_config = {
	.direction 		= kGPIO_DigitalInput,
	.outputLogic 	= 0,
	.interruptMode 	= kGPIO_IntRisingEdge,
};
/*!
 * @brief Interrupt service function of switch.
 */
void USER_GPIO_IRQHandler(void)
{
	// Clear flag on interrupt
	GPIO_PortClearInterruptFlags(USER_SW_GPIO, 1U << USER_SW_GPIO_PIN);
	// set global variable for signaling that button is pressed
	g_InputSignal = true;
	countPressed++;
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    /* Init input switch GPIO. */
    EnableIRQ(USER_SW_IRQ);
    GPIO_PinInit(USER_SW_GPIO, USER_SW_GPIO_PIN, &pin_gpio_button_config);
    /* Enable GPIO pin interrupt */
    GPIO_PortEnableInterrupts(USER_SW_GPIO, 1 << USER_SW_GPIO_PIN);

    NVIC_SetPriority(LPUART1_IRQn, 5);

    EventGroup_Button = xEventGroupCreate();
    if (EventGroup_Button == NULL)
    {
    	PRINTF("Event Group creation failed!\r\n");
    	while(1);
    }
    countSemaphore = xSemaphoreCreateBinary();
    if (countSemaphore == NULL)
	{
		/* There was insufficient heap memory available for the mutex to be
		created. */
		PRINTF("Semaphore Binary creation failed!\r\n");
		while(1);
	}
    cmdMutex = xSemaphoreCreateMutex();
	if (cmdMutex == NULL)
	{
		/* There was insufficient heap memory available for the mutex to be
		created. */
		PRINTF("Mutex creation failed!\r\n");
		while(1);
	}
    if (xTaskCreate(uart_task, "Uart_task", configMINIMAL_STACK_SIZE + 10, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Task creation failed!\r\n");
        while (1);
    }
    if (xTaskCreate(button_task, "Button_task", configMINIMAL_STACK_SIZE + 50, NULL, button_task_PRIORITY, NULL) != pdPASS)
    {
    	PRINTF("Task creation failed!\r\n");
    	while(1);
    }
    vTaskStartScheduler();
    for (;;);
}

/*!
 * @brief Task responsible for loopback.
 */
static void uart_task(void *pvParameters)
{
    int error;
    char *buffer = NULL;
    const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
    EventBits_t modeBits;
    lpuart_config.srcclk = DEMO_LPUART_CLK_FREQ;
    lpuart_config.base   = DEMO_LPUART;

    if (0 > LPUART_RTOS_Init(&handle, &t_handle, &lpuart_config))
    {
        vTaskSuspend(NULL);
    }

    /* Receive user input and send it back to terminal. */
    do
    {
    	modeBits = xEventGroupWaitBits(EventGroup_Button, MODE_0 | MODE_1, pdTRUE, pdFALSE, xTicksToWait);
    	if ((modeBits & MODE_0) == MODE_0 || (modeBits & MODE_1) == MODE_1)
    	{
    		if (xSemaphoreTake(cmdMutex, 0) == pdPASS)
    		{
    			for (unsigned int i = 0; i < buf_mode.size; i++)
    			{
    				buffer = buf_mode.data_to_send[i];
    				/* Send introduction message. */
					error = LPUART_RTOS_Send(&handle, (uint8_t *)buffer, strlen(buffer));
					if (kStatus_Success != error)
					{
						break;
					}
					delay(15000000);
    			}
    			freeBuffer();
    			xSemaphoreGive(cmdMutex);
    		}
    	}
    	else
    	{
    		continue;
    	}
    } while (1);

    LPUART_RTOS_Deinit(&handle);
    vTaskSuspend(NULL);
}

static void button_task(void *pvParameters)
{
	uint8_t tmp;
	while(1)
	{
		if (g_InputSignal)
		{
			delay(100000000);
			if (GPIO_PinRead(USER_SW_GPIO, USER_SW_GPIO_PIN) == 1)
			{
				tmp = countPressed;
				countPressed = 0;
				if (xSemaphoreTake(cmdMutex, 0) == pdPASS)
				{
					switch(tmp) {
						case 1:
							/* The semaphore was ‘taken’ successfully, so the resource it is
							guarding can be accessed safely. */
							buf_mode.size = (uint8_t)sizeof(mode1)/sizeof(mode1[0]);
							allocBufferArray(&buf_mode.data_to_send);
							for (unsigned int i = 0; i < buf_mode.size; i++)
							{
								copyBuffer((char*)mode1[i], &buf_mode.data_to_send[i]);
							}
							/* Access to the resource the semaphore is guarding is complete, so
							the semaphore must be ‘given’ back. */
							xSemaphoreGive(cmdMutex);
							/* Notify on event group */
							xEventGroupSetBits(EventGroup_Button, MODE_0);
							break;
						case 2:
							/* The semaphore was ‘taken’ successfully, so the resource it is
							guarding can be accessed safely. */
							buf_mode.size = (uint8_t)sizeof(mode2)/sizeof(mode2[0]);
							allocBufferArray();
							for (unsigned int i = 0; i < buf_mode.size; i++)
							{
								copyBuffer((char*)mode2[i], &buf_mode.data_to_send[i]);
							}
							/* Access to the resource the semaphore is guarding is complete, so
							the semaphore must be ‘given’ back. */
							xSemaphoreGive(cmdMutex);
							/* Notify on event group */
							xEventGroupSetBits(EventGroup_Button, MODE_1);
							break;
						default:
							xSemaphoreGive(cmdMutex);
							break;
					}
				}
			}
			g_InputSignal = false;
		}
	}
}

static void delay(uint32_t time)
{
    for (uint32_t i = 0; i < time; ++i)
    {
        __NOP(); /* delay */
    }
}

static unsigned int allocBufferArray()
{
	buf_mode.data_to_send = (char**) malloc(buf_mode.size * sizeof(char*));
	if (!buf_mode.data_to_send)
	{
		PRINTF("Out of memory");
		return -1;
	}
	return 0;
}

static unsigned int copyBuffer(char* data, char** des)
{
	*des = (char*)calloc(strlen(data), sizeof(char));
	if (!(*des))
	{
		PRINTF("Out of memory");
		return -1;
	}
	strcpy(*des, data);
	return 0;
}

static void freeBuffer(void)
{
	if (buf_mode.data_to_send != NULL)
	{
		for (unsigned int i = 0; i < buf_mode.size; i++)
		{
			free(buf_mode.data_to_send[i]);
			buf_mode.data_to_send[i] = NULL;
		}
		free(buf_mode.data_to_send);
		buf_mode.data_to_send = NULL;
	}
}
