/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* Freescale includes. */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_freertos.h"
#include "fsl_lpi2c.h"
#include "fsl_lpi2c_cmsis.h"

#include "clock_config.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_LPUART LPUART1
#define DEMO_LPUART_CLK_FREQ BOARD_DebugConsoleSrcFreq()
#define DEMO_LPUART_IRQn LPUART1_IRQn
/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 2)
#define button_task_PRIORITY (configMAX_PRIORITIES - 1)
#define i2c_b2b_task_PRIORITY (configMAX_PRIORITIES - 1)
#define BTN_CYCLE_RATE_MS       20
#define I2C_CYCLE_RATE_MS       20
/* Button definitions */
#define USER_SW_GPIO BOARD_USER_BUTTON_GPIO
#define USER_SW_GPIO_PIN BOARD_USER_BUTTON_GPIO_PIN
#define USER_SW_IRQ BOARD_USER_BUTTON_IRQ
#define USER_GPIO_IRQHandler BOARD_USER_BUTTON_IRQ_HANDLER
#define USER_SW_NAME BOARD_USER_BUTTON_NAME
/* Bit set for group event button */
#define MODE_0 (1 << 0)
#define MODE_1 (1 << 1)
/* Number of mode */
#define MAXMODE 2U
/* Mutex to protect script sent */
SemaphoreHandle_t cmdMutex = NULL;
/* Semaphore to protect count pressed */
SemaphoreHandle_t countSemaphore = NULL;

#define EXAMPLE_I2C_SLAVE Driver_I2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define I2C_MASTER_SLAVE_ADDR_7BIT (0x7EU)
#define I2C_DATA_LENGTH (200) /* MAX is 256 */
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void uart_task(void *pvParameters);
static void button_task(void *pvParameters);
static void i2c_b2b_task(void *pvParameters);
/*!
 * @brief delay a while.
 */
static void delay(void);
/*!
 * @brief copy data in mode to buffer transferred
 */
static unsigned int copyBuffer(char *data, char **des);
static void freeBuffer(void);
static unsigned int allocBufferArray();
/*!
 * @brief initialize i2c slave
 */
void initI2C_b2b(void);
/*!
 * @brief analyze data received by i2c from master
 */
static void receiveDataI2C_b2b(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Whether the SW is turned on */
volatile bool g_InputSignal = false;
volatile uint8_t countPressed = 0;
EventGroupHandle_t EventGroup_Button;

uint8_t g_slave_buff[I2C_DATA_LENGTH];
volatile bool g_SlaveCompletionFlag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
const char positionInit[] = {
    "#0P1450S500#1P1600S500#2P1800S500#3P1500S500#4P1550S500#5P1450S500\r\n"};
const char *mode1[] = {"#2P1450S1000\r\n",
                       "#0P700S1000\r\n",
                       "#4P600S1000#3P750S1000#1P1300S1000\r\n",
                       "#4P2000S1000\r\n",
                       "#1P1400S1000\r\n",
                       "#5P2350S1000\r\n",
                       "#5P600S1000#0P1450S1000\r\n",
                       "#5P1350S1000\r\n",
                       "#4P600S10000\r\n",
                       "#3P1450S1000#1P2000S1000#2P2000S1000\r\n"};
const char *mode2[] = {"#1P1400S1000#2P1650S1000#3P1250S1000#4P600S1000\r\n",
                       "#4P2000S1000\r\n",
                       "#1P1600S1000#2P1800S1000#3P1500S1000",
                       "#0P600S1000\r\n",
                       "#3P950S1000#5P2200S1000\r\n",
                       "#4P600S1000\r\n",
                       positionInit};
typedef struct scenario_buffer {
  char **data_to_send;
  uint8_t size;
} scenario_buffer_t;
scenario_buffer_t buf_mode;
uint8_t background_buffer[32];
uint8_t recv_buffer[4];

lpuart_rtos_handle_t handle;
struct _lpuart_handle t_handle;

lpuart_rtos_config_t lpuart_config = {
    .baudrate = 9600,
    .parity = kLPUART_ParityDisabled,
    .stopbits = kLPUART_OneStopBit,
    .buffer = background_buffer,
    .buffer_size = sizeof(background_buffer),
};

gpio_pin_config_t pin_gpio_button_config = {
    .direction = kGPIO_DigitalInput,
    .outputLogic = 0,
    .interruptMode = kGPIO_IntRisingEdge,
};

uint32_t LPI2C1_GetFreq(void)
{
    return LPI2C_CLOCK_FREQUENCY;
}

static void lpi2c_slave_callback(uint32_t event)
{
    switch (event)
    {
        /* The master has sent a stop transition on the bus */
        case ARM_I2C_EVENT_TRANSFER_DONE:
            g_SlaveCompletionFlag = true;
            break;
        default:
            break;
    }
}

/*!
 * @brief Interrupt service function of switch.
 */
void USER_GPIO_IRQHandler(void) {
  // Clear flag on interrupt
  GPIO_PortClearInterruptFlags(USER_SW_GPIO, 1U << USER_SW_GPIO_PIN);
  // set global variable for signaling that button is pressed
  g_InputSignal = true;
  countPressed++;
}

/*!
 * @brief Application entry point.
 */
int main(void) {
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
  if (EventGroup_Button == NULL) {
    PRINTF("Event Group creation failed!\r\n");
    while (1)
      ;
  }
  countSemaphore = xSemaphoreCreateBinary();
  if (countSemaphore == NULL) {
    /* There was insufficient heap memory available for the mutex to be
    created. */
    PRINTF("Semaphore Binary creation failed!\r\n");
    while (1)
      ;
  }
  cmdMutex = xSemaphoreCreateMutex();
  if (cmdMutex == NULL) {
    /* There was insufficient heap memory available for the mutex to be
    created. */
    PRINTF("Mutex creation failed!\r\n");
    while (1)
      ;
  }
  if (xTaskCreate(uart_task, "Uart_task", configMINIMAL_STACK_SIZE + 10, NULL,
                  uart_task_PRIORITY, NULL) != pdPASS) {
    PRINTF("Task creation failed!\r\n");
    while (1)
      ;
  }
  if (xTaskCreate(button_task, "Button_task", configMINIMAL_STACK_SIZE + 50,
                  NULL, button_task_PRIORITY, NULL) != pdPASS) {
    PRINTF("Task creation failed!\r\n");
    while (1)
      ;
  }
  if (xTaskCreate(i2c_b2b_task, "i2c_b2b_task", configMINIMAL_STACK_SIZE + 50, NULL,
		  	  	  i2c_b2b_task_PRIORITY, NULL) != pdPASS) {
    PRINTF("Task creation failed!\r\n");
    while (1)
      ;
  }
  vTaskStartScheduler();
  for (;;)
    ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void uart_task(void *pvParameters) {
  int error;
  char *buffer = NULL;
  const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
  EventBits_t modeBits;
  lpuart_config.srcclk = DEMO_LPUART_CLK_FREQ;
  lpuart_config.base = DEMO_LPUART;
  if (0 > LPUART_RTOS_Init(&handle, &t_handle, &lpuart_config)) {
    vTaskSuspend(NULL);
  }

  /* Receive user input and send it back to terminal. */
  do {
    modeBits = xEventGroupWaitBits(EventGroup_Button, MODE_0 | MODE_1, pdTRUE,
                                   pdFALSE, xTicksToWait);
    if ((modeBits & MODE_0) == MODE_0 || (modeBits & MODE_1) == MODE_1) {
      if (xSemaphoreTake(cmdMutex, 0) == pdPASS) {
        error = LPUART_RTOS_Send(&handle, (uint8_t *)positionInit,
                                 strlen(positionInit));
        if (kStatus_Success != error) {
          break;
        }
        delay();
        for (unsigned int i = 0; i < buf_mode.size; i++) {
          buffer = buf_mode.data_to_send[i];
          /* Send introduction message. */
          error = LPUART_RTOS_Send(&handle, (uint8_t *)buffer, strlen(buffer));
          if (kStatus_Success != error) {
            break;
          }
          delay();
        }
        freeBuffer();
        xSemaphoreGive(cmdMutex);
      }
    } else {
      continue;
    }
  } while (1);

  LPUART_RTOS_Deinit(&handle);
  vTaskSuspend(NULL);
}

static void button_task(void *pvParameters) {
  uint8_t tmp;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
	vTaskDelayUntil( &xLastWakeTime, BTN_CYCLE_RATE_MS );
    if (g_InputSignal) {
      delay();
      if (GPIO_PinRead(USER_SW_GPIO, USER_SW_GPIO_PIN) == 1) {
        tmp = countPressed;
        countPressed = 0;
        if (xSemaphoreTake(cmdMutex, 0) == pdPASS) {
          switch (tmp) {
          case 1:
            /* The semaphore was ‘taken’ successfully, so the resource it is
            guarding can be accessed safely. */
        	PRINTF("Mode 1");
            buf_mode.size = (uint8_t)sizeof(mode1) / sizeof(mode1[0]);
            allocBufferArray(&buf_mode.data_to_send);
            for (unsigned int i = 0; i < buf_mode.size; i++) {
              copyBuffer((char *)mode1[i], &buf_mode.data_to_send[i]);
            }
            /* Access to the resource the semaphore is guarding is complete, so
            the semaphore must be ‘given’ back. */
            xSemaphoreGive(cmdMutex);
            /* Notify on event group */
            xEventGroupSetBits(EventGroup_Button, MODE_0);
            break;
          case 2:
        	PRINTF("Mode 2");
            /* The semaphore was ‘taken’ successfully, so the resource it is
            guarding can be accessed safely. */
            buf_mode.size = (uint8_t)sizeof(mode2) / sizeof(mode2[0]);
            allocBufferArray();
            for (unsigned int i = 0; i < buf_mode.size; i++) {
              copyBuffer((char *)mode2[i], &buf_mode.data_to_send[i]);
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
    // pass state running to the task which has the same priority
    taskYIELD();
  }
}

static void i2c_b2b_task(void *pvParameters) {
    initI2C_b2b();
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
    	vTaskDelayUntil( &xLastWakeTime, I2C_CYCLE_RATE_MS );
    	receiveDataI2C_b2b();
		// pass state running to the task which has the same priority
		taskYIELD();
    }
}

static void delay() {
  uint32_t time = 100000000;
  for (uint32_t i = 0; i < time; ++i) {
	  continue;
  }
}

static unsigned int allocBufferArray() {
  buf_mode.data_to_send = (char **)malloc(buf_mode.size * sizeof(char *));
  if (!buf_mode.data_to_send) {
    PRINTF("Out of memory");
    return -1;
  }
  return 0;
}

static unsigned int copyBuffer(char *data, char **des) {
  *des = (char *)calloc(strlen(data), sizeof(char));
  if (!(*des)) {
    PRINTF("Out of memory");
    return -1;
  }
  strcpy(*des, data);
  return 0;
}

static void freeBuffer(void) {
	if (buf_mode.data_to_send != NULL) {
    for (unsigned int i = 0; i < buf_mode.size; i++) {
      free(buf_mode.data_to_send[i]);
      buf_mode.data_to_send[i] = NULL;
    }
    free(buf_mode.data_to_send);
    buf_mode.data_to_send = NULL;
  }
buf_mode.size = 0;
}

void initI2C_b2b(void) {
	/*Clock setting for LPI2C*/
	CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
	CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);

	/* Initialize the LPI2C slave peripheral */
	EXAMPLE_I2C_SLAVE.Initialize(lpi2c_slave_callback);
	EXAMPLE_I2C_SLAVE.PowerControl(ARM_POWER_FULL);

	/* Change the slave address */
	EXAMPLE_I2C_SLAVE.Control(ARM_I2C_OWN_ADDRESS, I2C_MASTER_SLAVE_ADDR_7BIT);
}

uint32_t FindIndex(char* str, char c)
{
    char* ptr;
    uint32_t index;

    ptr = strchr(str, c);
    if (ptr == NULL)
    {
        return -1;
    }

    index = ptr - str;
    return index;
}

void receiveDataI2C_b2b(void) {
	char* tempData;
	char cr = '\r';
	uint32_t currentIndex = 0, indexBuf = 0;
	if (!g_SlaveCompletionFlag)
			memset(g_slave_buff, 0, I2C_DATA_LENGTH);
	/* Start accepting I2C transfers on the LPI2C slave peripheral */
	EXAMPLE_I2C_SLAVE.SlaveReceive(g_slave_buff, I2C_DATA_LENGTH);
	if (g_SlaveCompletionFlag) {
		if (xSemaphoreTake(cmdMutex, 0) == pdPASS) {
			tempData = (char*)g_slave_buff;
			for (;;) {
				// Count number of chain request
				if ((currentIndex = FindIndex(tempData, '\r')) < I2C_DATA_LENGTH) {
					buf_mode.size ++;
					tempData += currentIndex + 1;
				}
				else {
					break;
				}
			}
			currentIndex = 0;
			indexBuf = 0;
			allocBufferArray();
			tempData = (char*)g_slave_buff;
			// Copy each chain request in buffer of slave i2c to buffer of command arm
			for (; indexBuf <= buf_mode.size; indexBuf++) {
				if ((currentIndex = FindIndex(tempData, '\r')) < I2C_DATA_LENGTH) {
					buf_mode.data_to_send[indexBuf] = malloc((currentIndex + 1) * sizeof(char));
					memset(buf_mode.data_to_send[indexBuf], 0, currentIndex + 1);
					strncpy(buf_mode.data_to_send[indexBuf], tempData, currentIndex);
					strncat(buf_mode.data_to_send[indexBuf], &cr, 1);
					tempData += currentIndex + 1;
					//PRINTF("%s\n", buf_mode.data_to_send[indexBuf]);
				}
				else {
					break;
				}
			}
		}
		xSemaphoreGive(cmdMutex);
        /* Notify on event group */
        xEventGroupSetBits(EventGroup_Button, MODE_1);
		g_SlaveCompletionFlag = false;
	}
}
