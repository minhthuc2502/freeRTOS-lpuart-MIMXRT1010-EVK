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
#define button_task_PRIORITY (configMAX_PRIORITIES - 1)
#define i2c_b2b_task_PRIORITY (configMAX_PRIORITIES - 2)
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

#define EXAMPLE_I2C_MASTER Driver_I2C1
#define EXAMPLE_LPI2C_DMAMUX_BASEADDR (DMAMUX)
#define EXAMPLE_LPI2C_DMA_BASEADDR (DMA0)
#define DMA0_IRQn DMA0_DMA16_IRQn
/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define I2C_MASTER_SLAVE_ADDR_7BIT (0x7EU)
#define I2C_DATA_LENGTH (220) /* MAX is 256 */
#define BTN_CYCLE_RATE_MS		20
#define I2C_CYCLE_RATE_MS		10
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void button_task(void *pvParameters);
static void i2c_b2b_task(void *pvParameters);
/*!
 * @brief delay a while.
 */
static void delay(void);

/*!
 * @brief initialize i2c slave
 */
void initI2C_b2b(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Whether the SW is turned on */
volatile bool g_InputSignal = false;
volatile uint8_t countPressed = 0;
EventGroupHandle_t EventGroup_Button;

AT_NONCACHEABLE_SECTION(uint8_t g_master_txBuff[I2C_DATA_LENGTH]);
AT_NONCACHEABLE_SECTION(uint8_t g_master_rxBuff[I2C_DATA_LENGTH]);
volatile bool g_MasterCompletionFlag = false;
/*******************************************************************************
 * Code
 ******************************************************************************/
const char positionInit[] = {
    "#0P1450S500#1P1600S500#2P1800S500#3P1500S500#4P1550S500#5P1450S500\r"};
char mode1[] = {"#2P1450S1000\r"
                       "#0P700S1000\r"
                       "#4P600S1000#3P750S1000#1P1300S1000\r"
                       "#4P2000S1000\r"
                       "#1P1400S1000\r"
                       "#5P2350S1000\r"
                       "#5P600S1000#0P1450S1000\r"
                       "#5P1350S1000\r"
                       "#4P600S10000\r"
                       "#3P1450S1000#1P2000S1000#2P2000S1000\r"};
char mode2[] = {"#1P1400S1000#2P1650S1000#3P1250S1000#4P600S1000\r"
                       "#4P2000S1000\r"
                       "#1P1600S1000#2P1800S1000#3P1500S1000\r"
                       "#0P600S1000\r"
                       "#3P950S1000#5P2200S1000\r"
                       "#4P600S1000\r"
					   "#0P1450S500#1P1600S500#2P1800S500#3P1500S500#4P1550S500#5P1450S500\r"};

gpio_pin_config_t pin_gpio_button_config = {
    .direction = kGPIO_DigitalInput,
    .outputLogic = 0,
    .interruptMode = kGPIO_IntRisingEdge,
};

uint32_t LPI2C1_GetFreq(void)
{
    return LPI2C_CLOCK_FREQUENCY;
}

static void lpi2c_master_callback(uint32_t event)
{
    switch (event)
    {
        /* The master has sent a stop transition on the bus */
        case ARM_I2C_EVENT_TRANSFER_DONE:
            g_MasterCompletionFlag = true;
            break;
        case ARM_I2C_EVENT_TRANSFER_INCOMPLETE:
            g_MasterCompletionFlag = true;
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

  cmdMutex = xSemaphoreCreateMutex();
  if (cmdMutex == NULL) {
    /* There was insufficient heap memory available for the mutex to be
    created. */
    PRINTF("Mutex creation failed!\r\n");
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
        	strcpy((char*)g_master_txBuff, mode1);
            /* Access to the resource the semaphore is guarding is complete, so
            the semaphore must be ‘given’ back. */
            xSemaphoreGive(cmdMutex);
            /* Notify on event group */
            xEventGroupSetBits(EventGroup_Button, MODE_0);
            break;
          case 2:
            /* The semaphore was ‘taken’ successfully, so the resource it is
            guarding can be accessed safely. */
          	strcpy((char*)g_master_txBuff, mode2);
        	//g_master_txBuff = (uint8_t*)mode2;
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

static void i2c_b2b_task(void *pvParameters) {
	EventBits_t modeBits;
	const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;		// 100ms

    initI2C_b2b();
    while (1) {
      modeBits = xEventGroupWaitBits(EventGroup_Button, MODE_0 | MODE_1, pdTRUE,
                                   pdFALSE, xTicksToWait);
      if ((modeBits & MODE_0) == MODE_0 || (modeBits & MODE_1) == MODE_1) {
        if (xSemaphoreTake(cmdMutex, 0) == pdPASS) {
          PRINTF("Begin send data to slave\r\n");
          /* Start transmitting I2C transfers on the LPI2C master peripheral */
          EXAMPLE_I2C_MASTER.MasterTransmit(I2C_MASTER_SLAVE_ADDR_7BIT, g_master_txBuff, I2C_DATA_LENGTH, false);
          /*wait for master complete*/
          //PreviousTime = xTaskGetTickCount();
          while (!g_MasterCompletionFlag)
          {
          }
          /*  Reset master completion flag to false. */
          g_MasterCompletionFlag = false;
          /* Start accepting I2C transfers on the LPI2C master peripheral */
          EXAMPLE_I2C_MASTER.MasterReceive(I2C_MASTER_SLAVE_ADDR_7BIT, g_master_rxBuff, I2C_DATA_LENGTH, false);
          /*wait for master complete*/
          while (!g_MasterCompletionFlag)
          {
          }
          /*  Reset master completion flag to false. */
          g_MasterCompletionFlag = false;
          xSemaphoreGive(cmdMutex);
          PRINTF("Finish send data to slave\r\n");
        }
      }
    }
}

static void delay() {
  uint32_t time = 100000000;
  for (uint32_t i = 0; i < time; ++i) {
	  continue;
  }
}

void initI2C_b2b(void) {
	/*Clock setting for LPI2C*/
	CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
	CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);

  /* DMAMux init and EDMA init */
#if (defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT)
    DMAMUX_Init(EXAMPLE_LPI2C_DMAMUX_BASEADDR);
#endif
  edma_config_t edmaConfig = {0}; 
  EDMA_GetDefaultConfig(&edmaConfig);
  EDMA_Init(EXAMPLE_LPI2C_DMA_BASEADDR, &edmaConfig);

  /* Initialize the LPI2C master peripheral */
  EXAMPLE_I2C_MASTER.Initialize(lpi2c_master_callback);
  EXAMPLE_I2C_MASTER.PowerControl(ARM_POWER_FULL);

  /* Change the default baudrate configuration */
  EXAMPLE_I2C_MASTER.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
}
