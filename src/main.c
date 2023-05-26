/***************************************************************************//**
 * @file
 * @brief FreeRTOS Blink Demo for Energy Micro EFM32GG_STK3700 Starter Kit
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "croutine.h"

#include "em_chip.h"
#include "bsp.h"
#include "bsp_trace.h"

#include "sleep.h"

#define STACK_SIZE_FOR_TASK    (configMINIMAL_STACK_SIZE + 10)
#define TASK_PRIORITY          (tskIDLE_PRIORITY + 1)

/* Structure with parameters for LedBlink */
typedef struct {
  /* Delay between blink of led */
  portTickType delay;
  /* Number of led */
  int          ledNo;
} TaskParams_t;

QueueHandle_t data_queue_sensor;
QueueHandle_t data_queue_processed;

/***************************************************************************//**
 * @brief Simple task which is blinking led
 * @param *pParameters pointer to parameters passed to the function
 ******************************************************************************/
static void LedBlink(void *pParameters)
{
  TaskParams_t     * pData = (TaskParams_t*) pParameters;
  const portTickType delay = pData->delay;

  for (;; ) {
    BSP_LedToggle(pData->ledNo);
    vTaskDelay(delay);
  }
}

static void sensor_task() {
    for (;;) {
        // Lee los datos del sensor
        uint16_t sensor_data = 0;

        if (!I2C_ReadRegister(0x02, &sensor_data)) { // ALG_RESULT_DATA
            printf("Error de lectura.\n");
        }

        sensor_data = rand() % 4000;

        printf("CO2 Level: %d\n", sensor_data);

        // Añade los datos a la cola
        xQueueSend(data_queue_sensor, &sensor_data, 0);

        // Espera un tiempo antes de volver a leer los datos
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void data_process_task() {
    for (;;) {
        // Lee los datos de la cola
        uint16_t sensor_data = 0;
        xQueueReceive(data_queue_sensor, &sensor_data, portMAX_DELAY);

        // Procesa los datos
        uint16_t processed_data = sensor_data * 1;

        // Añade los datos procesados a la cola
        xQueueSend(data_queue_processed, &processed_data, 0);
    }
}

static void led_control_task() {
    for (;;) {
        // Lee los datos de la cola
        uint16_t sensor_data = 0;
        xQueueReceive(data_queue_processed, &sensor_data, 0);

        // Obtiene el valor de CO2 de los datos
        uint16_t co2_ppm = sensor_data;

        // Controla el LED según el valor de CO2
        if (co2_ppm > 2500) {
        	BSP_LedSet(0);
            BSP_LedSet(1);
        }
        else if (co2_ppm > 1500) {
            BSP_LedSet(0);
            BSP_LedClear(1);
        }
        else if (co2_ppm > 0) {
            BSP_LedClear(0);
            BSP_LedClear(1);
        }
    }
}

void setup() {
    // Inicialitza la cua
    data_queue_sensor = xQueueCreate(10, 2);
    data_queue_processed = xQueueCreate(10, 2);

    while(!I2C_Test()) {}

    srand(time(NULL)); // Inicializar la semilla de números aleatorios con el tiempo actual

    // Crea les tasques
    xTaskCreate(sensor_task, (const char*) "Sensor Task", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(data_process_task, (const char*) "Data Process Task", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(led_control_task, (const char*) "LED Control Task", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);
}

/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();
  /* If first word of user data page is non-zero, enable Energy Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize LED driver */
  BSP_LedsInit();
  /* Setting state of leds*/
  BSP_LedSet(0);
  BSP_LedSet(1);
  BSP_LedClear(0);
  BSP_LedClear(1);

  /* Initialize SLEEP driver, no calbacks are used */
  SLEEP_Init(NULL, NULL);
#if (configSLEEP_MODE < 3)
  /* do not let to sleep deeper than define */
  SLEEP_SleepBlockBegin((SLEEP_EnergyMode_t)(configSLEEP_MODE + 1));
#endif

  /* Parameters value for tasks*/
  //static TaskParams_t parametersToTask1 = { pdMS_TO_TICKS(1000), 0 };
  //static TaskParams_t parametersToTask2 = { pdMS_TO_TICKS(500), 1 };

  /*Create two task for blinking leds*/
  //xTaskCreate(LedBlink, (const char *) "LedBlink1", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);
  //xTaskCreate(LedBlink, (const char *) "LedBlink2", STACK_SIZE_FOR_TASK, &parametersToTask2, TASK_PRIORITY, NULL);

  setup();


  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();

  return 0;
}

int _write(int file, const char *ptr, int len) {
    int x;
    for (x = 0; x < len; x++) {
       ITM_SendChar (*ptr++);
    }
    return (len);
}
