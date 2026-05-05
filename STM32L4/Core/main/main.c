#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "init.h"
#include "imu_data_input.h"
#include "nrf905_link.h"

static void blinky_handler(void*);
static void RTOS_Init(void);

/* global variables */

/* Tester task */
#define BLINKY_STACK_SIZE 128
static StackType_t blinky_stack[BLINKY_STACK_SIZE];
static StaticTask_t blinky_tcb;
static TaskHandle_t blinky_handle = NULL;
static UBaseType_t blinky_prio = 0;

/* idle task allocation overwrite */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   configSTACK_DEPTH_TYPE *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static void blinky_handler(void* pvParameters) {
    for(;;) {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void RTOS_Init(void) {
    blinky_handle = xTaskCreateStatic(blinky_handler, "blinky", BLINKY_STACK_SIZE, NULL,
                                    blinky_prio, blinky_stack, &blinky_tcb);
    if(blinky_handle == NULL) {
        Error_Handler();
    }

    if(imu_rtos_init() != 0) {
        Error_Handler();
    }

    if(nrf905_rtos_init() != 0) {
        Error_Handler();
    }
}

int main(void)
{
    HAL_Init();
    Peripherals_Init();
    RTOS_Init();
    vTaskStartScheduler();
    for(;;) {
        __WFI();
    }
}

