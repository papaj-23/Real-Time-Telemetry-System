#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "init.h"
#include "rtos_init.h"
#include "imu_data_input.h"
#include "mpu-6050.h"

static void Tester_handler(void*);
static void RTOS_Init(void);

/* global variables */




/* Tester task */
#define TESTER_STACK_SIZE 128
static StackType_t tester_stack[TESTER_STACK_SIZE];
static StaticTask_t tester_tcb;
static TaskHandle_t tester_handle = NULL;
static const task_init_t tester_init = {
    .entry = Tester_handler,
    .name = "tester",
    .stack_size = TESTER_STACK_SIZE,
    .args = NULL,
    .priority = 2,
    .stack = tester_stack,
    .tcb = &tester_tcb
};

static void Tester_handler(void* pvParameters) {
    for(;;) {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void RTOS_Init(void) {
    tester_handle = create_static_task(&tester_init);
    imu_rtos_init();
}

int main(void)
{
    HAL_Init();
    System_Init();
    RTOS_Init();
    vTaskStartScheduler();
    for(;;) {
        __WFI();
    }
}

