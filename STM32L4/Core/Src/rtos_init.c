#include "FreeRTOS.h"
#include "task.h"
#include "rtos_init.h"

/* Idle task */
static StaticTask_t IdleTaskTCB;
static StackType_t IdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &IdleTaskTCB;
    *ppxIdleTaskStackBuffer = IdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void delay_wrapper_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

TaskHandle_t create_static_task(const task_init_t *t)
{
    TaskHandle_t h = xTaskCreateStatic(
        t->entry,
        t->name,
        t->stack_size,
        t->args,
        t->priority,
        t->stack,
        t->tcb
    );
    configASSERT(h);
    return h;
}
