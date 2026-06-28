#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f4xx.h"

/* Kernel básico */
#define configUSE_PREEMPTION            1
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0
#define configCPU_CLOCK_HZ              ( SystemCoreClock )
#define configTICK_RATE_HZ              ( 1000 )
#define configMAX_PRIORITIES            ( 5 )
#define configMINIMAL_STACK_SIZE        ( (unsigned short)256 )  // aumentado para FPU
#define configMAX_TASK_NAME_LEN         ( 16 )
#define configUSE_16_BIT_TICKS          0
#define configIDLE_SHOULD_YIELD         1

/* Memória */
#define configTOTAL_HEAP_SIZE           ( (size_t)(16 * 1024) )  // aumentado
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION  0

/* Recursos */
#define configUSE_MUTEXES               1   // habilitado para UART
#define configUSE_TASK_NOTIFICATIONS    1
#define configUSE_TIMERS                0
#define configQUEUE_REGISTRY_SIZE       0

/* APIs incluídas */
#define INCLUDE_vTaskDelay              1
#define INCLUDE_vTaskDelayUntil         1
#define INCLUDE_vTaskDelete             1
#define INCLUDE_vTaskSuspend            1

/* Prioridades NVIC (STM32F4: 4 bits) */
#define configPRIO_BITS                 4
#define configKERNEL_INTERRUPT_PRIORITY         ( 15 << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (  5 << (8 - configPRIO_BITS) )

/* Mapear handlers exigidos pelo port */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* FPU */
#define configENABLE_FPU            1
#define configUSE_TASK_FPU_SUPPORT  1

#endif