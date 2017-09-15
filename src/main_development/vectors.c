// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "stm32l0xx.h"
/*#include "debug.h"*/

//Start-up code.
extern void __attribute__((noreturn, weak)) _start(void);

// Default interrupt handler
void __attribute__((section(".after_vectors"), noreturn)) __Default_Handler(void);

// Reset handler
void __attribute__((section(".after_vectors"), noreturn)) Reset_Handler(void);

/** Non-maskable interrupt (RCC clock security system) */
void NMI_Handler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** All class of fault */
void HardFault_Handler(void) __attribute__((interrupt, weak));

/** System service call via SWI instruction */
void SVC_Handler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** Pendable request for system service */
void PendSV_Handler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** System tick timer */
void SysTick_Handler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** Window watchdog interrupt */
void WWDG_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** PVD through EXTI line detection interrupt */
void PVD_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** RTC global interrupt */
void RTC_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** Flash global interrupt */
void FLASH_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** RCC global interrupt */
void RCC_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** EXTI Line0 interrupt */
void EXTI0_1_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** EXTI Line2 interrupt */
void EXTI2_3_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** EXTI Line4 interrupt */
void EXTI4_15_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** DMA1 Channel1 global interrupt */
void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** DMA1 Channel2 global interrupt */
void DMA1_Channel2_3_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** DMA1 Channel4 global interrupt */
void DMA1_Channel4_5_6_7_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

/** ADC1 and ADC2 global interrupt */
void ADC1_COMP_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void LPTIM1_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void USART4_5_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void TIM2_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void TIM3_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void TIM6_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void TIM7_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void TIM21_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void I2C3_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void TIM22_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void I2C1_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void I2C2_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void SPI1_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void SPI2_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void USART1_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void USART2_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

void LPUART1_IRQHandler(void) __attribute__((interrupt, weak, alias("__Default_Handler")));

// Stack start variable, needed in the vector table.
extern unsigned int __stack;

// Typedef for the vector table entries.
typedef void (*const pHandler)(void);

/** STM32F103 Vector Table */
__attribute__((section(".vectors"), used)) pHandler vectors[] = {
    (pHandler)&__stack, // The initial stack pointer
    Reset_Handler, // The reset handler
    NMI_Handler, // The NMI handler
    HardFault_Handler, // The hard fault handler
    0,
    0,
    0,
    0, // Reserved
    0, // Reserved
    0, // Reserved
    0, // Reserved
    SVC_Handler, // SVCall handler
    0, // Reserved
    0, // Reserved
    PendSV_Handler, // The PendSV handler
    SysTick_Handler, // The SysTick handler
    // ----------------------------------------------------------------------
    WWDG_IRQHandler, /* Window WatchDog              */
    PVD_IRQHandler, /* PVD through EXTI Line detection */
    RTC_IRQHandler, /* RTC through the EXTI line     */
    FLASH_IRQHandler, /* FLASH                        */
    RCC_IRQHandler, /* RCC                          */
    EXTI0_1_IRQHandler, /* EXTI Line 0 and 1            */
    EXTI2_3_IRQHandler, /* EXTI Line 2 and 3            */
    EXTI4_15_IRQHandler, /* EXTI Line 4 to 15            */
    0, /* Reserved                     */
    DMA1_Channel1_IRQHandler, /* DMA1 Channel 1               */
    DMA1_Channel2_3_IRQHandler, /* DMA1 Channel 2 and Channel 3 */
    DMA1_Channel4_5_6_7_IRQHandler, /* DMA1 Channel 4, Channel 5, Channel 6 and Channel 7*/
    ADC1_COMP_IRQHandler, /* ADC1, COMP1 and COMP2        */
    LPTIM1_IRQHandler, /* LPTIM1                       */
    USART4_5_IRQHandler, /* USART4 and USART 5           */
    TIM2_IRQHandler, /* TIM2                         */
    TIM3_IRQHandler, /* TIM3                         */
    TIM6_IRQHandler, /* TIM6 and DAC                 */
    TIM7_IRQHandler, /* TIM7                         */
    0, /* Reserved                     */
    TIM21_IRQHandler, /* TIM21                        */
    I2C3_IRQHandler, /* I2C3                         */
    TIM22_IRQHandler, /* TIM22                        */
    I2C1_IRQHandler, /* I2C1                         */
    I2C2_IRQHandler, /* I2C2                         */
    SPI1_IRQHandler, /* SPI1                         */
    SPI2_IRQHandler, /* SPI2                         */
    USART1_IRQHandler, /* USART1                       */
    USART2_IRQHandler, /* USART2                       */
    LPUART1_IRQHandler, /* LPUART1                      */
    0, /* Reserved                     */
    0, /* Reserved                     */
};

/** Default exception/interrupt handler */
void __attribute__((section(".after_vectors"), noreturn)) __Default_Handler(void)
{
#ifdef DEBUG
    while (1)
        ;
#else
    NVIC_SystemReset();

    while (1)
        ;
#endif
}

/** Reset handler */
void __attribute__((section(".after_vectors"), noreturn)) Reset_Handler(void)
{
    _start();

    while (1)
        ;
}
