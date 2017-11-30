// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include <stdbool.h>
#include <stdnoreturn.h>

#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif /* ifndef USE_FULL_LL_DRIVER */

#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l071xx.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_gpio.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_rcc.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_spi.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_system.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_tim.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_usart.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_utils.h"

#include "command.h"
#include "log.h"
#include "peripherals.h"
#include "print_queue.h"
#include "radio.h"
#include "radio_cc1125.h"
#include "spi.h"
#include "state.h"

volatile uint32_t millis = 0;
static queue_t usart1_queue;
static queue_t usart2_queue;
static queue_t event_queue;
queue_t spi1_tx_queue;
queue_t spi1_rx_queue;
volatile bool spi_transfer_completed = true;
volatile bool usart1_tx_completed = true;
volatile bool usart2_tx_completed = true;
volatile event_t e = EVT_SLEEP;

// error_catch serves as a debugging function that will catch all failed
// asserts and trap execution for the debugger
// FIXME: change error_catch into a reset?
noreturn void error_catch(void);
noreturn void error_catch(void)
{
    __disable_irq();
    // constant LED
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);

    while (1) {
        // if you can read this line in the debugger, you've made a terrible mistake
        __asm("BKPT");
    }
}

int main(void)
{
    // enable prefetch
    // 2018-08-04 FIXME: explain why
    LL_FLASH_EnablePrefetch();

    config_system_clocks();
    config_gpio();
    config_uart();
    config_spi();
    config_tim2_nvic();
    //config_rtc();

    queue_init(&usart1_queue);
    queue_init(&usart2_queue);
    queue_init(&spi1_tx_queue);
    queue_init(&spi1_rx_queue);

    radio_config_t radio_config = {
        .is_enabled = true,
        .tx_type = TX_TELEMETRY,
        .CANSAT_cfg = {
            .callsign = {
                // FIXME 2017-10-20: change source callsign to an actual callsign
                .source      = "CANSAT",
                .destination = "CQ",
                .via_relay   = "TELEM",
            },
            .baud = M9600_FSK,
        },
    };

    // set default state for GPIOs
    radio_LED_off();

    radio_set_mode(radio_config);

    LL_SPI_Enable(SPI1);

    // enable rx interrupt for USART1
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_DisableIT_TXE(USART1);
    LL_TIM_EnableIT_UPDATE(TIM2);

    while (1) {
      e = queue_pop(&event_queue);
      if (e != EVT_SLEEP) {
         radio_CC1125_config_radio();
         e = EVT_SLEEP;
      }
      else {
        // wait for USART to complete transmitting before sleeping the CPU
        // this avoids a race condition -- FIXME: 2017-10-31 DMA to fix?
        while (!usart1_tx_completed);
        while (!usart2_tx_completed);

        // goodnight sweet prince
        __WFI();
      }
    }

    return 0;
}

// 2018-04-08 FIXME: implement DMA for UART transmission
// putchar loads a circular buffer with characters to transmit (print_queue.c)
void usart_putchar(usart_num u, char c);
void usart_putchar(usart_num u, char c)
{
    if (u == USART_1) {
        queue_push(&usart1_queue, c);
        LL_USART_EnableIT_TXE(USART1);
        usart1_tx_completed = false;
    } else if (u == USART_2) {
        queue_push(&usart2_queue, c);
        LL_USART_EnableIT_TXE(USART2);
    } else {
        error_catch();
    }
}

// interrupt handlers
void TIM2_IRQHandler(void);
void TIM2_IRQHandler()
{
    __disable_irq();
    static uint32_t last_millis = 0;
    LL_TIM_ClearFlag_UPDATE(TIM2);

    millis++;
    if (millis - last_millis >= 500) {
        last_millis = millis;
        radio_LED_toggle();

        e = EVT_RUN; // wakeup every 500 ms
    }

    __enable_irq();
}

void USART1_IRQHandler(void);
void USART1_IRQHandler()
{
    __disable_irq();
    // receive
    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
        // RXNE flag is cleared by reading from the USART1 data register
        char c = LL_USART_ReceiveData8(USART1);
        /* if (c != '\0') { */
        parse_char(c);
        /* } */

        // echo char to serial
         if (c == '\r') {
             queue_push(&usart1_queue, '\r');
             queue_push(&usart1_queue, '\n');
         } else {
             pr_ch(USART_1, c);
         }
    }

    // transmit
    if (LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1)) {
        if (queue_is_empty(&usart1_queue)) {
            LL_USART_DisableIT_TXE(USART1);
            usart1_tx_completed = true;
        } else {
            // TXE flag cleared by writing to USART1 data register
            LL_USART_TransmitData8(USART1, queue_pop(&usart1_queue));
            LL_USART_EnableIT_TXE(USART1);
        }
    }

    __enable_irq();
}

void USART2_IRQHandler(void);
void USART2_IRQHandler()
{
    __disable_irq();
    // usart2 doesn't receive anything from the Stensat CANSAT (CANSAT doesn't send anything back, ever),
    // so we don't use the receive interrupt

    // transmit
    if (LL_USART_IsEnabledIT_TXE(USART2) && LL_USART_IsActiveFlag_TXE(USART2)) {
        if (queue_is_empty(&usart2_queue)) {
            LL_USART_DisableIT_TXE(USART2);
            usart2_tx_completed = true;
        } else {
            // TXE flag cleared by writing to USART2 data register
            LL_USART_TransmitData8(USART2, queue_pop(&usart2_queue));
            LL_USART_EnableIT_TXE(USART2);
        }
    } else {
        LL_USART_DisableIT_TXE(USART2);
    }
    __enable_irq();
}

void SPI1_IRQHandler(void);
void SPI1_IRQHandler(void)
{
  __disable_irq();

  // pull NSS/CSN high if transmit buffer is empty
  // and if all RX bytes are received
  if (queue_is_empty(&spi1_tx_queue) && !LL_SPI_IsActiveFlag_BSY(SPI1)) {
    spi_ss_high();
    spi_transfer_completed = true;
  }

  // transmit
  if (LL_SPI_IsEnabledIT_TXE(SPI1) && LL_SPI_IsActiveFlag_TXE(SPI1)) {
    if (queue_is_empty(&spi1_tx_queue)) {
      LL_SPI_DisableIT_TXE(SPI1);
    } else {
      spi_ss_low();
      // TXE flag cleared by writing to SPI1 data register
      LL_SPI_TransmitData8(SPI1, queue_pop(&spi1_tx_queue));
      LL_SPI_EnableIT_TXE(SPI1);
    }
  }

  // receive
  if (LL_SPI_IsActiveFlag_RXNE(SPI1)) {
    queue_push(&spi1_rx_queue, LL_SPI_ReceiveData8(SPI1));
    LL_SPI_EnableIT_RXNE(SPI1);
  }

  __enable_irq();
}

// fstack-protector
uintptr_t __stack_chk_guard = 0xdeadbeef;

void __stack_chk_fail(void);
void __stack_chk_fail(void)
{
    error_catch();
}
