// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

/* 2018-08-04 FIXME: use 14.7456 MHz crystal instead of internal oscillator */
#define HSE_VALUE ((uint32_t)14745600)

//#define USE_HSI

//#define SPI_BITBANG

#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif /* ifndef USE_FULL_LL_DRIVER */

#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_bus.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_rcc.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_spi.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_tim.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_usart.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_utils.h"

#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_hal_cortex.h"

#include "error.h"
#include "peripherals.h"
#include "spi.h"

// 2018-08-04 FIXME: use external clock instead of internal oscillator
void config_system_clocks(void)
{
    typedef struct pll_t {
        LL_UTILS_PLLInitTypeDef prescale;
        LL_UTILS_ClkInitTypeDef bus;
    } pll_t;

#ifdef USE_HSI
    pll_t pll_init = {
        // we obviously use the PLL, and set it to have no effect on the 16 MH HSI clock
        .prescale = {.PLLMul = LL_RCC_PLL_MUL_4, .PLLDiv = LL_RCC_PLL_DIV_4 },
        // default system clock = 16 MHz HSI, but we use the 2MHz after prescale
        // (see stm32cube clock config chart)
        .bus = {
            .AHBCLKDivider = LL_RCC_SYSCLK_DIV_8,
            .APB1CLKDivider = LL_RCC_APB1_DIV_1,
            .APB2CLKDivider = LL_RCC_APB2_DIV_1,
        }
    };
#else
    // use HSE also FIXME: u know what to do Trent pls
    pll_t pll_init = {
        // use same settings for PLL as in HSI
        .prescale = {.PLLMul = LL_RCC_PLL_MUL_4, .PLLDiv = LL_RCC_PLL_DIV_4 },
        // reconfigure bus frequencies
        .bus = {
            .AHBCLKDivider = LL_RCC_SYSCLK_DIV_1,
            .APB1CLKDivider = LL_RCC_APB1_DIV_8,
            .APB2CLKDivider = LL_RCC_APB2_DIV_8,
        }
    };
#endif
    // set up clocks - OSC_IN and OSC_OUT on PH0 and PH1,
    // HSE input clock is 14.7456 MHz
    // set PLL as system clock, using HSE as source
    /*LL_PLL_ConfigSystemClock_HSE(HSE_VALUE, LL_UTILS_HSEBYPASS_OFF, &pll_init.prescale, &pll_init.bus);*/
#ifdef USE_HSI
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
    LL_PLL_ConfigSystemClock_HSI(&pll_init.prescale, &pll_init.bus);
#else
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
    LL_PLL_ConfigSystemClock_HSE(HSE_VALUE,
            LL_UTILS_HSEBYPASS_OFF,
            &pll_init.prescale,
            &pll_init.bus);
#endif

    /*LL_Init1msTick(SystemCoreClock);*/

    // enable peripheral clocks
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB | LL_IOP_GRP1_PERIPH_GPIOC);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

void config_gpio(void)
{
    // set up peripherals - GPIO for control/output:
    // | Pin  | Pin Name        | Description
    // | ---- | --------------- | -----------------------------------------
    // | PB0  | CC1125_Tx_or_Rx | Switches Rx and Tx chain (0 = Rx, 1 = Tx)
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);

    // | PB4  | CANSAT_Enable   | Enables power switch to CANSAT
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_DOWN);

    // | Pin  | Pin Name        | Description
    // | ---- | --------------- | -----------------------------------------
    // | PA1  | CC1125_Enable   | Enables power switch to CC1125
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_DOWN);

    // | PA15 | PA_Enable       | Enables (logic to the gate of) power amplifier
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);

    // | PA8  | N2420_Enable    | Enables power switch to N2420
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_DOWN);

    // | Pin  | Pin Name        | Description
    // | ---- | --------------- | -----------------------------------------
    // | PC13 | MCU_Alive       | Connected as current source to heartbeat LED
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);

    /*   - UART for CANSAT - USART2 on PA2 and PA3 */
    /*LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);*/
    /*LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);*/
    /*LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);*/

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_4); // USART2_TX
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_4); // USART2_RX

    /*   - UART for n2420 - USART1 on PA9/10*/
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_4); // USART1_TX
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_4); // USART1_RX

    /* turn off CANSAT and CC1125 */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

void config_uart(void)
{
    typedef struct usart_t {
        LL_USART_InitTypeDef init;
        LL_USART_ClockInitTypeDef clock;
    } usart_t;

    usart_t usart_init;

    /* USART configuration */
    LL_USART_StructInit(&usart_init.init);
    LL_USART_ClockStructInit(&usart_init.clock);

    // usart 1 config
    LL_USART_ClockInit(USART1, &usart_init.clock);
    LL_USART_Init(USART1, &usart_init.init);
    LL_USART_Enable(USART1);

    // usart 2 config
    LL_USART_ClockInit(USART2, &usart_init.clock);
    LL_USART_Init(USART2, &usart_init.init);
    LL_USART_Enable(USART2);

    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);

    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);

    uint32_t usart1_periph_clk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE);
    uint32_t usart2_periph_clk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART2_CLKSOURCE);

    // there is no way to recover from this error
    // (2017-08-07 FIXME: recover from this error)
#ifdef USE_HSI
    assert(usart1_periph_clk == 2E6);
    assert(usart2_periph_clk == 2E6);
#else
    // if using HSE, frequencies are 1.8432 MHz
    assert(usart1_periph_clk == 1843200);
    assert(usart2_periph_clk == 1843200);
#endif

    LL_USART_SetBaudRate(USART1, usart1_periph_clk, LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_SetBaudRate(USART2, usart1_periph_clk, LL_USART_OVERSAMPLING_16, 38400);
}

void config_tim2_nvic(void)
{
    LL_TIM_InitTypeDef tim_init;
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);

    LL_TIM_StructInit(&tim_init);
    LL_TIM_Init(TIM2, &tim_init);

    LL_TIM_EnableCounter(TIM2);
#ifdef USE_HSI
    // The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
    /* // 2 MHz / (19999 + 1) => 100 Hz */
    /* LL_TIM_SetPrescaler(TIM2, 19999); */
    /* LL_TIM_SetAutoReload(TIM2, 100); */

    // 2 MHz / (1999 + 1) => 1000 Hz
    LL_TIM_SetPrescaler(TIM2, 1999);
    LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);
    // default counter mode is to count down from autoreload value to zero
    // therefore, tim2_nvic fires once a second
    LL_TIM_SetAutoReload(TIM2, 1);
#else
    /* // 1.8432 MHz / (18431 + 1) => 100 Hz */
    /* LL_TIM_SetPrescaler(TIM2, 18431); */
    /* LL_TIM_SetAutoReload(TIM2, 100); */

    // 1.8432 MHz / (1842 + 1) => 1000.1 Hz
    LL_TIM_SetPrescaler(TIM2, 1842);
    LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);
    // default counter mode is to count down from autoreload value to zero
    // therefore, tim2_nvic fires once a second
    LL_TIM_SetAutoReload(TIM2, 1);
#endif
}

// 2017-08-04 FIXME: use hardware SPI
void config_spi(void)
{
    // set up peripherals - alternate pin functions are on page 45/136
    // of DM00141136 - STM32L071x8 datasheet
    //   - SPI for CC1125 - SPI1 on PA4/5/6/7

#ifdef SPI_BITBANG
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);

    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
#else
    // write proper GPIO registers: configure GPIO for MOSI/MISO/SCK pins

    // spi NSS pin doesn't work as intended
    // why? because reasons - see
    // https://stackoverflow.com/questions/35780290/how-to-use-hardware-nss-spi-on-stm32f4
    // and
    // https://community.st.com/thread/39153-spi-master-nss-always-low-in-stm32f4
    // so we use GPIO control instead (and we'll use it in the SPI1 interrupt)

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);

    // if SPI NSS did work, we would use:
    /*LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE); */
    /*LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_4, LL_GPIO_AF_0); // SPI1_NSS */

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_0); // SPI1_SCK
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_0); // SPI1_MISO
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_0); // SPI1_MOSI

    // write to SPI_CR2 and SPI_CR2 registers with configuration settings
    LL_SPI_InitTypeDef spi_init;
    LL_SPI_StructInit(&spi_init);
    spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
    spi_init.Mode = LL_SPI_MODE_MASTER;
    spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    spi_init.ClockPolarity = LL_SPI_POLARITY_LOW;
    spi_init.ClockPhase = LL_SPI_PHASE_1EDGE;
    spi_init.NSS = LL_SPI_NSS_SOFT; // use software control b/c hardware setting
    // isn't working - FIXME: 2017-10-13
    spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
    spi_init.BitOrder = LL_SPI_MSB_FIRST;
    /* spi_init.TIMode = SPI_TIMODE_DISABLE; */
    /* spi_init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; */
    /* spi_init.CRCPolynomial = 7; */

    ErrorStatus e = LL_SPI_Init(SPI1, &spi_init);
    if (e != SUCCESS) {
        error_catch();
    }

    LL_SPI_Enable(SPI1);

    HAL_NVIC_EnableIRQ(SPI1_IRQn); 
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0); 

    LL_SPI_EnableIT_RXNE(SPI1); // it's okay to enable rxne b/c nothing will be received 
    /*LL_SPI_EnableIT_TXE(SPI1);  // we shouldn't enable txe until we have something to send though*/

#endif // end SPI_BITBANG
}
