/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the STMicroelectronics STM3210E-EVAL evaluation board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM3210E_EVAL
#define BOARD_NAME              "VSS2"
#define SERIAL_LOADER_IF        SD1

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            8000000


/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 * Note: Older board revisions should define STM32F10X_HD instead, please
 *       verify the STM32 model mounted on your board. The change also
 *       affects your linker script.
 */
#define STM32F411xE
#define STM32F4XX

#define STM32_VDD 300U

#define ADXL355_USE_SPI 1

//#define STM32_I2S_SPI3_MODE STM32_I2S_MODE_MASTER | STM32_I2S_MODE_TX
//#define STM32_SDC_USE_SDMMC1    true
/*
  IO Pin Assignnment
*/

#define GPIOA_UART2_CTS      0U
#define GPIOA_UART2_RTS      1U
#define GPIOA_UART2_TX       2U
#define GPIOA_UART2_RX       3U
#define GPIOA_SPI1_CS        4U
#define GPIOA_SPI1_SCK       5U
#define GPIOA_SPI1_MISO      6U
#define GPIOA_SPI1_MOSI      7U
#define GPIOA_SPI2_CS       8U
#define GPIOA_UART1_TX       9U
#define GPIOA_UART1_RX       10U
#define GPIOA_BMI_INT1       11U
#define GPIOA_BMI_INT2         12U
#define GPIOA_SWCLK          13U
#define GPIOA_SWDIO          14U
#define GPIOA_BMI_DRDY         15U

#define GPIOB_0              0U
#define GPIOB_1              1U
#define GPIOB_2        2U
#define GPIOB_SPI3_SCK       3U
#define GPIOB_SPI3_MISO      4U
#define GPIOB_SPI3_MOSI      5U
#define GPIOB_I2C1_SCL       6U
#define GPIOB_I2C1_SDA       7U
#define GPIOB_ADXL_INT2       8U
#define GPIOB_ADXL_INT1          9U
#define GPIOB_SD_CD      10U
#define GPIOB_I2C2_SDA      11U
#define GPIOB_FS_CS0       12U
#define GPIOB_SPI2_SCK      13U
#define GPIOB_SPI2_MISO     14U
#define GPIOB_SPI2_MOSI     15U
                            
#define GPIOC_AD0    0U
#define GPIOC_AD1   1U
#define GPIOC_AD2    2U
#define GPIOC_LOG_LED    3U
#define GPIOC_4    4U
#define GPIOC_5    5U
#define GPIOC_FS_CS1    6U
#define GPIOC_FS_CS2    7U
#define GPIOC_SDIO_D0    8U
#define GPIOC_SDIO_D1    9U
#define GPIOC_SDIO_D2   10U
#define GPIOC_SDIO_D3   11U
#define GPIOC_SDIO_CLK   12U
#define GPIOC_USER_BTN   13U
#define GPIOC_OSC32_IN    14U
#define GPIOC_OSC32_OUT   15U

#define GPIOD_0    0U
#define GPIOD_1   1U
#define GPIOD_SDIO_CMD    2U
#define GPIOD_3    3U
#define GPIOD_4    4U
#define GPIOD_5    5U
#define GPIOD_6    6U
#define GPIOD_7    7U
#define GPIOD_8    8U
#define GPIOD_9    9U
#define GPIOD_10    10U
#define GPIOD_11    11U
#define GPIOD_12    12U
#define GPIOD_13    13U
#define GPIOD_14    14U
#define GPIOD_15    15U

#define GPIOE_0   0U
#define GPIOE_1   1U
#define GPIOE_2   2U
#define GPIOE_3   3U
#define GPIOE_4   4U
#define GPIOE_5   5U
#define GPIOE_6   6U
#define GPIOE_7   7U
#define GPIOE_SW11   8U
#define GPIOE_SW12   9U
#define GPIOE_SW13   10U
#define GPIOE_SW14   11U
#define GPIOE_SW21   12U
#define GPIOE_SW22   13U
#define GPIOE_SW23   14U
#define GPIOE_SW24   15U
/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))


/*
 * GPIOA setup:
 *
 * PA0  - GPIOA_UART4_TX      (AF.8)
 * PA1  - GPIOA_UART4_RX      (AF.8)
 * PA2  - GPIOA_2             (IN)
 * PA3  - GPIOA_3             (IN)
 * PA4  - GPIOA_SPI1_CS       (OUT.PP)
 * PA5  - GPIOA_SPI1_SCK      (AF.5)
 * PA6  - GPIOA_SPI1_MISO     (AF.5)
 * PA7  - GPIOA_SPI1_MOSI     (AF.5)
 * PA8  - GPIOA_ADC_INT       (IN.PU)
 * PA9  - GPIOA_UART1_TX      (AF.7)
 * PA10 - GPIOA_UART1_RX      (AF.7)
 * PA11 - GPIOA_11            (IN)
 * PA12 - GPIOA_12            (IN)
 * PA13 - GPIOA_SWDIO         (AF.0)
 * PA14 - GPIOA_SWCLK         (AF.0)
 * PA15 - GPIOA_YLED          (OUT.PP)
 */

#define VAL_GPIOA_MODER             (PIN_MODE_OUTPUT(GPIOA_UART2_CTS)         |\
                                     PIN_MODE_OUTPUT(GPIOA_UART2_RTS)         |\
                                     PIN_MODE_INPUT(GPIOA_UART2_TX )                  |\
                                     PIN_MODE_OUTPUT(GPIOA_UART2_RX )                    |\
                                        PIN_MODE_OUTPUT(GPIOA_SPI1_CS  )             |\
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK )         |\
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO)        |\
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI)        |\
                                         PIN_MODE_OUTPUT(GPIOA_SPI2_CS )              |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_TX )         |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_RX )         |\
                                         PIN_MODE_INPUT(GPIOA_BMI_INT1   )                   |\
                                         PIN_MODE_INPUT(GPIOA_BMI_INT2   )                   |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO    )            |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK    )            |\
                                        PIN_MODE_INPUT(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_UART2_CTS)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_RTS)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_TX )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_RX )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_CS  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI2_CS )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_TX )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RX )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BMI_INT1   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BMI_INT2   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO    )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK    )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_UART2_CTS)           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART2_RTS)           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART2_TX )           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART2_RX )           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_CS  )           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_SCK )            |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MISO)           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MOSI)           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI2_CS )           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART1_TX )           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART1_RX )           |\
                                     PIN_OSPEED_HIGH(GPIOA_BMI_INT1   )               |\
                                     PIN_OSPEED_HIGH(GPIOA_BMI_INT2   )           |\
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO    )           |\
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK    )           |\
                                     PIN_OSPEED_HIGH(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_UART2_CTS)         |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_RTS)         |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_TX )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_RX )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_CS  )           |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_SCK )            |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_MISO)           |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_MOSI)         |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI2_CS )           |\
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_TX )         |\
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_RX )         |\
                                     PIN_PUPDR_FLOATING(GPIOA_BMI_INT1   )             |\
                                     PIN_PUPDR_FLOATING(GPIOA_BMI_INT2   )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO    )         |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK    )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_UART2_CTS)               |\
                                     PIN_ODR_LOW(GPIOA_UART2_RTS)               |\
                                     PIN_ODR_HIGH(GPIOA_UART2_TX )               |\
                                     PIN_ODR_HIGH(GPIOA_UART2_RX )               |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_CS  )               |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_SCK )                |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_MISO)               |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI)               |\
                                     PIN_ODR_HIGH(GPIOA_SPI2_CS )               |\
                                     PIN_ODR_HIGH(GPIOA_UART1_TX )               |\
                                     PIN_ODR_HIGH(GPIOA_UART1_RX )               |\
                                     PIN_ODR_HIGH(GPIOA_BMI_INT1   )                   |\
                                     PIN_ODR_HIGH(GPIOA_BMI_INT2   )               |\
                                     PIN_ODR_HIGH(GPIOA_SWDIO    )               |\
                                     PIN_ODR_HIGH(GPIOA_SWCLK    )               |\
                                     PIN_ODR_HIGH(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_UART2_CTS,0)            |\
                                     PIN_AFIO_AF(GPIOA_UART2_RTS,0)            |\
                                     PIN_AFIO_AF(GPIOA_UART2_TX ,0)            |\
                                     PIN_AFIO_AF(GPIOA_UART2_RX ,0)            |\
                                     PIN_AFIO_AF(GPIOA_SPI1_CS  ,0)            |\
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK ,5)             |\
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO,5)            |\
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI,5))            
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_SPI1_CS ,0)             |\
                                     PIN_AFIO_AF(GPIOA_UART1_TX ,7)             |\
                                     PIN_AFIO_AF(GPIOA_UART1_RX ,7)             |\
                                     PIN_AFIO_AF(GPIOA_BMI_INT1   ,0)                 |\
                                     PIN_AFIO_AF(GPIOA_BMI_INT2   ,0)             |\
                                     PIN_AFIO_AF(GPIOA_SWDIO    ,0)             |\
                                     PIN_AFIO_AF(GPIOA_SWCLK    ,0)             |\
                                     PIN_AFIO_AF(GPIOA_BMI_DRDY   ,0))
/*
 * GPIOB setup:
 *
 */
#define VAL_GPIOB_MODER             (    PIN_MODE_INPUT(GPIOB_0        )        |\
                                         PIN_MODE_INPUT(GPIOB_1        )        |\
                                        PIN_MODE_OUTPUT(GPIOB_2  )     |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI3_SCK )        |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI3_MISO)       |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI3_MOSI)        |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL )       |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA )       |\
                                         PIN_MODE_INPUT(GPIOB_ADXL_INT2 )       |\
                                         PIN_MODE_INPUT(GPIOB_ADXL_INT1    )          |\
                                     PIN_MODE_INPUT(GPIOB_SD_CD )       |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2C2_SDA )       |\
                                        PIN_MODE_OUTPUT(GPIOB_FS_CS0  )       |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_SCK )       |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MISO)          |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OTYPER            ( PIN_OTYPE_PUSHPULL(GPIOB_0        )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_1        )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_2  )    |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI3_SCK )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI3_MISO)      |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI3_MOSI)       |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL )       |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_ADXL_INT2 )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_ADXL_INT1    )       |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_SD_CD )       |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SDA )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_FS_CS0  )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_SCK )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MISO)       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_0        )          |\
                                     PIN_OSPEED_HIGH(GPIOB_1        )          |\
                                     PIN_OSPEED_HIGH(GPIOB_2  )       |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI3_SCK )          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI3_MISO)         |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI3_MOSI)          |\
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL )          |\
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA )          |\
                                     PIN_OSPEED_HIGH(GPIOB_ADXL_INT2 )          |\
                                     PIN_OSPEED_HIGH(GPIOB_ADXL_INT1    )          |\
                                     PIN_OSPEED_HIGH(GPIOB_SD_CD )          |\
                                     PIN_OSPEED_HIGH(GPIOB_I2C2_SDA )          |\
                                     PIN_OSPEED_HIGH(GPIOB_FS_CS0  )          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_SCK )          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MISO)          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_0        )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_1        )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_2  )    |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI3_SCK )       |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_SPI3_MISO)      |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI3_MOSI)       |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA )       |\
                                     PIN_PUPDR_PULLUP(GPIOB_ADXL_INT2 )       |\
                                     PIN_PUPDR_PULLUP(GPIOB_ADXL_INT1    )       |\
                                     PIN_PUPDR_PULLUP(GPIOB_SD_CD )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2C2_SDA )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_FS_CS0  )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_SCK )       |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_SPI2_MISO)       |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_0        )             |\
                                     PIN_ODR_HIGH(GPIOB_1        )             |\
                                     PIN_ODR_HIGH(GPIOB_2  )          |\
                                     PIN_ODR_HIGH(GPIOB_SPI3_SCK )             |\
                                     PIN_ODR_HIGH(GPIOB_SPI3_MISO)            |\
                                     PIN_ODR_HIGH(GPIOB_SPI3_MOSI)             |\
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL )             |\
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA )             |\
                                     PIN_ODR_HIGH(GPIOB_ADXL_INT2 )             |\
                                     PIN_ODR_HIGH(GPIOB_ADXL_INT1    )             |\
                                     PIN_ODR_HIGH(GPIOB_SD_CD )             |\
                                     PIN_ODR_HIGH(GPIOB_I2C2_SDA )             |\
                                     PIN_ODR_HIGH(GPIOB_FS_CS0  )             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_SCK )             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_MISO)             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_0        ,0)           |\
                                     PIN_AFIO_AF(GPIOB_1        ,0)           |\
                                     PIN_AFIO_AF(GPIOB_2  ,0)        |\
                                     PIN_AFIO_AF(GPIOB_SPI3_SCK ,6)           |\
                                     PIN_AFIO_AF(GPIOB_SPI3_MISO,6)          |\
                                     PIN_AFIO_AF(GPIOB_SPI3_MOSI,6)           |\
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL ,4 )            |\
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA ,4 ))           
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_ADXL_INT2 ,0)           |\
                                     PIN_AFIO_AF(GPIOB_ADXL_INT1    ,0)           |\
                                     PIN_AFIO_AF(GPIOB_SD_CD ,0)           |\
                                     PIN_AFIO_AF(GPIOB_I2C2_SDA ,0)           |\
                                     PIN_AFIO_AF(GPIOB_FS_CS0  ,0 )           |\
                                     PIN_AFIO_AF(GPIOB_SPI2_SCK ,5)           |\
                                     PIN_AFIO_AF(GPIOB_SPI2_MISO,5)            |\
                                     PIN_AFIO_AF(GPIOB_SPI2_MOSI,5))

/*
 * GPIOC setup:
 *
 */
 
#define VAL_GPIOC_MODER             ( PIN_MODE_ANALOG(GPIOC_AD0   )        |\
                                     PIN_MODE_ANALOG(GPIOC_AD1  )        |\
                                     PIN_MODE_ANALOG(GPIOC_AD2  )        |\
                                      PIN_MODE_OUTPUT(GPIOC_LOG_LED)        |\
                                      PIN_MODE_OUTPUT(GPIOC_4)        |\
                                      PIN_MODE_INPUT(GPIOC_5        )        |\
                                      PIN_MODE_OUTPUT(GPIOC_FS_CS1        )        |\
                                      PIN_MODE_OUTPUT(GPIOC_FS_CS2        )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D0  )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D1  )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D2  )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D3  )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_CLK )        |\
                                      PIN_MODE_INPUT(GPIOC_USER_BTN  )        |\
                                      PIN_MODE_INPUT(GPIOC_OSC32_IN )        |\
                                      PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_AD0   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD1  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD2  )        |\
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LOG_LED)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_4)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_5        )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_FS_CS1        )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_FS_CS2        )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CLK )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_USER_BTN  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_AD0   )           |\
                                     PIN_OSPEED_HIGH(GPIOC_AD1  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_AD2  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_LOG_LED)           |\
                                     PIN_OSPEED_HIGH(GPIOC_4)           |\
                                     PIN_OSPEED_HIGH(GPIOC_5        )           |\
                                     PIN_OSPEED_HIGH(GPIOC_FS_CS1        )           |\
                                     PIN_OSPEED_HIGH(GPIOC_FS_CS2        )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D0  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D1  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D2  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D3  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_CLK )           |\
                                     PIN_OSPEED_HIGH(GPIOC_USER_BTN  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_IN )           |\
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_AD0   )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_AD1  )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_AD2  )        |\
                                     PIN_PUPDR_FLOATING(GPIOC_LOG_LED )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_4 )        |\
                                       PIN_PUPDR_PULLUP(GPIOC_5        )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_FS_CS1        )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_FS_CS2        )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D0  )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D1  )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D2  )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D3  )        |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_SDIO_CLK )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_USER_BTN  )        |\
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN )        |\
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_AD0   )              |\
                                     PIN_ODR_HIGH(GPIOC_AD1  )              |\
                                     PIN_ODR_HIGH(GPIOC_AD2  )              |\
                                     PIN_ODR_HIGH(GPIOC_LOG_LED)              |\
                                     PIN_ODR_HIGH(GPIOC_4)              |\
                                     PIN_ODR_HIGH(GPIOC_5        )              |\
                                     PIN_ODR_HIGH(GPIOC_FS_CS1        )              |\
                                     PIN_ODR_HIGH(GPIOC_FS_CS2        )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_D0  )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_D1  )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_D2  )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_D3  )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_CLK )              |\
                                     PIN_ODR_HIGH(GPIOC_USER_BTN  )              |\
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN )              |\
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_AD0   ,0   )               |\
                                     PIN_AFIO_AF(GPIOC_AD1  ,0   )               |\
                                     PIN_AFIO_AF(GPIOC_AD2  ,0  )               |\
                                     PIN_AFIO_AF(GPIOC_LOG_LED,0)               |\
                                     PIN_AFIO_AF(GPIOC_4,0  )               |\
                                     PIN_AFIO_AF(GPIOC_5        ,0  )               |\
                                     PIN_AFIO_AF(GPIOC_FS_CS1        ,0      )               |\
                                     PIN_AFIO_AF(GPIOC_FS_CS2        ,0      ))              
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SDIO_D0  ,12      )               |\
                                     PIN_AFIO_AF(GPIOC_SDIO_D1  ,12     )               |\
                                     PIN_AFIO_AF(GPIOC_SDIO_D2  ,12  )               |\
                                     PIN_AFIO_AF(GPIOC_SDIO_D3  ,12 )               |\
                                     PIN_AFIO_AF(GPIOC_SDIO_CLK ,12 )               |\
                                     PIN_AFIO_AF(GPIOC_USER_BTN  ,0     )               |\
                                     PIN_AFIO_AF(GPIOC_OSC32_IN ,0  )               |\
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT,0 ))

// GPIOD

#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_0  )        |\
                                     PIN_MODE_INPUT(GPIOD_1  )        |\
                                     PIN_MODE_ALTERNATE(GPIOD_SDIO_CMD  )        |\
                                     PIN_MODE_INPUT(GPIOD_3  )        |\
                                     PIN_MODE_INPUT(GPIOD_4  )        |\
                                     PIN_MODE_INPUT(GPIOD_5  )        |\
                                     PIN_MODE_INPUT(GPIOD_6  )        |\
                                     PIN_MODE_INPUT(GPIOD_7  )        |\
                                     PIN_MODE_INPUT(GPIOD_8  )        |\
                                     PIN_MODE_INPUT(GPIOD_9  )        |\
                                     PIN_MODE_INPUT(GPIOD_10 )        |\
                                     PIN_MODE_INPUT(GPIOD_11 )        |\
                                     PIN_MODE_INPUT(GPIOD_12 )        |\
                                     PIN_MODE_INPUT(GPIOD_13 )        |\
                                     PIN_MODE_INPUT(GPIOD_14 )        |\
                                     PIN_MODE_INPUT(GPIOD_15 ))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_0 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_1 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDIO_CMD )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_3 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_4 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_5   		)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_6   		)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_7 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_8   		)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_9   		)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_10)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_11)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_12)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_13)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_14)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_0 	)           |\
                                     PIN_OSPEED_HIGH(GPIOD_1 )           |\
                                     PIN_OSPEED_HIGH(GPIOD_SDIO_CMD  )           |\
                                     PIN_OSPEED_HIGH(GPIOD_3 	)           |\
                                     PIN_OSPEED_HIGH(GPIOD_4 	)           |\
                                     PIN_OSPEED_HIGH(GPIOD_5    		)           |\
                                     PIN_OSPEED_HIGH(GPIOD_6    		)           |\
                                     PIN_OSPEED_HIGH(GPIOD_7 	)           |\
                                     PIN_OSPEED_HIGH(GPIOD_8    		)           |\
                                     PIN_OSPEED_HIGH(GPIOD_9    		)           |\
                                     PIN_OSPEED_HIGH(GPIOD_10)           |\
                                     PIN_OSPEED_HIGH(GPIOD_11)           |\
                                     PIN_OSPEED_HIGH(GPIOD_12)           |\
                                     PIN_OSPEED_HIGH(GPIOD_13)           |\
                                     PIN_OSPEED_HIGH(GPIOD_14)           |\
                                     PIN_OSPEED_HIGH(GPIOD_15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_0 )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_1 )        |\
                                     PIN_PUPDR_PULLUP(GPIOD_SDIO_CMD )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_3 )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_4 )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_5   		)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_6   		)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_7 )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_8   		)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_9   		)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_10)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_11)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_12)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_13)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_14)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_0 )              |\
                                     PIN_ODR_HIGH(GPIOD_1 )              |\
                                     PIN_ODR_HIGH(GPIOD_SDIO_CMD )              |\
                                     PIN_ODR_HIGH(GPIOD_3 )              |\
                                     PIN_ODR_HIGH(GPIOD_4 )              |\
                                     PIN_ODR_HIGH(GPIOD_5 		)              |\
                                     PIN_ODR_HIGH(GPIOD_6 		)              |\
                                     PIN_ODR_HIGH(GPIOD_7 )              |\
                                     PIN_ODR_HIGH(GPIOD_8 		)              |\
                                     PIN_ODR_HIGH(GPIOD_9 		)              |\
                                     PIN_ODR_HIGH(GPIOD_10)            |\
                                     PIN_ODR_HIGH(GPIOD_11)              |\
                                     PIN_ODR_HIGH(GPIOD_12)              |\
                                     PIN_ODR_HIGH(GPIOD_13)              |\
                                     PIN_ODR_HIGH(GPIOD_14)              |\
                                     PIN_ODR_HIGH(GPIOD_15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_0 ,0)           |\
                                     PIN_AFIO_AF(GPIOD_1 ,0)    |\
                                     PIN_AFIO_AF(GPIOD_SDIO_CMD ,12)       |\
                                     PIN_AFIO_AF(GPIOD_3 ,0)     |\
                                     PIN_AFIO_AF(GPIOD_4 ,0)       |\
                                     PIN_AFIO_AF(GPIOD_5 ,0)           |\
                                     PIN_AFIO_AF(GPIOD_6 ,0)               |\
                                     PIN_AFIO_AF(GPIOD_7 ,0))           
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_8 ,0)            |\
                                     PIN_AFIO_AF(GPIOD_9 ,0)           |\
                                     PIN_AFIO_AF(GPIOD_10,0) |\
                                     PIN_AFIO_AF(GPIOD_11,0)|\
                                     PIN_AFIO_AF(GPIOD_12,0)|\
                                     PIN_AFIO_AF(GPIOD_13,0)    |\
                                     PIN_AFIO_AF(GPIOD_14,0) |\
                                     PIN_AFIO_AF(GPIOD_15,0))

// GPIOE

#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_0   )        |\
                                     PIN_MODE_INPUT(GPIOE_1   )        |\
                                     PIN_MODE_INPUT(GPIOE_2   )        |\
                                     PIN_MODE_INPUT(GPIOE_3   )        |\
                                     PIN_MODE_INPUT(GPIOE_4   )        |\
                                     PIN_MODE_INPUT(GPIOE_5   )        |\
                                     PIN_MODE_INPUT(GPIOE_6   )        |\
                                     PIN_MODE_INPUT(GPIOE_7   )        |\
                                     PIN_MODE_INPUT(GPIOE_SW11)        |\
                                     PIN_MODE_INPUT(GPIOE_SW12)        |\
                                     PIN_MODE_INPUT(GPIOE_SW13)        |\
                                     PIN_MODE_INPUT(GPIOE_SW14)        |\
                                     PIN_MODE_INPUT(GPIOE_SW21)        |\
                                     PIN_MODE_INPUT(GPIOE_SW22)        |\
                                     PIN_MODE_INPUT(GPIOE_SW23)        |\
                                     PIN_MODE_INPUT(GPIOE_SW24))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_0   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_1   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_2   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_3   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_4   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_5   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_6   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_7   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW11)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW12)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW13)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW14)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW21)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW22)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW23)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW24))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_0   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_1   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_2   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_3   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_4   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_5   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_6   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_7   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW11)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW12)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW13)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW14)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW21)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW22)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW23)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW24))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_0   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_1   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_2   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_3   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_4   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_5   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_6   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_7   )        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW11)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW12)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW13)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW14)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW21)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW22)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW23)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW24))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_0   )              |\
                                     PIN_ODR_HIGH(GPIOE_1   )              |\
                                     PIN_ODR_HIGH(GPIOE_2   )              |\
                                     PIN_ODR_HIGH(GPIOE_3   )              |\
                                     PIN_ODR_HIGH(GPIOE_4   )              |\
                                     PIN_ODR_HIGH(GPIOE_5   )              |\
                                     PIN_ODR_HIGH(GPIOE_6   )              |\
                                     PIN_ODR_HIGH(GPIOE_7   )              |\
                                     PIN_ODR_HIGH(GPIOE_SW11)              |\
                                     PIN_ODR_HIGH(GPIOE_SW12)              |\
                                     PIN_ODR_HIGH(GPIOE_SW13)              |\
                                     PIN_ODR_HIGH(GPIOE_SW14)              |\
                                     PIN_ODR_HIGH(GPIOE_SW21)              |\
                                     PIN_ODR_HIGH(GPIOE_SW22)              |\
                                     PIN_ODR_HIGH(GPIOE_SW23)              |\
                                     PIN_ODR_HIGH(GPIOE_SW24))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_0,0   )               |\
                                     PIN_AFIO_AF(GPIOE_1,0 )               |\
                                     PIN_AFIO_AF(GPIOE_2,0)               |\
                                     PIN_AFIO_AF(GPIOE_3,0)              |\
                                     PIN_AFIO_AF(GPIOE_4,0)               |\
                                     PIN_AFIO_AF(GPIOE_5,0)               |\
                                     PIN_AFIO_AF(GPIOE_6,0   )               |\
                                     PIN_AFIO_AF(GPIOE_7,0   ))              
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_SW11,0     )               |\
                                     PIN_AFIO_AF(GPIOE_SW12,0    )               |\
                                     PIN_AFIO_AF(GPIOE_SW13,0 )               |\
                                     PIN_AFIO_AF(GPIOE_SW14,0)               |\
                                     PIN_AFIO_AF(GPIOE_SW21,0)               |\
                                     PIN_AFIO_AF(GPIOE_SW22,0     )               |\
                                     PIN_AFIO_AF(GPIOE_SW23,0  )               |\
                                     PIN_AFIO_AF(GPIOE_SW24,0 ))


#define VAL_GPIOF_MODER         0
#define VAL_GPIOF_OTYPER        0
#define VAL_GPIOF_OSPEEDR       0
#define VAL_GPIOF_PUPDR         0
#define VAL_GPIOF_ODR           0
#define VAL_GPIOF_AFRL          0
#define VAL_GPIOF_AFRH          0

#define VAL_GPIOG_MODER         0
#define VAL_GPIOG_OTYPER        0
#define VAL_GPIOG_OSPEEDR       0
#define VAL_GPIOG_PUPDR         0
#define VAL_GPIOG_ODR           0
#define VAL_GPIOG_AFRL          0
#define VAL_GPIOG_AFRH          0

#define VAL_GPIOH_MODER         0
#define VAL_GPIOH_OTYPER        0
#define VAL_GPIOH_OSPEEDR       0
#define VAL_GPIOH_PUPDR         0
#define VAL_GPIOH_ODR           0
#define VAL_GPIOH_AFRL          0
#define VAL_GPIOH_AFRH          0

#define VAL_GPIOI_MODER         0
#define VAL_GPIOI_OTYPER        0
#define VAL_GPIOI_OSPEEDR       0
#define VAL_GPIOI_PUPDR         0
#define VAL_GPIOI_ODR           0
#define VAL_GPIOI_AFRL          0
#define VAL_GPIOI_AFRH          0

#define SPI1_CS_LOW()   palClearPad(GPIOA,GPIOA_SPI1_CS)
#define SPI1_CS_HIGH()   palSetPad(GPIOA,GPIOA_SPI1_CS)
#define SPI2_CS_LOW()   palClearPad(GPIOB,GPIOB_SPI2_CS)
#define SPI2_CS_HIGH()   palSetPad(GPIOB,GPIOB_SPI2_CS)
#define SPI3_CS_LOW()   palClearPad(GPIOA,GPIOA_SPI3_CS)
#define SPI3_CS_HIGH()   palSetPad(GPIOA,GPIOA_SPI3_CS)

#define LED_ON()        palClearPad(GPIOC,3)
#define LED_OFF()       palSetPad(GPIOC,3)
#define LED_TOG()       palTogglePad(GPIOC,3)

//#define LED_ON()        
//#define LED_OFF()       
//#define LED_TOG()       

//#define DBG_LED_ON()        palClearPad(GPIOB,2)
//#define DBG_LED_OFF()       palSetPad(GPIOB,2)
//#define DBG_LED_TOG()       palTogglePad(GPIOB,2)

#define DBG_LED_ON()        
#define DBG_LED_OFF()       
#define DBG_LED_TOG()       

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
