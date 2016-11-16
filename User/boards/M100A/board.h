/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdio.h>
#include "stm8l15x.h"
#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "gpio.h"
#include "spi.h"
#include "uart.h"
#include "radio.h"
#include "sx1276/sx1276.h"
#include "gpio-board.h"
#include "timer-board.h"
#include "sx1276-board.h"
#include "uart-board.h"
#include "m100a-board.h"
#include "crc16.h"

#define USEDEBUG

#ifdef USEDEBUG
#define DEBUG   printf
#else
#define DEBUG
#endif

/*!
 * Define indicating if an external IO expander is to be used
 */
//#define BOARD_IOE_EXT

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0  
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

#define TimerTime_t                                uint32_t
/*!
 * Board IO Test pins definitions
 */
#define LED_1                                       PB_1
#define LED_2                                       PB_2
#define KEY_1                                       PB_3
#define KEY_2                                       PD_3
#define RX_LED                                      PB_2        
#define TX_LED                                      PB_3        
/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 PB_1
// SPI_2
#define RADIO_MISO                                  PD_4
#define RADIO_MOSI                                  PD_5
#define RADIO_SCLK                                  PD_6
#define RADIO_NSS                                   PD_7

// SPI_1
#define SPI1_MISO                                   PB_7
#define SPI1_MOSI                                   PB_6
#define SPI1_SCLK                                   PB_5
#define SPI1_NSS                                    PB_4

#define RADIO_DIO_0                                 PE_0
#define RADIO_DIO_1                                 PE_1
#define RADIO_DIO_2                                 PE_2
#define RADIO_DIO_3                                 PE_3        // UART2_RX
#define RADIO_DIO_4                                 PE_4        // UART2_TX
#define RADIO_DIO_5                                 PE_5

#define IO_0                                        PB_0
#define IO_1                                        PF_0
#define IO_2                                        PA_4
#define IO_4                                        PA_5
#define IO_5                                        PA_7

#define LORA_RX_LED                                 PB_2        // TIM2_CH2
#define LORA_TX_LED                                 PB_3        // TIM2_ETR        

#define STM8_CTS                                    PC_4        // I'll transmit if okay
#define STM8_RTS                                    PC_7        // I am ready to receive
#define MODE_SET                                    PA_6

#define RADIO_ANT_SWITCH_LF                         PD_0        // FEM_CPS

#define OSC_LSE_IN                                  PC_5
#define OSC_LSE_OUT                                 PC_6

#define OSC_HSE_IN                                  PA_2
#define OSC_HSE_OUT                                 PA_3

#define I2C_SCL                                     PC_1
#define I2C_SDA                                     PC_0

#define UART1_TX                                    PC_3
#define UART1_RX                                    PC_2 

#define UART3_TX                                    PE_6
#define UART3_RX                                    PE_7

#define RF_RXTX                                     PD_0        // FEM_CPS

#define SWIM                                        PA_0
                                     
#define PIN_NC_1                                    PD_1
#define PIN_NC_2                                    PD_2
#define PIN_NC_3                                    PD_3


/*!
 * MCU objects
 */
extern Uart_t Uart1;
extern Uart_t Uart3;


/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );


#endif // __BOARD_H__
