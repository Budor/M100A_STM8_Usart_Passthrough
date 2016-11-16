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
#include "board.h"

/*
 * MCU objects
 */

Uart_t Uart1;
Uart_t Uart3;

/**
  * @brief  Configure peripheral clock 
  * @param  None
  * @retval None
  */
static void CLK_Config(void)
{
    /* Select HSE as system clock source */
    CLK_SYSCLKSourceSwitchCmd(ENABLE);
    CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSE);

    /*High speed external clock prescaler: 1*/
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);

    while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSE)
    {}
}

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = FALSE;

void BoardInitMcu( void )
{
    if( McuInitialized == FALSE )
    {
        CLK_Config( );

        SpiInit( &SX1276.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );

        SX1276IoInit( );
        
        TimerHwInit( );
        
        enableInterrupts( );
        McuInitialized = TRUE;
    }
}
