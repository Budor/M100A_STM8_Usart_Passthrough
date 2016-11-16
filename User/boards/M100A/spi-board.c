/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board SPI driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "spi-board.h"
#include "stm8l15x_spi.h"
#include "stm8l15x_gpio.h"

void SpiInit( Spi_t *obj, SPIName spi, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{   
    SPI_TypeDef *spix;
    /* Enable SPI clock */
    if( spi == SPI_1 )
    {
        CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
        spix = SPI1;
    }
    else if( spi == SPI_2 )
    {
        CLK_PeripheralClockConfig(CLK_Peripheral_SPI2, ENABLE);
        spix = SPI2;
    }
    else
    {
        while(1);
    }
    
    SPI_DeInit(obj->Spi);
    
    GetGpioDefine(&obj->Mosi, mosi);
    GetGpioDefine(&obj->Miso, miso);
    GetGpioDefine(&obj->Sclk, sclk);
    
    obj->Spi = ( SPI_TypeDef* )spi;
    
    /* Set the MOSI,MISO and SCK at high level */
    GPIO_ExternalPullUpConfig(obj->Mosi.port, obj->Mosi.pin, ENABLE);
    GPIO_ExternalPullUpConfig(obj->Miso.port, obj->Miso.pin, ENABLE);
    GPIO_ExternalPullUpConfig(obj->Sclk.port, obj->Sclk.pin, ENABLE);

    if( nss == NC )
    {
        /* SPI_INTERFACE Config speed=16MHz / 2*/ 
        SPI_Init(spix, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_8, SPI_Mode_Master,
                 SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_Direction_2Lines_FullDuplex,
                 SPI_NSS_Soft, (uint8_t)0x07);
    }
    else
    {
        /* SPI_INTERFACE Config speed=16MHz / 2*/ 
        SPI_Init(spix, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_8, SPI_Mode_Slave,
                 SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_Direction_2Lines_FullDuplex,
                 SPI_NSS_Soft, (uint8_t)0x07);
    
    }
    /* SPI_INTERFACE enable */
    SPI_Cmd(obj->Spi, ENABLE);   
}

void SpiDeInit( Spi_t *obj )
{
    SPI_Cmd( obj->Spi, DISABLE );
    SPI_DeInit( obj->Spi );
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    if( ( obj == NULL ) || ( obj->Spi ) == NULL )
    {
        while( 1 );
    }
    
    while( SPI_GetFlagStatus( obj->Spi, SPI_FLAG_TXE ) == RESET );
    SPI_SendData( obj->Spi, outData );
    while( SPI_GetFlagStatus( obj->Spi, SPI_FLAG_RXNE ) == RESET );
    return SPI_ReceiveData( obj->Spi );
}

