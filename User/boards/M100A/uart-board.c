/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board UART driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"

#include "uart-board.h"

#define COMn    2

/*!
 * FIFO buffers size
 */
#define FIFO_RX_SIZE                                256
#define FIFO_TX_SIZE                                256

uint8_t UsartRxBuffer[FIFO_RX_SIZE];
uint8_t UsartTxBuffer[FIFO_TX_SIZE];

USART_TypeDef* COM_USART[COMn] =
{
    USART1, 
    USART3
};

const uint8_t COM_USART_CLK[COMn] =
{
    CLK_Peripheral_USART1,
    CLK_Peripheral_USART3
};

void UartMcuInit( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx )
{
    obj->UartId = uartId;

    GetGpioDefine(&obj->Rx, rx);
    GetGpioDefine(&obj->Tx, tx);
    
    /* Enable USART clock */
    CLK_PeripheralClockConfig((CLK_Peripheral_TypeDef)COM_USART_CLK[obj->UartId], ENABLE);
    
    USART_DeInit( COM_USART[obj->UartId] );
    
    /* Configure USART Tx as alternate function push-pull  (software pull up)*/
    GPIO_ExternalPullUpConfig(obj->Tx.port, obj->Tx.pin, ENABLE);

    /* Configure USART Rx as alternate function push-pull  (software pull up)*/
    GPIO_ExternalPullUpConfig(obj->Rx.port, obj->Rx.pin, ENABLE);  
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    USART_Mode_TypeDef                          usart_mode;
    USART_StopBits_TypeDef                      usart_stop_bits;
    USART_Parity_TypeDef                        usart_parity;
    
    FifoInit( &obj->FifoRx, UsartRxBuffer, FIFO_RX_SIZE );
    FifoInit( &obj->FifoTx, UsartTxBuffer, FIFO_TX_SIZE );
    
    if( mode == TX_ONLY )
    {
        if( obj->FifoTx.Data == NULL )
        {
            while( 1 );
        }

        usart_mode = (USART_Mode_TypeDef)USART_Mode_Tx;
    }
    else if( mode == RX_ONLY )
    {
        if( obj->FifoRx.Data == NULL )
        {
            while( 1 );
        }

        usart_mode = (USART_Mode_TypeDef)USART_Mode_Rx;
    }
    else
    {
        if( ( obj->FifoTx.Data == NULL ) || ( obj->FifoRx.Data == NULL ) )
        {
            while( 1 );
        }

        usart_mode = (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx);
    }

    if( wordLength == UART_8_BIT )
    {
        obj->UsartWordLength = USART_WordLength_8b;
    }
    else 
    {
        obj->UsartWordLength = USART_WordLength_9b;
    }

    if( stopBits == UART_1_STOP_BIT )
    {
        usart_stop_bits = USART_StopBits_1;
    }
    else if( stopBits == UART_2_STOP_BIT )
    {
        usart_stop_bits = USART_StopBits_2;
    }
    else if( stopBits == UART_1_5_STOP_BIT )
    {
        usart_stop_bits = USART_StopBits_1_5;
    }

    if( parity == NO_PARITY )
    {
        usart_parity = USART_Parity_No;
    }
    else if( parity == EVEN_PARITY )
    {
        usart_parity = USART_Parity_Even;
    }
    else
    {
        usart_parity = USART_Parity_Odd;
    }

    /* USART configuration */
    USART_Init(COM_USART[obj->UartId], baudrate,
               obj->UsartWordLength,
               usart_stop_bits,
               usart_parity,
               usart_mode);
    
    if(obj->UartId == UART_1)
    {
        /* Enable the USART Receive interrupt: this interrupt is generated when the USART
          receive data register is not empty */
        USART_ITConfig(COM_USART[obj->UartId], USART_IT_RXNE, ENABLE);
        /* Enable the USART Transmit interrupt */
        USART_ITConfig(COM_USART[obj->UartId], USART_IT_TXE, ENABLE);
        /* Enable the USART Overrun Error interrupt */
        USART_ITConfig(COM_USART[obj->UartId], USART_IT_OR, ENABLE);
        
        ITC_SetSoftwarePriority(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQn, ITC_PriorityLevel_3);
        
        ITC_SetSoftwarePriority(USART1_RX_TIM5_CC_IRQn, ITC_PriorityLevel_3);
    }

    /* Enable USART */
    USART_Cmd( COM_USART[obj->UartId], ENABLE );
}

void UartMcuDeInit( Uart_t *obj )
{
    USART_DeInit( COM_USART[obj->UartId] );

    GPIO_ExternalPullUpConfig(obj->Tx.port, obj->Tx.pin, DISABLE);
    GPIO_ExternalPullUpConfig(obj->Rx.port, obj->Rx.pin, DISABLE);
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data )
{
    if( IsFifoFull( &obj->FifoTx ) == FALSE )
    {
        FifoPush( &obj->FifoTx, data );
        // Enable the USART Transmit interrupt
        USART_ITConfig( COM_USART[obj->UartId], USART_IT_TXE, ENABLE );
        return 0; // OK
    }
    return 1; // Busy
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data )
{
    if( IsFifoEmpty( &obj->FifoRx ) == FALSE )
    {
        *data = FifoPop( &obj->FifoRx );
        return 0;
    }
    return 1;
}


/**
  * @brief USART1 TX / TIM5 Update/Overflow/Trigger/Break Interrupt  routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler,27)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    uint8_t data;

    if( USART_GetITStatus( USART1, USART_IT_TXE ) != RESET )
    {    
        if( IsFifoEmpty( &Uart1.FifoTx ) == FALSE )
        {
            data = FifoPop( &Uart1.FifoTx );
            //  Write one byte to the transmit data register 
            if( Uart1.UsartWordLength == USART_WordLength_8b )
            {
                USART_SendData8( USART1, data );
            }
            else
            {
                USART_SendData9( USART1, data );
            }
        }
        else
        {
            // Disable the USART Transmit interrupt
            USART_ITConfig( USART1, USART_IT_TXE, DISABLE );
        }
        if( Uart1.IrqNotify != NULL )
        {
            Uart1.IrqNotify( UART_NOTIFY_TX );
        }
    }
}

/**
  * @brief USART1 RX / Timer5 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    uint8_t data;

    if( USART_GetITStatus( USART1, USART_IT_RXNE ) != RESET )
    {    
        if( Uart1.UsartWordLength == USART_WordLength_8b)
        {
            data = USART_ReceiveData8( USART1 );
        }
        else
        {
            data = USART_ReceiveData9( USART1 );
        }
        if( IsFifoFull( &Uart1.FifoRx ) == FALSE )
        {
            // Read one byte from the receive data register
            FifoPush( &Uart1.FifoRx, data );
        }
        if( Uart1.IrqNotify != NULL )
        {
            Uart1.IrqNotify( UART_NOTIFY_RX );
        }
    }
    
    if( USART_GetITStatus( USART1, USART_IT_OR ) != RESET )
    {
        if( Uart1.UsartWordLength == USART_WordLength_8b)
        {
            USART_ReceiveData8( USART1 );
        }
        else
        {
            USART_ReceiveData9( USART1 );
        }
    }
}