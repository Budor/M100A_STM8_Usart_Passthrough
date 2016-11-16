/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: MCU RTC timer and low power modes management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <math.h>
#include "board.h"
#include "timer-board.h"

/*!
 * Hardware Time base in us
 */
#define HW_TIMER_TIME_BASE                              100 //us 

/*!
 * Hardware Timer tick counter
 */
TimerTime_t TimerTickCounter = 1;

/*!
 * Saved value of the Tick counter at the start of the next event
 */
static TimerTime_t TimerTickCounterContext = 0;

/*!
 * Value trigging the IRQ
 */
volatile TimerTime_t TimeoutCntValue = 0;

/*!
 * Increment the Hardware Timer tick counter
 */
void TimerIncrementTickCounter( void );

/*!
 * Counter used for the Delay operations
 */
volatile uint32_t TimerDelayCounter = 0;

/*!
 * Return the value of the counter used for a Delay
 */
uint32_t TimerHwGetDelayValue( void );

/*!
 * Increment the value of TimerDelayCounter
 */
void TimerIncrementDelayCounter( void );


void TimerHwInit( void )
{
    // config time2 100us
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);
    ITC_SetSoftwarePriority(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQn, ITC_PriorityLevel_2);
    TIM2_TimeBaseInit(TIM2_Prescaler_1, TIM2_CounterMode_Up, 1599);
    TIM2_ClearFlag(TIM2_FLAG_Update);
    TIM2_ITConfig(TIM2_IT_Update, ENABLE);
    TIM2_Cmd(ENABLE);
    
    // config time3 1ms
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);
    ITC_SetSoftwarePriority(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQn, ITC_PriorityLevel_3);
    TIM3_TimeBaseInit(TIM3_Prescaler_1, TIM3_CounterMode_Up, 15999);
    TIM3_ClearFlag(TIM3_FLAG_Update);
    TIM3_ITConfig(TIM3_IT_Update, ENABLE);
    TIM3_Cmd(ENABLE);
    
}

void TimerHwDeInit( void )
{
    /* Deinitialize the timer */
    TIM2_DeInit();
}

uint32_t TimerHwGetMinimumTimeout( void )
{
    return (uint32_t)( ceil( 2 * HW_TIMER_TIME_BASE ) );
}

void TimerHwStart( uint32_t val )
{
    TimerTickCounterContext = TimerHwGetTimerValue( );

    if( val <= HW_TIMER_TIME_BASE + 1 )
    {
        TimeoutCntValue = TimerTickCounterContext + 1;
    }
    else
    {
        TimeoutCntValue = TimerTickCounterContext + ( ( val - 1 ) / HW_TIMER_TIME_BASE );
    }
}

void TimerHwStop( void )
{
    TIM2_ITConfig( TIM2_IT_CC1, DISABLE );
    TIM2_Cmd( DISABLE );
}

void TimerHwDelayMs( uint32_t delay )
{
    uint32_t delayValue = 0;

    delayValue = delay;

    TimerDelayCounter = 0;

    TIM3_ITConfig( TIM3_IT_Update, ENABLE );
    TIM3_Cmd( ENABLE );

    while( TimerHwGetDelayValue( ) < delayValue )
    {
    }

    TIM3_ITConfig( TIM3_IT_Update, DISABLE );
    TIM3_Cmd( DISABLE );
}

TimerTime_t TimerHwGetElapsedTime( void )
{
     return( ( ( TimerHwGetTimerValue( ) - TimerTickCounterContext ) + 1 )  * HW_TIMER_TIME_BASE );
}

TimerTime_t TimerHwGetTimerValue( void )
{
    TimerTime_t val = 0;

    val = TimerTickCounter;

    return( val );
}

TimerTime_t TimerHwGetTime( void )
{

    return TimerHwGetTimerValue( ) * HW_TIMER_TIME_BASE;
}

uint32_t TimerHwGetDelayValue( void )
{
    uint32_t val = 0;

    val = TimerDelayCounter;

    return( val );
}

void TimerIncrementTickCounter( void )
{
    TimerTickCounter++;
}

void TimerIncrementDelayCounter( void )
{
    TimerDelayCounter++;
}

/*!
 * Timer IRQ handler
 */
INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler,19)
{
    if( TIM2_GetITStatus( TIM2_IT_Update ) != RESET )
    {
        TimerIncrementTickCounter( );
        TIM2_ClearITPendingBit( TIM2_IT_Update );
    
        if( TimerTickCounter == TimeoutCntValue )
        {
            TimerIrqHandler( );
        }
    }
}

/*!
 * Timer IRQ handler
 */
INTERRUPT_HANDLER(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler,21)
{
    if( TIM3_GetITStatus( TIM3_IT_Update ) != RESET )
    {
        TimerIncrementDelayCounter( );
        TIM3_ClearITPendingBit( TIM3_IT_Update );
    }
}

void TimerHwEnterLowPowerStopMode( void )
{
#ifndef USE_DEBUGGER
    wfi( );
#endif
}
