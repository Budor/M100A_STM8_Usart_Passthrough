/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board GPIO driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "gpio-board.h"
#include "stm8l15x_gpio.h"

static GpioIrqHandler *GpioIrq[8];

bool GetGpioDefine(Gpio_t *obj, PinNames pin)
{
    GPIO_TypeDef* port[9] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};

    if( pin == NC ) 
    {
        return FALSE;
    }
    else
    {
        obj->portIndex = (pin / 8);
        obj->port = port[obj->portIndex];
        
        obj->pinIndex = pin;
        obj->pin = (PinNames)( 0x01 << ( obj->pinIndex % 8 ) );
    }
    return TRUE;
}

void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
    if( GetGpioDefine(obj, pin) == FALSE ) 
    {
        return;
    }
    
    if( mode == PIN_INPUT_NO_IT )
    {
        if(config == PIN_PUSH_PULL)
        {
            GPIO_Init(obj->port, obj->pin, GPIO_Mode_In_PU_No_IT);
        }
        else if(config == PIN_FLOATING)
        {
            GPIO_Init(obj->port, obj->pin, GPIO_Mode_In_FL_No_IT);
        }
    }
    else if( mode == PIN_INPUT_IT )
    {
        if(config == PIN_PUSH_PULL)
        {
            GPIO_Init(obj->port, obj->pin, GPIO_Mode_In_PU_IT);
        }
        else if(config == PIN_FLOATING)
        {
            GPIO_Init(obj->port, obj->pin, GPIO_Mode_In_FL_IT);
        }
    }
    else if(mode == PIN_OUTPUT)
    {
        if(config == PIN_PUSH_PULL)
        {
            if(type == PIN_PULL_UP)
            {
                GPIO_Init(obj->port, obj->pin, GPIO_Mode_Out_PP_High_Fast);
            }
            else if(type == PIN_PULL_DOWN)
            {
                GPIO_Init(obj->port, obj->pin, GPIO_Mode_Out_PP_Low_Fast);
            }
            else
            {
                while(1); // other type
            }
        }
        else if(config == PIN_OPEN_DRAIN)
        {
            if(type == PIN_HI_Z)
            {
                GPIO_Init(obj->port, obj->pin, GPIO_Mode_Out_OD_HiZ_Fast);
            }
            else if(type == PIN_PULL_DOWN)
            {
                GPIO_Init(obj->port, obj->pin, GPIO_Mode_Out_OD_Low_Fast);
            }
            else
            {
                while(1);       // other type       
            }
        } 
        else
        {
            while(1);   // other config
        }
        GpioMcuWrite( obj, value );
    }
    else
    {
        while(1); // other mode 
    }
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    EXTI_Trigger_TypeDef                trigger_type;
    ITC_PriorityLevel_TypeDef           priority_level;
    IRQn_TypeDef irq[8] =               {EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn, EXTI5_IRQn, EXTI6_IRQn, EXTI7_IRQn};
    EXTI_Pin_TypeDef exti_pin[8] =      {EXTI_Pin_0, EXTI_Pin_1, EXTI_Pin_2, EXTI_Pin_3, EXTI_Pin_4, EXTI_Pin_5, EXTI_Pin_6, EXTI_Pin_7};

    if( irqHandler == NULL )
    {
        return;
    }

    GpioIrq[(obj->pinIndex % 8) & 0x0F] = irqHandler;

    if( irqMode == IRQ_RISING_EDGE )
    {
        trigger_type = EXTI_Trigger_Rising;
    }
    else if( irqMode == IRQ_FALLING_EDGE )
    {
        trigger_type = EXTI_Trigger_Falling;
    }
    else if( irqMode == IRQ_RISING_FALLING_EDGE )
    {
        trigger_type = EXTI_Trigger_Rising_Falling;
    }
    else
    {
        while( 1 );
    }

    if( irqPriority == IRQ_LOW_PRIORITY )
    {
        priority_level = ITC_PriorityLevel_1;
    }
    else if( irqPriority == IRQ_MEDIUM_PRIORITY )
    {
        priority_level = ITC_PriorityLevel_2;
    }
    else if( irqPriority == IRQ_HIGH_PRIORITY )
    {
        priority_level = ITC_PriorityLevel_3;
    }
    else
    {
        while( 1 );
    }
    GPIO_Init(obj->port, obj->pin, GPIO_Mode_In_FL_IT);
    // 中断方式
    EXTI_SetPinSensitivity(exti_pin[(obj->pinIndex % 8)], trigger_type);
    // 优先级
    ITC_SetSoftwarePriority(irq[(obj->pinIndex % 8)], priority_level);
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    GpioIrq[(obj->pinIndex % 8) & 0x0F] = NULL;
    
    GPIO_Init(obj->port, obj->pin, GPIO_Mode_In_FL_No_IT);
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
    if( ( obj == NULL ) || ( obj->port == NULL ) )
    {
        nop();
        while( 1 );
    }
    // Check if pin is not connected
    if( obj->pinIndex == NC )
    {
        return;
    }
    if( value == 0 )
    {
        GPIO_ResetBits( obj->port, obj->pin );
    }
    else
    {
        GPIO_SetBits( obj->port, obj->pin );
    }
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
    if( obj == NULL )
    {
        while( 1 );
    }
    // Check if pin is not connected
    if( obj->pinIndex == NC )
    {
        return 0;
    }
    return GPIO_ReadInputDataBit( obj->port, (GPIO_Pin_TypeDef)obj->pin );
}

/**
  * @brief External IT PIN0 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI0_IRQHandler,8)
{
    if( EXTI_GetITStatus( EXTI_IT_Pin0 ) != RESET )
    {
        if( GpioIrq[0] != NULL )
        {
            GpioIrq[0]( );
        }
        EXTI_ClearITPendingBit( EXTI_IT_Pin0 );
    }
}

/**
  * @brief External IT PIN1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI1_IRQHandler,9)
{
    if( EXTI_GetITStatus( EXTI_IT_Pin1 ) != RESET )
    {
        if( GpioIrq[1] != NULL )
        {
            GpioIrq[1]( );
        }
        EXTI_ClearITPendingBit( EXTI_IT_Pin1 );
    }
}

/**
  * @brief External IT PIN2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI2_IRQHandler,10)
{
    if( EXTI_GetITStatus( EXTI_IT_Pin2 ) != RESET )
    {
        if( GpioIrq[2] != NULL )
        {
            GpioIrq[2]( );
        }
        EXTI_ClearITPendingBit( EXTI_IT_Pin2 );
    }
}

/**
  * @brief External IT PIN3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI3_IRQHandler,11)
{
    if( EXTI_GetITStatus( EXTI_IT_Pin3 ) != RESET )
    {
        if( GpioIrq[3] != NULL )
        {
            GpioIrq[3]( );
        }
        EXTI_ClearITPendingBit( EXTI_IT_Pin3 );
    }
}

/**
  * @brief External IT PIN4 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI4_IRQHandler,12)
{
    if( EXTI_GetITStatus( EXTI_IT_Pin4 ) != RESET )
    {
        if( GpioIrq[4] != NULL )
        {
            GpioIrq[4]( );
        }
        EXTI_ClearITPendingBit( EXTI_IT_Pin4 );
    }
}

/**
  * @brief External IT PIN5 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI5_IRQHandler,13)
{
    if( EXTI_GetITStatus( EXTI_IT_Pin5 ) != RESET )
    {
        if( GpioIrq[5] != NULL )
        {
            GpioIrq[5]( );
        }
        EXTI_ClearITPendingBit( EXTI_IT_Pin5 );
    }
}
#if 0
/**
  * @brief External IT PIN6 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI6_IRQHandler,14)
{
    if( EXTI_GetITStatus( EXTI_IT_Pin6 ) != RESET )
    {
        if( GpioIrq[6] != NULL )
        {
            GpioIrq[6]( );
        }
        EXTI_ClearITPendingBit( EXTI_IT_Pin6 );
    }
}

/**
  * @brief External IT PIN7 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI7_IRQHandler,15)
{
    if( EXTI_GetITStatus( EXTI_IT_Pin7 ) != RESET )
    {
        if( GpioIrq[7] != NULL )
        {
            GpioIrq[7]( );
        }
        EXTI_ClearITPendingBit( EXTI_IT_Pin7 );
    }
}
#endif