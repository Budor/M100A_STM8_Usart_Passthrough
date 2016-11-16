#include "stm8l_eeprom.h"

void earse_eeprom(uint16_t config_param_size)
{
    uint8_t i;
    
    FLASH_Unlock(FLASH_MemType_Data);
    /* Wait until Flash Program area unlocked flag is set*/
    while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET)
    {}
    
    FLASH_ProgramByte((uint32_t)EEPROM_FLAG_ADDR, 0);
    
    for(i = 0; i < config_param_size; i++)
    {
        FLASH_ProgramByte((uint32_t)EEPROM_START_ADDR + i, 0);
    }
    
    FLASH_Lock(FLASH_MemType_Data);
}

uint8_t save_config_param(uint8_t *pdata, uint8_t length)
{
    uint8_t i;
    
    FLASH_Unlock(FLASH_MemType_Data);
    /* Wait until Flash Program area unlocked flag is set*/
    while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET)
    {
    }
    
    FLASH_ProgramByte((uint32_t)EEPROM_FLAG_ADDR, SET_CONFIGURE_FLAG);
    
    for(i = 0; i < length; i++)
    {
        FLASH_ProgramByte((uint32_t)EEPROM_START_ADDR + i, pdata[i]);
    }
    
    for(i = 0; i < length; i++)
    {
        if( pdata[i] != FLASH_ReadByte((uint32_t)EEPROM_START_ADDR + i))
        {
            return 1;
        }
    }
    
    FLASH_Lock(FLASH_MemType_Data);
    
    return 0;
}

void get_eeprom_data(uint8_t *buffer, uint8_t size)
{
    uint8_t i = 0;
    
    FLASH_Unlock(FLASH_MemType_Data);
    /* Wait until Flash Program area unlocked flag is set*/
    while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET)
    {}
    
    for(i = 0; i < size; i++)
    {
        buffer[i] = FLASH_ReadByte((uint32_t)EEPROM_START_ADDR + i);
    }
    
    FLASH_Lock(FLASH_MemType_Data);
}