#ifndef __STM8L_EEPROM_H__
#define __STM8L_EEPROM_H__

#include "stm8l15x_flash.h"

#define SET_CONFIGURE_FLAG                          0xAB
#define EEPROM_FLAG_ADDR                            0x1000
#define EEPROM_START_ADDR                           0x1001

void earse_eeprom(uint16_t config_param_size);
uint8_t save_config_param(uint8_t *pdata, uint8_t length);
void get_eeprom_data(uint8_t *buffer, uint8_t size);

#endif