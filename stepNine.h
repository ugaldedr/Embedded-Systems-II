/*
 *  stepNine.h
 *
 *  Created on: Apr 25, 2019
 *  Author: Dario Ugalde
 */
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#ifndef STEPNINE_H_
#define STEPNINE_H_

uint32_t EEPROMread()
{
    uint32_t value = 0;

    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EEBLOCK_R = 0;
    EEPROM_EEOFFSET_R = 0;
    value = EEPROM_EERDWR_R;

    return value;
}

void EEPROMwrite(uint32_t value)
{
    EEPROM_EEBLOCK_R = 0;
    EEPROM_EEOFFSET_R = 0;
    EEPROM_EERDWR_R = value;
}



#endif /* STEPNINE_H_ */
