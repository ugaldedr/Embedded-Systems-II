/*
 *  stepFive.h
 *
 *  Created on: Apr 17, 2019
 *  Author: Dario Ugalde
 */
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#ifndef STEPFIVE_H_
#define STEPFIVE_H_

void setZeroDMXData(uint8_t* Ddata)
{
    uint16_t x = 0;
    for(x= 0;x < 512;x++)
    {
        Ddata[x] = 0;
    }
}

#endif /* STEPFIVE_H_ */
