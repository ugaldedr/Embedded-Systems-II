/*
 *  stepFour.h
 *
 *  Created on: Apr 16, 2019
 *  Author: Dario Ugalde
 */
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#ifndef STEPFOUR_H_
#define STEPFOUR_H_

// Determine if the user input is a valid command
bool isCommand(char* strVerb, uint8_t minArgs)
{
    if(strcmp("device", strVerb) == 0  && minArgs == 0)
    {
        return true;
    }
    else if(strcmp("controller", strVerb) == 0  && minArgs == 0)
    {
        return true;
    }
    else if(strcmp("clear", strVerb) == 0  && minArgs == 0)
    {
        return true;
    }
    else if(strcmp("set", strVerb) == 0  && minArgs == 2)
    {
        return true;
    }
    else if(strcmp("get", strVerb) == 0  && minArgs == 1)
    {
        return true;
    }
    else if(strcmp("on", strVerb) == 0  && minArgs == 0)
    {
        return true;
    }
    else if(strcmp("off", strVerb) == 0  && minArgs == 0)
    {
        return true;
    }
    else if(strcmp("max", strVerb) == 0  && minArgs == 1)
    {
        return true;
    }
    else
    {
        putsUart0("\r\nUser input is not a valid command or was misunderstood.\r\n");
        return false;
    }
}

// Determine what address is being modified when "set" is called
uint16_t getValue(uint8_t argNo)
{
    return 0;
}

// Determine the data value that will replace the data at element determined in getValue
char* getString(uint8_t argNo)
{
    return 0;
}

#endif /* STEPFOUR_H_ */
