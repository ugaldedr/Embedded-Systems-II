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

uint16_t getValue(char* strInput, uint32_t* spot)
{
    char string[10];
    uint32_t loc = *(spot);
    uint8_t c = 0;
    while(strInput[loc] != ' ' && strInput[loc] != ',' && strInput[loc] != 0)
    {
        string[c] = strInput[loc];
        if(!isdigit(string[c]))
        {
            return 512;
        }
        c++;
        loc++;
    }
    string[c] = 0;
    loc = loc - c;
    return atoi(string);
}

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

// Determine the data value that will replace the data at element determined in getValue
char* getString(uint8_t argNo)
{
    return 0;
}

#endif /* STEPFOUR_H_ */
