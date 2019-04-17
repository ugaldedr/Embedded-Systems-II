/*
 *  stepThree.h
 *
 *  Created on: Apr 15, 2019
 *  Author: Dario Ugalde
 */
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#ifndef STEPTHREE_H_
#define STEPTHREE_H_

// String parser-lite function to find interesting positions within the input data
uint32_t* parseStr(char* strInput)
{
    uint8_t i;
    static uint32_t pos[3];
    uint8_t count = 0;

    if(isalpha(strInput[0])) // Check edge case to see if first element is of interest
    {
        pos[0] = 0;
        count++;
    }
    for(i = 0;i < strlen(strInput);i++) // Iterate over input data and count up elements of interest
    {
        if((!(isalpha(strInput[i])) && !(isdigit(strInput[i]))) && (isalpha(strInput[i + 1])))
        {
            pos[count] = i + 1;
            count++;
        }
        if((!(isalpha(strInput[i])) && !(isdigit(strInput[i]))) && (isdigit(strInput[i + 1])))
        {
            pos[count] = i + 1;
            count++;
        }
    }

    return pos;
}

uint32_t getFieldCount(char* strInput)
{
    uint8_t i;
    uint8_t count = 0;

    if(isalpha(strInput[0])) // Check edge case to see if first element is of interest
    {
        count = count + 1;
    }
    for(i = 0;i < strlen(strInput);i++) // Iterate over input data and count up elements of interest
    {
        if((!(isalpha(strInput[i])) && !(isdigit(strInput[i]))) && (isalpha(strInput[i + 1])))
        {
            count = count + 1;
        }
        if((!(isalpha(strInput[i])) && !(isdigit(strInput[i]))) && (isdigit(strInput[i + 1])))
        {
            count = count + 1;
        }
    }
    return count;
}

#endif /* STEPTHREE_H_ */
