/*
 *  stepTwo.h
 *
 *  Created on: Apr 15, 2019
 *  Author: Dario Ugalde
 */
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define MAX_CHARS 80

#ifndef STEPTWO_H_
#define STEPTWO_H_

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

// Function to get instruction input from user
void getsUart0(char* str, uint8_t maxChars)
{
    uint8_t count = 0;
    while(count <= 80)
    {
        char c = getcUart0();
        if(c == 8) // Handling backspaces as input
        {
            if(count > 0)
            {
                count = count - 1;
            }
            else
            {
                continue;
            }
        }

        if(c == 13) // Handling carriage returns as input
        {
            str[count] = 0;
            break;
        }

        if(c >= 32) // Handling printable characters as input
        {
            str[count] = tolower(c);
            count = count + 1;
        }
        else
        {
            continue;
        }

        if(count == MAX_CHARS) // Terminate input after 80 characters of input
        {
            str[count] = 0;
            break;
        }
        else
        {
            continue;
        }
    }
}



#endif /* STEPTWO_H_ */
