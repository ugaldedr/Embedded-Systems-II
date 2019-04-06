#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

#define RED_LED_MASK 2
#define MAX_CHARS 80

uint8_t dmxData [512];

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF;

    // Configure LED pin
    GPIO_PORTF_DIR_R = RED_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R = RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = RED_LED_MASK;  // enable LED

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

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

// String parser-lite function to find interesting positions within the input data
uint8_t* parseStr(char* strInput)
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

    return getData(strInput, count);
}

uint8_t* getData(char* strInput, uint8_t count)
{
    uint8_t pos[count]; // HAVING ISSUES HERE, C DOESN'T LIKE VARIABLE SIZE ARRAYS; MUST BE DETERMINED AT COMPILE TIME, CANNOT USE DYNAMIC MEMORY BECAUSE EMBEDDED SYSTEM
    uint8_t i = 0;
    if(isalpha(strInput[0]))
    {
        pos[0] = 0;
    }
    for(i = 0;i < strlen(strInput);i++) // Iterate over data and store locations of elements of interest
    {
        if((!(isalpha(strInput[i])) && !(isdigit(strInput[i]))) && (isalpha(strInput[i + 1])))
        {
            pos[i] = i;
        }
        if((!(isalpha(strInput[i])) && !(isdigit(strInput[i]))) && (isdigit(strInput[i + 1])))
        {
            pos[i] = i;
        }
    }
    return pos;
}
int main(void)
{
    // Initialize hardware
    initHw();

    RED_LED = 1;
    waitMicrosecond(500);
    RED_LED = 0;
    waitMicrosecond(500);

    char strInput [MAX_CHARS + 1];
    uint8_t* pos;
    while(1)
    {
        getsUart0(strInput,MAX_CHARS);
        putsUart0("\r\nCommand obtained:\r\n");
        putsUart0(strInput);
        putsUart0("\r\n");
        pos = parseStr(strInput);
        uint8_t i = 0;
        for(i = 0;i < strlen(pos);i++)
        {
            putsUart0(strInput[pos[i]]);
            putsUart0("\r\n");
        }
    }


	return 0;
}
