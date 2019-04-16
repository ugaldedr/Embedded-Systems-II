#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#include "stepOne.h"
#include "stepTwo.h"
#include "stepThree.h"
#include "stepFour.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

#define RED_LED_MASK 2
#define MAX_CHARS 80
#define NULL 0

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

char* getVerb(char* strInput, uint8_t* pos)
{
    char string[80];
    uint8_t c = pos[0];
    uint8_t count = 0;
    while(isalpha(strInput[c]))
    {
        string[count] = strInput[c];
        count++;
        c++;
    }
    string[count] = 0;
    return string;
}

int main(void)
{
    // Initialize hardware
    initHw();

    flashLED();

    char strInput [MAX_CHARS + 1];
    uint32_t* pos = NULL;
    char* strVerb = NULL;
    uint32_t fieldCount = 0;
    uint32_t minArgs = 0;
    while(1)
    {
        putsUart0("Enter a command:\r\n");
        getsUart0(strInput,MAX_CHARS);
        putsUart0("\r\nCommand obtained:\r\n");
        putsUart0(strInput);
        putsUart0("\r\n");
        pos = parseStr(strInput);
        fieldCount = getFieldCount(strInput);
        minArgs = fieldCount - 1;
        strVerb = getVerb(strInput, pos);
        putsUart0(strVerb);
        putsUart0("\r\n");
        if(isCommand(strVerb, minArgs)){
            putsUart0("This is a valid command.");
            putsUart0("\r\n");
        }

    }

	return 0;
}
