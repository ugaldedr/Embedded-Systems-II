#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#include "stepOne.h"
#include "stepTwo.h"
#include "stepThree.h"
#include "stepFour.h"
#include "stepFive.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

#define RED_LED_MASK 2
#define MAX_CHARS 80
#define NULL 0

uint8_t dmxData [512];
uint8_t phase = 0;
uint16_t max = 512;

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOF;

    // Enable clocks
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer


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

    // Configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1
    GPIO_PORTB_DEN_R |= 3;                           // make PB 0 and 1 run digital function
    GPIO_PORTB_AFSEL_R |= 3;                         // setting AFSEL PB 0 and 1 high to use UART1
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB0_U1RX;

    // Configure UART1 to 115200 baud 8N1 format (must be 3 clocks from clock enable adn config writes)
    UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40MHz)
    UART1_IBRD_R = 5;                               // r = 40 MHz / (Nx250kHz), set floor(r)=5, where N=16
    UART1_FBRD_R = 0;                               // round(fract(r)*64)=0
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_STP2; // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure Timer 1
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x1B80;                         // set load value to 7040 to interrupt every 176us
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

}

// 176us timer interrupt used to handle step 6
void timer1ISR()
{
    if(phase == 0)
    {
        GPIO_PORTB_PCTL_R = GPIO_PCTL_PB1_U1TX;          // set TX pin to high
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_TAILR_R = 0x1B80;                         // set load value to 480 to interrupt every 12us
        phase = 1;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    }
    if(phase == 1)
    {
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        GPIO_PORTB_PCTL_R = GPIO_PCTL_PB0_U1RX;
        phase = 2;
    }
    if(phase == 2)
    {

    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

int main(void)
{
    // Initialize hardware
    initHw();

    flashLED();

    // Data variables
    char strInput [MAX_CHARS + 1];
    char* strVerb = NULL;
    char* value = NULL;
    uint16_t add = 0;
    uint16_t data = 0;
    uint32_t fieldCount = 0;
    uint32_t minArgs = 0;
    uint32_t* pos = NULL;

    setZeroDMXData(dmxData);
    while(1)
    {
        putsUart0("Enter a command:\r\n");
        getsUart0(strInput,MAX_CHARS);
        putsUart0("\r\nUser Input:\r\n");
        putsUart0(strInput);
        putsUart0("\r\n");
        pos = parseStr(strInput);
        fieldCount = getFieldCount(strInput);
        minArgs = fieldCount - 1;
        strVerb = getVerb(strInput, pos);
        if(minArgs >= 1)
        {
            add = getValue(strInput, pos + 1);
            if(add > 511)
            {
                putsUart0("Invalid DMX address.");
                putsUart0("\r\n");
                continue;
            }
        }
        if(minArgs == 2)
        {
            data = getValue(strInput, pos + 2);
            if(data > 511)
            {
                putsUart0("Invalid data.");
                putsUart0("\r\n");
                continue;
            }
        }
        if(isCommand(strVerb, minArgs))
        {
            if(!strcmp(strVerb,"device"))
            {

            }
            if(!strcmp(strVerb,"controller"))
            {

            }
            if(!strcmp(strVerb,"clear"))
            {
                setZeroDMXData(dmxData);
            }
            if(!strcmp(strVerb,"set"))
            {
                dmxData[add] = data;
            }
            if(!strcmp(strVerb,"get"))
            {
                char number[10];
                sprintf(number, "%d", dmxData[add]);
                putsUart0(number);
                putsUart0("\r\n");
                continue;
            }
            if(!strcmp(strVerb,"max"))
            {
                max = add;
            }
            if(!strcmp(strVerb,"on"))
            {

            }
            if(!strcmp(strVerb,"off"))
            {

            }
        }
        else
        {
            putsUart0("User input is not a valid command or was misunderstood.\r\n");
            continue;
        }
    }

	return 0;
}
