#include <stdint.h>
#include <stdio.h>
#include <stdlib.h> // atoi
#include <string.h> // strlen, strcmp
#include <stdbool.h>
#include <ctype.h> // tolower, isdigit, isalpha
#include "tm4c123gh6pm.h"

#include "stepOne.h"
#include "stepTwo.h"
#include "stepThree.h"
#include "stepFour.h"
#include "stepNine.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

#define RED_LED_MASK 2
#define MAX_CHARS 80
#define NULL 0


// Global Variables
// ************************************
uint8_t dmxData [512];
uint8_t RXdmxData[512];
uint16_t phase = 0;
uint16_t RXphase = 0;
uint16_t max = 500;
uint32_t controllerMode = 0xFFFFFFFF;
// ************************************

void setZeroDMXData()
{
    uint16_t x = 0;
    for(x=0;x < 512;x++)
    {
        dmxData[x] = 255;
    }
}

uint16_t initEEPROM()
{
    SYSCTL_RCGCEEPROM_R |= SYSCTL_RCGCEEPROM_R0;     // turn-on EEPROM

    // Configure EEPROM
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    if(EEPROM_EESUPP_R & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY))
    {
        return 1;
    }
    SYSCTL_SREEPROM_R = SYSCTL_SREEPROM_R0;
    SYSCTL_SREEPROM_R &= ~SYSCTL_SREEPROM_R0;
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    if(EEPROM_EESUPP_R & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY))
    {
        return 1;
    }
    return 0;
}

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOF;

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer

    // EEPROM setup
    initEEPROM();
    controllerMode = EEPROMread();

    // DMX Data setup
    setZeroDMXData();

    // Configure LED pin
    GPIO_PORTF_DIR_R = RED_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R = RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = RED_LED_MASK;  // enable LED

    // Configure directions for PORTC pins
    GPIO_PORTC_DIR_R = 0x60; // C5,C6 are outputs, C4 is input
    GPIO_PORTC_DEN_R = 0x70; // C4-6 are all digital
    GPIO_PORTC_AFSEL_R = 0x00; // All GPIO by default

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

    if(controllerMode == 0)
    {
        // Configure Timer 1
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for one-shot mode (count down)
        TIMER1_TAILR_R = 0x1B80;                         // set load value to 7040 to interrupt every 176us
        TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
        NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

        // Configure UART1 pins
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
        GPIO_PORTC_AFSEL_R = 0x00;                       // still GPIO mode
        GPIO_PORTC_DATA_R = 0x60;                        // Pins C5,6

        // Configure UART1 to 250000 baud, 8N2 format (must be 3 clocks from clock enable and config writes)
        UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
        UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART1_IBRD_R = 10;                                // r = 40 MHz / (Nx250kkHz), set floor(r)=21, where N=16
        UART1_FBRD_R = 0;                                // round(fract(r)*0)=0
        UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2; // configure for 8N2
        UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN | UART_CTL_EOT;   // enable TX and module
        UART1_IM_R |= UART_IM_TXIM;                      // turn on tx interrupt
        NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 22(UART1)
    }
    else
    {
        // Configure UART1 pins
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
        GPIO_PORTC_AFSEL_R |= 0x10;                      // C4 is UART mode
        GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_U1RX;         // C4 is UART mode

        // Configure UART1 to 250000 baud, 8N2 format (must be 3 clocks from clock enable and config writes)
        UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
        UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART1_IBRD_R = 10;                                // r = 40 MHz / (Nx250kkHz), set floor(r)=21, where N=16
        UART1_FBRD_R = 0;                                // round(fract(r)*0)=0
        UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2 | UART_LCRH_FEN; // configure for 8N2
        UART1_IM_R = UART_IM_RXIM | UART_IM_BEIM;       // turn on rx and break interrupts
        UART1_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN;   // enable RX and module
        NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 22(UART1)
    }
}

// timer interrupt used to handle step 6
void timer1ISR()
{
    if(phase == 0)
    {
        //putsUart0("Phase 0\r\n");

        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt
        TIMER1_TAILR_R = 0x170;                          // set load value to 480 (1E0) to interrupt every 12us
        GPIO_PORTC_DATA_R = 0x60;                        // turn on pin C5 for 12us, keep C6 on
        phase = 1;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    }
    else if(phase == 1)
    {
        //putsUart0("Phase 1\r\n");

        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt
        TIMER1_TAILR_R = 0x1B80;                         // set load value 176us

        // Configure UART1 pins
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
        GPIO_PORTC_AFSEL_R = 0x20;                       // default, added for clarity
        GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX;

        // Configure UART1 to 250000 baud, 8N2 format (must be 3 clocks from clock enable and config writes)
        UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
        UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART1_IBRD_R = 10;                               // r = 40 MHz / (Nx250kkHz), set floor(r)=21, where N=16
        UART1_FBRD_R = 0;                               // round(fract(r)*0)=0
        UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2; // configure for 8N2
        UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN | UART_CTL_EOT;   // enable TX and module
        UART1_IM_R |= UART_IM_TXIM;                     // turn on tx interrupt
        NVIC_EN0_R |= 1 << (INT_UART1-16);              // turn-on interrupt 22(UART1)

        while (UART1_FR_R & UART_FR_TXFF);              // start code
            UART1_DR_R = 0;

        phase = 2;
    }
}

void Uart1Isr()
{
    //putsUart0("Entering UART1ISR\r\n");

    //while (UART1_FR_R & UART_FR_TXFF);
    //    UART1_DR_R = 0xFF;

    if(UART1_RIS_R & UART_RIS_TXRIS)
    {
        char number[10];
        sprintf(number, "%d", phase);
        //putsUart0(number);
        //putsUart0("\r\n");

        UART1_ICR_R = UART_ICR_TXIC;                           // clear the UART1 transmit interrupt
        if(phase - 2 < max)
        {
            while (UART1_FR_R & UART_FR_TXFF);
                UART1_DR_R = dmxData[phase - 2];               // transmit dmxData along UART1
                phase++;                                       // increment phase to transmit next value
        }
        else
        {
            while (UART1_FR_R & UART_FR_TXFF);
            NVIC_EN0_R &= ~(1 << (INT_UART1-16));              // turn-off interrupt 22(UART1)
            phase = 0;                                         // reset the phase variable for next transmission
            GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC5_U1TX;          // turn-off U1TX pin
            UART1_CTL_R = 0;                                   // turn-off UART1
            GPIO_PORTC_AFSEL_R &= ~0x20;                       // turn-off AFSEL for UART1
            GPIO_PORTC_DATA_R = 0x40;                          // TX pin is low for 176us, keep C6 on
            SYSCTL_RCGCUART_R &= ~SYSCTL_RCGCUART_R1;          // disable clocks to UART1
            TIMER1_CTL_R |= TIMER_CTL_TAEN;                    // start timer1 for next transmission
        }
    }
    else if(UART1_RIS_R & (UART_RIS_RXRIS | UART_RIS_BERIS))
    {
        char number[10];
        sprintf(number, "%d", RXphase);
        //putsUart0(number);
        //putsUart0("\r\n");

        uint8_t data = UART1_DR_R;
        if(UART1_DR_R & UART_DR_BE)
            RXphase = 1;
        else
        {
            switch(RXphase)
            {
                case 0: break;
                case 1: if(data == 0)
                            RXphase = 2;
                        break;

            default: dmxData[RXphase - 2] = data;
                     RXphase++;
            }
        }
        UART1_ICR_R = UART_ICR_RXIC | UART_ICR_BEIC;
    }
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

    if(controllerMode == 0)
        putsUart0("Currently set to controller mode.\r\n");
    else
        putsUart0("Currently set to device mode.\r\n");

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
                EEPROMwrite(0xFFFFFFFF);
                putsUart0("Setting board as device.\r\n");
            }
            if(!strcmp(strVerb,"controller"))
            {
                EEPROMwrite(0x0);
                putsUart0("Setting board as controller.\r\n");
            }
            if(!strcmp(strVerb,"clear"))
            {
                setZeroDMXData();
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
