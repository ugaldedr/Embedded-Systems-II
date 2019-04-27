#include <stdint.h>
#include <string.h> // strcmp
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define MAX_CHARS 80


// Global Variables
// ************************************
uint8_t dmxData[512];
char strInput[MAX_CHARS + 1];
uint8_t argIndex[3];
uint8_t fieldCount = 0;
char strVerb[25];
uint16_t timeout = 4;
uint16_t address = 1;
uint16_t add = 0;
uint16_t data = 0;
uint16_t max = 512;
uint16_t phase = 0;
uint16_t RXphase = 0;
uint32_t deviceMode = 0xFFFFFFFF;
// ************************************

void setZeroDMXData()
{
    uint32_t x = 0;
    for(x=0;x < 512;x++)
    {
        dmxData[x] = 0;
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

// Function to read EEPROM
uint32_t EEPROMread()
{
    uint32_t value = 0;

    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EEBLOCK_R = 0;
    EEPROM_EEOFFSET_R = 0;
    value = EEPROM_EERDWR_R;

    return value;
}

// Function to write to EEPROM
void EEPROMwrite(uint32_t value)
{
    EEPROM_EEBLOCK_R = 0;
    EEPROM_EEOFFSET_R = 0;
    EEPROM_EERDWR_R = value;
}

uint32_t EEPROMreadAddr()
{
    uint32_t value = 0;

    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EEBLOCK_R = 0;
    EEPROM_EEOFFSET_R = 1;
    value = EEPROM_EERDWR_R;

    return value;
}

// Function to write to EEPROM
void EEPROMwriteAddr(uint32_t value)
{
    EEPROM_EEBLOCK_R = 0;
    EEPROM_EEOFFSET_R = 1;
    EEPROM_EERDWR_R = value;
}

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // DMX Data setup
    setZeroDMXData();

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOF;

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2;       // turn-on timer

    // EEPROM setup
    initEEPROM();
    deviceMode = EEPROMread();

    // Configure LED pin
    GPIO_PORTF_DIR_R = RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R = RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;  // enable LED

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

    if(deviceMode == 0)
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
        RED_LED = 1;                                     // turn on TX LED

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
    else // device mode
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
        UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2; // configure for 8N2
        UART1_IM_R = UART_IM_RXIM | UART_IM_BEIM;       // turn on rx and break interrupts
        UART1_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN;   // enable RX and module
        NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 22(UART1)

        // Configure Timer 1
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for one-shot mode (count down)
        TIMER2_TAILR_R = 0x1312D00;                      // set load value to 0X1312D00 to interrupt every 500ms (.5 sec)
        TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
        NVIC_EN0_R |= 1 << (INT_TIMER2A-16);             // turn-on interrupt 38 (TIMER2A)
        TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
        GREEN_LED = 1;
    }
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

// Flash LED for 500ms
void flashLED()
{
    RED_LED = 1;
    waitMicrosecond(500);
    RED_LED = 0;
    waitMicrosecond(500);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
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

// Function to get instruction input from user
void getsUart0()
{
    uint8_t count = 0;
    while(count <= 80)
    {
        char c = getcUart0();

        if(c == 8) // Handling for backspaces
        {
            if(count > 0)
            {
                count--;
            }
        }
        if(c >= 32) // Handling for printable characters
        {
            if((c >='A' && c <='Z') || (c >='a' && c <='z')) // Handling for letters
                if(c >='A' && c <='Z') // Handling for uppercase letters
                   strInput[count] = c + 32;
                else // Handling for lowercase letters
                   strInput[count] = c;
            else if(c == 32) // Handling for spaces
                strInput[count] = 32;
            else if(c >= '0' && c <= '9') // Handling for numbers
                strInput[count] = c;
            else // Handling for non-alphanumberic characters
                strInput[count] = 32;

            count++;
        }

        putcUart0(c); // Display input back to user

        if(count == MAX_CHARS || c == 13) // Terminate input after 80 characters of input or carriage return received
        {
            strInput[count] = 0;
            break;
        }
    }
}

// Function to determine if a char is number
uint8_t is_digit(char c)
{
    if(c >= '0' && c <= '9')
        return c;
    return 0;
}

// Function to determine if a char is a letter
uint8_t is_alpha(char c)
{
    if((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
    {
        if(c >= 'A' && c <= 'Z')
            return c + 32;
        else
            return c;
    }
    return 0;
}

// Parser lite function to store user input in buffer
void parseStr()
{
    uint8_t count = 1;
    if(is_alpha(strInput[0])) // base case if command is first input
    {
        argIndex[0] = 0;
        fieldCount++;
    }
    while(strInput[count] != 0)
    {
        if(is_alpha(strInput[count]) && (strInput[count - 1] == ' ')) // look for user commands
        {
            argIndex[fieldCount] = count;
            fieldCount++;
        }
        if(is_digit(strInput[count]) && (strInput[count - 1] == ' ')) // look for user arguments
        {
            argIndex[fieldCount] = count;
            fieldCount++;
        }
        count++;
    }
}

// Function to extract user command
void getVerb()
{
    uint8_t i = argIndex[0];
    uint8_t count = 0;
    while(is_alpha(strInput[i]))
    {
        strVerb[count] = strInput[i];
        count++;
        i++;
    }
    strVerb[count] = 0;
}

// Determine if the user input is a valid command
bool isCommand(uint8_t minArgs)
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
    else if(strcmp("address", strVerb) == 0  && minArgs == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Function to convert string numbers into uint16_t ints
uint16_t ATOI(char* num)
{
    uint8_t i = 0;
    uint16_t result = 0;
    while(num[i] != 0)
    {
        result = result * 10 + (num[i] -'0');
        i++;
    }
    if(result > 511)
        return 511;
    else
        return result;

}

char* ITOA (uint16_t value, char *result)
{
    char* ptr = result, *ptr1 = result, tmp_char;
    uint16_t tmp_value;

    do {
        tmp_value = value;
        value /= 10;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * 10)];
    } while ( value );

    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

// Function to return the value of a user argument
uint16_t getValue(uint8_t index)
{
    char string[10];
    uint8_t c = 0;
    while(strInput[index] != ' ' && strInput[index] != 0)
    {
        string[c] = strInput[index];
        if(!is_digit(string[c]))
            return 512;
        c++;
        index++;
    }
    string[c] = 0;
    return ATOI(string);
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
        RED_LED = 1;                                     // TX LED on
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

void timer2ISR()
{
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt
    if(timeout>0)
    {
        timeout--;
    }else
    {
        GREEN_LED = !GREEN_LED;
    }
}
void Uart1Isr()
{
    //putsUart0("Entering UART1ISR\r\n");

    //while (UART1_FR_R & UART_FR_TXFF);
    //    UART1_DR_R = 0xFF;

    if(UART1_RIS_R & UART_RIS_TXRIS)
    {
        //char number[10];
        //ITOA(number, phase);
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
            RED_LED = 0;                                       // TX LED is OFF
            SYSCTL_RCGCUART_R &= ~SYSCTL_RCGCUART_R1;          // disable clocks to UART1
            TIMER1_CTL_R |= TIMER_CTL_TAEN;                    // start timer1 for next transmission
        }
    }
    else if(UART1_RIS_R & (UART_RIS_RXRIS | UART_RIS_BERIS))
    {

        uint8_t rxdata = UART1_DR_R;
        if(UART1_DR_R & UART_DR_BE)
        {
            RXphase = 1;
            GREEN_LED = 1;
            timeout = 4;
            if(dmxData[EEPROMreadAddr()] == 0)
                BLUE_LED = 0;
            else
                BLUE_LED = 1;
        }
        else
        {
            switch(RXphase)
            {
                case 0: break;
                case 1: if(rxdata == 0)
                            RXphase = 2;
                        break;

            default: dmxData[RXphase - 2] = rxdata;
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

    // Flash LED to check successful initialization
    flashLED();

    if(deviceMode)
    {
        putsUart0("Initialized in device mode.\r\nAddress: ");
        char number[10];
        ITOA(EEPROMreadAddr(), number);
        putsUart0(number);
        putsUart0("\r\n");
    }
    else
        putsUart0("Initialized in controller mode.\r\n");

    while(1)
    {
        // Obtaining command from user
        putsUart0("\r\nEnter a command:\r\n");
        getsUart0();

        GREEN_LED = !GREEN_LED;
        waitMicrosecond(5000);  // Green LED on received CR

        putsUart0("\r\n");

        // Extracting data from input
        fieldCount = 0;
        parseStr();

        // Make sure user input is of valid field count
        if(fieldCount > 3)
        {
           putsUart0("\r\nToo many fields for valid command.\r\n");
           putsUart0("Please try again.\r\n");
           continue;
        }

        // Store the user's command
        getVerb();
        if(fieldCount >= 2)
            add = getValue(argIndex[1]);
        if(fieldCount == 3)
            data = getValue(argIndex[2]);

        GREEN_LED = !GREEN_LED;
        if(data > 255 || add > 511)
        {
            putsUart0("Invalid arguments.\r\n");
            putsUart0("Please try again.\r\n");
            continue;
        }

        // Make sure the user input is a valid command and execute command
        if(isCommand(fieldCount - 1))
        {
            if(!strcmp(strVerb,"device"))
            {
                EEPROMwrite(0xFFFFFFFF);
                putsUart0("Setting board as device.\r\n");
            }
            else if(!strcmp(strVerb,"controller"))
            {
                EEPROMwrite(0x0);
                putsUart0("Setting board as controller.\r\n");
            }
            else if(!strcmp(strVerb,"clear"))
            {
                setZeroDMXData();
            }
            else if(!strcmp(strVerb,"set"))
            {
                dmxData[add] = data;
            }
            else if(!strcmp(strVerb,"get"))
            {
                char number[10];
                ITOA(dmxData[add], number);
                putsUart0(number);
                putsUart0("\r\n");
            }
            else if(!strcmp(strVerb,"max"))
            {
                max = add;
            }
            else if(!strcmp(strVerb,"address"))
            {
                char number[10];
                EEPROMwriteAddr(add);
                putsUart0("New address saved to EEPROM: ");
                ITOA(EEPROMreadAddr(), number);
                putsUart0(number);
                putsUart0("\r\n");
            }
            else if(!strcmp(strVerb,"on"))
            {
                if(deviceMode == 0)
                {
                    GPIO_PORTC_DIR_R = 0x60;                    // turn-on AFSEL for UART1
                    GPIO_PORTF_DEN_R = RED_LED_MASK;            // enable LED pin
                }
            }
            else if(!strcmp(strVerb,"off"))
            {
                if(deviceMode == 0)
                {
                    GPIO_PORTC_DIR_R = 0x00;            // turn-on AFSEL for UART1
                    GPIO_PORTF_DEN_R = 0x00;            // disable LED pin
                }
            }
        }
        else
        {
            putsUart0("Not a valid command.\r\n");
            putsUart0("Please try again.\r\n");
            continue;
        }
    }
    return 0;
}
