// Serial Example
// Sowmya Srinivasa

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "Project_SowmyaSrinivasa.h"

// Bitband aliases
#define RX_GREEN_LED      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define TX_RED_LED        (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 2*4)))
#define rs485_DEN      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))


// Bitband aliases for input devices
#define PUSH_BUTTON    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

// Bitband aliases for output devices

#define RED_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))



//UART1 masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1
#define GPIO_DEN   16

// PortE masks
#define GREEN_LED_MASK 2
#define RED_LED_MASK 4
#define PUSH_BUTTON_MASK 16
#define AIN3_MASK 1

#define DELIMETER 'd'
#define ALPHA     'a'
#define NUMBER    'n'
#define MAX_CHARS 80
#define MAX_FIELDS 5
#define MAX_DATAPACKETS 10
#define MAX_COUNT 5
bool CS, RANDOM, ack = 0, busy, testCS, testDone;
int8_t msgInProgress = -1;
uint8_t msgPhase = 0, rxPhase = 0, myAddress = 1, txCount, txLEDtimeout = 0, rxLEDtimeout = 0;
uint32_t CurrentRandomNumber = 0, minBackoffTime = 10, timerValue = 10;

// UI message variables
#define UI_BUFFER_LENGTH 30
char UImessage[UI_BUFFER_LENGTH];
uint8_t rxIndex = 0;
uint8_t txIndex = 0;

////Channel Array
//#define MAX_CHANNEL 3
//volatile uint32_t* ACT_Channel[MAX_CHANNEL] = { &RED_LED, &GREEN_LED, &PUSH_BUTTON };

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

typedef struct _TX485MSG
{
    uint8_t dstAddress;
    uint8_t srcAddress;
    uint8_t cmd;
    uint8_t size;
    uint8_t data1[10];
    uint8_t channel;
    uint8_t seqID;
    uint8_t checkSum;
    bool valid;
    uint8_t timeToTransmit;
    uint8_t transmitCount;
} TX485MSG;

TX485MSG message[MAX_DATAPACKETS];

typedef struct _RX485MSG
{
    uint8_t dstAddress;
    uint8_t srcAddress;
    uint8_t cmd;
    uint8_t size;
    uint8_t data1[10];
    uint8_t channel;
    uint8_t seqID;
    uint8_t checkSum;
} RX485MSG;

RX485MSG rxMsg;

typedef struct _action
{
    uint8_t command;
    uint8_t time1;
    uint8_t level1;
    uint8_t time2;
    uint8_t level2;
    uint8_t count;
    bool valid;
    uint8_t phase;
    uint8_t deltaT;
} action;

action act;

//COMMANDS
#define CMD_SET 0x00
#define CMD_SQUARE 0x03
#define CMD_PULSE 0x02

#define CMD_DATAREQUEST 0x30
#define CMD_DATAREPORT 0x31
#define CMD_ACKNOWLEDGE 0x70
#define CMD_POLLREQUEST 0x78
#define CMD_POLLRESPONSE 0x79
#define CMD_SETADDRESS 0x7A
#define CMD_RESET 0x7F

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5; //PORTE and PORTF
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0; // Timer0
    SYSCTL_RCGCEEPROM_R = SYSCTL_RCGCEEPROM_R0; // EEPROM
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;    // ADC
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTE_DIR_R  |=  GREEN_LED_MASK | RED_LED_MASK; //
    GPIO_PORTF_DIR_R  &= ~PUSH_BUTTON_MASK;
    GPIO_PORTE_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R  |=  GREEN_LED_MASK | RED_LED_MASK ;
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK;
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;
    //GPIO_PORTB_DEN_R |= GPIO_DEN;

    // Timer
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER0_TAILR_R = 400000;                         // set load value to 400000 for 1 Hz interrupt rate
    TIMER0_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R |= 1 << (INT_TIMER0A-16);             // turn-on interrupt 35 (TIMER0A) in NVIC


    //EEPROM configuration


    //EEPROM_EESIZE_R = 0x10000;
    EEPROM_EEBLOCK_R = 0;
    EEPROM_EEOFFSET_R = 0;

    //ADC Configuration

    // Configure AIN3 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN3_MASK;                 // select alternative functions for AN3 (PE0)
    GPIO_PORTE_DEN_R &= ~AIN3_MASK;                  // turn off digital operation on pin PE0
    GPIO_PORTE_AMSEL_R |= AIN3_MASK;                 // turn on analog operation on pin PE0

}

// initialize Uart1
void initUart1()
{

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks

    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    //Configure UART1 pins
    GPIO_PORTB_DIR_R |= UART_TX_MASK | GPIO_DEN;           // enable output on UART1 TX pin
    GPIO_PORTB_DIR_R &= ~(UART_RX_MASK);           // enable input on UART1 RX pin
    GPIO_PORTB_DR2R_R |= UART_TX_MASK | GPIO_DEN; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= UART_TX_MASK | UART_RX_MASK ;//| GPIO_DEN;    // enable digital on UART1 pins
    GPIO_PORTB_AFSEL_R |= UART_TX_MASK | UART_RX_MASK; // use peripheral to drive PB0, PB1
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M | GPIO_PCTL_PB0_M); // clear bits 0-7
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX;
    // select UART1 to drive pins PB0 and PB1: default, added for clarity



    //Configure UART1 to 38400 baud
    UART1_CTL_R = 0;                 // turn-off UART to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART1_IBRD_R = 65; // r = 40 MHz / (Nx138400Hz), set floor(r)=65, where N=16
    UART1_FBRD_R = 07;                                  // round(fract(r)*64)=07
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_SPS | UART_LCRH_PEN;
    UART1_IM_R |= UART_IM_TXIM | UART_IM_RXIM;                        // enabling interrupt

    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_EOT ;//| UART_CTL_LBE;
    // enable TX, RX, and module

    NVIC_EN0_R |= 1 << (INT_UART1 - 16);
}

void initAdc0Ss3()
{
    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    _delay_cycles(16);

    // Configure ADC
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSCTL3_R = ADC_SSCTL3_END0 | ADC_SSCTL3_TS0;// mark first sample as the end, read temperature at the first sample
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

void initMsg()
{
    uint8_t i;
    for (i = 0; i <= MAX_DATAPACKETS; i++)
    {
        message[i].valid = false;
        message[i].timeToTransmit = 0;
    }
}

// Set SS3 analog input
void setAdc0Ss3Mux(uint8_t input)
{
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_SSMUX3_R = input;                           // Set analog input for single sample
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

// Request and read one sample from SS3
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    while (ADC0_SSFSTAT3_R & ADC_SSFSTAT3_EMPTY);
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

int readEEPROM()
{
    if(EEPROM_EERDWR_R == 0xFFFFFFFF)
    {
        myAddress = 1;
    }
    else
    {
        myAddress = EEPROM_EERDWR_R;                           // source address from EEPROM
    }
    return myAddress;
}

void writeEEPROM(uint8_t newaddress)
{
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EEOFFSET_R = 0;
    EEPROM_EERDWR_R = rxMsg.data1[0];
}

void getsUart0(USER_DATA *data)
{
    int count = 0;
    char ch;

    while (count <= (MAX_CHARS))
    {
        ch = getcUart0();
        if (ch == 8 || ch == 127)
        {
            if (count > 0)
            {
                count--;

            }
            continue;
        }

        if (ch == 13)
        {
            data->buffer[count] = '\0';
            break;                       // exit when enter key is pressed
        }

        data->buffer[count] = ch;       // Update buffer array
        count++;

        if (count == MAX_CHARS)
        {
            data->buffer[count + 1] = '\0';
            break;                        // exit when max chars reached
        }
    }
}

void parseFields(USER_DATA *data)
{
    char prev_type, current_type;
    prev_type = DELIMETER;
    data->fieldCount = 0;
    int i = 0;

    while (data->buffer[i] != '\0')
    {

        if ((data->buffer[i] >= 97 && data->buffer[i] <= 122)
                || (data->buffer[i] >= 65 && data->buffer[i] <= 90))
        {
            current_type = ALPHA; // Setting fieldType = 'a';
        }

        else if ((data->buffer[i] >= 48 && data->buffer[i] <= 57)
                || data->buffer[i] == 45 || data->buffer[i] == 46)
        {
            current_type = NUMBER; // Setting fieldType = 'n';
        }
        else
        {
            current_type = DELIMETER;
            data->buffer[i] = '\0';
        }

        if (prev_type == DELIMETER
                && (current_type == ALPHA || current_type == NUMBER))
        {
            data->fieldType[data->fieldCount] = current_type;
            data->fieldPosition[data->fieldCount] = i;
            data->fieldCount++;
        }

        prev_type = current_type;
        i++;

        if (data->fieldCount == MAX_FIELDS)
        {
            break;
        }
    }
}

char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{

    if (fieldNumber <= data->fieldCount)
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
        return '\0';
    }
}

int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    if ((fieldNumber <= data->fieldCount)
            && (data->fieldType[fieldNumber] == 'n'))
    {
        uint8_t temp, i = 0;
        uint32_t val = 0;

        char *str = &data->buffer[data->fieldPosition[fieldNumber]];
        while (str[i] != '\0')
        {
            temp = str[i] - 48;
            val = (val * 10) + temp;
            i++;
        }
        return val;
    }
    return 0;
}

bool strCompare(char *str1, const char *str2)
{
    int i = 0;
    while (str1[i] == str2[i])
    {
        if (str1[i] == '\0' || str2[i] == '\0')
        {
            break;
        }
        i++;
    }
    if (str1[i] == '\0' && str2[i] == '\0')
    {
        return true;
    }
    return false;
}

bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    if (strCompare((&data->buffer[data->fieldPosition[0]]), strCommand))
    {
        if (minArguments < data->fieldCount)
            return true;
    }
    return false;
}

void itostring(int n, char* s)
{
    int j;
    //static char s[20];
    char tempChar;
    j = 0;

    while(n)
    {
        s[j++] = n % 10 + '0';
        n /= 10;
    }
    s[j] = '\0';
    int i=0;
    j--;
    while(i<j)
    {
       tempChar = s[j];
       s[j] = s[i];
       s[i] = tempChar;

       i++;
       j--;
    }
}

void timer0Isr()
{
    uint8_t i;
    if(testDone > 0)
    {
        testCS--;
        if(testCS == 0)
        {
            testDone = true;
        }
    }
    for(i = 0; i <= MAX_DATAPACKETS; i++)
    {
        if(message[i].valid == true)
        {
            if(message[i].timeToTransmit > 0)
            {
                message[i].timeToTransmit--;

                if(message[i].timeToTransmit == 0)
                {
                    if(UART1_FR_R & UART_FR_TXFE)
                    {
                        if((UART1_RIS_R & UART_RIS_TXRIS) == 0x00)
                        {
                            send485Byte();
                            break;
                        }
                    }
                }
            }
        }
    }

    if(txLEDtimeout > 0)// change
    {
        TX_RED_LED = 1;
        txLEDtimeout--;
    }
    else if(txLEDtimeout == 0)
    {
        TX_RED_LED = 0;
    }

    if(rxLEDtimeout > 0)// change
    {
        RX_GREEN_LED = 1;
        rxLEDtimeout--;
    }
    else if(txLEDtimeout == 0)
    {
        RX_GREEN_LED = 0;
    }

    for(i = 0; i<=10; i++)
    {
        act.deltaT++;
        if(act.phase == 0)
        {
            if(act.deltaT == act.time1) // change
            {
                act.phase++;

                if((rxMsg.cmd & 0x7F) == CMD_PULSE)
                {
                    //
                    act.valid = false;
                }
                if((rxMsg.cmd & 0x7F) == CMD_SQUARE)
                {

                }
            }
        }
        else if(act.phase == 1)
        {
            if(act.deltaT == act.time2)
            {
//                //
                act.deltaT = 0;
                act.phase = 0;
            }
        }
    }

    TIMER0_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

bool getParity(uint32_t n)
{

        bool parity = 0;

        while (n)
        {

            parity = !parity;
            n      = n & (n - 1);
        }
        return parity;
}

uint8_t generateSeed()
{
    uint16_t temp = readAdc0Ss3();
    temp ^= 0x5555;
    myAddress = readEEPROM();
    temp += myAddress;
    uint8_t seed = temp & 0xFF;
    return seed;
}

uint8_t getNextRandomValue(uint8_t prevRandom)
{
    uint8_t temp = prevRandom & 0xAA;
    bool parity = getParity(temp);
    prevRandom = (prevRandom>>1);
    uint8_t newRandom = prevRandom | (parity<<7);
    return newRandom;
}

uint8_t powerOfTwo(uint8_t exponent)
{
    uint8_t result = 1;
    while(exponent != 0)
    {
        result *= 2;
        exponent--;
    }
    return result;
}

int CalculateRetransmitTime()
{
    int timeToTx;
    if(RANDOM == false)
    {
        timeToTx = minBackoffTime + (powerOfTwo(txCount)*timerValue);
    }
    else
    {
        // find out next value in the sequence
        CurrentRandomNumber = getNextRandomValue(CurrentRandomNumber);
        while(CurrentRandomNumber > 0 && CurrentRandomNumber >(powerOfTwo(txCount)*timerValue))
        {
            CurrentRandomNumber = getNextRandomValue(CurrentRandomNumber);
        }
        timeToTx = minBackoffTime + CurrentRandomNumber;
    }
    return timeToTx;
}



void send485Byte()
{
    uint8_t QueueByte;
    static uint8_t i = 0, j = 0;

    if(msgInProgress == -1)
    {
        GPIO_PORTB_DEN_R |= GPIO_DEN;
        rs485_DEN = 1;
        if((message[i].valid == true) && (message[i].timeToTransmit == 0))
        {
            msgPhase = 0;
            msgInProgress = i;
        }
    }

    if((msgInProgress > -1) && (msgInProgress < MAX_DATAPACKETS))
    {

        if(msgPhase == 0)
        {
            if(CS)
            {
                if(testDone)
                {
                   testDone = false;
                    if(!busy)
                    {
                        QueueByte = message[msgInProgress].dstAddress;
                        msgPhase++;                    }
                }
                else
               {
                    busy = false;
                   testDone = false;
                   testCS = 2;
                }
           }
            QueueByte = message[msgInProgress].dstAddress;
            msgPhase++;
        }
        else if(msgPhase == 1)
        {
            QueueByte = message[msgInProgress].srcAddress;
            msgPhase++;
        }
        else if(msgPhase == 2)
        {
            QueueByte = message[msgInProgress].seqID;
            msgPhase++;
        }
        else if(msgPhase == 3)
        {
            QueueByte = message[msgInProgress].cmd;
            msgPhase++;
        }
        else if(msgPhase == 4)
        {
            QueueByte = message[msgInProgress].channel;
            msgPhase++;
        }
        else if(msgPhase == 5)
        {
            QueueByte = message[msgInProgress].size;
            msgPhase++;
        }
        else if(msgPhase == 6)
        {
            if(j<message[msgInProgress].size)
            {
                QueueByte = message[msgInProgress].data1[j++];
            }
            else
            {
                j = 0;
                msgPhase++;
            }
        }
        if(msgPhase == 7)
        {
            QueueByte = message[msgInProgress].checkSum;
            msgPhase++;
        }


        UART1_DR_R = QueueByte;
        //msgPhase++;

        if(msgPhase == 8)
        {
            msgPhase = 0;
            //msgInProgress = -1;// change
            i = (i+1)%(MAX_DATAPACKETS);
            TX_RED_LED = 1;
            txLEDtimeout = 100;
            if((message[msgInProgress].cmd & 0x80) == 0x00)
            {
                message[msgInProgress].valid = false;
                msgInProgress = -1;
            }
            else
            {
                txCount++;
                if(txCount == MAX_COUNT)
                {
                    message[msgInProgress].valid = false;
                    TX_RED_LED = 1;
                    sendUImsg("msg SeqID Failed\0");
                }
                else
                {
                    message[msgInProgress].timeToTransmit = CalculateRetransmitTime();
                    msgInProgress = -1;
                }
            }
        }
    }
    else
    {
        waitMicrosecond(1000000);
        GPIO_PORTB_DEN_R &= ~GPIO_DEN;
        rs485_DEN = 0;
    }
}


void sendRS485(uint8_t dstAddr, uint8_t cmd, uint8_t channel, uint8_t size,
               uint8_t data1[], bool ack)
{
    static uint8_t seqID = 0;
    uint8_t i, j;
    uint32_t checkSum = 0;

    for (j = 0; j <= MAX_DATAPACKETS; j++)
    {
        if (message[j].valid == false)
        {
            message[j].dstAddress = dstAddr;
            message[j].srcAddress = readEEPROM();
            message[j].channel = channel;
            message[j].size = size;
            if(ack == 0)
            {
                message[j].cmd = cmd;
            }
            else
            {
                message[j].cmd = cmd & (ack << 7);
            }
            message[j].seqID = seqID;
            checkSum = message[j].dstAddress+message[j].srcAddress+message[j].channel+message[j].size+message[j].cmd+message[j].seqID;

            for (i = 0; i < size; i++)
            {
                message[j].data1[i] = data1[i];
                checkSum  += message[j].data1[i];
            }
            seqID++;
            message[j].checkSum = ~(checkSum);
            message[j].transmitCount = 0;

            //marking msg as valid
            message[j].valid = true;

            while (!(UART1_FR_R & UART_FR_TXFE));
            send485Byte();
            break;
        }
    }
}

void uart0ISR()
{

    if(UART0_FR_R & UART_FR_TXFE)
    {
        if(txIndex != rxIndex)
        {
            UART0_DR_R = UImessage[rxIndex];
            rxIndex = (rxIndex + 1) % UI_BUFFER_LENGTH;
        }
        //UART0_ICR_R |= UART_ICR_TXIC ;
        else
        {
            UART0_IM_R &= ~UART_IM_TXIM;
            UART0_ICR_R |= UART_ICR_TXIC ;//| UART_ICR_RXIC;
        }
    }
}

void sendUImsg()
{
    if(UART0_FR_R & UART_FR_TXFE)              // wait if uart0 tx fifo full
    {
        UART0_DR_R = UImessage[rxIndex];
        rxIndex = (rxIndex + 1) % UI_BUFFER_LENGTH;
        UART0_IM_R |= UART_IM_TXIM;
    }
}


//void sendUImsg(char* str)
void appendUImsg(char* str)
{
    uint8_t i = 0;
    UART0_IM_R &= ~UART_IM_TXIM;
    bool full = ((txIndex+1) % UI_BUFFER_LENGTH) == rxIndex;
    while(str[i] != '\0' && (!full))
    {
        UImessage[txIndex] = str[i++];
        txIndex = (txIndex + 1) % UI_BUFFER_LENGTH;
        full = ((txIndex+1) % UI_BUFFER_LENGTH) == rxIndex;
    }

//    if(UART0_FR_R & UART_FR_TXFE)              // wait if uart0 tx fifo full
//    {
//        UART0_DR_R = UImessage[rxIndex];
//        rxIndex = (rxIndex + 1) % UI_BUFFER_LENGTH;
//        UART0_IM_R |= UART_IM_TXIM;
//
//    }
}

// uncomment later when you can test the code

//volatile uint32_t* getChannel(uint8_t act_channel)
//{
//    uint8_t i;
//    volatile uint32_t* channel;
//
//    for(i=0;i<=MAX_CHANNEL;i++)
//    {
//        if(act_channel == i)
//        {
//            channel = ACT_Channel[i];
//        }
//    }
//    return channel;
//}

void process_rxData(RX485MSG *rxMsg)
{
    uint8_t cmd;
    if(rxMsg->cmd & 0x80)
    {
        bool ack = 0;
        cmd = CMD_ACKNOWLEDGE;
        uint8_t data = rxMsg->seqID;
        sendRS485(rxMsg->dstAddress, cmd, 0, 0, &data, ack);
    }
    else if(CMD_ACKNOWLEDGE)
    {
        uint8_t i;
        for(i=0; i<=MAX_DATAPACKETS; i++)
        {
            if((message[i].seqID == rxMsg->data1[0]) && (message[i].dstAddress == rxMsg->srcAddress))
            {
                message[i].valid = false;
            }
        }
    }
    //    process msg step9

    if((rxMsg->cmd & 0x7F) == CMD_SET) // if set command is received
    {
        if(rxMsg->channel == 0)
            RED_LED = rxMsg->data1[0];
        if(rxMsg->channel == 1)
            GREEN_LED = rxMsg->data1[0];
        if(rxMsg->channel == 2)
            PUSH_BUTTON = rxMsg->data1[0];
        appendUImsg("messageUI \0");
            //waitMicrosecond(1000);
            char a[10];
            itostring(456, &a[0]);
            appendUImsg(&a[0]);
            //waitMicrosecond(1000);
            appendUImsg(" test\0");
            sendUImsg();
        sendUImsg("rxData set channel");
    }

    if((rxMsg->cmd & 0x7F) == CMD_DATAREQUEST)
    {
        uint8_t data1[0];
        if(rxMsg->channel == 0)
            data1[0] = RED_LED;
        if(rxMsg->channel == 1)
            data1[0] = GREEN_LED;
        if(rxMsg->channel == 2)
            data1[0] = PUSH_BUTTON;
        uint8_t cmd = CMD_DATAREPORT;
        sendRS485(rxMsg->dstAddress, cmd, 0, 1, &data1[0], 0);
        //sendUImsg("Report sent");
    }

    if((rxMsg->cmd & 0x7F) == CMD_DATAREPORT)
    {
        //sendUImsg("Channel Value\0");
    }

    if((rxMsg->cmd & 0x7F) == CMD_SETADDRESS)
    {
        myAddress = rxMsg->data1[0];
        writeEEPROM(myAddress);
    }

    if((rxMsg->cmd & 0x7F) == CMD_RESET)
    {
        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    }

    if((rxMsg->cmd & 0x7F) == CMD_POLLREQUEST)
    {
        uint8_t data2[0];
        uint8_t dstAddr = rxMsg->srcAddress;
        uint8_t cmd = 0x79;
        ack = 0;
        data2[0] = myAddress;
        sendRS485(dstAddr, cmd, 0, 0, &data2[0], ack);
    }

    if((rxMsg->cmd & 0x7F) == CMD_POLLRESPONSE)
    {
        //sendUImsg("poll received from address");
    }

    if((rxMsg->cmd & 0x7F) == CMD_SQUARE)
    {
        act.command = rxMsg->cmd;
        act.level1 = rxMsg->data1[0];
        act.level2 = rxMsg->data1[1];
        act.time1 = rxMsg->data1[2];
        act.time2 = rxMsg->data1[3];
        uint8_t temp1 = rxMsg->data1[4]<<8;
        uint8_t temp2 = rxMsg->data1[5];
        act.count = temp1 | temp2;
    }
}
void uart1ISR()
{
    //    static uint8_t j = 0;
    //    uint8_t  checkSum_C = 0;
    if((UART1_RIS_R & UART_RIS_TXRIS) == UART_RIS_TXRIS)
    {
        if((UART1_FR_R & UART_FR_TXFE))
            //if(UART1_FR_R & UART_FR_TXFE)
        {
            send485Byte();
            UART1_ICR_R |= UART_ICR_TXIC;         // clear interrupt flag
        }
    }
    else
    {
        static uint8_t j = 0, checkSum_C = 0;
        //while (UART1_FR_R & UART_FR_RXFE);
        if(UART1_FR_R & UART_FR_RXFF)
        {
            busy = true;
            //uint16_t address = UART1_DR_R;
            //if((rxPhase == 0) && ((address & UART_DR_PE) == UART_DR_PE) && (((address & 0xFF) == 0xFF) || ((address && 0xFF) == myAddress)))
            if((rxPhase == 0))
            {
                uint16_t address = UART1_DR_R;
                rxMsg.dstAddress = address & 0xFF;
                rxPhase++;

            }
            else if(rxPhase>0)
            {
                if(rxPhase == 1)
                {
                    rxMsg.srcAddress = UART1_DR_R & 0xFF;
                    rxPhase++;
                }
                else if(rxPhase == 2)
                {
                    rxMsg.seqID = UART1_DR_R & 0xFF;
                    rxPhase++;
                }
                else if(rxPhase == 3)
                {
                    rxMsg.cmd = UART1_DR_R & 0xFF;
                    rxPhase++;
                }
                else if(rxPhase == 4)
                {
                    rxMsg.channel = UART1_DR_R & 0xFF;
                    rxPhase++;
                }
                else if(rxPhase == 5)
                {
                    rxMsg.size = UART1_DR_R & 0xFF;
                    rxPhase++;
                }
                else if(rxPhase == 6)
                {
                    if(j<rxMsg.size)
                    {
                        rxMsg.data1[j] = UART1_DR_R & 0xFF;
                        checkSum_C += rxMsg.data1[j++];
                    }
                    else
                    {
                        j = 0;
                        rxPhase++;
                    }
                }
                if(rxPhase == 7)
                {
                    rxMsg.checkSum = UART1_DR_R & 0xFF;
                    rxPhase += rxMsg.size+1;
                    UART1_ICR_R |= UART_ICR_RXIC;
                    checkSum_C += rxMsg.dstAddress+rxMsg.srcAddress+rxMsg.cmd+rxMsg.seqID+rxMsg.channel+rxMsg.size;
                    rxPhase = 0;
                    if((checkSum_C & rxMsg.checkSum) == 0)
                    {

                        RX_GREEN_LED = 1;
                        rxLEDtimeout = 100;
                        //sendUImsg("Received\r\n");
                        process_rxData(&rxMsg);

                    }
                    else
                    {
                        RX_GREEN_LED = 1;
                    }
                    checkSum_C = 0;
                }
            }
        }
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    initHw();
    initUart0();
    initUart1();
    initMsg();
    initAdc0Ss3();
    setAdc0Ss3Mux(3);
    setUart0BaudRate(115200, 40e6);
    //putsUart0("Serial Example\n");
    USER_DATA data;
    RANDOM = true;
    CurrentRandomNumber = generateSeed();



//    appendUImsg("messageUI \0");
//    //waitMicrosecond(1000);
//    char a[10];
//    itostring(456, &a[0]);
//    appendUImsg(&a[0]);
//    //waitMicrosecond(1000);
//    appendUImsg(" test\0");
//    sendUImsg();




    while (true)
    {
        //        rxIndex = 0;
        //        txIndex = 0;

        getsUart0(&data);
        //putsUart0(data.buffer);
        parseFields(&data);
        bool valid, ack;

        // Echo back the parsed field data (type and fields)
        uint8_t i;
        for (i = 0; i < data.fieldCount; i++)
        {
            putcUart0(data.fieldType[i]);
            putcUart0('\t');
            putsUart0(&data.buffer[data.fieldPosition[i]]);
            putcUart0('\n');
        }

        //Step1

        if (isCommand(&data, "reset", 0))
        {

            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            valid = true;
        }

        //putsUart0("load\n");

        if (isCommand(&data, "cs", 1))
        {
            char *firstArgument = getFieldString(&data, 1);
            {
                if (strCompare(firstArgument, "ON"))
                {
                    CS = 1;
                }
                else if (strCompare(firstArgument, "OFF"))
                {
                    CS = 0;
                }
                else
                    putsUart0("Invalid Argument\n");
            }
            valid = true;
        }

        if (isCommand(&data, "random", 1))
        {
            char *firstArgument = getFieldString(&data, 1);
            {
                if (strCompare(firstArgument, "ON"))
                {
                    RANDOM = 1;
                }
                else if (strCompare(firstArgument, "OFF"))
                {
                    RANDOM = 0;
                }
                else
                    putsUart0("Invalid Argument\n");
            }
            valid = true;
        }

        if (isCommand(&data, "ack", 1))
        {
            char *firstArgument = getFieldString(&data, 1);

            if (strCompare(firstArgument, "ON"))
            {
                ack = true;
            }
            else if (strCompare(firstArgument, "OFF"))
            {
                ack = false;
            }
        }

        if (isCommand(&data, "set", 3))
        {
            //parseFields(&data);
            uint8_t destAddress = getFieldInteger(&data, 1);
            uint8_t channel = getFieldInteger(&data, 2);
            uint8_t data1[10];
            data1[0] = (uint8_t) getFieldInteger(&data, 3);
            uint8_t size = 1;
            uint8_t cmd = CMD_SET;
            sendRS485(destAddress, cmd, channel, size, &data1[0], ack);
        }

        process_rxData(&rxMsg);

        if (isCommand(&data, "reset", 1))
        {
            //parseFields(&data);
            uint8_t destAddress = getFieldInteger(&data, 1);
            uint8_t cmd = CMD_RESET;
            sendRS485(destAddress, cmd, 0, 0, 0, ack);
        }


        if (isCommand(&data, "square", 8))
        {

            uint8_t destAddress = getFieldInteger(&data, 1);
            uint8_t channel = getFieldInteger(&data, 2);
            // Square Value1 Value2 Time1(2B) Time2(2B) Cycles(2B)

            uint8_t data1[10];
            data1[0] = (uint8_t) getFieldInteger(&data, 1);
            data1[1] = (uint8_t) getFieldInteger(&data, 2);
            uint32_t x = getFieldInteger(&data, 3);
            data1[2] = x & 0xFF;
            data1[3] = (x >> 8) & 0xFF;
            x = getFieldInteger(&data, 4);
            data1[4] = x & 0xFF;
            data1[5] = (x >> 8) & 0xFF;
            x = getFieldInteger(&data, 5);
            data1[6] = x & 0xFF;
            data1[7] = (x >> 8) & 0xFF;
            uint8_t size = 8;
            uint8_t cmd = CMD_SQUARE;

            sendRS485(destAddress, cmd, channel, size, &data1[0], ack);
        }
//
       if (isCommand(&data, "pulse", 3))
        {
                        uint8_t destAddress = getFieldInteger(&data, 1);
                        uint8_t channel = getFieldInteger(&data, 2);
                        // Pulse = value1(1 byte), value2(2bytes)
                        uint8_t data1[3];
                        data1[0] = (uint8_t) getFieldInteger(&data, 1);
                        uint32_t x = getFieldInteger(&data, 2);
                        data1[1] = x & 0xFF;
                        data1[2] = (x >> 8) & 0xFF;
                        uint8_t size = 3;
                        uint8_t cmd = CMD_PULSE;

                        sendRS485(destAddress, cmd, channel, size, &data1[0], ack);
        }

       if (isCommand(&data, "DataRequest", 0))
               {
                               uint8_t destAddress = getFieldInteger(&data, 1);
                               uint8_t channel = getFieldInteger(&data, 2);
                               //Data Request = 0 data values
                               uint8_t size = 0;
                               uint8_t cmd = CMD_DATAREQUEST;
                               sendRS485(destAddress, cmd, channel, size, 0, ack);
               }

       if (isCommand(&data, "PollRequest", 0))
                     {
                                     uint8_t destAddress = getFieldInteger(&data, 1);
                                     uint8_t channel = getFieldInteger(&data, 2);
                                     //Data Request = 0 data values
                                     uint8_t size = 0;
                                     uint8_t cmd = CMD_POLLREQUEST;
                                     sendRS485(destAddress, cmd, channel, size, 0, ack);
                     }



        if (!valid)
        {
            putsUart0("Invalid command\n");
        }
    }
}
