/* 
 * File:   main.c
 * Author: Maciej Napora
 *
 * Created on 04 February 2020, 22:02
 */


#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
#pragma config DEBUG = OFF         //

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic.h>
#include <pic16f877a.h>

#define _XTAL_FREQ 20000000 //Specify the XTAL crystal FREQ
#define TRUE 0x01
#define FALSE 0x00
#define IRT PORTAbits.RA0
#define DEBUG PORTAbits.RA1
#define TOGGLE_DEBUG   if(TRUE == DEBUG){DEBUG = FALSE;} else { DEBUG = TRUE; }
#define DHT22 PORTBbits.RB4
#define jN 20
#define iN 8
/*
 * 
 */


unsigned int active_RF = 0;
unsigned int i = 0;
unsigned int j = 0;

enum State_IR_38KHz{low, high};
enum State_IR_38KHz state_IR_38KHz = low;

enum State_IR_Command_Send{waiting_for_signal, decode, sending};
enum State_IR_Command_Send state_IR_command_send = waiting_for_signal;

enum State_DHT22{   
                    idle, bring_down_for_500us, host_up, sensor_down, sensor_up, 
                    int_RH, dec_RH, int_temp, dec_temp, chk_sum
                };
enum State_DHT22 state_DHT = idle;

/* flag for generating infrared command */
unsigned int flag_IR = FALSE;
/* 100us task flag */
unsigned int flag_IRTMR2IF = FALSE;
/* variable holding a recent command */
unsigned int USART_command = 0x00;
/* 100us task flag */
unsigned int flag_DHT22_acquire = FALSE;
/* DHT data available */
unsigned int flag_DHT22_data_available = FALSE;
/* counter of DHT22 bits */
unsigned int DHT22_bit = 0;
/* time elapsed since DHT22 pin change, DHT22_change_counter == 8 -> 5 us */
volatile unsigned int DHT22_change_counter = 0;
unsigned int DHT22_last_state;

//unsigned int DHT_data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

#define DHT_N_INTERVAL 42
unsigned int DHT_data_time[DHT_N_INTERVAL];

/* Encode the ON/OFF command of the heater; Associated with 0x4F or ASCII 'O' */
const unsigned int signal_O[jN] = { 
                            0b11111100, 0b00000000, 0b00000001, 0b11111110,
                            0b10101010, 0b10101011, 0b10111011, 0b10111011,
                            0b10111011, 0b10111010, 0b10111010, 0b10111010,
                            0b10101110, 0b10111011, 0b10101110, 0b11101110,
                            0b11111111, 0b11111111, 0b11111111, 0b11111111 
};

/* Encode the set temperature of heater; Associated with 0x53 or ASCII 'S' */
const unsigned int signal_S[jN] = { 
                            0b11111100, 0b00000000, 0b00000001, 0b11111110,
                            0b10101010, 0b10101011, 0b10111011, 0b10111011, 
                            0b10111011, 0b10111010, 0b10101110, 0b10101010, 
                            0b10111011, 0b10101110, 0b11101110, 0b11101110,
                            0b11111111, 0b11111111, 0b11111111, 0b11111111 
                        };

/* Encode the confirm temperature; Associated with 0x4B or ASCII 'K' */
const unsigned int signal_K[jN] = { 
                            0b11111100, 0b00000000, 0b00000001, 0b11111110,
                            0b10101010, 0b10101011, 0b10111011, 0b10111011, 
                            0b10111011, 0b10111010, 0b10111011, 0b10101010, 
                            0b10101110, 0b10101110, 0b11101110, 0b11101110,
                            0b11111111, 0b11111111, 0b11111111, 0b11111111
                        };

/* Encode the increase of the temperature; Associated with 0x2B or ASCII '+' */
const unsigned int signal_plus[jN] = { 
                            0b11111100, 0b00000000, 0b00000001, 0b11111110,
                            0b10101010, 0b10101011, 0b10111011, 0b10111011, 
                            0b10111011, 0b10111010, 0b11101011, 0b10101010, 
                            0b10101011, 0b10101110, 0b11101110, 0b11101110,
                            0b11111111, 0b11111111, 0b11111111, 0b11111111
                        };

/* Encode the decrease of the temperature; Associated with 0x2D or ASCII '-' */
const unsigned int signal_minus[jN] = { 
                            0b11111100, 0b00000000, 0b00000001, 0b11111110,
                            0b10101010, 0b10101011, 0b10111011, 0b10111011, 
                            0b10111011, 0b10111010, 0b10101011, 0b10101010, 
                            0b10111011, 0b10111010, 0b11101110, 0b11101110,
                            0b11111111, 0b11111111, 0b11111111, 0b11111111
                        };

static unsigned int * encoded_command;

void UART_send_char(char);
void UART_send_string(char*);
char UART_get_char();
void generate_IR_command();
void get_pointer_to_command(unsigned int received_byte);
void task_100us();
unsigned int set_DHT_reading(
                      unsigned int, 
                      enum State_DHT22,
                      unsigned int
                    );

int main(int argc, char** argv) {

    PORTA = 0x00;
    TRISA = 0x00;
    ADCON1 = 0x06;
    IRT = 0;
    
    /*Configure port B*/
    PORTB = 0x00;
    TRISB = 0x00;
    DHT22 = TRUE;
    
    /* Configure timer 0 and its interrupt */
    /* Set prescaler to 128*/
    OPTION_REG = (OPTION_REG & 0b11000000)| (0b00111111 & 0b00000010);
    TMR0 = 0xFF;       
    TMR0IE = TRUE;       //Enable timer interrupt bit in PIE1 register

    /* Configure timer 1 */
    TMR1L = 0x00;
    TMR1H = 0x00;
    T1CON = 0b00110001; // set the timer 2 configuration
//    TMR1IE = TRUE;
    
    /* Configure timer 2 and its interrupt */
    PR2 = 23;
    TMR2 = 0;
    T2CON = 0b00000111; // set the timer 2 configuration
    TMR2IE = TRUE;
    
    /* Enable interrupt for USART read */
    RCIE = TRUE;
    
    GIE = 1;          //Enable Global Interrupt
    PEIE = 1;         //Enable the Peripheral Interrupt
    IRT = 1;
    
    /* RB Port Change Interrupt Enable bit */
    RBIE = 1;
    /* Setting I/O pins for UART */
    TRISC6 = 0;         // TX Pin set as output
    TRISC7 = 1;         // RX Pin set as input
    /* End of I/O pins set */
    /* Initialise SPBRG register for required 
    baud rate and set BRGH for fast baud_rate */
    SPBRG = 129;        /* ((_XTAL_FREQ/16)/Baud_rate) - 1 */
    BRGH  = 1;          // for high baud_rate
    /* End of baud_rate setting */
    /* Enable Asynchronous serial port */
    SYNC  = 0;    // Asynchronous
    SPEN  = 1;    // Enable serial port pins
    /* Asynchronous serial port enabled */
    /* Lets prepare for transmission & reception */
    TXEN  = 1;    // enable transmission
    CREN  = 1;    // enable reception
    /* UART module up and ready for transmission and reception */
    /* Select 8-bit mode */  
    TX9   = 0;    // 8-bit reception selected
    RX9   = 0;    // 8-bit reception mode selected
    /* 8-bit mode selected */ 
    //while( 0x46 != UART_get_char() );
    
    DEBUG = FALSE;
    
    while(TRUE)
    {    
        task_100us();
    }
}

void __interrupt() ISR(void)
{  
    volatile unsigned int portb;
    volatile unsigned int current_DHT22_state = FALSE;
    unsigned int temp_TMR1L;
    if(TRUE == TMR2IF)
    {
        flag_IRTMR2IF = TRUE;
        /* Timer counting up */
        TMR2 = 0;
        TMR2IF = FALSE;       // Clear timer interrupt flag
    }
    if(TRUE == RBIF)
    {
        portb = PORTB;
        //asm("MOVF PORTB, W");
        RBIF = FALSE;
        current_DHT22_state = (portb >> 4) & 0x01;
        temp_TMR1L = TMR1L;
        TMR1L = 0x00;
        if(    (DHT22_last_state == TRUE) 
            && (current_DHT22_state == FALSE) 
            && (DHT22_bit < DHT_N_INTERVAL)
            && (bring_down_for_500us == state_DHT)
          )
        {
            DHT_data_time[DHT22_bit] = temp_TMR1L;
            DHT22_bit++;
        }
        else if(DHT22_bit == DHT_N_INTERVAL)
        {
            flag_DHT22_data_available = TRUE;
        }
        
        DHT22_last_state = current_DHT22_state;     
    }
    if(TRUE == TMR0IF)
    {
        flag_IR = TRUE;
        /* Timer counting up, interrupt on overflow from 0xFF to 0x00 */
        TMR0 = 0xFB;
        TMR0IF = FALSE;       // Clear timer interrupt flag
        if(low == state_IR_38KHz)
        {
            IRT = 0;
            state_IR_38KHz = high;
        }
        else
        {
            IRT = 0x01 & active_RF;
            state_IR_38KHz = low;
        }
    }
    if(TRUE == RCIF)
    {
        USART_command = RCREG;
        if(USART_command != 0x41)
        {
            state_IR_command_send = decode;
        }
        else if(USART_command == 0x41)
        {
           flag_DHT22_acquire = TRUE; 
        }
        RCIF = FALSE;
    }
}

unsigned int set_DHT_reading(
                      unsigned int DHT22_change_counter, 
                      enum State_DHT22 state_DHT,
                      unsigned int variable
                    )
{
    unsigned int shift = 0;
    
    if( int_RH == state_DHT)
    {
        shift = 0;
        
    }
    else if( dec_RH == state_DHT )
    {
        shift = 8;

    }
    else if( int_temp == state_DHT )
    {
        shift = 16;

    }
    else if( dec_temp == state_DHT )
    {
        shift = 24;
    }
    else if( chk_sum == state_DHT )
    {
        shift = 32;
    }
    else
    {
        
    }
    
    if( ( DHT22_change_counter > 0 ) && ( DHT22_change_counter < 40 ) )
    {
        variable = variable | (0x00<<(DHT22_bit-shift));
        DHT22_bit++;
        
    }
    else if( ( DHT22_change_counter > 40) && ( DHT22_change_counter < 54 ) )
    {
        variable = variable | (0x01<<(DHT22_bit-shift));
        DHT22_bit++;
    }

    return variable;
}

void generate_IR_command(void)
{    
        if(decode == state_IR_command_send)
        {
            get_pointer_to_command(USART_command);
            state_IR_command_send = sending;
        }
        if(sending == state_IR_command_send)
        {
            if(j<jN)
            {
                if(i<iN)
                {
                    active_RF = 0x01 & ~(encoded_command[j]>>(7-i));
                    i++;
                    if(iN == i)
                    {
                        i = 0;
                        j++;
                    }
                }
                if(jN == j)
                {
                    j = 0;
                    state_IR_command_send = waiting_for_signal;
                }
            }
        }
}

void get_pointer_to_command(unsigned int received_byte)
{
    if(0x4F == received_byte)
    {
            encoded_command = signal_O;
    }
    else if (0x53 == received_byte)
    {
        encoded_command = signal_S;
    }
    else if (0x4B == received_byte)
    {
        encoded_command = signal_K;
    }
    else if (0x2B == received_byte)        
    {
            encoded_command = signal_plus;
    }
    else if (0x2D == received_byte)        
    {
            encoded_command = signal_minus;
    }
}

/* Task executed every 100 us */
void task_100us()
{
    static unsigned int counter_IR_command_100us = 0;
    volatile static unsigned int counter_DHT22_100us = 0;
    enum State_IR_2kHz{low, high};
    static enum State_IR_2kHz state_IR_2KHz = low;
    unsigned int DHT22_int_RH = 0;
    unsigned int DHT22_dec_RH = 0;
    unsigned int DHT22_int_T = 0;
    unsigned int DHT22_dec_T = 0;
    unsigned int DHT22_chksum = 0;
    if( TRUE == flag_IRTMR2IF)
    {
        flag_IRTMR2IF = FALSE;

        /* Body of the task */
        if(counter_IR_command_100us < 5)
        {
            counter_IR_command_100us++;
        }
        else
        {
            generate_IR_command();
            counter_IR_command_100us = 0;
        }
        

        if(TRUE == flag_DHT22_acquire)
        {
            flag_DHT22_acquire = FALSE;
            for(unsigned int i= 0; i < DHT_N_INTERVAL; i++)
                DHT_data_time[i] = 0;
            DHT22_bit = 0;
            state_DHT = bring_down_for_500us;
            DHT22 = FALSE;
            counter_DHT22_100us = 0;
        }
        if( bring_down_for_500us == state_DHT )
        {
            
            if(counter_DHT22_100us < 6)
            {
                counter_DHT22_100us++;
            }
            else if(counter_DHT22_100us == 6)
            {
                DHT22 = TRUE;
                TRISB = 0x10;
            }
        }
        if(TRUE == flag_DHT22_data_available)
        {
            counter_DHT22_100us = 0;
            TRISB = 0x00;
            flag_DHT22_data_available = FALSE;
            state_DHT = idle;
            unsigned int bit_value = 0;
            DHT22_int_RH = 0;
            DHT22_dec_RH = 0;
            DHT22_int_T = 0;
            DHT22_dec_T = 0;
            DHT22_chksum = 0;
            for(unsigned int i = 0; i < 8; i++)
            {
                if(DHT_data_time[i + 2] > 30)
                {
                    bit_value = 1;
                }
                else
                {
                    bit_value = 0;
                }
                //UART_send_char(i);
                DHT22_int_RH = DHT22_int_RH | ( bit_value << (7 - i) ) ;        
            }
            UART_send_char(DHT22_int_RH);
            for(unsigned int i = 0; i < 8; i++)
            {
                if(DHT_data_time[i + 10] > 30)
                {
                    bit_value = 1;
                }
                else
                {
                    bit_value = 0;
                }
                //UART_send_char(i);
                DHT22_dec_RH = DHT22_dec_RH | ( bit_value << (7 - i) ) ;        
            }
            UART_send_char(DHT22_dec_RH);
            for(unsigned int i = 0; i < 8; i++)
            {
                if(DHT_data_time[i + 18] > 30)
                {
                    bit_value = 1;
                }
                else
                {
                    bit_value = 0;
                }
                //UART_send_char(i);
                DHT22_int_T = DHT22_int_T | ( bit_value << (7 - i) ) ;        
            }
            UART_send_char(DHT22_int_T);
            for(unsigned int i = 0; i < 8; i++)
            {
                if(DHT_data_time[i + 26] > 30)
                {
                    bit_value = 1;
                }
                else
                {
                    bit_value = 0;
                }
                //UART_send_char(i);
                DHT22_dec_T = DHT22_dec_T | ( bit_value << (7 - i) ) ;        
            }
            UART_send_char(DHT22_dec_T);
            for(unsigned int i = 0; i < DHT_N_INTERVAL;i++)
            {
                UART_send_char(DHT_data_time[i]);
            }
        }
    }

}

/* Function to send one byte of date to UART */
void UART_send_char(char bt)  
{
    while(!TXIF);  // hold the program till TX buffer is free
    TXREG = bt; //Load the transmitter buffer with the received value
}

/* Function to convert string to byte */
void UART_send_string(char* st_pt)
{
    while(*st_pt)
    {
        UART_send_char(*st_pt++); //process it as a byte data
    }    
}

/* Function to get one byte of date from UART */
char UART_get_char()   
{
    if(OERR) // check for Error 
    {
        CREN = 0; //If error -> Reset 
        CREN = 1; //If error -> Reset 
    }
    
    while(!RCIF);  // hold the program till RX buffer is free
    
    return RCREG; //receive the value and send it to main function
}