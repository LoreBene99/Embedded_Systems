/*
 * File:   Assignment.c
 * Author: Marco
 *
 * Created on 6 novembre 2022, 15.59
 */


#include <xc.h>
#include <stdio.h>

// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)


#define TIMER1 1
#define TIMER2 2
#define SIZE 16

//setup the period of a timer
void tmr_setup_period(int timer, int ms);
//wait until the timer is expired
void tmr_wait_period(int timer);
//wait for a certain amount of time using a given timer
void tmr_wait_ms(int timer, int ms);
//clear first row of the LCD
void clear_first_row_lcd();
//print the data ont the second row of the LCD
void print_second_row_lcd(int counter);
//safely send the data to the LCD
void send_lcd(char character);
//send data using UART
void send_uart(int counter);
//setup board and simulation
void setup_board(int counter);
//UART2 receiver interrupt routine
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt();

//9600 baud significa invio di 9600 simboli (bit) al secondo. Un char in UART
//sono 10 bit, 8 di char + start e stop. Quindi 960 caratteri al secondo, 
//circa 1 ogni 1ms. Durante l'algoritmo possiamo ricevere quindi al massimo 7 char
//che dovranno essere scritti.
char circular_buffer[SIZE];
int read_idx = 0;
int write_idx = 0;
int to_be_read_counter = 0;

int main(void) {

    //counter of received characters
    int counter = 0;
    //counter of bytes wrote on the first row
    int counter_first_row = 0;

    //setup UART2, SPI1, interrupts and LCD
    setup_board(counter);

    //main loop
    while (1) {

        //wait main period
        tmr_wait_period(TIMER2);

        //simulate algorithm that needs 7ms
        tmr_wait_ms(TIMER1, 7);

        //if button S5 has been pressed
        if (IFS0bits.INT0IF) {
            //reset INT0 flag
            IFS0bits.INT0IF = 0;

            //send number of char recv via uart
            send_uart(counter);
        }

        //if button S6 has been pressed
        if (IFS1bits.INT1IF) {
            //reset INT1 flag
            IFS1bits.INT1IF = 0;

            //reset total counter
            counter = 0;
            //reset first row counter
            counter_first_row = 0;
            //clear first row
            clear_first_row_lcd();
            //print the second row of lcd
            print_second_row_lcd(counter);
            //move cursor to the correct position in the first row
            send_lcd(0x80);
        }

        //if there is a character that has to be read from the buffer
        if (to_be_read_counter) {

            //read incoming character
            char received = circular_buffer[read_idx++];
            //when end is reached start again from the beggining of the array
            read_idx = read_idx % SIZE;
            //increment counter for char that has to be read
            to_be_read_counter--;


            //if CR or LF is received clear first row
            if (received == '\r' || received == '\n') {
                //reset overflow flag
                U2STAbits.OERR = 0;
                //reset first row counter
                counter_first_row = 0;
                //clear first row
                clear_first_row_lcd();
            } else {

                //if first row is full, clear and start again
                if (counter_first_row == 16) {

                    //reset first row counter
                    counter_first_row = 0;
                    //clear first row
                    clear_first_row_lcd();
                }

                //if an overflow is occurred
                if (U2STAbits.OERR == 1) {
                    //[...]
                    //reset overflow flag and discard all char
                    U2STAbits.OERR = 0;
                }

                //write character on LCD
                send_lcd(received);

                //increment number of characters received
                counter++;
                counter_first_row++;

                //print the second row of lcd
                print_second_row_lcd(counter);

                //move cursor to the correct position in the first row
                send_lcd(0x80 + counter_first_row);
            }
        }
    }
    return 0;
}

void setup_board(int counter) {
    //time to initialize lcd
    tmr_wait_ms(TIMER1, 1000);

    //setup timer to synchronize main to 100hz
    tmr_setup_period(TIMER2, 10);

    // set UART2
    U2BRG = 11; // (7372800 / 4) / (16 * 9600) ? 1
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
    U2STAbits.URXISEL = 1;

    //enable UART receiver interrupt
    IEC1bits.U2RXIE = 1;

    //set SPI1
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6; // 1:2 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI

    //print the second row of lcd
    print_second_row_lcd(counter);
    //move cursor to first position of first row
    send_lcd(0x80);
}

void tmr_setup_period(int timer, int ms) {

    int pr = (((7372800 / 4) / 64) / (1000.0 / ms)); // (7.3728 MHz / 4) / 64 / (1000/ms)

    switch (timer) {
        case 1:
            TMR1 = 0;
            PR1 = pr;
            T1CONbits.TCKPS = 2;
            IFS0bits.T1IF = 0; // set interrupt to zero
            T1CONbits.TON = 1; // starts the timer!
            break;
        case 2:
            TMR2 = 0;
            PR2 = pr;
            T1CONbits.TCKPS = 2;
            IFS0bits.T2IF = 0; //set interrupt to zero
            T2CONbits.TON = 1; // starts the timer!
            break;
        default:
            break;
    }
}

void tmr_wait_period(int timer) {
    switch (timer) {
        case 1:
            while (IFS0bits.T1IF == 0) {
            }
            IFS0bits.T1IF = 0; //reset flag

            break;
        case 2:
            while (IFS0bits.T2IF == 0) {
            }
            IFS0bits.T2IF = 0; //reset flag
            break;
        default:
            break;
    }
}

void tmr_wait_ms(int timer, int ms) {

    tmr_setup_period(timer, ms);
    tmr_wait_period(timer);
    switch (timer) {
        case 1:
            T1CONbits.TON = 0; // stops the timer!
            TMR1 = 0;
            break;
        case 2:
            T2CONbits.TON = 0; // stops the timer!
            TMR2 = 0;
            break;
        default:
            break;
    }

}

void clear_first_row_lcd() {

    //move cursor to first position of first row
    send_lcd(0x80);

    for (int i = 0; i < 16; i++) {
        send_lcd(' ');
    }

    //move cursor to first position of first row
    send_lcd(0x80);
}

void print_second_row_lcd(int counter) {
    //buffer for printing on LCD
    char buffer[16];

    //create string
    sprintf(buffer, "Char Recv: %d%d%d", counter / 100, (counter / 10) % 10, counter % 10);


    //move cursor second row
    send_lcd(0xC0);

    //write on lcd
    for (int i = 0; buffer[i] != '\0'; i++) {
        send_lcd(buffer[i]);
    }
}

void send_lcd(char character) {

    // wait until SPI buffer is not full
    while (SPI1STATbits.SPITBF == 1);
    //send to lcd
    SPI1BUF = character;

}

void send_uart(int counter) {

    //buffer for printing on LCD
    char buffer[3];

    //create string
    sprintf(buffer, "%d", counter);

    for (int i = 0; buffer[i] != '\0'; i++)
        //send via uart the number of characters received
        U2TXREG = buffer[i];
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt
() {
    if (to_be_read_counter < SIZE) {
        // reset interrupt flag
        IFS1bits.U2RXIF = 0;
        //read incoming char
        circular_buffer[write_idx++] = U2RXREG;
        //when end is reached start again from the beggining of the array
        write_idx = write_idx % SIZE;
        //increment counter for char that has to be read
        to_be_read_counter++;
    }

}
