/*
 * File:   Assignment.c
 * Author: Marco Macchia, Lorenzo Benedetti, Samuele Pedrazzi, Matteo Aicardi, Davide Bruzzo
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
#define TIMER3 3
#define SIZE 16


//struct for circular buffer

typedef struct {
    char data[SIZE];
    int read_idx;
    int write_idx;
} circularBuffer;

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
//convert int to string
void my_itoa(int num, char buffer[]);

//global circular buffer
volatile circularBuffer cb;
//counter that counts how many char the main loop should read from che circular buffer
int to_be_read_counter = 0;
//flag raised when s5 is pressed
int s5_pressed = 0;

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
        if (s5_pressed) {

            //reset flag
            s5_pressed = 0;

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
        while (to_be_read_counter > 0) {

            //read incoming character
            char received = cb.data[cb.read_idx++];
            //when end is reached start again from the beggining of the array
            cb.read_idx = cb.read_idx % SIZE;

            //disable UART2 Interrupt
            IEC1bits.U2RXIE = 0;
            //increment counter for char that has to be read
            to_be_read_counter--;

            //re-enable UART2 interrupt
            IEC1bits.U2RXIE = 1;

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
                    //reset overflow flag and discard all char
                    U2STAbits.OERR = 0;
                }

                //write character on LCD
                send_lcd(received);

                //increment number of characters received
                counter++;
                counter_first_row++;

                //max value of counter is 999. If exceeded reset
                counter = counter % 1000;

            }
        }
        //print the second row of lcd
        print_second_row_lcd(counter);

        //move cursor to the correct position in the first row
        send_lcd(0x80 + counter_first_row);
    }
    return 0;
}

void setup_board(int counter) {

    //time to initialize lcd
    tmr_wait_ms(TIMER1, 1000);

    // set UART2
    // (7372800 / 4) / (16 * 9600) - 1
    U2BRG = 11;
    // enable UART
    U2MODEbits.UARTEN = 1;
    // enable U2TX
    U2STAbits.UTXEN = 1;
    U2STAbits.URXISEL = 1;

    //enable UART receiver interrupt
    IEC1bits.U2RXIE = 1;

    //set SPI1
    // master mode
    SPI1CONbits.MSTEN = 1;
    // 8 bit mode
    SPI1CONbits.MODE16 = 0;
    // 1:1 primary prescaler
    SPI1CONbits.PPRE = 1;
    // 1:2 secondary prescaler 
    SPI1CONbits.SPRE = 6;
    SPI1STATbits.SPIEN = 1;
    // enable SPI

    //enable INT0 interrupt (button S5)
    IEC0bits.INT0IE = 1;

    //set S5 button (E8 pin) as input
    TRISEbits.TRISE8 = 1;

    //print the second row of lcd
    print_second_row_lcd(counter);
    //move cursor to first position of first row
    send_lcd(0x80);

    //setup timer to synchronize main to 100hz
    tmr_setup_period(TIMER2, 10);
}

void tmr_setup_period(int timer, int ms) {

    // (7.3728 MHz / 4) / 64 / (1000/ms)
    int pr = (((7372800 / 4) / 64) / (1000.0 / ms));

    switch (timer) {
        case 1:
            TMR1 = 0;
            PR1 = pr;
            T1CONbits.TCKPS = 2;
            // set interrupt flag to zero
            IFS0bits.T1IF = 0;
            // starts the timer
            T1CONbits.TON = 1;
            break;
        case 2:
            TMR2 = 0;
            PR2 = pr;
            T2CONbits.TCKPS = 2;
            // set interrupt flag to zero
            IFS0bits.T2IF = 0;
            // starts the timer
            T2CONbits.TON = 1;
            break;
        case 3:
            TMR3 = 0;
            PR3 = pr;
            T3CONbits.TCKPS = 2;
            // set interrupt flag to zero
            IFS0bits.T3IF = 0;
            // starts the timer
            T3CONbits.TON = 1;
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
            //reset flag
            IFS0bits.T1IF = 0;

            break;
        case 2:
            while (IFS0bits.T2IF == 0) {
            }
            //reset flag
            IFS0bits.T2IF = 0;
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
            // stops the timer
            T1CONbits.TON = 0;
            TMR1 = 0;
            break;
        case 2:
            // stops the timer
            T2CONbits.TON = 0;
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
        //write empty char on the LCD
        send_lcd(' ');
    }

    //move cursor to first position of first row
    send_lcd(0x80);
}

void print_second_row_lcd(int counter) {
    //buffer for printing on LCD
    char buffer[16] = "Char Recv: ";

    //move cursor second row
    send_lcd(0xC0);

    //write on lcd
    for (int i = 0; buffer[i] != '\0'; i++) {
        send_lcd(buffer[i]);
    }

    //convert counter in char
    my_itoa(counter, buffer);

    //write on lcd
    for (int i = 0; buffer[i] != '\0'; i++) {
        send_lcd(buffer[i]);
    }

    //wipe remaining char in second row
    for (int i = 0; i < 2 && counter < 100; i++)
        send_lcd(' ');

}

void send_lcd(char character) {

    // wait until SPI buffer is not full
    while (SPI1STATbits.SPITBF == 1);
    //send to lcd
    SPI1BUF = character;

}

void send_uart(int counter) {

    //buffer for printing on LCD
    char buffer[4];
    my_itoa(counter, buffer);

    //create string
    //sprintf(buffer, "%d", counter);

    for (int i = 0; buffer[i] != '\0'; i++)
        //send via uart the number of characters received
        U2TXREG = buffer[i];
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt
() {
    // reset interrupt flag
    IFS1bits.U2RXIF = 0;

    //read incoming char
    char received = U2RXREG;

    //if there are empty spaces on the circular buffer
    if (to_be_read_counter < SIZE) {
        //store incoming char on circular buffer
        cb.data[cb.write_idx++] = received;
        //when end is reached start again from the beggining of the array
        cb.write_idx = cb.write_idx % SIZE;
        //increment counter for char that has to be read
        to_be_read_counter++;
    }

}

void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt
() {
    // reset interrupt flag
    IFS0bits.INT0IF = 0;

    //disable INT0 interrupt
    IEC0bits.INT0IE = 0;

    //start timer 3
    tmr_setup_period(TIMER3, 30);

    //enable timer 3 interrupt
    IEC0bits.T3IE = 1;
}

void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt
() {
    // reset interrupt T3 flag
    IFS0bits.T3IF = 0;

    // reset interrupt INT0 flag
    IFS0bits.INT0IF = 0;

    //stop T3 timer
    T3CONbits.TON = 0;
    TMR3 = 0;

    //disable T3 interrupt
    IEC0bits.T3IE = 0;

    //enable INT0 interrupt
    IEC0bits.INT0IE = 1;

    if (PORTEbits.RE8) {
        //raise button pressed flag
        s5_pressed = 1;
    }
}

void my_itoa(int num, char buffer[]) {

    //convert number into a string using ascii char codes
    
    if (num < 10) {
        buffer[0] = 48 + num % 10;
        buffer[1] = '\0';
    } else if (num < 100) {
        buffer[0] = 48 + (num / 10) % 10;
        buffer[1] = 48 + num % 10;
        buffer[2] = '\0';
    } else {
        buffer[0] = 48 + num / 100;
        buffer[1] = 48 + (num / 10) % 10;
        buffer[2] = 48 + num % 10;
        buffer[3] = '\0';
    }
}
