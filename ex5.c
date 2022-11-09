/*
 * File:   ex5.c
 * Author: Marco
 *
 * Created on 3 novembre 2022, 11.51
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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define TIMER1 1
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer,int ms);

void ex1(){
    tmr_setup_period(TIMER1, 1000);
    //UART2
    U2BRG = 11; // (7372800 / 4) / (16 * 9600) ? 1
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
    U2STAbits.URXISEL = 1;
    
    //SPI1
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6; // 1:2 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
    
    while(1){
        if(U2STAbits.URXDA){
        //read character
            SPI1BUF = U2RXREG;
        }
    }
}

void ex2(){
    tmr_setup_period(TIMER1, 1000);
    //UART2
    U2BRG = 11; // (7372800 / 4) / (16 * 9600) ? 1
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
    U2STAbits.URXISEL = 1;
    
    while(1){
        if(U2STAbits.URXDA){
        //read character
            U2TXREG = U2RXREG;
        }
    }
}

void ex3(){
    tmr_setup_period(TIMER1, 1000);
    //UART2
    U2BRG = 11; // (7372800 / 4) / (16 * 9600) ? 1
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U2TX (must be after UARTEN)
    U2STAbits.URXISEL = 1;
    
    //SPI1
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6; // 1:2 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
    
    while(1){
        if(U2STAbits.URXDA){
        //read character
            char character = U2RXREG;
            U2TXREG = character;
            SPI1BUF = character;
        }
    }
}
int main(void) {
    
    //ex1();
    //ex2();
    ex3();

    return 0;
}

void tmr_setup_period(int timer, int ms){
    
    int pr = (((7372800/4)/64)/(1000.0/ms)); // (7.3728 MHz / 4) / 64 / (1000/ms)
    
    switch (timer){
        case 1:
            TMR1 = 0;
            PR1 = pr;
            T1CONbits.TCKPS=2;
            IFS0bits.T1IF = 0; // set interrupt to zero
            T1CONbits.TON = 1; // starts the timer!
            break;
        case 2:
            TMR2 = 0;
            PR2 = pr;
            T1CONbits.TCKPS=2;
            IFS0bits.T2IF = 0; //set interrupt to zero
            T2CONbits.TON = 1; // starts the timer!
            break;
        default:
            break;
    }
}
void tmr_wait_period(int timer){
    switch (timer){
        case 1:
            while(IFS0bits.T1IF ==0){
            }
            IFS0bits.T1IF =0; //reset flag
            
            break;
        case 2:
            while(IFS0bits.T2IF ==0){
            }
            IFS0bits.T2IF =0; //reset flag
            break;    
        default:
            break;
    }
}

void tmr_wait_ms(int timer,int ms){

    tmr_setup_period(timer,ms);
    tmr_wait_period(timer);
    switch (timer){
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
