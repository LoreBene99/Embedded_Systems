/*
 * File:   ex3.c
 * Author: Marco
 *
 * Created on 11 ottobre 2022, 11.13
 */

#include <xc.h>

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


#include "xc.h"

#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer,int ms);

void ex1() {
// initialization code    
    TRISBbits.TRISB0 = 0; //set D3 as output
    TRISBbits.TRISB1 = 0; //set D4 as output
    LATBbits.LATB0 = 1; //set led
    tmr_setup_period(TIMER1, 500);
    tmr_setup_period(TIMER2, 250);
    IEC0bits.T2IE = 1; //enable interrupt

    LATBbits.LATB0 = 0; //set led
    while(1){
        tmr_wait_period(TIMER1);
        LATBbits.LATB0 = !LATBbits.LATB0; //invert led
    }
}

void ex2() {
    IEC0bits.INT0IE = 1; // enable INT0 interrupt!
    
    TRISBbits.TRISB0 = 0; //set D3 as output
    TRISBbits.TRISB1 = 0; //set D4 as output
    TRISEbits.TRISE8 = 1; // set button S5 as input
    
    LATBbits.LATB0 = 1; //set led
    tmr_setup_period(TIMER1, 500);
    LATBbits.LATB0 = 0; //set led
    while(1){        
        tmr_wait_period(TIMER1);
        LATBbits.LATB0 = !LATBbits.LATB0; //invert led        
    }
}

int main() {
    
    //ex1();
    ex2();

}

void __attribute__ ((__interrupt__, __auto_psv__ )) _T2Interrupt
() {
    IFS0bits.T2IF = 0; // reset interrupt flag
    
    LATBbits.LATB1 = !LATBbits.LATB1; //invert led
}

void __attribute__ ((__interrupt__, __auto_psv__ )) _INT0Interrupt
() {
    IFS0bits.INT0IF = 0; // reset interrupt flag
    tmr_setup_period(TIMER3, 10);
    IEC0bits.INT0IE = 0; //disable interrupt
    IEC0bits.T3IE = 1; //enable interrupt
}

void __attribute__ ((__interrupt__, __auto_psv__ )) _T3Interrupt
() {
    IFS0bits.T3IF = 0; // reset interrupt flag
    if(PORTEbits.RE8==1)
        LATBbits.LATB1 = !LATBbits.LATB1; //invert led
    IEC0bits.INT0IE = 1; //enable interrupt
    T3CONbits.TON = 0; // stops the timer!
    
}

void tmr_setup_period(int timer, int ms){
    
    int pr = (((7372800/4)/256)/(1000.0/ms)); // (7.3728 MHz / 4) / 64 / (1000/ms)
    
    switch (timer){
        case 1:
            TMR1 = 0;
            PR1 = pr;
            T1CONbits.TCKPS0=1; // prescaler
            T1CONbits.TCKPS1=1; // prescaler
            T1CONbits.TON = 1; // starts the timer!
            break;
        case 2:
            TMR2 = 0;
            PR2 = pr;
            T2CONbits.TCKPS0=1; // prescaler
            T2CONbits.TCKPS1=1; // prescaler
            T2CONbits.TON = 1; // starts the timer!
            break;
        case 3:
            TMR3 = 0;
            PR3 = pr;
            T3CONbits.TCKPS0=1; // prescaler
            T3CONbits.TCKPS1=1; // prescaler
            T3CONbits.TON = 1; // starts the timer!
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
        case 3:
            while(IFS0bits.T3IF ==0){
            }
            IFS0bits.T3IF =0; //reset flag
            break;
        default:
            break;
    }
}

