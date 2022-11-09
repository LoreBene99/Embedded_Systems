/*
 * File:   ex2.c
 * Author: Marco
 *
 * Created on 6 ottobre 2022, 10.28
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
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer,int ms);
void wait_and_check_btn(int timer);

void ex1(){
    // initialization code
    TRISBbits.TRISB0 = 0; //set D3 as output
    tmr_setup_period(TIMER1, 500);
    while (1) {
        // code to blink LED
        LATBbits.LATB0 = !LATBbits.LATB0; //invert led
        tmr_wait_period(TIMER1);
    }
}

void ex2(){
    // initialization code
    TRISBbits.TRISB0 = 0; //set D3 as output
    LATBbits.LATB0 = 1; //set led
    tmr_wait_ms(TIMER2, 1000);
    LATBbits.LATB0 = 0; //set led
    tmr_wait_ms(TIMER2, 5000);
    LATBbits.LATB0 = 1; //set led
    tmr_wait_ms(TIMER2, 500);
    LATBbits.LATB0 = 0; //set led
}

int number = 1; //number of blinks
int E8value = 0; //value of button  
int E8valuePrev = 0; //value of button precedent
int pressTimes = 0; //number of time slots when the button is pressed

int main() {

    //ex2();
    TRISBbits.TRISB0 = 0; //set D3 as output
    TRISEbits.TRISE8 = 1; // set button S5 as input
    
    
    LATBbits.LATB0=0; //start with led off
    tmr_setup_period(TIMER1, 100);
    while(1){
    
        for(int i=0;i<10;i++)
        {
            if(i<number){
                LATBbits.LATB0 = 1; //led on
                wait_and_check_btn(TIMER1);
                LATBbits.LATB0 = 0; //led on
            }
            
            wait_and_check_btn(TIMER1);
        }
    
    }
    
}

void tmr_setup_period(int timer, int ms){
    
    int pr = (((7372800/4)/256)/(1000.0/ms)); // (7.3728 MHz / 4) / 64 / (1000/ms)
    
    switch (timer){
        case 1:
            TMR1 = 0;
            PR1 = pr;
            T1CONbits.TCKPS0=1; // prescaler
            T1CONbits.TCKPS1=1; // prescaler
            IFS0bits.T1IF = 0; // set interrupt to zero
            T1CONbits.TON = 1; // starts the timer!
            break;
        case 2:
            TMR2 = 0;
            PR2 = pr;
            T2CONbits.TCKPS0=1; // prescaler
            T2CONbits.TCKPS1=1; // prescaler
            IFS0bits.T2IF = 0; //set interrupt to zero
            T2CONbits.TON = 1; // starts the timer!
            break;
        case 3:
            TMR3 = 0;
            PR3 = pr;
            T2CONbits.TCKPS0=1; // prescaler
            T2CONbits.TCKPS1=1; // prescaler
            IFS0bits.T3IF = 0; //set interrupt to zero
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

void wait_and_check_btn(int timer){

    E8valuePrev = PORTEbits.RE8;
            
    tmr_wait_period(timer);
            
    E8value = PORTEbits.RE8;
            
    if(E8valuePrev==0&&E8value==0) pressTimes++;
    else if(E8valuePrev==0&&E8value==1){
                
        number=1+(number)%3;
                
        if(pressTimes>=30)
            number=0;
        
        pressTimes=0;
                
    }
}



