/*
 * File:   ex4.c
 * Author: Marco
 *
 * Created on 18 ottobre 2022, 11.34
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
    tmr_wait_ms(TIMER1,1000); //wait 1000ms as LCD standard
    
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6; // 1:2 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
    
    char string [] = "HELLO WORLD";
    
    for(int i=0;string[i]!='\0';i++)
    {
        while(SPI1STATbits.SPITBF == 1); // wait until not full
        SPI1BUF = string[i];
    }
    while(1);

}

void ex2(){
    int elapsed_seconds = 0;
    tmr_setup_period(TIMER1, 1000);
    char buf[4];
    
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6; // 1:2 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
    
    while(1){
        tmr_wait_period(TIMER1);
        
        while(SPI1STATbits.SPITBF == 1); // wait until not full
        SPI1BUF=0x80;
        
        sprintf(buf,"%d",elapsed_seconds);
        
        for(int i=0;buf[i]!='\0';i++)
        {
            while(SPI1STATbits.SPITBF == 1); // wait until not full
            SPI1BUF = buf[i];
        }
        elapsed_seconds++;
         
        
    }

}

int elapsed_seconds = 0;

void ex3(){
    tmr_setup_period(TIMER1, 1000);
    char buf[4];
    
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6; // 1:2 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
    
    IEC0bits.INT0IE = 1; // enable INT0 interrupt!
    
    while(1){
        tmr_wait_period(TIMER1);
        
        while(SPI1STATbits.SPITBF == 1); // wait until not full
        SPI1BUF=0x80;
        
        sprintf(buf,"%d",elapsed_seconds);
        
        for(int i=0;buf[i]!='\0';i++)
        {
            while(SPI1STATbits.SPITBF == 1); // wait until not full
            SPI1BUF = buf[i];
        }
        elapsed_seconds++;
         
        
    }

}

void __attribute__ ((__interrupt__, __auto_psv__ )) _INT0Interrupt
() {
    IFS0bits.INT0IF = 0; // reset interrupt flag
    elapsed_seconds=0;
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
