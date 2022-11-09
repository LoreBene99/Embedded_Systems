/*
 * File:   ex1.c
 * Author: Marco
 *
 * Created on 27 settembre 2022, 11.23
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

void ex1(){
    TRISBbits.TRISB0 = 0; //set D3 as output
    LATBbits.LATB0 = 1;
    while(1);
  
}

void ex2(){
    TRISBbits.TRISB0 = 0; //set D3 as output
    TRISEbits.TRISE8 = 1; // set E8 as input
    int E8value = 0; //value of button
    
    while (1){
        E8value = PORTEbits.RE8;
        if(E8value==1) LATBbits.LATB0 = 0;
        else LATBbits.LATB0 = 1;
    }
    
}

void ex3(){
    TRISBbits.TRISB0 = 0; //set D3 as output
    TRISEbits.TRISE8 = 1; // set E8 as input
    int E8value = 0; //value of button  
    int E8valuePrev = 0; //value of button precedent
    int th = 1000;
    int count = 0;
    while (1)
    {
        E8value = PORTEbits.RE8;
        if (E8value==0) count++;
        if(E8valuePrev==0&&E8value==1) {
            if (count>=th)
                LATBbits.LATB0 = !LATBbits.LATB0;
            count=0;
            
        }
        E8valuePrev=E8value;
    }
}

int main(void) {
    
//    ex1();
//    ex2();
    ex3();
    return 0;
}

