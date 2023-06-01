/*
 * File:   newmain.c
 * Author: schwe
 *
 * Created on 30 de mayo de 2023, 05:08 PM
 */

//CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT                        //Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF                                   //Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF                                  //Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF                                  //RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF                                     //Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                                    //Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF                                  //Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF                                   //Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF                                  //Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                                    //Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V                               //Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF                                    //Flash Program Memory Self Write Enable bits (Write protection off)

//Librerías
#include <xc.h>
#include <pic16f887.h>

//Definición de variables
#define _XTAL_FREQ 4000000
uint8_t PWM1;
uint8_t contPWM1;
uint8_t PWM2;
uint8_t contPWM2;

//Prototipos
void setup(void);
void main(void);
int mapear (int valor, int min, int max, int nmin, int nmax);

//Interrupcion
void __interrupt() isr(void) {
    if (PIR1bits.ADIF){
        if (ADCON0bits.CHS == 0B0010){
            PWM1 = mapear(ADRESH, 0, 255, 1, 5);
        }
        else if (ADCON0bits.CHS == 0B0011){
            PWM2 = mapear(ADRESH, 0, 255, 1, 5);
        }
    }
    
    if (INTCONbits.T0IF){
        if (contPWM1 < PWM1){
            PORTCbits.RC0 = 1;
        }
        else{
            PORTCbits.RC0 = 0;
        }
        contPWM1++;
        if (contPWM1 > 9){
            contPWM1 = 0;
        }
        TMR0 = 131;
    }
    
    if (PIR1bits.TMR1IF){
        if (contPWM2 < PWM2){
            PORTCbits.RC3 = 1;
        }
        else{
            PORTCbits.RC3 = 0;
        }
        contPWM2++;
        if (contPWM2 > 9){
            contPWM2 = 0;
        }
        TMR1H = 255;                                
        TMR1L = 143;
    }
    
    INTCONbits.T0IF = 0;
    PIR1bits.TMR1IF = 0;
    PIR1bits.ADIF = 0;                                      //Limpia la bandera de interrupción
    return;
}

//Setup General
void setup(void){
    //Oscilador
    OSCCON = 0B01100000;                                    //Oscilador a 4Mhz
        
    //Interrupciones
    INTCON = 0B11100000;                                    //Int globales, PEIE, TMR0 activadas
    PIE1 = 0B01000111;                                      //Int ADC, CCP1, TMR2, TMR1 activadas
    
    //ADC
    ADCON0 = 0B01001001;                                     //Fosc/8 (2us), AnCH2, ADC encendido 
    ADCON1 = 0B00000000;                                    //Voltajes de referencia y formato a la izquierda
    ANSEL = 0B00001111;                                     //Pines analógicos AN0, AN1, AN2, AN3
    ANSELH = 0;
    
    //TIMER0
    OPTION_REG = 0B00000001;                                //4 prescaler
    TMR0 = 131;                                             //0.5 ms
    
    //TIMER1
    T1CON = 0B00100001;                                     //4 prescaler
    TMR1H = 255;                                
    TMR1L = 143;                                            //0.5 ms

    //Entradas y salidas
    TRISA = 0B00001111;                                     //A0, A1, A2, A3 como entradas
    TRISB = 0B00000000;                                     //Salidas
    TRISC = 0B00000000;
    TRISD = 0B00000000;
    
    //Valores iniciales de variables y puertos
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PWM1 = 0;
    contPWM1 = 0;
    PWM2 = 0;
    contPWM2 = 0;

    return;
}

//Loop
void main(void) {
    setup();
    while(1){
        if (ADCON0bits.GO == 0){
            if (ADCON0bits.CHS == 0B0010){
                ADCON0bits.CHS = 0B0011;
            }
            else if (ADCON0bits.CHS == 0B0011){
                ADCON0bits.CHS = 0B0010;
            }
            __delay_ms(100);
            ADCON0bits.GO = 1;
        }
    }
}

//Funciones
int mapear (int valor, int min, int max, int nmin, int nmax){
    int nvalor = nmin + (valor - min) * (long)(nmax - nmin) / (max - min);
    return nvalor;
}
