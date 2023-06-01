/*
 * File:   Prueba_Serial.c
 * Author: schwe
 *
 * Created on 29 de mayo de 2023, 06:09 PM
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
#include <stdio.h>

//Definición de variables
#define _XTAL_FREQ 4000000
uint8_t indicador;

//Prototipos
void setup(void);
void main(void);

//Interrupcion
void __interrupt() isr(void) {
    if (PIR1bits.RCIF){                                     //Si se puede recibir valor de UART
        indicador = RCREG;                                  //Guardar valor en indicador
        PORTA = indicador;
        if (indicador == 0B00110001){                       //Si se recibio 1 serial
            PORTDbits.RD1 = 1;
        }
        else if (indicador == 0B00110010){                  //Si se recibio 2 serial
            PORTDbits.RD2 = 1;
        }
        else if (indicador == 0B00110011){                  //Si se recibio 3 serial
            PORTDbits.RD3 = 1;
        }
        else if (indicador == 0B00110100){                  //Si se recibio 4 serial
            PORTDbits.RD4 = 1;
        }
    }
    
    PIR1bits.RCIF = 0;
    return;
}

//Setup General
void setup(void){
    //Oscilador
    OSCCON = 0B01100000;                                    //Oscilador a 4Mhz
        
    //Interrupciones
    INTCON = 0B11000000;                                    //Int globales, PEIE  activadas
    PIE1 = 0B00100000;                                      //Int UART activada
    PIR1 = 0B00000000;

    //UART
    TXSTA = 0B00000100;
    BAUDCTL = 0B00001000;
    RCSTA = 0B10010000;
    SPBRG = 103;
    SPBRGH = 0;
    
    //Entradas y salidas
    ANSEL = 0;
    ANSELH = 0;
    TRISA = 0B00000000;
    TRISB = 0B00000000;
    TRISD = 0B00000000;
    
    //Valores iniciales de variables y puertos
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    
    return;
}

//Loop
void main(void) {
    setup();
    while(1){
        __delay_ms(100);
    }
}