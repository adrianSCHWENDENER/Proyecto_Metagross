/*
 * File:   Metagross.c
 * Author: schwe
 *
 * Created on 31 de mayo de 2023, 08:29 PM
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
uint8_t indicador;
int CCP1;
int CCP2;
int antiR1;
int antiR2;
int READ;
int Modo_Libre;

//Prototipos
void setup(void);
void main(void);
int mapear(int valor, int min, int max, int nmin, int nmax);
void escribirEEPROM(uint8_t direccion, uint8_t data);
void escribirEEPROMH(uint8_t direccion, int DATABIG);
uint8_t leerEEPROM(uint8_t direccion);
int leerEEPROMH(uint8_t direccion);
void Ataque(int tipo);

//Interrupcion
void __interrupt() isr(void) {
    if (PIR1bits.ADIF){
        if (ADCON0bits.CHS == 0B0000){
            if (READ == 0 && Modo_Libre == 1){
                CCP1 = mapear(ADRESH, 0, 255, 124, 628);
            }
            else if(READ == 1 && Modo_Libre == 1){
                CCP1 = leerEEPROMH(0x04);
            }
            CCPR1L = (unsigned char)(CCP1>>2);        
            CCP1CONbits.DC1B0 = (CCP1 & 0B01);
            CCP1CONbits.DC1B1 = (CCP1 & 0B10)>>1;
        }
        else if (ADCON0bits.CHS == 0B0001){
            if (READ == 0 && Modo_Libre == 1){
                CCP2 = mapear(ADRESH, 0, 255, 124, 628);
            }
            else if (READ == 1 && Modo_Libre == 1){
                CCP2 = leerEEPROMH(0x05);
            }
            CCPR2L = (unsigned char)(CCP2>>2);        
            CCP2CONbits.DC2B0 = (CCP2 & 0B01);
            CCP2CONbits.DC2B1 = (CCP2 & 0B10)>>1;
        }
        else if (ADCON0bits.CHS == 0B0010){
            if (READ == 0 && Modo_Libre == 1){
                PWM1 = mapear(ADRESH, 0, 255, 1, 5);
            }
            else if (READ == 1 && Modo_Libre == 1){
                PWM1 = leerEEPROM(0x06);
            }
        }
        else if (ADCON0bits.CHS == 0B0011){
            if (READ == 0 && Modo_Libre == 1){
                PWM2 = mapear(ADRESH, 0, 255, 1, 5);
            }
            else if (READ == 1 && Modo_Libre == 1){
                PWM2 = leerEEPROM(0x07);
            }
        }
    }
    
    if (PIR1bits.TMR2IF){
        PIR1bits.TMR2IF = 0;
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
    
    if (PIR1bits.RCIF){                                     //Si se puede recibir valor de UART
        indicador = RCREG;                                  //Guardar valor en indicador
        if (indicador == 0B00110000){                       //Si se recibio 0 serial
            CCP1 = 375;
            CCP2 = 375;
            PWM1 = 3;
            PWM2 = 3;
            Modo_Libre = 0;
        }
        else if (indicador == 0B00110001){                  //Si se recibio 1 serial
            Modo_Libre = 1;
        }
        else if (indicador == 0B00110010){                  //Si se recibio 2 serial
            Ataque(1);
        }
        else if (indicador == 0B00110011){                  //Si se recibio 3 serial
            Ataque(2);
        }
    }
    INTCONbits.T0IF = 0;
    PIR1bits.RCIF = 0;
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
    PIE1 = 0B01100111;                                      //Int ADC, UART, CCP1, TMR2 match activadas
    
    //UART
    TXSTA = 0B00000100;
    BAUDCTL = 0B00001000;
    RCSTA = 0B10010000;
    SPBRG = 103;
    SPBRGH = 0;

    //ADC
    ADCON0 = 0B01000001;                                    //Fosc/8 (2us), AnCH0, ADC encendido 
    ADCON1 = 0B00000000;                                    //Voltajes de referencia y formato a la izquierda
    ANSEL = 0B00001111;                                     //Pines analógicos AN0, AN1, AN2, AN3
    ANSELH = 0;
    
    //PORTB
    TRISB = 0B00000011;
    WPUB = 0B00000011;
    
    //TIMER0
    OPTION_REG = 0B00000001;                                //4 prescaler
    TMR0 = 143;                                             //0.5 ms
    
    //TIMER1
    T1CON = 0B00100001;                                     //4 prescaler
    TMR1H = 255;                                
    TMR1L = 143;                                            //0.5 ms
    
    //PWM
    TRISCbits.TRISC2 = 1;
    PR2 = 250;                                              //Periodo de 4ms
    CCP1CON = 0B00001100;                                   //PWM mode, P1A, B, C y D normales, LSbs en 00
    CCPR1L = 0B00000000;                                    //Duty cicle inicial en 0
    CCP2CON = 0B00001100;                                   //PWM mode, LSbs en 00
    CCPR2L = 0B00000000;                                    //Duty cicle inicial en 0
    
    //TIMER2
    T2CON = 0B00000111;                                     //TMR2 activado, prescaler 16
    while (!PIR1bits.TMR2IF){                               //Esperar a que TMR2 haga overflow
    ;
    }
    PIR1 = 0;
    PIR2 = 0;
    
    //Entradas y salidas
    TRISA = 0B00001111;                                     // A0, A1, A2 y A3 como entradas
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC3 = 0;
    
    TRISD = 0;
    
    //Valores iniciales de variables y puertos
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    CCP1 = 375;
    CCP2 = 375;
    PWM1 = 3;
    contPWM1 = 0;
    PWM2 = 3;
    contPWM2 = 0;
    READ = 0;
    Modo_Libre = 0;
    return;
}

//Loop
void main(void) {
    setup();
    while(1){
        if (ADCON0bits.GO == 0){
            if (ADCON0bits.CHS == 0B0000){
                ADCON0bits.CHS = 0B0001;
            }
            else if (ADCON0bits.CHS == 0B0001){
                ADCON0bits.CHS = 0B0010;
            }
            else if (ADCON0bits.CHS == 0B0010){
                ADCON0bits.CHS = 0B0011;
            }
            else if (ADCON0bits.CHS == 0B0011){
                ADCON0bits.CHS = 0B0000;
            }
            __delay_ms(100);
            ADCON0bits.GO = 1;
        }
        
        if (!PORTBbits.RB0){                                //Si se apacho boton
            antiR1 = 1;
        }
        if (PORTBbits.RB0 && antiR1 == 1){                  //Si el boton esta apagado y el antirrebote encendido
            escribirEEPROMH(0x04, CCP1);                    //Escribir en la EEPROM
            escribirEEPROMH(0x05, CCP2);                    //Escribir en la EEPROM
            escribirEEPROM(0x06, PWM1);                     //Escribir en la EEPROM
            escribirEEPROM(0x07, PWM2);                     //Escribir en la EEPROM
            antiR1 = 0;                                     //Apagar antirrebote
        }

        if (!PORTBbits.RB1){
            antiR2 = 1;
        }
        if (PORTBbits.RB1 && antiR2 == 1){                   //Si el boton esta apagado y el antirrebote encendido
            if (READ == 0){
                READ = 1;
            }
            else{
                READ = 0;
            }
            antiR2 = 0;                                      //Apagar antirrebote
        }        
    }
}

//Funciones
int mapear (int valor, int min, int max, int nmin, int nmax){
    int nvalor = nmin + (valor - min) * (long)(nmax - nmin) / (max - min);
    return nvalor;
}

uint8_t leerEEPROM (uint8_t direccion){
    EEADR = direccion;                                      //Guardar direccion
    EECON1bits.EEPGD = 0;                                   //Ir a la direccion
    EECON1bits.RD = 1;                                      //Iniciar lectura
    return EEDATA;                                          //Regresar valor leido
}

void escribirEEPROM (uint8_t direccion, uint8_t data){
    if (!EECON1bits.WR){                                    //Si ya termino de escribir
        EEADR = direccion;                                  //Guardar direccion
        EEDATA = data;                                      //Guardar data
        EECON1bits.EEPGD = 0;                               //Ir a la direccion
        EECON1bits.WREN = 1;                                //Habilitar escritura
        INTCONbits.GIE = 0;                                 //Deshabilitar interrupciones
        EECON2 = 0x55;
        EECON2 = 0xAA;
        EECON1bits.WR = 1;                                  //Iniciar escritura
        INTCONbits.GIE = 1;                                 //Habilitar interrupciones
        EECON1bits.WREN = 0;                                //Deshabilitar escritura
    }
    return;
}

int leerEEPROMH (uint8_t direccion){
    int DATABIG;
    EEADR = direccion;                                      //Guardar direccion
    EECON1bits.EEPGD = 0;                                   //Ir a la direccion
    EECON1bits.RD = 1;                                      //Iniciar lectura
    DATABIG = (EEDATH << 8) | EEDATA;
    return DATABIG;                                          //Regresar valor leido
}

void escribirEEPROMH (uint8_t direccion, int DATABIG){
    if (!EECON1bits.WR){                                    //Si ya termino de escribir
        EEADR = direccion;                                  //Guardar direccion
        EEDATA = DATABIG & 0xFF;                            //Guardar data
        EEDATH = (DATABIG >> 8) & 0xFF;
        EECON1bits.EEPGD = 0;                               //Ir a la direccion
        EECON1bits.WREN = 1;                                //Habilitar escritura
        INTCONbits.GIE = 0;                                 //Deshabilitar interrupciones
        EECON2 = 0x55;
        EECON2 = 0xAA;
        EECON1bits.WR = 1;                                  //Iniciar escritura
        INTCONbits.GIE = 1;                                 //Habilitar interrupciones
        EECON1bits.WREN = 0;                                //Deshabilitar escritura
    }
    return;
}

void Ataque(int tipo){
    if (tipo == 1){
        CCP1 = 501;
        CCP2 = 501;
        PWM1 = 1;
        PWM2 = 1;
    }
    else if (tipo == 2){
        CCP1 = 438;
        CCP2 = 438;
        PWM1 = 3;
        PWM2 = 3;
    }
}
