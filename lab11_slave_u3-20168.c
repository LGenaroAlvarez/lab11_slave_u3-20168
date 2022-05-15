/*
 * File:   lab11_slave_u3-20168.c
 * Author: luisg
 *
 * Created on May 14, 2022, 10:28 AM
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF           // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF          // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF          // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF             // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF            // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF          // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF           // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF          // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF            // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V       // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF            // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

//DEFINICION DE FRECUENCIA PARA DELAY
#define _XTAL_FREQ 1000000          // FRECUENCIA PARA DELAYS (1MHz)


//DEFINICION DE ALIAS PARA PINES
#define incB PORTBbits.RB0
#define decB PORTBbits.RB1

//VARIABLES GLOBALES
uint8_t cont_u3 = 0;                // VARIABLE DE CONTADOR CON PUSHBUTTONS
uint8_t A = 1;                      // VARIABLES PARA ANTIRREBOTE
uint8_t B = 1;
uint8_t C = 1;
uint8_t D = 1;

//PROTO FUNCIONES
void setup(void);                   // FUNCION DE SETUP

//INTERRUPCIONES
void __interrupt() isr(void){
    //----------------------------------SLAVE---------------------------------------------
    if (PIR1bits.SSPIF){            // REVISAR INTERRUPCION DE RECEPCION DE DATOS
        SSPBUF = cont_u3;           // CARGAR VALOR DE CONTADOR A LA SALIDA DEL PIC
        PIR1bits.SSPIF = 0;         // LIMPIEZA DE BANDERA DE INTERRUPCION DE SPI
    }
    if(INTCONbits.RBIF){
        A = 1;
        C = 1;
        if (!incB){                 // REVISAR SI RB0 FUE PRESIONADO
            A = 0;
            B = A;
        }
            if (B != A){            // REVISAR SI RB0 FUE LIBERADO
            B = A;
            cont_u3++;              // INCREMENTAR PORTA
        }
        else if(!decB){             // REVISAR SI RB1 FUE PRESIONADO
            C = 0;
            D = C;
        }
        if (D != C){                // REVISAR SI RB1 FUE LIBERADO
            D = C;
            cont_u3--;              // DECREMENTAR PORTA
        }
        INTCONbits.RBIF = 0;    // LIMPIAR BANDERA DE INTERRUPCION EN PORTB
    }
    //------------------------------------------------------------------------------------
    return;
}

void main(void) {
    //EJECUCION CONFIG
    setup();

    while(1){
                                    // 
    }
    return;
}

//CONFIGURACION PRINCIPAL
void setup(void){
    ANSEL = 0;                      // I/O DIGITALES
    ANSELH = 0;                     // I/O DIGITALES

    TRISA = 0b00100000;             // RA5 COMO ENTRADA
    PORTA = 0;                      // LIMPIEZA DE PORTA


    //OSCCONFIC
    OSCCONbits.IRCF = 0b0100;       // FRECUENCIA DE OSCILADOR INTERNO (1MHz)
    OSCCONbits.SCS  = 1;            // RELOJ INTERNO

    //-------------------EN CASO DE RA0 = 0, PIC EN MODO ESCLAVO--------------------------

    TRISC = 0b00011000;         // ENTRADA DE DATOS Y SINCRONIZADOR DE RELOJ COMO ENTRADA, SALIDA DE DATOS COMO SALIDA
    PORTC = 0;                  // LIMPIEZA DE PORTD

    //SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;   // SS HABILITADO, SPI EN MODO ESCLAVO
    SSPCONbits.CKP = 0;         // RELOJ INACTIVO EN 0
    SSPCONbits.SSPEN = 1;       // HABILITACION DE PINES DE SPI
    //SSPSTAT <7:6>
    SSPSTATbits.CKE = 1;        // ENVIO DE DATO EN FLANCO DE SUBIDA
    SSPSTATbits.SMP = 0;        // DATO AL FINAL DE PULSO DE RELOJ (EN 0 DEBIDO A MODO ESCLAVO)

    //CONFIG PUSHBUTTONS EN PORTB
    TRISBbits.TRISB0 = 1;       // RB0 COMO INPUT
    TRISBbits.TRISB1 = 1;       // RB1 COMO INPUT
    PORTB = 0;                  // LIMPIEZA DEL PUERTOB
    OPTION_REGbits.nRBPU = 0;   // HABILITAR WEAK PULLUP EN PUERTO B
    WPUBbits.WPUB0 = 1;         // HABILITAR RESISTENCIA EN RB0
    WPUBbits.WPUB1 = 1;         // HABILITAR RESISTENCIA EN RB1

    //CONFIG DE INTERRUPCIONES
    PIR1bits.SSPIF = 0;         // LIMPIAR BANDERA DE INTERRUPCIONES DE SP1
    PIE1bits.SSPIE = 1;         // ACTIVAR INTERRUPCIONES DE SPI
    INTCONbits.GIE = 1;         // ACTIVAR INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 1;        // ACTIVAR INTERRUPCIONES DE PERIFERICOS
    INTCONbits.RBIE = 1;        // HABILITAR INTERRUPCIONES EN PORTB
    IOCBbits.IOCB0 = 1;         // HABILITAR INTERRUPCION EN CAMBIO PARA RB0
    IOCBbits.IOCB1 = 1;         // HABILITAR INTERRUPCION EN CAMBIO PARA RB1
    INTCONbits.RBIF = 0;        // LIMPIAR BANDERA DE INTERRUPCION EN PORTB
}