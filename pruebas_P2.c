/*
 * File:   proyecto_2.c
 * Author: Carlos Daniel Valdez Coreas
 * Descripción: Control de 4 servomotores. 2 se controlan con el módulo PWM
 * en CCP1 y CCP2, y los otros dos se controlan con un PWM manual utilizando
 * TMR0. El programa tiene un modo manual, en el que los servos se controlan
 * utilizando potenciómetros, y tiene un modo EEPROM en el que se pueden
 * reproducir posiciones guardadas anteriormente. También posee un modo serial
 * para poder ser controlado desde la computadora mediante UART.
 * Created on 1 de mayo del 2023
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1

#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdio.h>         // for sprintf



/*
 * Constantes
 */
#define _XTAL_FREQ 8000000
#define _tmr0_n 255 //TMR0 load value


/*
 * Variables
 */
uint8_t TMR0_count; //TMR0 counter
uint8_t pulse_width; //Manual PWM pulse width
uint8_t pulse_width2; //Manual PWM pulse width



uint8_t ADH1, ADH2,ADH3,ADH4,ADL1,ADL2 ;             //Servomotor 1, Servomotor 2, motor 1, motor 2
unsigned char EPROM_sm1_H   =   0x01; 
unsigned char EPROM_sm1_L   =   0x02;
unsigned char EPROM_sm2_H   =   0x03;
unsigned char EPROM_sm2_L   =   0x04; 
unsigned char EPROM_m1      =   0x05; 
unsigned char EPROM_m2      =   0x06;
uint8_t c1_L,c2_L,c3,c4,c1_h,c2_h;



unsigned char contador =1;
unsigned char UA_1; // Car?cter a enviar
char text[5];
const char message[] = "PIC16F887 microcontroller UART example" ;







void incrementarContador()
{
    if (PORTAbits.RA4 == 1) // Verifica si el bot?n est? presionado
    {
        
        __delay_ms(50); // Espera para evitar rebotes
        if (PORTAbits.RA4 == 0) // Verifica nuevamente si el bot?n est? presionado
        {
            PORTD =0b00000011;
            contador++; // Incrementa el contador
            if (contador > 3)
            {
                contador = 1; // Reinicia el contador si alcanza el valor m?ximo de 4
            }
        }
    }
}


// Prototipos de funciones

void UART_Init(const uint32_t baud_rate)
{
  int16_t n = ( _XTAL_FREQ / (16 * baud_rate) ) - 1;
  
  if (n < 0)
    n = 0;
 
  if (n > 255)  // low speed
  {
    n = ( _XTAL_FREQ / (64 * baud_rate) ) - 1;
    if (n > 255)
      n = 255;
    SPBRG = n;
    TXSTA = 0x20;  // transmit enabled, low speed mode
  }
 
  else   // high speed
  {
    SPBRG = n;
    TXSTA = 0x24;  // transmit enabled, high speed mode
  }
 
  RCSTA = 0x90;  // serial port enabled, continues receive enabled
 
}

__bit UART_Data_Ready()
{
  return RCIF;  // return RCIF bit (register PIR1, bit 5)
}

uint8_t UART_GetC()
{
  while (RCIF == 0) ;  // wait for data receive
  if (OERR)  // if there is overrun error
  {  // clear overrun error bit
    CREN = 0;
    CREN = 1;
  }
  return RCREG;        // read from EUSART receive data register
}
 
void UART_PutC(const char data)
{
  while (TRMT == 0);  // wait for transmit shift register to be empty
  TXREG = data;       // update EUSART transmit data register
}

void UART_Print(const char *data)
{
  uint8_t i = 0;
  while (data[i] != '\0')
    UART_PutC (data[i++]);
}

unsigned char readEEPROM(unsigned char  address)
{
   
  EEADR = address; //Address to be read
  EECON1bits.EEPGD = 0;//Selecting EEPROM Data Memory
  EECON1bits.RD = 1; //Initialise read cycle
  return EEDATA; //Returning data
}

void writeEEPROM(unsigned char  address, unsigned char  dataEE)
{ 
  
  EEADR = address; //Address to write
  EEDATA = dataEE; //Data to write
  EECON1bits.EEPGD = 0; //Selecting EEPROM Data Memory
  EECON1bits.WREN = 1; //Enable writing of EEPROM
  
  INTCONbits.GIE=0; //Diables the interrupt
  EECON2=0x55; //Required sequence for write to internal EEPROM
  EECON2=0xAA; //Required sequence for write to internal EEPROM
  EECON1bits.WR = 1; //Initialise write cycle
  
  EECON1bits.WREN = 0; //To disable write
  INTCONbits.GIE=1; //Enable the interrupt
  
  while(PIR2bits.EEIF == 0)//Checking for complition of write operation
  {
    NOP(); //do nothing
  }
  PIR2bits.EEIF = 0; //Clearing EEIF bit
}

void setup(void);

/*
 * Interrupciones
 */

void __interrupt() isr (void)
{
    if(PIR1bits.ADIF)     //ADC
    {   
        if (ADCON0bits.CHS == 0)
        {
            // interrupcion
            ADH1   = ADRESH;
            ADL1  = ADRESL; 
            CCPR1L = (ADRESH>>0)+70;
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = (ADRESL>>7);
            
        }
        else if (ADCON0bits.CHS == 1)
        {
            // interrupcion
            ADH2   = ADRESH;
            ADL2  = ADRESL;
            CCPR2L = (ADRESH>>0)+70;
            CCP2CONbits.DC2B1 = ADRESH & 0b01;
            CCP2CONbits.DC2B0 = (ADRESL>>7);
            
        }
        else if (ADCON0bits.CHS == 2){
            ADH3   = ADRESH;
            pulse_width = ADRESH;
           
        }
        else if (ADCON0bits.CHS == 3){
            ADH4   = ADRESH;
            pulse_width2 = ADRESH;
            
        }
        
        PIR1bits.ADIF = 0;
    }
    //Interrupcion TMR0
    if(T0IF == 1){  //TMR0
        TMR0 = _tmr0_n;
        TMR0_count = TMR0_count+45;
        T0IF = 0;
    }
// Interrupciones del Puerto B
    if (INTCONbits.RBIF ){
        PORTD =0;
        int x = PORTB;
        INTCONbits.RBIF = 0;
        
        //Interrupcion Boton1
        if (PORTBbits.RB1 == 1){
            //Eprom del servo1
            
            writeEEPROM(EPROM_sm1_H, ADH1);
            c1_h    = readEEPROM(EPROM_sm1_H);
            
            writeEEPROM(EPROM_sm1_L, ADL1);
            c1_L    = readEEPROM(EPROM_sm1_H);

            //Eprom del servo2

            writeEEPROM(EPROM_sm2_H, ADH2);
            c2_h    = readEEPROM(EPROM_sm2_H); 
            writeEEPROM(EPROM_sm2_L, ADL2);
            c2_L    = readEEPROM(EPROM_sm2_L);

            //Eprom motor Dc1

            writeEEPROM(EPROM_m1, ADH3); 
            c3    = readEEPROM(EPROM_m1);


                 //Eprom motor Dc2

            writeEEPROM(EPROM_m2, ADH4);
            c4    = readEEPROM(EPROM_m2);
            
            INTCONbits.RBIF = 0;
            PORTD++;//= 0xFF;
            INTCONbits.GIE=1; 
            
            
        }
    
        //Interrupcion Boton2

        if (PORTBbits.RB2 == 1){
            
            
        }
    }
    
         
}




/*
 * Main
 */

void main (void)
{
    setup();                 //Configuracion Inicial
    ADCON0bits.GO = 1;       //Inicia la conversion ADC
    
    while(1){
        incrementarContador();
        if (contador == 1){
            PORTD = 0b00000001;
            if (ADCON0bits.GO == 0)
            {
                if (ADCON0bits.CHS == 0)
                    ADCON0bits.CHS = 1;
                else if(ADCON0bits.CHS == 1)
                    ADCON0bits.CHS = 2;
                else if(ADCON0bits.CHS == 2)
                    ADCON0bits.CHS = 3;
                else if(ADCON0bits.CHS == 3)
                    ADCON0bits.CHS = 0;
                __delay_us(50);
                ADCON0bits.GO = 1;
            }
        
            if (TMR0_count > pulse_width){
                PORTCbits.RC3 = 0;
            }    
            else
                PORTCbits.RC3 = 1;

            if (TMR0_count > pulse_width2){
                PORTCbits.RC4 = 0;
            }
            else
                PORTCbits.RC4 = 1; 
        }
        if (contador == 2){
            PORTD = 0b00000010;
            //SERVOMOTOR POSICION 1
            
            
            CCPR1L = (c1_h>>1)+124;
            CCP1CONbits.DC1B1 = c1_h & 0b01;
            CCP1CONbits.DC1B0 = (c1_L>>7);

            CCPR2L = (c2_h>>1)+124;
            CCP2CONbits.DC2B1 = c2_h & 0b01;
            CCP2CONbits.DC2B0 = (c2_L>>7);
            
            //PIR1bits.ADIF = 0;
            INTCONbits.RBIF = 0;
            
            if (TMR0_count > c3){
                PORTCbits.RC3 = 0;
            }    
            else
                PORTCbits.RC3 = 1;

            if (TMR0_count > c4){
                PORTCbits.RC4 = 0;
            }
            else
                PORTCbits.RC4 = 1; 
            
        }
        if (contador == 3){
            PORTD = 0b00000100;
            OSCCON = 0x70;    // set internal oscillator to 8MHz
 
            UART_Init(9600);  // initialize UART module with 9600 baud

            __delay_ms(2000);  // wait 2 seconds

            UART_Print("MODO UART \r\n");  // UART print

            __delay_ms(1000);  // wait 1 second

            UART_Print("\r\n");  // start new line
            while(1){
                if ( UART_Data_Ready() ){  // if a character available
                    
                    uint8_t c = UART_GetC();  // read from UART and store in 'c'
                    UART_PutC(c);  // send 'c' via UART (return the received character back)

                    if(c == 'A' || c == 'a'){
                        
                        PORTD = 0b00001100;
                        CCPR1L = (255>>1)+124;
                        CCP1CONbits.DC1B1 = 255 & 0b01;
                        CCP1CONbits.DC1B0 = (255>>7);
                        
                        
                        CCPR2L = (0>>1)+124;
                        CCP2CONbits.DC2B1 = 0 & 0b01;
                        CCP2CONbits.DC2B0 = (0>>7);
                        __delay_us(2000);
                    }
                    else if(c == 'b' || c == 'B')
                    {
                        PORTD = 0b00000110;
                        CCPR1L = (0>>1)+124;
                        CCP1CONbits.DC1B1 = 255 & 0b01;
                        CCP1CONbits.DC1B0 = (255>>7);
                        
                        
                        CCPR2L = (25>>1)+124;
                        CCP2CONbits.DC2B1 = 0 & 0b01;
                        CCP2CONbits.DC2B0 = (0>>7);
                        __delay_us(2000);
                    }
                    if(c == 'C' || c == 'c'){
                        
                        PORTD = 0b00001100;
                        CCPR1L = (255>>1)+124;
                        CCP1CONbits.DC1B1 = 255 & 0b01;
                        CCP1CONbits.DC1B0 = (255>>7);
                        
                        
                        CCPR2L = (0>>1)+124;
                        CCP2CONbits.DC2B1 = 0 & 0b01;
                        CCP2CONbits.DC2B0 = (0>>7);
                        __delay_us(2000);
                    }
                }
                if (PORTAbits.RA4 == 1) // Verifica si el bot?n est? presionado
                {
                    contador++; // Incrementa el contador
                    break;
                }
            }  
            
        }
            
    }
}

//Funciones


void setup(void)
{
    // configuraci?n de entradas y salidas
    ANSEL = 0b00001111;
    ANSELH = 0;
    
    TRISA = 0xFF;
    
    
    TRISCbits.TRISC3 = 0; //RC3 Output
    TRISCbits.TRISC4 = 0; //RC4 Output
    
    TRISD = 0;
    
    PORTD = 0b00000000;
    
    
    
    // Configuraci?n del oscilador
    OSCCONbits.IRCF = 0b0111; //8MHz
    OSCCONbits.SCS = 1;
    
    //configuraci?n del TMR0 
    OPTION_REGbits.PS = 0b100;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.T0CS = 0;    
    TMR0 = _tmr0_n;
    
    // Configuraci?n del ADC
    ADCON1bits.ADFM = 0;        //justificaci?n a la izquierda
    ADCON1bits.VCFG0 = 0;       //Vref en VSS y VDD
    ADCON1bits.VCFG1 = 0;
    
    ADCON0bits.ADCS = 0b10;     //FOSC/32
    ADCON0bits.CHS = 0;
    ADCON0bits.ADON = 1;
    __delay_us(50);
    
    // configuraci?n del PWM
    TRISCbits.TRISC2 = 1;       // RC2/CCP1 como entrada
    TRISCbits.TRISC1 = 1;       // RC1/CCP2 como entrada
    PR2 = 255;                  // configuraci?n del periodo
    
    
    CCP1CONbits.P1M = 0;        // Configuraci?n del modo PWM
    CCP1CONbits.CCP1M = 0b1100;
    
    CCP2CONbits.CCP2M = 0b1100;
    
    CCPR1L = 0x0f;              //ciclo de trabajo inicial
    CCP1CONbits.DC1B = 0;
    
    CCPR2L = 0x0f;              //ciclo de trabajo inicial
    CCP2CONbits.DC2B0 = 0;
    CCP2CONbits.DC2B1 = 0;
    
    PIR1bits.TMR2IF = 0;        //se toma la bandera
    T2CONbits.T2CKPS = 0b11;    //prescaler a 1:16
    T2CONbits.TMR2ON = 1; 
    
    while(PIR1bits.TMR2IF == 0);  //esperar un ciclo del TMR2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;       //Salida del PWM
    TRISCbits.TRISC1 = 0; 
    
    
    // Configuraci?n de las interrupciones
    
    //Configuraci?n para la interrupci?n del ADC
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    //Configuraci?n para la interrupci?n de los botones
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    //Configuraci?n para las interrupciones globales
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    IOCBbits.IOCB1 = 1;
    
    
    
    
    SPBRG = 12;
    SPBRGH = 0;
    BRGH = 0;   //TABLE 12-3: BAUD RATE FORMULAS
    BRG16 = 0;  //8bit boud rate generator
    //Asynchronous serial port
    SYNC = 0;
    SPEN = 1;
    //Enable transmission
    TXEN = 1;
    TXIF = 0;   //Clear flag
    //Enable reception
    CREN = 1;
    
    return;
}

