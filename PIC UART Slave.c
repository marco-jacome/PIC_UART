/*
 * File:   
 * Author: Marco Jacome
 *
 * Created on August 2, 2022, 1:47 PM
 */


#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits->Oscillator not enabled
#pragma config RSTOSC = HFINT1    // Power-up default value for COSC bits->HFINTOSC
#pragma config CLKOUTEN = OFF    // Clock Out Enable bit->CLKOUT function is disabled; I/O or oscillator function on OSC2
#pragma config CSWEN = ON    // Clock Switch Enable bit->Writing to NOSC and NDIV is allowed
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config MCLRE = ON    // Master Clear Enable bit->MCLR/VPP pin function is MCLR; Weak pull-up enabled
#pragma config PWRTE = OFF    // Power-up Timer Enable bit->PWRT disabled
#pragma config WDTE = OFF    // Watchdog Timer Enable bits->WDT disabled; SWDTEN is ignored
#pragma config LPBOREN = OFF    // Low-power BOR enable bit->ULPBOR disabled
#pragma config BOREN = ON    // Brown-out Reset Enable bits->Brown-out Reset enabled, SBOREN bit ignored
#pragma config BORV = LOW    // Brown-out Reset Voltage selection bit->Brown-out voltage (Vbor) set to 2.45V
#pragma config PPS1WAY = ON    // PPSLOCK bit One-Way Set Enable bit->The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable bit->Stack Overflow or Underflow will cause a Reset
#pragma config DEBUG = OFF    // Debugger enable bit->Background debugger disabled

// CONFIG3
#pragma config WRT = OFF    // User NVM self-write protection bits->Write protection off
#pragma config LVP = ON    // Low Voltage Programming Enable bit->Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.

// CONFIG4
#pragma config CP = OFF    // User NVM Program Memory Code Protection bit->User NVM code protection disabled
#pragma config CPD = OFF    // Data NVM Memory Code Protection bit->Data NVM code protection disabled

#include <xc.h>
#include <pic16f18344.h>
#include <stdint.h>


//--------------------------------
// Global Variables
#define _XTAL_FREQ 8000000


//--------------------------------
// Functions Declarations
void UART_RX_Init(void);
void UART_Read(void);


//--------------------------------
// Main Routine
void main(void)
{

// Clock Source Configuration
    OSCCON1 = 0x60; // HFINTOSC   
    OSCFRQ = 0x04;  // HFFRQ 8_MHz;


 // LED Indicator Output Port Configuration
  TRISB  = 0x20; // Output Pins RB4, RB6, RB7. Input Pin RB5 for RX
  ANSELB = 0x00; // Digital input/output


 // PPS Cofiguration for USART 
	RB5PPS = 0b01101; // RB5 Input PPS for RX
  PPSLOCKbits.PPSLOCKED = 0; // PPS is locked. PPS selections can not be changed.

 void UART_RX_Init(void);

  while(1)
  {
   void UART_Read(void);
  
      PORTBbits.RB6 = (0b0000100 & RC1REG) ? 1 : 0; // Set bits for MSB
      PORTBbits.RB7 = (0b0000010 & RC1REG) ? 1 : 0; // Set bits 
      PORTBbits.RB4 = (0b0000001 & RC1REG) ? 1 : 0; // Set bits for LSB 
   }
  return;
  
}



//--------------------------------
// Functions Definitions

void UART_RX_Init(void)
{
	
// Transmit Status and Control Register Configuration
//--------------------------------------------------
 TX1STAbits.SYNC = 0; // Set for Asynchronous Mode
 TX1STAbits.BRGH = 1; // Set for High-Speed Baud Rate

// Receive Status and Control Register Configuration
//--------------------------------------------------
  RC1STAbits.SPEN = 1; // Serial Port Enabled
	RC1STAbits.RX9 = 0; // Set to 8-bit Reception
	RC1STAbits.CREN = 1; // Enable Continous Receive until CREN bit is cleared



// Baud Rate and Control Register
//--------------------------------------------------
// We will use 12 to set Baud rate period in 8-bit mode 
 BAUD1CONbits.BRG16 = 1; // 16-bit Baud Rate Generator is used to reduce
												// baud rate error.
 BAUD1CONbits.SCKP = 0; // Idle state for transmit (TX) is a high level
 
 // Baud Rate Generator Registor configured for Baud Rate 9600 @ 8MHz Fosc
//-----------------------------------------------------------------------
 SPBRGL = 12; // Set The Baud Rate To Be 9600 bps
 SPBRGH = 0;
  }

//Function to read one byte of data from UART
void UART_Read(void)   
{
    if(RC1STAbits.OERR) // check for Error 
    {
       RC1STAbits.CREN = 0; // Enable Continous Receive until CREN bit is cleared
       RC1STAbits.CREN = 1; // Enable Continous Receive until CREN bit is cleared	
    }

    while(!RCIF);  // hold the program till RX buffer is free
    


}