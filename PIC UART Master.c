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
uint8_t Data = 0; // The Data Byte



//--------------------------------
// IO Pins Defines (Mappings)
#define Up RA0
#define Down RA1
#define Send RA2

//--------------------------------
// Functions Declarations
void UART_TX_Init(void);
void UART_Write(uint8_t);


//--------------------------------
// Main Routine
void main(void)
{

// Clock Source Configuration
    OSCCON1 = 0x60; // HFINTOSC   
    OSCFRQ = 0x04;  // HFFRQ 8_MHz;
  
 // Push-Button Input Port Configuration 
  TRISA = 0xFF;  // Input Port (3-Pins)
  ANSELA = 0x00; // Digital Output
// RA0 - UP 
// RA1 - DOWN
// RA2 - SEND

 // LED Indicator Output Port Configuration
  TRISB  = 0x00; // Output Pins RB4, RB6, RB7.
  ANSELB = 0x00; // Digital input/output
 // RB4 (LSB)
 // RB7 Bit   
 // RB6 (MSB)
  
 // Serial port Configuration
  TRISC = 0x00; // RC0, RC1 & RC2 Are Input Pins (Push Buttons)
  ANSELC = 0x00; // Digital Output

 // PPS Cofiguration for USART 
  RC4PPS =  0b10100; // RC4 source is TX/CK(1) 
  PPSLOCKbits.PPSLOCKED = 0; // PPS is locked. PPS selections can not be changed.



  // USART TX Configuraiton 
  UART_TX_Init(); // Initialize The UART in Master Mode @ 9600bps


  while(1)
  {
    if (Up) // Increment The Data Value
    {
      Data++;
      __delay_ms(250); // Delay for debouncing 
      PORTBbits.RB6 = (0b0000100 & Data) ? 1 : 0; // Set bits for MSB
      PORTBbits.RB7 = (0b0000010 & Data) ? 1 : 0; // Set bits 
      PORTBbits.RB4 = (0b0000001 & Data) ? 1 : 0; // Set bits for LSB 
    }

 
    if (Down) // Decrement the Data Value
    {
      Data--;
      __delay_ms(250); // Delay for debouncing 
      PORTBbits.RB6 = (0b0000100 & Data) ? 1 : 0; // Clear bits for MSB
      PORTBbits.RB7 = (0b0000010 & Data) ? 1 : 0; // Clear bits 
      PORTBbits.RB4 = (0b0000001 & Data) ? 1 : 0; // Clear bits for LSB 
    }
    

/*----------------------TX------------------------*/  
// TMRT = 1 TSR Empty. TMRT  = 0 TSR Full
  

		while(!TRMT); // While TMRT is NOT Empty
	  TX1REG = Data;
  } // END WHILE

  return;
} //------------------------END OF MAIN------------



// Functions Definitions
void UART_TX_Init(void)
{
// Transmit Status and Control Register Configuration
//--------------------------------------------------
 
//TX1STAbits.CSRC = x; Clock Source bit; Unused in ASYNCH mode 
  TX1STAbits.SYNC = 0; // Set for Asynchronous Mode
  TX1STAbits.TX9 = 0; // Set to 8-bit Transmission Mode
  TX1STAbits.TXEN = 1; // Enable UART Transmission
//TX1STAbits.SENDB = x; Send Break Char Bit
  TX1STAbits.BRGH = 1; // Set for High-Speed Baud Rate
//TX1STAbits.TMRT = x; TX shift Register Status bit
//TX1STAbits.TX9D = x; Ninth bit of TX data


// Receive Status and Control Register Configuration
//--------------------------------------------------
 RC1STAbits.SPEN = 1; // Serial Port Enabled

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
