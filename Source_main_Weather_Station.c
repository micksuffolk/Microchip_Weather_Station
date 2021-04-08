/*
 * Weather Station, by Mike Suffolk.
 * This application resides on a PIC16F887 microcontroller.
 * Interfacing to a Bosch BME280 Temp/Press/Humidity sensor via I2C
 * communications it displays the sensor readings on a 20x4 LCD display
 * via 4-bit LCD interface.
 */



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////                       PIC Configuration                         ///////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Setup PIC with required parameters...
#pragma config FOSC     = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE     = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE    = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN    = ON        // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP      = OFF       // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD      = OFF       // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT      = OFF       // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP       = OFF       // Flash Program Memory Code Protection bit (Code protection off)
#pragma config MCLRE    = 0x00      // MCLR internally tied to Vdd
#pragma config IESO     = 0x00      // Internal / External switchover disabled
#pragma config FCMEN    = 0x01      // Fail safe clock monitor enabled
#pragma config DEBUG    = 0x01      // In-circuit debugger disabled
#pragma config BOR4V    = 0x01      // Brown out reset at 4.0V
#define _XTAL_FREQ      8000000    // Oscillator freq. Hz for __delay_ms() function, 20MHz.



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////                    Link tags to I/O points                      ///////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Define ports (I/O pins) used by the LCD header file...
#define LCD_RS          PORTDbits.RD0 // LCD RS Signal
#define LCD_EN          PORTDbits.RD1 // LCD EN Signal
#define LCD_D4          PORTDbits.RD2 // LCD D4 Signal
#define LCD_D5          PORTDbits.RD3 // LCD D5 Signal
#define LCD_D6          PORTDbits.RD4 // LCD D6 Signal
#define LCD_D7          PORTDbits.RD5 // LCD D7 Signal
#define I2C_Status_LED  PORTEbits.RE0 // I2C Request/Write Status LED
#define LCD_Backlight   PORTBbits.RB5 // LCD Back light
#define Push_Button     PORTBbits.RB2 // Push Button



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////                      Include Header files                       ///////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Included header files...
#include <xc.h>
#include "PIC_I2C_Header_File.h"
#include "PIC_LCD_Header_File.h"
#include <stdlib.h>
#include <string.h>



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////                        Define variables                         ///////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Program variables & definitions...
bit                 Initialise_complete;
unsigned short      I2C_Read_Byte[26];
signed long         adc_T;
signed long         adc_P;
unsigned int        adc_H;
signed long         T;
unsigned long       P;
signed long         H;
signed char         T_char;
signed char         T_decimal_char;
unsigned short      P_short;
signed char         P_decimal_char;
signed char         H_char;
signed long         var1;
signed long         var2;
signed long         t_fine;
unsigned short      dig_T1;
signed short        dig_T2;
signed short        dig_T3;
unsigned short      dig_P1;
signed short        dig_P2;
signed short        dig_P3;
signed short        dig_P4;
signed short        dig_P5;
signed short        dig_P6;
signed short        dig_P7;
signed short        dig_P8;
signed short        dig_P9;
unsigned char       dig_H1;
signed short        dig_H2;
unsigned char       dig_H3;
signed short        dig_H4;
signed short        dig_H5;
signed char         dig_H6;
char                string_A[10];
char                string_B[10];
unsigned char       Prg_Counter;

void main(void)
{



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////                       Initialise Program                        ///////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



while(!Initialise_complete)
{
// First line of code delays the processor for 1000ms
// Allows power supplies to switch on and stabilise.
__delay_ms(100);
    
//  Oscillator Frequency Select Bits
    OSCCONbits.IRCF = 0b111;
    OSCCONbits.SCS = 0;
    OSCCONbits.HTS = 1;
    OSCCONbits.LTS = 1;
    OSCCONbits.OSTS = 1;
    OSCTUNEbits.TUN = 0b00000;
    
// Initialise I2C Master with 100KHz clock
    SSPCON  = 0b00101000;                   // I2C Master, SDA & SCL pins used
    SSPCON2 = 0b00000000;                   // Default values
    SSPADD = (_XTAL_FREQ/(4*100000))-1;     // Setting Clock Speed, 100KHz clock
    SSPSTAT = 0b00000000;                   // Clear the status register

//  Disable AI channels which share the pins used as digital inputs or outputs
    ANSEL = 0x00;   // Analogues - None
    ANSELH = 0x00;  // Analogues - None

    //  Setup channels as digital outputs or inputs
    TRISA = 0x00; // Digital Input - None
    TRISB = 0x04; // Digital Input - Push button at RB2
    TRISC = 0x18; // Digital Input - I2C SCL & SDA at RC3 & RC4
    TRISD = 0x00; // Digital Input - None
    TRISE = 0x00; // Digital Input - None
    __delay_ms(100);

//  Setup digital outputs as ON/OFF initially    
    PORTA = 0x00; // Digital Outputs - Off
    PORTB = 0x00; // Digital Outputs - Off
    PORTC = 0x00; // Digital Outputs - Off
    PORTD = 0x00; // Digital Outputs - Off
    PORTE = 0x00; // Digital Outputs - Off
    __delay_ms(100);

// Initialise LCD display
    LCD_Backlight = 1;  // LCD Back light
    Lcd_Start();        // Startup LCD Display.
    Lcd_Clear();        // Clear LCD display to start with.
    Lcd_Set_Cursor(1,1);
    Lcd_Print_String("M.Suffolk");
    Lcd_Set_Cursor(2,1);
    Lcd_Print_String("Weather");
    Lcd_Set_Cursor(3,1);
    Lcd_Print_String("Monitoring Station");
    Lcd_Set_Cursor(4,1);
    Lcd_Print_String("Soft. Rev. 20210208");
    __delay_ms(100);

// Initialise BME280 sensor
    I2C_Begin();     // Initiate start condition on I2C bus
    I2C_Write(0xEC); // BME280 address = 0x76 RW Bit = 0...  (Binary 11101100)
    I2C_Write(0xF2); // We want to write to register address F2h
    I2C_Write(0x01); // We want to write data 01h
    I2C_Write(0xF4); // We want to write to register address F4h
    I2C_Write(0x25); // We want to write data 25h
    I2C_Write(0xF5); // We want to write to register address F5h
    I2C_Write(0x00); // We want to write data 00h
    I2C_End();
    __delay_ms(200);

//  Read BME280 Calibration Values...
    I2C_Begin();     // Initiate start condition on I2C bus
    I2C_Write(0xEC); // BME280 address = 0x76 RW Bit = 0...  (Binary 11101100)
    I2C_Write(0x88); // We want to read register address 88h onwards
    I2C_Begin();     // Repeat the start condition on I2C bus
    I2C_Write(0xED); // BME280 address = 0x76 RW Bit = 1...  (Binary 11101101)
    I2C_Read_Byte[0]    = I2C_Read(1);
    I2C_Read_Byte[1]    = I2C_Read(1);
    I2C_Read_Byte[2]    = I2C_Read(1);
    I2C_Read_Byte[3]    = I2C_Read(1);
    I2C_Read_Byte[4]    = I2C_Read(1);
    I2C_Read_Byte[5]    = I2C_Read(1);
    I2C_Read_Byte[6]    = I2C_Read(1);
    I2C_Read_Byte[7]    = I2C_Read(1);
    I2C_Read_Byte[8]    = I2C_Read(1);
    I2C_Read_Byte[9]    = I2C_Read(1);
    I2C_Read_Byte[10]   = I2C_Read(1);
    I2C_Read_Byte[11]   = I2C_Read(1);
    I2C_Read_Byte[12]   = I2C_Read(1);
    I2C_Read_Byte[13]   = I2C_Read(1);
    I2C_Read_Byte[14]   = I2C_Read(1);
    I2C_Read_Byte[15]   = I2C_Read(1);
    I2C_Read_Byte[16]   = I2C_Read(1);
    I2C_Read_Byte[17]   = I2C_Read(1);
    I2C_Read_Byte[18]   = I2C_Read(1);
    I2C_Read_Byte[19]   = I2C_Read(1);
    I2C_Read_Byte[20]   = I2C_Read(1);
    I2C_Read_Byte[21]   = I2C_Read(1);
    I2C_Read_Byte[22]   = I2C_Read(1);
    I2C_Read_Byte[23]   = I2C_Read(1);
    I2C_Read_Byte[24]   = I2C_Read(1);
    I2C_Read_Byte[25]   = I2C_Read(0);
    I2C_End();

//  BME280 Temperature Calibration variables...
    dig_T1 = I2C_Read_Byte[0] + (I2C_Read_Byte[1] * 256);
    dig_T2 = I2C_Read_Byte[2] + (I2C_Read_Byte[3] * 256);
    dig_T3 = I2C_Read_Byte[4] + (I2C_Read_Byte[5] * 256);

//  BME280 Pressure Calibration variables...
    dig_P1 = I2C_Read_Byte[6] + (I2C_Read_Byte[7] * 256);
    dig_P2 = I2C_Read_Byte[8] + (I2C_Read_Byte[9] * 256);
    dig_P3 = I2C_Read_Byte[10] + (I2C_Read_Byte[11] * 256);
    dig_P4 = I2C_Read_Byte[12] + (I2C_Read_Byte[13] * 256);
    dig_P5 = I2C_Read_Byte[14] + (I2C_Read_Byte[15] * 256);
    dig_P6 = I2C_Read_Byte[16] + (I2C_Read_Byte[17] * 256);
    dig_P7 = I2C_Read_Byte[18] + (I2C_Read_Byte[19] * 256);
    dig_P8 = I2C_Read_Byte[20] + (I2C_Read_Byte[21] * 256);
    dig_P9 = I2C_Read_Byte[22] + (I2C_Read_Byte[23] * 256);

//  BME280 Humidity Calibration variables (first part)...
    dig_H1 = I2C_Read_Byte[25];

//  Zero the bytes used to store the data read from I2C device...    
    I2C_Read_Byte[0]    = 0;
    I2C_Read_Byte[1]    = 0;
    I2C_Read_Byte[2]    = 0;
    I2C_Read_Byte[3]    = 0;
    I2C_Read_Byte[4]    = 0;
    I2C_Read_Byte[5]    = 0;
    I2C_Read_Byte[6]    = 0;
    I2C_Read_Byte[7]    = 0;
    I2C_Read_Byte[8]    = 0;
    I2C_Read_Byte[9]    = 0;
    I2C_Read_Byte[10]   = 0;
    I2C_Read_Byte[11]   = 0;
    I2C_Read_Byte[12]   = 0;
    I2C_Read_Byte[13]   = 0;
    I2C_Read_Byte[14]   = 0;
    I2C_Read_Byte[15]   = 0;
    I2C_Read_Byte[16]   = 0;
    I2C_Read_Byte[17]   = 0;
    I2C_Read_Byte[18]   = 0;
    I2C_Read_Byte[19]   = 0;
    I2C_Read_Byte[20]   = 0;
    I2C_Read_Byte[21]   = 0;
    I2C_Read_Byte[22]   = 0;
    I2C_Read_Byte[23]   = 0;
    I2C_Read_Byte[24]   = 0;
    I2C_Read_Byte[25]   = 0;
    __delay_ms(200);

//  Read remaining BME280 Humidity Calibration Values...
    I2C_Begin();     // Initiate start condition on I2C bus
    I2C_Write(0xEC); // BME280 address = 0x76 RW Bit = 0...  (Binary 11101100)
    I2C_Write(0xE1); // We want to read register address E1h onwards
    I2C_Begin();     // Repeat the start condition on I2C bus
    I2C_Write(0xED); // BME280 address = 0x76 RW Bit = 1...  (Binary 11101101)
    I2C_Read_Byte[0]    = I2C_Read(1);
    I2C_Read_Byte[1]    = I2C_Read(1);
    I2C_Read_Byte[2]    = I2C_Read(1);
    I2C_Read_Byte[3]    = I2C_Read(1);
    I2C_Read_Byte[4]    = I2C_Read(1);
    I2C_Read_Byte[5]    = I2C_Read(1);
    I2C_Read_Byte[6]    = I2C_Read(0);
    I2C_End();

//  BME280 Humidity Calibration variables (second part)...
    dig_H2 = I2C_Read_Byte[0] + (I2C_Read_Byte[1] * 256);
    dig_H3 = I2C_Read_Byte[2];
    dig_H4 = (I2C_Read_Byte[4] & 0x0F) + (I2C_Read_Byte[3] * 16);
    dig_H5 = (I2C_Read_Byte[4] & 0xF0) + (I2C_Read_Byte[5] * 16);
    dig_H6 = I2C_Read_Byte[6];

//  Zero the bytes used to store the data read from I2C device...    
    I2C_Read_Byte[0]    = 0;
    I2C_Read_Byte[1]    = 0;
    I2C_Read_Byte[2]    = 0;
    I2C_Read_Byte[3]    = 0;
    I2C_Read_Byte[4]    = 0;
    I2C_Read_Byte[5]    = 0;
    I2C_Read_Byte[6]    = 0;
    __delay_ms(200);
    
//  Clear LCD display ready for main program to start...
    __delay_ms(5000);
    Lcd_Clear();    // Clear LCD display to start with.
    
// Set tag TRUE to inform program that initialisation of the PIC is complete.
    Initialise_complete = 1;
    
}   // End of while initialise complete = Not true.



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////                          Main Program                           ///////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



// Perform an endless program loop.
while(Initialise_complete == 1)
{


    
///////////////////////////////////////////////////////////////////////////////
///////                             BME280                              ///////
///////////////////////////////////////////////////////////////////////////////
    
//  I2C Communication.
//  Write to BME280 to trigger forced mode sensor update...
I2C_Begin();     // Initiate start condition on I2C bus
I2C_Write(0xEC); // BME280 address = 0x76 RW Bit = 0...  (Binary 11101100)
I2C_Write(0xF4); // We want to write to register address F4h
I2C_Write(0x25); // We want to write data 25h
I2C_End();
__delay_ms(200);

//  Read Temperature, Pressure, & Humidity ADC values...
I2C_Begin();     // Initiate start condition on I2C bus
I2C_Write(0xEC); // BME280 address = 0x76 RW Bit = 0...  (Binary 11101100)
I2C_Write(0xF7); // We want to read register address F7h onwards
I2C_Begin();     // Repeat the start condition on I2C bus
I2C_Write(0xED); // BME280 address = 0x76 RW Bit = 1...  (Binary 11101101)
I2C_Read_Byte[0]    = I2C_Read(1);
I2C_Read_Byte[1]    = I2C_Read(1);
I2C_Read_Byte[2]    = I2C_Read(1);
I2C_Read_Byte[3]    = I2C_Read(1);
I2C_Read_Byte[4]    = I2C_Read(1);
I2C_Read_Byte[5]    = I2C_Read(1);
I2C_Read_Byte[6]    = I2C_Read(1);
I2C_Read_Byte[7]    = I2C_Read(0);
I2C_End();

//  ADC Raw value for pressure...
adc_P = (((I2C_Read_Byte[0] & 0xFF) * 65536) + 
        ((I2C_Read_Byte[1] & 0xFF) * 256) + 
        (I2C_Read_Byte[2] & 0xF0)) / 16;
    
//  ADC Raw value for temperature...
adc_T = (((I2C_Read_Byte[3] & 0xFF) * 65536) + 
        ((I2C_Read_Byte[4] & 0xFF) * 256) + 
        (I2C_Read_Byte[5] & 0xF0)) / 16;
    
//  ADC Raw value for humidity...
adc_H = (((I2C_Read_Byte[6] & 0xFF) * 256) + 
        ((I2C_Read_Byte[7] & 0xFF)));

//  Zero the bytes used to store the data read from I2C device...    
I2C_Read_Byte[0]    = 0;
I2C_Read_Byte[1]    = 0;
I2C_Read_Byte[2]    = 0;
I2C_Read_Byte[3]    = 0;
I2C_Read_Byte[4]    = 0;
I2C_Read_Byte[5]    = 0;
I2C_Read_Byte[6]    = 0;
I2C_Read_Byte[7]    = 0;
__delay_ms(200);

//  Temperature Calculation...
var1 = ((((adc_T >> 3) - ((signed long)dig_T1 << 1))) * ((signed long)dig_T2)) >> 11;
var2 = (((((adc_T >> 4) - ((signed long)dig_T1)) * ((adc_T >> 4) - ((signed long)dig_T1))) >> 12) * ((signed long)dig_T3)) >> 14;
t_fine = var1 + var2;
T = (t_fine * 5 + 128) >> 8;
T_char = T/100; // Whole DegC for LCD display.
T_decimal_char = ((T-((T_char)*100))/10); // first decimal point value.

//  Pressure Calculation...    
var1 = (((signed long)t_fine) >> 1) - (signed long)64000;
var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((signed long)dig_P6);
var2 = var2 + ((var1*((signed long)dig_P5)) << 1);
var2 = (var2 >> 2) + (((signed long)dig_P4) << 16);
var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((signed long)dig_P2) * var1)>>1)) >> 18;
var1 = ((((32768 + var1)) * ((signed long)dig_P1)) >> 15);
if (var1 == 0)
{P = 0;}
else
{P = (((unsigned long)(((signed long)1048576) - adc_P) - (var2 >> 12)))*3125;
if (P < 0x80000000)
{P = (P << 1) / ((unsigned long)var1);   }
else
{P = (P / (unsigned long)var1) * 2;  }
var1 = (((signed long)dig_P9) * ((signed long)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
var2 = (((signed long)(P >> 2)) * ((signed long)dig_P8)) >> 13;
P = (unsigned long)((signed long)P + ((var1 + var2 + dig_P7) >> 4));}
P_short = P/100; // Whole hPa value for LCD display.
P_decimal_char = abs((P/10)-((P_short)*10)); // first decimal point value.

//  Humidity Calculation...    
H = (((double)t_fine) - 76800);
H = (adc_H - (((double)dig_H4) * 64 + ((double)dig_H5) / 16384 * H)) * (((double)dig_H2) / 65536 * (1 + ((double)dig_H6) / 67108864 * H * (1 + ((double)dig_H3) / 67108864 * H)));
H = H * (1 - ((double)dig_H1) * H / 524288);
H_char = H; // Whole humidity value for LCD display.



///////////////////////////////////////////////////////////////////////////////
///////                         LCD Control                             ///////
///////////////////////////////////////////////////////////////////////////////

//  Print line one of LCD display...
Lcd_Set_Cursor(1,1);
Lcd_Print_String("Temp.");
itoa(string_A, (abs(T_char)), 10);
itoa(string_B, (abs(T_decimal_char)), 10); // d.p. accuracy
strcat(string_A, ".");
strcat(string_A, string_B);
    if (T < 0)                      // Insert the negative sign where necessary.
    {   Lcd_Set_Cursor(1,9);
       Lcd_Print_String("-");  }
    else
    {   Lcd_Set_Cursor(1,9);
       Lcd_Print_String(" ");  }
Lcd_Set_Cursor(1,10);
Lcd_Print_String("           ");
Lcd_Set_Cursor(1,10);
Lcd_Print_String(string_A);
Lcd_Set_Cursor(1,17);
Lcd_Print_String("DegC");
    
//  Print line two of LCD display...
Lcd_Set_Cursor(2,1);
Lcd_Print_String("Humid.");
itoa(string_A, (H_char), 10);
Lcd_Set_Cursor(2,10);
Lcd_Print_String("           ");
Lcd_Set_Cursor(2,10);
Lcd_Print_String(string_A);
Lcd_Set_Cursor(2,18);
Lcd_Print_String("%RH");
    
//  Print line three of LCD display...
Lcd_Set_Cursor(3,1);
Lcd_Print_String("Press.");
utoa(string_A, (P_short), 10);
itoa(string_B, (P_decimal_char), 10); // d.p. accuracy
strcat(string_A, ".");
strcat(string_A, string_B);
Lcd_Set_Cursor(3,10);
Lcd_Print_String("           ");
Lcd_Set_Cursor(3,10);
Lcd_Print_String(string_A);
Lcd_Set_Cursor(3,18);
Lcd_Print_String("hPa");

//  Print line four of LCD display...
Lcd_Set_Cursor(4,1);
Lcd_Print_String(" Weather Monitoring");



///////////////////////////////////////////////////////////////////////////////
///////             Wrap things up for this scan cycle                  ///////
///////////////////////////////////////////////////////////////////////////////

//  Increment the program counter...
//  this counter will count up then reset.
//  Used for the following functions...
//  LCD back light timeout.
if (Prg_Counter < 30)               // While counter is less than setpoint
{
    Prg_Counter = Prg_Counter + 1;  // Increment the counter one at a time
}
else                                // when counter reaches the setpoint
{
    LCD_Backlight = 0;              // LCD Back light OFF.
}



//  Weather Station digital input (push button)...
//  Reset the program counter to 0;
//  Turn the LCD back light ON.
if (Push_Button == 1)
{
    Prg_Counter = 0;
    LCD_Backlight = 1;
}



}   // End of while initialise complete = true.



}   // End of main program.