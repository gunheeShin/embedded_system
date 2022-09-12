//------------------------------------------------------
// File name: GPIO_LCD_ADC
// YOUTUBE LINK: https://youtu.be/7Jdpv_H0E70
// Programmed by Gun-Hee Shin on Sep 2021
//------------------------------------------------------


//--------------------connect info-----------------
//   ATmega2560                 Peripheral Board  
//      5V                             5V
//      GND                            GND
//      A0~A7                          LED1~LED*
//      12                             RS
//      11                             E
//      5                              D4
//      4                              D5
//      3                              D6
//      2                              D7
//      A8                             ANA1  
//-------------------------------------------------

//--------------------------------------------------abstract----------------------------------------------------
// analog signals are transmitted to micorcontroller(Atmega2560) through potentiometer on the peripheral board
// Atmega2560 converts the analog signal into a digital signal of 0-5V.
// magnitude of the converted signal is displayed on the LCD monitor on peripheral board
//--------------------------------------------------------------------------------------------------------------



#include <LiquidCrystal.h>
#include <avr/io.h>
#include <iomxx0_1.h>
#define RANGE 1023
int data=0;                                 // varaible that save the value received from potentiometer on the peripheral board
int level=0;                                // variables for classifying potentiometer values
int a=128;                                  // variables to distinguish level
char led[8]={0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};

LiquidCrystal lcd(12,11,5,4,3,2);

void setup()
{
    lcd.begin(16,2);

    lcd.setCursor(0,0);
    lcd.print("voltage= ");

    DDRF |=0xFF;
}

void loop()
{
   data= analogRead(A8);
   lcd.setCursor(10,0);
   lcd.print((float)data/RANGE*5);

    if(0<=(float)data&&(float)data<a-1)                 // level 0, light the first LED, display "Low voltage" on LCD monitor on the peripheral board
    {
        level=0;
        PORTF=led[level];
        lcd.setCursor(0,1);
        lcd.print("Low voltage");
    }
    else if(a<=(float)data&&(float)data<a*2-1)          // level 1, light the second LED, display "Low voltage" on LCD monitor on the peripheral board
    {
        level=1;
        PORTF=led[level];
        lcd.setCursor(0,1);
        lcd.print("Low voltage");
    }
    else if(a*2<=(float)data&&(float)data<a*3-1)        // level 2, light the third LED, display "Low voltage" on LCD monitor on the peripheral board
    {
        level=2;
        PORTF=led[level];
        lcd.setCursor(0,1);
        lcd.print("Low voltage");
    }
    else if(a*3<=(float)data&&(float)data<a*4-1)        // level 3, light the fourth LED, display "Good voltage" on LCD monitor on the peripheral board
    {
        level=3;
        PORTF=led[level];
        lcd.setCursor(0,1);
        lcd.print("Good voltage");
    }
    else if(a*4<=(float)data&&(float)data<a*5-1)        // level 4, light the fifth LED, display "Good voltage" on LCD monitor on the peripheral board
    {
        level=4;
        PORTF=led[level];
        lcd.setCursor(0,1);
        lcd.print("Good voltage");
    }
    else if(a*5<=(float)data&&(float)data<a*6-1)        // level 5, light the sixth LED, display "Good voltage" on LCD monitor on the peripheral board
    {
        level=5;
        PORTF=led[level];
        lcd.setCursor(0,1);
        lcd.print("Good voltage");
    }
    else if(a*6<=(float)data&&(float)data<a*7-1)        // level 6, light the seventh LED, display "Good voltage" on LCD monitor on the peripheral board
    {
        level=6;
        PORTF=led[level];
        lcd.setCursor(0,1);
        lcd.print("Good voltage");
    }
    else if(a*7<=(float)data&&(float)data<a*8-1)        // level 7, light the eight LED, display "Danger voltage" on LCD monitor on the peripheral board
    {
        level=7;
        PORTF=led[level];
        lcd.setCursor(0,1);
        lcd.print("Danger voltage");
    }

}
	