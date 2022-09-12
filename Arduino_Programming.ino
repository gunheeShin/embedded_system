//------------------------------------------------------
// File name: Arduino_Programming
// YOUTUBE LINK: https://youtu.be/dHASlqc_Ezs
// Programmed by Gun-Hee Shin on Sep 2021
//------------------------------------------------------

//--------------------connect info-----------------
//   ATmega2560                 Peripheral Board  
//      5V                             5V
//      GND                            GND
//      14                             LED1
//      15                             LED2
//      16                             LED3
//      17                             LED4
//      18                             LED5
//      19                             LED6
//      20                             LED7
//      21                             LED8
//-------------------------------------------------

//--------------------------------------------------------------abstract---------------------------------------------------------------
//The user changes the lighting method of the Peripheral Board according to the serial input of the variables "a, b, c, d, 1, 2, 3, 4".
//The variable "a, b, c, d" determines the number of LEDs to be lit, and the variable "1, 2, 3, 4" determines the lighting speed.
//--------------------------------------------------------------------------------------------------------------------------------------


char set1[8]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
char set2[8]={0x03,0x06,0x0C,0x18,0x30,0x60,0xC0,0x81};
char set3[8]={0x07,0x0E,0x1C,0x38,0x70,0xE0,0xc1,0x83};
char set4[8]={0x0F,0x1E,0x3C,0x78,0xF0,0xE1,0xC3,0x87};
char count=0;
char data=0;           // variables that receive serial data.
char data1=97;         // the variable receiving the variable "a,b,c,d", and the initial value is set to "a"
char data2=49;         // the variable that receives the variable "1, 2, 3, 4", and the initial value is set to "1"
char tempdata1=0;      // variables that save the previous variable value when there is no serial input ("a,b,c,d")
char tempdata2=0;      // variables that save the previous variable value when there is no serial input ("1,2,3,4")

void setup()
{
    for(int i=14;i<=21;i++) pinMode(i,OUTPUT);    
    Serial.begin(115200);

}

void loop()
{
    if(Serial.available()>0)                     
    {
        data=Serial.read();                     
        if(data>=97&&data<=100) tempdata1= data; // if data is one of "a,b,c,d", save it to tempdata1
        else tempdata2=data;                     // if data is one of "1,2,3,4", save it to tempdata2

        data1=tempdata1;
        data2=tempdata2;
    } 
    else 
    {
        data1=tempdata1;                         // if there is no serial input in the buffer, data1 and data2 are used as data1 and data2 of the previous loop
        data2=tempdata2;
    }
    
    if(data1==97) for(int i=0; i<8;i++) digitalWrite(i+14,set1[count]& 1<<i);        // data1 = 'a'
    else if(data1==98) for(int i=0; i<8;i++) digitalWrite(i+14,set2[count]& 1<<i);   // data1 = 'b'
    else if(data1==99) for(int i=0; i<8;i++) digitalWrite(i+14,set3[count]& 1<<i);   // data1 = 'c'
    else if(data1==100) for(int i=0; i<8;i++) digitalWrite(i+14,set4[count]& 1<<i);  // data1 = 'd'
    count++;
    if(count==8) count=0;


    if(data2==49) delay(500);       // data2 = '1'
    else if (data2==50) delay(250); // data2 = '2'
    else if (data2==51) delay(125); // data2 = '3'
    else if (data2==52) delay(62);  // data2 = '4'
}
