//------------------------------------------------------
// File name: WAV Player Project
// YOUTIBE LINK: https://youtu.be/4CrqWBFyTRY
// Programmed by Gun-Hee Shin on DEC 2021
//------------------------------------------------------


//--------------------connect info----------------------
//   ATmega2560                 Peripheral Board 

//                                   (LCD) 
//      5V                             5V
//      GND                            GND
//      12                             D4
//      11                             D4
//      5                              D5
//      4                              D6
//      3                              D7
//                                (Rotary Encoder)
//      18(PD3,INT3)                   PH_A
//      19(PD2)                        PH_B
//      20(PD1,INT1)                   ENC_SW
//                                    (SPI)
//      50(PB3,MISO)                   MISO
//      51(PB2,MOSI)                   MISO
//      52(PB1,SCK)                    SCK
//      53(PB0,SS)                     SD_CS

//    ATmega2560                    Amp Board
//      5V                             5V
//      GND                            GND
//      10(PB4,OC2A)                   PWM_H(LEFT)
//      9(PH6,OC2B)                    PWM_L(LEFT)
//      6(PH3,OC4A)                    PWM_H(RIGHT)
//      7(PH4,OC4B)                    PWM_L(RIGHT)

//    Amp Board                      Speaker
//      Left speaker                   Speaker #1
//      Right speaker                  Speaker #2
//------------------------------------------------------


#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>
#include <iomxx0_1.h> 

#define PhaseA 18 
#define PhaseB 19
#define enc_sw 20
#define SD_CS 53

LiquidCrystal lcd(12,11,5,4,3,2);

uint32_t i = 0;                   // Variables used to store data in a sound source file in the music_info function.
volatile int music_cnt = 0;       // Variables used to save the order of sound sources during ExtInterrupt.
volatile int music_num = 0;       // Variables to determine the number of wav sound sources in the SD card.
volatile int buttoncnt = 0;       // Variables that determine play/stop according to the number of clicks on the button.

uint8_t BitPerSample[2] = {0,};   // Variables to be read from the header of WAV file if sampling bit is 8/16 bit
uint8_t NumChannels[2] = {0,};    // Variables to be read from the header of WAV file if channel is stereo/mono
uint8_t SampleRate[4] = {0,};     // Variables to be read from the header of WAV file if sampling rate is 22050/44100Hz
uint8_t Subchunk2Size[4] = {0,};  // Variables representing the size of music data.

uint8_t buffer[2][3200];          // Variables to store music data in the loop function.
char Music_name[10][13];          // Variables to store the name of music
             
uint16_t bps = 0;                 // bits per second
uint32_t sr = 0;                  // Sampling rate, sample rate는 4byte이므로 수로 바꿀때는 32bit로
char nc = 0;                      // Number of channel
uint32_t count = 0;  
int b=0;                          

File myFile;                      // music file from SD card

void printFilename(File dir);
void music_info(File myfile);

void printFilename(File dir) {   // stores the file name of the music
  String fname;
  while (true) {
    File entry =  dir.openNextFile(); // open the next file in SD card
    if (! entry) {                    // break if there's no next file
      break;
    }
    if (entry.isDirectory()) {        
    }
    else {
      fname = entry.name();
      if (fname.endsWith("WAV")) {    // select only wav file in SD card
        fname.toCharArray(Music_name[music_num], fname.length()+1); // convert string to char arrangement to display on LCD
        music_num++;
      }
    }
  }
}

void music_info(File myfile)      // reads informations of music from header of WAV file
{
    i=0;
    while (1) {

    if (i >= 22 && i <= 23) {
      NumChannels[i - 22] = myfile.read();
    }

    else if (i > 23 && i < 28) {
      SampleRate[i - 24] = myfile.read();
    }

    else if (i >= 34 && i <= 35) {
      BitPerSample[i - 34] = myfile.read();
    }
    else if (i > 39 && i < 44) {
      Subchunk2Size[i - 40] = myfile.read();
    }
    else {
      myfile.read();
    }
    i++;
    if (i == 44) {
      break;
    }
    delay(50);
  }

//-----------save BitPerSample data to variables bps--------//
  uint32_t data1 = 0x00000000;
  data1 |= (BitPerSample[0] | BitPerSample[1] << 8); //the data form is little endian
  Serial.print("Bit per Sampling :");
  Serial.println(data1);
  bps = data1;
//----------------------------------------------------------//

//------------save NumChannels data to variables nc---------//
  uint32_t data2 = 0x00000000;
  data2 |= (NumChannels[0] | NumChannels[1] << 8); //the data form is little endian
  Serial.print("Num_Chan :");
  Serial.println(data2);
  nc = data2;
//----------------------------------------------------------//

  data1 = 0x00000000; 
  data2 = 0x00000000;

//----------save SampleRate data to variables sr-----------// 
  data1 = (SampleRate[0] | SampleRate[1] << 8);
  data2 |= (SampleRate[2] | SampleRate[3] << 8); 
  data1 = data2 << 16 | data1;  //the data form is little endian
  data1 = 0x0000FFFF & data1;   // Correct incorrect values that may occur due to bitwise AND operation.
  Serial.print("Sampling_Rate :");
  Serial.println(data1);
  sr = data1;
//---------------------------------------------------------//


//----------display Subchunk2Size data on the serial monitor-----------// 
  data1 = 0x00000000;
  data2 = 0x00000000;
  data1 |= (Subchunk2Size[0] | Subchunk2Size[1] << 8);
  data2 |= (Subchunk2Size[2] | Subchunk2Size[3] << 8);
  data1 = data2 << 16 | data1; // the data form is little endian
  data1 = 0x0000FFFF & data1; // Correct incorrect values that may occur due to bitwise AND operation.
  Serial.print("total music data :");
  Serial.println(data1);
//---------------------------------------------------------------------//

}

void ExtInterruptInit(){          // set up ExtInteruupt
  
//------ INT1 Interrupt / falling edge detection ------//
  EICRA &= ~_BV(ISC10);
  EICRA |= _BV(ISC11);  // falling edge
  EIFR |=_BV(INTF1);    // Clear INT1 Flag
  EIMSK |= _BV(INT1);   // Enable INT1
//-----------------------------------------------------//

//------ INT3 Interrupt / falling edge detection ------//
  EICRA &= ~_BV(ISC30);
  EICRA |= _BV(ISC31);  // falling edge
  EIFR |=_BV(INTF3);    // Clear INT3 Flag
  EIMSK |= _BV(INT3);   // Enable INT3
//-----------------------------------------------------//
}

void Timer_Counter2() {           // set up 8-bit Timer/Counter2

  TCCR2B &= ~_BV(WGM22);  
  TCCR2A |= _BV(WGM21);   // WGM22=0, WGM21=1, WGM20=1
  TCCR2A |= _BV(WGM20);   // Fast PWM, 8-bit / TOP : 0x00FF

  TCCR2A |= _BV(COM2A1);  // COM2A1=1, COM2A0=0 
  TCCR2A &= ~_BV(COM2A0); // clear OC2A on Compare Match, set OC2A at TOP

  TCCR2A |= _BV(COM2B1);  // COM2B1=1, COM2B0=0
  TCCR2A &= ~_BV(COM2B0); // clear OC2B on Compare Match, set OC2B at TOP

  TCCR2B &= ~_BV(CS22);   
  TCCR2B &= ~_BV(CS21);   // CS22=0,CS21=0,CS20=1
  TCCR2B |= _BV(CS20);    // No prescale factor
  
  TCNT2 = 0x00;           // timer/counter 2 reset
}

void Timer_Counter4() {           // set up 8-bit Timer/Counter4

  TCCR4B &= ~_BV(WGM43);
  TCCR4B |= _BV(WGM42); 
  TCCR4A &= ~_BV(WGM41);  // WGM43=0 ,WGM42=1 ,WGM41=0 ,WGM40=1
  TCCR4A |= _BV(WGM40);   // Fast PWM, 8-bit / TOP : 0x00FF

  TCCR4B &= ~_BV(CS42); 
  TCCR4B &= ~_BV(CS41);   // CS12=0, CS11=0, CS10=1
  TCCR4B |= _BV(CS40);    // No prescale factor

  TCCR4A |= _BV(COM4A1);  // COM4A1=1, COM4A0=0 
  TCCR4A &= ~_BV(COM4A0); // clear OC4A on Compare Match, set OC4A at TOP

  TCCR4A |= _BV(COM4B1);  // COM4B1=1, COM4B0=0
  TCCR4A &= ~_BV(COM4B0); // clear OC4B on Compare Match, set OC4B at TOP

  TCNT4 = 0x0000; // timer/counter reset
}

void Timer_Counter1(){            // set up 16-bit Timer/Counter1
  
  TCCR1B &= ~_BV(WGM13); 
  TCCR1B |= _BV(WGM12);  
  TCCR1A &= ~_BV(WGM11);    // WGM13=0, WGM12=1, WGM11=0, WGM10=0
  TCCR1A &= ~_BV(WGM10);    // CTC mode, 16bit / TOP : OCR1A
  
  TCCR1A |= _BV(COM1A1);    // COM1A1 = 1,COM1A0 = 0
  TCCR1A &= ~_BV(COM1A0);   // clear OC1A on compare match
  
  TCCR1B &= ~_BV(CS12);  
  TCCR1B &= ~_BV(CS11);   // CS12=0, CS11=0, CS10=1
  TCCR1B |= _BV(CS10);    // No prescale factor

  if (sr==44100){
    OCR1A = 181;          // 44100HZ -> OCR1A=181
  }
  else{
    OCR1A = 362;          // 22050HZ -> OCR1A=362
  }

  TCNT1 = 0x0000;         // timer/counter reset

  TIMSK1 |= _BV(OCIE1A);  // interrupt enable
  TIFR1 |= _BV(OCF1A);  
}

ISR(INT1_vect)                    // set up External ISR(interrupt service routine)
{
    //----------------------------------------------------------------------------------------
    // this ISR is operated only when the rotary encoder swith is pressed
    // initial value of the button is 0 and 'play'.
    // play/stop is determined according to the accumulated value of the button being pressed.
    // if the accumulated value is even, it is 'play' and if it is odd, it is 'stop'
    //----------------------------------------------------------------------------------------
  if (buttoncnt%2==0){
    lcd.setCursor(0,1);
    lcd.print("PLAYING");   // display "PLAY" on LCD when playing a music.

    // open the saved music file whose name is 'Music_name[music_cnt]' in printDirectory(File dir) function 
    myFile= SD.open(Music_name[music_cnt]); 
    
    music_info(myFile);     // get the header information of the music

    EIMSK &= ~_BV(INT3);    // disable interruption due to iNT3; no changing the music during playing
    TIMSK1 |= _BV(OCIE1A);  // timer/counter interrupt 1 enable, play the music
    }

  else{
    cli();                  // disable Timer/Counter1 interrupt
    myFile.close();
    lcd.setCursor(0,1);
    lcd.print("STOPPED");   // display "STOPPED" on LCD when music is stopped
    EIMSK |= _BV(INT3);     // Enable interruption due to iNT3; only can change the music when music is stopped 
    TIMSK1 &= ~_BV(OCIE1A); // timer/counter 1 disable on stopped
    }
    buttoncnt++;
    sei();                  // enable Timer/Counter1 interrupt
} 

ISR(INT3_vect)                    // set up External ISR(interrupt service routine)
{
    //-------------------------------------------------------------------
    // this ISR is operated only when the rotary encoder is ratated
    // When the encoder rotates to the right, 0 signal is sended, 
    // and when the encoder rotates to the left, 1 signal is sended
    // rotate signal sended through pin PhaseB on perpheral board
    //--------------------------------------------------------------------

  cli(); //disable Timer/Counter1 interrupt
  if(digitalRead(PhaseB)) {
    music_cnt--;
  }
  else{
    music_cnt++; 
  }

  if(music_cnt < 0){
    music_cnt = music_num-1;
  }
  if(music_cnt > music_num-1){
    music_cnt = 0;
  }

  lcd.clear();

  Serial.println(Music_name[music_cnt]);  // display on serial monitor
  lcd.setCursor(0,0);
  lcd.print(Music_name[music_cnt]);       // display on LCD monitor
  
  lcd.setCursor(0,1);
  lcd.print("STOPPED"); 
  count=0;  // to read data from the beginning
  sei();    // enable Timer/Counter1 interrupt
}

ISR(TIMER1_COMPA_vect)            // set up Timer ISR(interrupt service routine)
{

//---------------------------------------------------------------------------------------------------------------------------------
// the data received first has a small weight because music data from WAV file is consist of little endian.
// if the BitPerSample of WAV file is 16bit, first byte data goes to OCRB(PWM_L) and next byte data goes to OCRA(PWM_H)
// data of 8-bit WAV file is unsigned data, however data of 16-bit WAV file is signed data
// so when playing 16-bit WAV file, we should transform signed data to unsigned data by adding 0x8000
// if the Numchannels of Wave file is 2(stereo), we sholud consider that the left channel data is ahead of the right channel data.
// remind frequency of updating OCR data in this routine is determined by OCR1A in Timer_Counter1() function
//---------------------------------------------------------------------------------------------------------------------------------

  if(bps==16&&nc==2){     // 16bit&stereo WAV file

    uint16_t data3=0x0000;
    uint16_t data4=0x0000;
    data3 |= (buffer[i][count] | buffer[i][count+1]<<8);
    data4 |= (buffer[i][count+2] | buffer[i][count+3]<<8);
    data3= data3+0x8000;
    data4= data4+0x8000;
    OCR2A= data3>>8;
    OCR2B= data3 & 0xff;
    OCR4A= data4 >>8;
    OCR4B= data4 & 0xff; 

    count= count+4;

    if(count >= 3000 &&b==0) count=0, i=1, b=1;
    else if(count>=3000&&b==1) count=0, i=0, b=0;
  }

  else if(bps==16&&nc==1){ // 16bit&mono WAV file

    uint16_t data3=0x0000;
    data3 |= (buffer[i][count] | buffer[i][count+1]<<8);
    data3= data3+0x8000;
    OCR2A= data3>>8;
    OCR2B= data3 & 0xff;
    OCR4A= OCR2A;
    OCR4B= OCR2B;

    count= count+2;
    if(count >= 3000 &&b==0) count=0, i=1, b=1;
    else if(count>=3000&&b==1) count=0, i=0, b=0;

  }

  else if(bps==8&&nc==2){ // 8bit&stereo WAV file

    OCR2A = 0; 
    OCR2B = buffer[i][count];
    count ++;

    OCR4A = 0;
    OCR4B = buffer[i][count];
    count++;

    if(count>=3000&&b==0) count=0, i=1, b=1;
    else if(count>=3000&&b==1) count=0, i=0, b=0;

  } 

  else {                  // 8bit&mono WAV file

    OCR2A=0;
    OCR2B=buffer[i][count];

    OCR4A=0;
    OCR4B=OCR2B;

    count ++;
    if(count>=3000&&b==0) count=0, i=1, b=1;
    else if(count>=3000&&b==1) count=0, i=0, b=0;

  }

}

void setup() {

  pinMode(PhaseA, INPUT);
  pinMode(PhaseB, INPUT);   
  pinMode(enc_sw, INPUT);   
  pinMode(10, OUTPUT);      
  pinMode(9, OUTPUT);       
  pinMode(6, OUTPUT);       
  pinMode(7, OUTPUT);       
  lcd.begin(16,2);

  ExtInterruptInit();
  sei();
  
  Serial.begin(115200);
  while (!Serial) {
    ; 
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  
  Timer_Counter4();               // Timer/Counter 4 8bit PWM initialization
  Timer_Counter2();               // Timer/Counter 2 8bit PWM initialization
  Timer_Counter1();               // Timer/Counter 1 timer interrupt initialization

  myFile = SD.open("/"); 
 printFilename(myFile);         // save all the name of music
  Serial.println(Music_name[0]);  // display the first name of music on serial monitor
  
  myFile.close();

  lcd.setCursor(0,0);
  lcd.print(Music_name[0]);       // display the first name of music on LCD monitor
  lcd.setCursor(0,1);
  lcd.print("PLAYING");           // display letter 'PLAYING' on LCD monitor

}

void loop()
{
    
    myFile.read(buffer[0],3000);
    myFile.read(buffer[1],3000);

}