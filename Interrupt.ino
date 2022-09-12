//------------------------------------------------------
// File name: Interrupt
// YOUTUBE LINK: https://youtu.be/97zHu4CqOR0
// Programmed by Gun-Hee Shin on Nov 2021
//------------------------------------------------------

//--------------------Pin 연결----------------------
//   ATmega2560                      ATmega 
//  11(PB5,OC1A)                   18(PD3,INT3)
//-------------------------------------------------

//-----------------------abstract---------------------------
// measure the duty ratio of PWM and send it to PC via serial
// plot the received duty ratio at 0.02 seconds intervals
//----------------------------------------------------------

#include <iomxx0_1.h> 

int cnt = 0;

float sample_time = 0.02;
uint32_t start_time;        // time when the new iteration should begin
uint32_t MicrosSampleTime;  // Integer variable for the sampling time in microseconds

uint32_t temptime1=0;
uint32_t temptime2=0;
unsigned int new_duty=0 ;

int dutycalc = 0;

void ExtInterruptInit()
{
  EICRA |= _BV(ISC30);
  EICRA &= ~_BV(ISC31);   // ISC31, ISC30 = 0,1 : Any edge of INT3 generates an interrupt request
  
  EIFR |= _BV(INTF3);     // Clear INT3 Flag
  EIMSK |= _BV(INT3);     // Enable INT3
}


ISR(INT3_vect)    //External interrupt service routine: measure the duty ratio of PWM
{
  // measure the rising edge, falling edge interval
  if ((cnt % 2) != 0) // If the number interupt is even, it a rising edge
  {
    temptime1 = TCNT1;
  }
  else                // If the number interupt is odd, it a falling edge
  {
    temptime2 = TCNT1; 
  }
  cnt++;

  // duty calculation
  dutycalc = (temptime1 - temptime2)/19.99;  // duty ratio is obtained by the difference between the rising edge and falling edge TCNT1 values.
  
  if (dutycalc < 0){
    dutycalc = -dutycalc; // get absolute value of measured duty
  }
  if (dutycalc > 100){
    dutycalc = 95;
  }
  else if(dutycalc < 5){
    dutycalc = 5; 
  }
  
} 


void setup() { 
  pinMode(11, OUTPUT);
  pinMode(18, INPUT);

//------------------------------------------------------
// Timer/Counter mode selection : Fast PWM, TOP : ICR1
// WGM13, WGM12, WGM11, WGM10 = 1,1,1,0
//------------------------------------------------------
  TCCR1A |= _BV(WGM11); 
  TCCR1A &= ~_BV(WGM10); 
  TCCR1B |= _BV(WGM13); 
  TCCR1B |= _BV(WGM12); 

//---------------------------------------------------------------
// CS12, CS11, CS10 = 0,1,1 (prescale ration : clkIO/8 = 2MHz)
//---------------------------------------------------------------
  TCCR1B &= ~_BV(CS12); 
  TCCR1B |= _BV(CS11); 
  TCCR1B &= ~_BV(CS10); 

  TCCR1A |= _BV(COM1A1);
  TCCR1A &= ~_BV(COM1A0); 

  ICR1 = 1999;
  TCNT1 = 0; // clear the count value to zero

  Serial.begin(115200);
  MicrosSampleTime = (uint32_t)(sample_time*1e6);

  ExtInterruptInit();
  sei(); 

  start_time = micros() + MicrosSampleTime;
}


void loop() {
  
  if(Serial.available())
  {
    delay(100);
    cli();
    new_duty=Serial.read(); // user inputs the desired duty value through serial.read()
            
    TCNT1 = 0; 
    OCR1A = (new_duty*0x14); // set an OCR1A value to generate PWM of the input duty ratio
    sei();  
  }
  
  Serial.print(dutycalc);   // display calculated duty ratio in ISR on serial monitor
  Serial.print(" ");
  Serial.println(new_duty); // display nwe duty ratio on serail monitor
  while(!((start_time-micros()) & 0x80000000)); 
  start_time += MicrosSampleTime; 
      
}
