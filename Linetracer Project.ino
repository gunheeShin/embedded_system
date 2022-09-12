//------------------------------------------------------
// File name: Line Tracer Project
// YOUTUBE LINK: https://youtu.be/JJVlXocFmmk
// Programmed by Gun-Hee Shin on DEC 2021
//------------------------------------------------------


//--------------------connect info----------------------
//   ATmega2560                 Step Motor Driver 

//                                   
//      5V                             VDD
//      GND                            GND
//      21                             Step(Left)
//      20                             Dir(Left)
//      19                             Step(Right)
//      18                             Dir(Left)
//      17                             En

//   ATmega2560                  Line Scan Camera
//      22(PA0)                        CLK
//      23(PA1)                        SI
//      A0(PF0, ADC0)                  AO
//      VDD                            5V
//      GND                            GND
//------------------------------------------------------

#include <iomxx0_1.h>
#define left_motor PD0
#define left_dir PD1
#define right_motor PD2
#define right_dir PD3
#define CLK PA0
#define SI PA1
#define En 17
#define scan_TH 350 

// Variables used to set sampling time of function loop()
float ts=0.01; 
uint32_t MicrosSampleTime;
uint32_t time_out;

// Variables used to find position of robot using camera data 
volatile unsigned int camera_data[129]={0};   // variable used to store converted camera data through ADC pin
volatile unsigned int left_edge;              // the position where the binarized camera data changed 0 to 1
volatile unsigned int right_edge;             // the position where the binarized camera data changed 1 to 0

// Variables used in DDA algorithm
volatile unsigned int cnt1[2] = {0,};
volatile unsigned int cnt2[2] = {0,};

volatile unsigned int B[2] = {0,};
volatile unsigned int M[2] = {0,};

// varialbles that control the step motor
int vel=5000;           // velocity of robot
float con_freq=0.0;     //output of PID control that adjust the angular velocity of the motor
int WL,WR;              //left and right motor angular velocity
float Position = 0.0;   // robot current position


void ADCInit();         // set the ADC register
void ADC_Get(int i);    // read camera analog data through ADC pin
void DDA_parm(int WL, int WR);  // set the parameters used in DDA algorithm
void timer_counter_1(); // set up 16-bit Timer/Counter1
void PIDInit();         // set the parameters used in PID control
void PIDcalc();         // set the rotation velocity of motor to locate robot at desired position


struct PID{                       // PID control parameters struct
    float Ref;
    float state;
    float Kp,Ki,Kd;
    float Max,Min;
    float Intg;
    float Deriv;
    float Err;
    float Err_past;
    float samT;
    float Out_tmp;
    float Out;
    float SatErr;
    float Kc;
} PID;

void ADCInit()                    // set the ADC register
{
  //-----------------------------------------------------------
  // ADMUX - ADC Multiplexer Selection Register
  // Bit 7:6 - REFS1:0 : Reference Voltage Selection Bits
  // Bit 5 - ADLAR : ADC Left Adjust Result
  // Bit 4:0 - MUS4:0 : Analog Channel and Gain Selection Bits
  //-----------------------------------------------------------
  ADMUX = 0;
  ADMUX &= ~_BV(REFS1); 
  ADMUX |= _BV(REFS0);
  ADMUX &= ~_BV(ADLAR); 

  //-----------------------------------------------
  // ADCSRA - ADC Control and Status Register A
  // Bit 7 - ADEN : ADC Enable
  // Bit 6 - ADSC : ADC Start Conversion
  // Bit 5 - ADATE : ADC Auto Trigger Enable
  // Bit 4 - ADIF : ADC Interrupt Flag
  // Bit 3 - ADIE : ADC Interrupt Enable
  // Bit 2:0 - ADPS2:0 : ADC Prescaler Select Bits
  //------------------------------------------------
  ADCSRA |= _BV(ADEN); 
  ADCSRA &= ~_BV(ADSC); 
  ADCSRA &= ~_BV(ADATE); 
  ADCSRA |= _BV(ADIF); 
  ADCSRA &= ~_BV(ADIE); 
  ADCSRA |= _BV(ADPS2); 
  ADCSRA &= ~_BV(ADPS1);
  ADCSRA |= _BV(ADPS0);
}

void ADC_Get(unsigned char num)   // read camera analog data through ADC pin
{
  //-------------------------------------------------------------
  // wait until the conversion analog to digital is finished
  // ADSC bit is read as 1 while the conversion is in progress
  // therfore when ADSC bit is read as 0, the convesion is done
  //--------------------------------------------------------------
  ADCSRA |= (1<<ADSC); // Start conversion
  while(ADCSRA & (1<<ADSC));
  camera_data[num] = ADCW;
}

void DDA_parm(int WL, int WR)     // set the parameters used in DDA algorithm
{
  //---------------------------------------------------------------------
  // carrier frequency is frequency of timer/counter interruption(50KHz)
  // WL, WR is set by output of PID control in function loop()
  //---------------------------------------------------------------------
  TCNT1 = 0;
  B[0] = WL+0.0001;
  M[0] = int(50000 / B[0]) >> 1;
  B[1] = WR+0.0001;
  M[1] = int(50000 / B[1]) >> 1;
}

void timer_counter_1()            // set up 16-bit Timer/Counter1
{
  TCCR1A &= ~_BV(WGM11);    
  TCCR1A &= ~_BV(WGM10);    
  TCCR1B |= _BV(WGM12);     // WGM13=0, WGM12=1, WGM11=0, WGM10=0    
  TCCR1B &= ~_BV(WGM13);    // CTC mode, 16bit / TOP : OCR1A

  TCCR1B &= ~_BV(CS10);       
  TCCR1B |= _BV(CS11);      // CS12=0, CS11=1, CS10=0  
  TCCR1B &= ~_BV(CS12);     // prescale ration : clk/8

  OCR1A = 39;               // frequency of timer/counter interruption(50KHz)
  TCNT1 = 0x0000;

  TIMSK1 |= _BV(OCIE1A);    // interrupt enable
  TIFR1 |= _BV(OCF1A);      // clear OCF1A. To clear it, we need to write logic 1 to it.
}

ISR(TIMER1_COMPA_vect)            // set up timer ISR(interrupt service routine)
{
  //----------------------------------------------------------------------
  // DDA algorithm is operated in this timer ISR
  // DDA algorithm is operated twice for left and right motor
  // carrier frequency is frequency of timer/counter interruption(50KHz)
  //----------------------------------------------------------------------
    for (int i = 0; i < 2; i++)
    {
        cnt1[i]++;
    }
    for (int i = 0; i < 2; i++)
    {
        if (cnt1[i] > M[i])
        {
            digitalWrite(left_motor,0);
            digitalWrite(right_motor,0);
        }

        cnt2[i] += B[i]; 

        if (cnt2[i] >= 50000)
        {

            digitalWrite(left_motor,1);
            digitalWrite(right_motor,1);
            cnt2[i] -= 50000; 

            cnt1[i] = 0;
        }
    }
}

void PIDInit()                    // set the parameters used in PID control
{
  //---------------------------------------------------------------------
  // these parameters are valid only when the velocity of robot is 5000
  // parameters shold be changed when velocity of robot changes
  //---------------------------------------------------------------------
  PID.Ref = 0;
  PID.state = 0;
  PID.Kp = 32;
  PID.Ki = 3;
  PID.Kd = 0.5;
  PID.Err = 0;
  PID.Err_past = 0;
  PID.samT = ts;    //0.01s
  PID.Max = vel;
  PID.Min = -vel;
  PID.Intg = 0;
  PID.Deriv = 0;
  PID.Out_tmp = 0;
  PID.Out = 0;
  PID.SatErr = 0;
  PID.Kc = 1;
}

void PIDcalc()                    // set the rotation velocity of motor to locate robot at desired position
{
  //----------------------------------------------------------------------------
  // input of PID control is desired robot position which is middle of the line
  // output of PID control is added to right motor rotation velocity
  // PID control is used to set the error zero fast and correctly
  // P,I,D gain should be set appropriately through trial and error
  //-----------------------------------------------------------------------------

  PID.Err = PID.Ref - PID.state;                                            // Error calculation
  PID.Out_tmp = PID.Kp * PID.Err + PID.Ki * PID.Intg + PID.Kd * PID.Deriv;  // PID Output
  
  if (PID.Out_tmp > PID.Max) PID.Out = PID.Max;
  else if (PID.Out_tmp < PID.Min) PID.Out = PID.Min;
  else PID.Out = PID.Out_tmp;

  PID.SatErr = PID.Out - PID.Out_tmp;  //Anti -windup
  
  PID.Intg += PID.samT * (PID.Err + PID.Kc * PID.SatErr);   // Error input to the Integral controller.
  PID.Deriv = (PID.Err - PID.Err_past) / PID.samT;          //Error input to the Derivative controller.

  PID.Err_past = PID.Err;
}

void setup()
{
    Serial.begin(115200);
    ADCInit();
    timer_counter_1();
    void PIDInit();
    pinMode(right_motor,OUTPUT);
    pinMode(left_motor,OUTPUT);
    pinMode(right_dir,OUTPUT);
    pinMode(left_dir,OUTPUT);
    pinMode(En,OUTPUT);
    digitalWrite(left_dir,1);
    digitalWrite(right_dir,0);
    digitalWrite(En,0);

    sei();
    MicrosSampleTime = (uint32_t)(ts*1e6);
    time_out = micros() + MicrosSampleTime;
}

void loop()
{
    int i;

    // initialize reading camera data 
    PORTA |= (1 << SI);   // SI : HIGH
    PORTA |= (1 << CLK);  // CLK : HIGH
    PORTA &= ~(1 << SI);  // SI : LOW
    
    delay(1);

    for (i = 0; i < 129; i++)
  {
    ADC_Get(i);           // read data input of A0 pin
    PORTA &= ~(1 << CLK); // CLK low
    PORTA |= (1 << CLK);  // CLK high

    //------------------------------------------------------------------------------------
    // convert decimal ADC camera data to binary data based on the threshold value(scan_TH)
    // white space : 1 black line: 0
    // the position where the binarized camera data changed 0 to 1 is left_edge
    // the position where the binarized camera data changed 1 to 0 is right_edge
    // current position of robot is middle of left_edge and right_edge
    //-------------------------------------------------------------------------------------
    if (camera_data[i] > scan_TH) {
      camera_data[i] = 1;
    }
    else {
      camera_data[i] = 0;
    }

    if(i>0)
    {
      if(camera_data[i-1]^camera_data[i]) {
        if(camera_data[i]==0) left_edge=i;
        else right_edge=i;
      }
    }
  }

    PORTA &= ~(1 << CLK);  //CLK low
    Position= (float)(right_edge+left_edge)/2; // current position of the robot
    
    PID.Ref=64; // desired position of robot is middle of the line
    PID.state=Position;  

    PIDcalc();

    con_freq=PID.Out;
    WR= con_freq+vel;
    WL=(2*vel)-WR;

    DDA_parm(WL,WR);    // update DDA parameters using updated WR,WL

    while (!((time_out - micros()) & 0x80000000));
    time_out += MicrosSampleTime;

}
