//------------------------------------------------------
// File name: Serialplot_Class
// YOUTUBE LINK: https://youtu.be/_PgYV1EbYm8
// Programmed by Gun-Hee Shin on Oct 2021
//------------------------------------------------------

//--------------------Pin 연결----------------------
//   ATmega2560                 Peripheral Board  
//      5V                             5V
//      GND                            GND
//      A0                             ANA1
//-------------------------------------------------

//----------------------------------------------abstract------------------------------------------------------
// analog signals are transmitted to micorcontroller(Atmega2560) through potentiometer on the peripheral board
// atmega2560 display the ADC signal in the serial plotter
// implement delay function using class
// real time signal and the 0.3s delay signal are simultaneously plotted in the serial plotter
// sampling time of function loop() is 0.01s
//------------------------------------------------------------------------------------------------------------


int rdata;                   // raw data
float sample_time=0.01;
uint32_t start_time;
uint32_t Microsampletime;


class Dclass                // class that expresses delay block
{
    public:
    int arraysize;          // variable that represent the number of array block based on Td, Ts
    int tempdata[300];      // array used to return the delayed value, the number of blocks shold greater than Td/Ts
                            // in this algorithm, it was leisurely set as (0.3/0.01)*10.
    int ddata;              // variables that store delayed data
    float inputdata;        // raw input data to delay block

    Dclass(float Td, float Ts);
    ~Dclass();
    void delayfunc();
};

Dclass *Dclass1;            

void setup()
{
    Serial.begin(115200);
    Dclass1= new Dclass(0.3,0.01);                   //dynamic memory allocation
    Microsampletime = (uint32_t)(sample_time*1e6);   
    start_time = micros() + Microsampletime;         
}

void loop()
{
    rdata=analogRead(A0);        // save raw data from potentiometers
    Dclass1->inputdata=rdata;    // input of delay class
    Dclass1->delayfunc();        // execute the delayfunc(), which is a member function of the delay class.

    Serial.print(rdata);        // display raw data on serial monitor
    Serial.print(" ");
    Serial.println(Dclass1->ddata);  // display delayed data on serial monitor
    while( !((start_time-micros()) & 0x80000000));
    start_time += Microsampletime;
}

Dclass::Dclass(float Td, float Ts)
{
    arraysize = (int)(Td/Ts);
    for(int i=0; i<arraysize;i++)
    {
        tempdata[i]=0;            // refresh the block in the array to zero.
    }

}

void Dclass::delayfunc()                       
{
    for(int i=arraysize;i>0;i--) 
    {
    tempdata[i]=tempdata[i-1];  // the value after Td exists in the (int)Td/Ts-1 array column
                                // move all the values of the array one by one to the right
    }
    tempdata[0]=inputdata;
    ddata=tempdata[arraysize];  // save a new raw data to the first column and a save ddatato (int)Td/Ts column  
}

Dclass :: ~Dclass()
{

}


