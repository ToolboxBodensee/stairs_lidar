#include <stdarg.h>
#include<SoftwareSerial.h>//header file of software serial port
#include <FastLED.h>

#define LED_PIN  6
#define BRIGHTNESS 1 //1 to 255
#define NUM_LEDS 60
#define MAX_LAENGE 295 //in cm

CRGB leds[NUM_LEDS];// Define the array of leds
SoftwareSerial SerialS(2,3); //define software serial port name as SerialS and define pin2 as RX and pin1 as TX
uint16_t  dist;//actual distance measurements of LiDAR
int strength;//signal strength of LiDAR
int check;//save check value
int i;
int ii;
int uart[9];//save data measured by LiDAR
const uint16_t XY(const uint8_t& x, const uint8_t& y);
const int HEADER=0x59; //frame header of data package
double difflaenge;
int leuchte;



//----------------------------------------------------------------------------------------
void setup() {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  Serial.begin(9600);//set bit rate of serial port connecting Arduino with computer
  SerialS.begin(115200);//set bit rate of serial port connecting LiDAR with Arduino
  Serial.println("hello - starting...");
  FastLED.clear();
  difflaenge = MAX_LAENGE - 30;
}

void loop() 
{
  /*
  for(int ii = 0;ii<NUM_LEDS;ii++){
    leds[ii-1] = CRGB::Green;
    leds[ii-3] = CRGB::Black;
    FastLED.show();
    delay(250);
  */
  

//---------------------------
  if (SerialS.available())//check if serial port has data input
  {
    if(SerialS.read()==HEADER)//assess data package frame header 0x59
    {
      uart[0]=HEADER;
      if(SerialS.read()==HEADER)//assess data package frame header 0x59
      {
        uart[1]=HEADER;
        for(i=2;i<9;i++)//save data in array
        {
          uart[i]=SerialS.read();
        }
        check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
        if(uart[8]==(check&0xff))//verify the received data as per protocol
        {
          dist=uart[2]+uart[3]*256;//calculate distance value
          strength=uart[4]+uart[5]*256;//calculate signal strength value
          Serial.print("dist = ");
          Serial.print(dist);//output measure distance value of LiDAR
          Serial.print(" cm");
          Serial.print('\t');
          Serial.print("strength = ");
          Serial.print(strength);//output signal strength value
          Serial.print('\n');
          
          leuchte = ((dist-30)/difflaenge*NUM_LEDS);
          if(leuchte <NUM_LEDS)
            leds[leuchte] = CRGB::Blue;
          FastLED.show();
          Serial.print('\t');
          Serial.print(difflaenge);
          Serial.print('\t');
          Serial.print(NUM_LEDS);
          Serial.print('\t');
          Serial.print(leuchte);
          Serial.print('\t');
          //delay(1000);
        }
      }
    }
  }
}

