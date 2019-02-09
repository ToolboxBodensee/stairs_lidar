#include <FastLED.h>
#include <stdarg.h>
#include<SoftwareSerial.h>//header file of software serial port

#define LED_PIN  6

#define COLOR_ORDER GRB
#define CHIPSET WS2811

#define BRIGHTNESS 255

// Breite und Höhe
#define kMatrixWidth 11 //----------.Breite
#define kMatrixHeight 17 //----------Höhe

//Button Variablen
#define buttonPin1 10//Switch Oben und Mitte 
#define buttonPin2 11 //Switch unten und Mitte
#define buttonPin3 9 //Rot unten
#define buttonPin4 8 // Rot oben

//Zeit Variablen
#define xTime 10 //---------------------------Intervall 1
#define onewayTime 42 //----------------------Intervall 2

//------------------------------------------------------

// Param for different pixel layouts
#define kMatrixSerpentineLayout true
#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
CRGB leds_plus_safety_pixel[ NUM_LEDS + 1];
CRGB* const leds( leds_plus_safety_pixel + 1);
SoftwareSerial SerialS(14,12); //define software serial port name as SerialS and define pin2 as RX and pin1 as TX

void animation();
void debugPrint(const char* output, ...);
int dist;//actual distance measurements of LiDAR
int strength;//signal strength of LiDAR
int check;//save check value
int i;
int uart[9];//save data measured by LiDAR
const uint16_t XY(const uint8_t& x, const uint8_t& y);
const int HEADER=0x59;//frame header of data package

//--------------------------------SETUP ------------------------
void setup() {
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setBrightness( BRIGHTNESS );
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  Serial.begin(9600);//set bit rate of serial port connecting Arduino with computer
  //debugPrint("Hello World!");
  SerialS.begin(115200);//set bit rate of serial port connecting LiDAR with Arduino
  Serial.println("hello - starting...");
}

//----------------------------------LOOP -------------------
void loop() 
{ 
  //Wenn Schalter oben ist, starte Animation.
  if (digitalRead(buttonPin1) == HIGH) {
    animation();
  } 
  //LEDs ausschalten, wenn der switch unten ist (Pinkontakt 2+3)
  else if (digitalRead(buttonPin2) == HIGH) {
    powerOff();
  }
  {
  //Serial.println("loop");
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
  }
  }
  }
  }
  }
}


//-------------------------------Animationsfunktion ------------
void animation() {
  static uint8_t xA;
  static uint8_t xB;
  static unsigned long xMillis;
  static long onewayMillis;
  static uint8_t hue;
  static uint8_t y;
  static uint8_t x;
  static uint8_t gegenstrecke = kMatrixHeight-1;
  debugPrint("x: %d   y: %d   gegenstrecke: %d", x, y, gegenstrecke);
  
  unsigned long currentMillis = millis();
  if (currentMillis - xMillis > xTime) {
    xMillis = currentMillis;
    ++x;
    if (x > 5) {
      x = 0;
      ++y;
      fadeToBlackBy(leds, NUM_LEDS, 20);
      if (y > kMatrixHeight-1) {
        y = 0;
      }
    }
    xA = 5 - x;
    xB = 5 + x;
  }
  if (currentMillis - onewayMillis > onewayTime) {
    onewayMillis = currentMillis;
    if (gegenstrecke == 0) {
      gegenstrecke = kMatrixHeight-1;
    } else {
      --gegenstrecke;
    }
  }
    
  ++hue;
  leds[ XY(xA, y)] = CHSV( hue, 255, 255 );
  leds[ XY(xB, y)] = CHSV( hue, 255, 255 );
  leds[ XY(5, gegenstrecke)] = CHSV( hue, 255, 255 );
  FastLED.show();
}
//----------------------------- Debug Funktion ----------------

void debugPrint(const char* output, ...) {
  va_list args;
  char buffer[1024];
  if (digitalRead(buttonPin1) && digitalRead(buttonPin2) && (sprintf (buffer, output, args) >= 0) ) {
    Serial.println(buffer);
  }
}

//----------------------------------Power Off -------------
void powerOff() {
  FastLED.clear();
  FastLED.show();
}

//------------------------ Matrix definitionen ----------
const uint16_t XY(const uint8_t& x, const uint8_t& y) {
  uint16_t returnValue = y * kMatrixWidth;
  
  if( kMatrixSerpentineLayout && y & 0x01) {
    // Odd rows run backwards
    returnValue += kMatrixWidth - 1 - x;
  } else {
    // Even rows run forwards
    returnValue += x;
  }
  return returnValue;
}

