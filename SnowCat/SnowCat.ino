/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <printf.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <U8x8lib.h>

#include <Servo.h>

#include <Adafruit_NeoPixel.h>

//************************** PINS *************************************

#define LED_PIN 2

#define MOTOR_LEFT 4
#define MOTOR_RIGHT 3

#define DISPLAY_DC 10
#define DISPLAY_RST 9
#define DISPLAY_CS 6

#define RADIO_CE 7
#define RADIO_CSN 8

//Encoder 1 is directly soldered to the Arduino on PIN A0-A2
#define ENC1_SW A0
#define ENC1_1 A1
#define ENC1_2 A2

//************************ STRUCTS ***********************************

struct RECEIVE_DATA_STRUCTURE{
  //struct __attribute__((__packed__)) SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO

  bool tgl1;
  bool tgl2;
  bool tgl3;
  bool tgl4;
  bool tgl5;
  bool tgl6;

  int16_t encVal1;
  int16_t encVal2;
  int16_t encVal3;
  int16_t encVal4;

  bool encSw1;
  bool encSw2;
  bool encSw3;
  bool encSw4;

  // Joystick
  bool joy1Btn;
  int16_t joy1X;
  int16_t joy1Y;
  int16_t joy1Z;

  bool joy2Btn;
  int16_t joy2X;
  int16_t joy2Y;
  int16_t joy2Z;
};

struct SEND_DATA_STRUCTURE{
   //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};



//*********************** CONSTANTS ***********************************
const long STARTUP_DELAY = 1000;

const byte LED_COUNT = 5;

const byte LED_MODES = 2;

const int MOTOR_OFF = 91;
const int MOTOR_MAX = 175;
const int MOTOR_MIN = 5;
const int CENTER = 5;

const uint8_t address[][6] = {"1Node", "2Node"};

//********************** VARIABLES **********************************

bool startup = true;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
unsigned long remoteMillis; //last time I heard from the remote

bool remoteState = false;
bool remoteStateOld = true;

int ledBrightness = 255;

int posEncoder = 0;
int posEncoderPrev = 0;
bool prevEnVal = HIGH;
bool buttonPressed = LOW;
bool prevButtonPressed = HIGH;

//*********************** OBJECTS ***********************************


SEND_DATA_STRUCTURE data_send;
RECEIVE_DATA_STRUCTURE data_remote;

RECEIVE_DATA_STRUCTURE data_prev;


RF24 radio(RADIO_CE, RADIO_CSN); // CE, CSN

U8X8_SSD1306_128X64_NONAME_4W_HW_SPI screen( DISPLAY_CS, DISPLAY_DC, DISPLAY_RST);

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

Servo motorLeft;
Servo motorRight;


//*******************************************************************
//*******************************************************************
//*******************************************************************
void setup() {
  Serial.begin(115200);
  
  pinMode(ENC1_SW, INPUT_PULLUP);
  pinMode(ENC1_1, INPUT_PULLUP);
  pinMode(ENC1_2, INPUT_PULLUP);

  screen.begin();
  screen.setFont(u8x8_font_chroma48medium8_r );  

  
  
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  for(int i=0;i<LED_COUNT;i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show();            // Turn OFF all pixels ASAP

  startupMessage("Init I/O..." , 4);
  startupMessage("Init radio..." , 3);

  while (!radio.begin()) {
    screen.clear();
    screen.drawString(0,3, "Radio error!");
    strip.setPixelColor(3, strip.Color(ledBrightness, 0, 0));
    strip.show();
    delay(500);
  }

  startupMessage("Init receiver..." , 2);
   
  radio.openWritingPipe(address[1]); // 00001
  radio.openReadingPipe(1, address[0]); // 00002
  radio.setPayloadSize(sizeof(RECEIVE_DATA_STRUCTURE));
  //radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  printf_begin();
  radio.printPrettyDetails();

  motorLeft.attach(MOTOR_LEFT);
  motorRight.attach(MOTOR_RIGHT);

  motorLeft.write(30);
  motorRight.write(30);

  startupMessage("Init motors" , 1);
  
  startupMessage("Startup done" , 0);
  startupMessage("" , -1);
  
}


void loop() {

  currentMillis = millis();

  readEncoder(posEncoder, digitalRead(ENC1_2), digitalRead(ENC1_1), prevEnVal);
  
  if (currentMillis - previousMillis >= 10) {  // start timed event
  if (radio.available()) { //try to receive data
     remoteMillis = currentMillis;
     data_prev = data_remote;
     radio.read(&data_remote, sizeof(RECEIVE_DATA_STRUCTURE));
     remoteMillis = currentMillis;
  }

   // is the remote disconnected for too long ?
    if (currentMillis - remoteMillis > 100) {
      remoteState = false;
    }
    else {
      remoteState = true;
    }

    if(startup) {
      remoteState = false;
      remoteStateOld = true;
      startup = false;
    }

    // if disconnected
    if (!remoteState && remoteStateOld) {      
      remoteStateOld = remoteState;         
      //Serial.println("Disconnected");
      motorLeft.write(MOTOR_OFF);
      motorRight.write(MOTOR_OFF);

      screen.clear();
      screen.drawString(0, 3, "  DISCONNECTED");

      for(int i=0;i<LED_COUNT;i++) {
        strip.setPixelColor(i, strip.Color(ledBrightness, 0, 0));
      }
      strip.show(); 
      
    } else if(remoteState){

      if(remoteState && !remoteStateOld) {
        remoteStateOld = remoteState;
        
        screen.clear();
        screen.drawString(0, 3, "   CONNECTED");  
        
        for(int i=0;i<LED_COUNT;i++) {
          strip.setPixelColor(i, strip.Color(0, ledBrightness, 0));
        }
        strip.show(); 
      }

      // MOTORS
      if(data_remote.tgl6) {
         motorLeft.write(MOTOR_OFF);
         motorRight.write(MOTOR_OFF);
      } else {
        long x1 = data_remote.joy1X-512;
        long y1 = data_remote.joy1Y-512;
        long z1 = data_remote.joy1Z-512;
        if(abs(z1)<CENTER) {
          z1 = 0;
        }

        double motorL = max(-512, min(512, y1 - min(0, -x1) - z1));
        double motorR = max(-512, min(512, y1 - min(0,x1) + z1));
         
        motorL = map(motorL, -512, 512, MOTOR_MIN, MOTOR_MAX);
        motorR = map(motorR, -512, 512, MOTOR_MIN, MOTOR_MAX );

       if(abs((180/2)-motorL)<CENTER) {
          motorL = MOTOR_OFF;
        }
        if(abs((180/2)-motorR)<CENTER) {
          motorR = MOTOR_OFF;
        }
        
        motorLeft.write(motorL);
        motorRight.write(motorR);
      }
    }
  } //end of timed loop
  
} //end of main loop

void startupMessage(char* message, int ledPin) {
  screen.clear();
  screen.drawString(0 ,3 , message);
  
  strip.setPixelColor(ledPin, strip.Color(0, 0, ledBrightness));
  if(ledPin<=4) {
    strip.setPixelColor(ledPin+1, strip.Color(0, ledBrightness, 0));
  }
  
  strip.show();
  long t = millis();
  while(t+STARTUP_DELAY>millis()&&digitalRead(ENC1_SW)) { } //delay, but shortcut with button
}

void readEncoder(int& val, bool val1, bool val2, bool& valAPrev) {
  if ((valAPrev == HIGH) && (val1 == LOW)) {
    if (val2 == HIGH) {
      val = max(-999, val-1);      
    } else {
      val = min(999, val+1);
    }
  }
  valAPrev = val1;
}
