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

#include <Servo.h>

#include <Adafruit_NeoPixel.h>

//************************** PINS *************************************

#define LED_PIN 2

#define RADIO_CE 7
#define RADIO_CSN 8

#define MOTOR_LEFT 5
#define MOTOR_RIGHT 6

//************************ END PINS ***********************************

//*********************** CONSTANTS ***********************************

const byte LED_COUNT = 40;

const byte LED_MODES = 2;

const int MOTOR_OFF = 91;
const int MOTOR_MAX = 175;
const int MOTOR_MIN = 5;
const int CENTER = 5;

const uint8_t address[][6] = {"1Node", "2Node"};

//********************* END CONSTANTS *********************************

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

//*********************** OBJECTS ***********************************


SEND_DATA_STRUCTURE data_send;
RECEIVE_DATA_STRUCTURE data_remote;

RECEIVE_DATA_STRUCTURE data_prev;


RF24 radio(RADIO_CE, RADIO_CSN); // CE, CSN

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

Servo motorLeft;
Servo motorRight;

//********************** VARIABLES **********************************

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
unsigned long remoteMillis; //last time I heard from the remote

bool remoteState;
bool remoteStateOld;

int ledBrightness = 255;
int ledMode = 0;
bool ledRefresh = true;
int ledCounter = 0;
unsigned long ledModeDelta = 0;
unsigned long ledDelta = 0;

//******************** END VARIABLES ********************************

void setup() {
  Serial.begin(115200);

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP

  while (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    delay(500);
  }

  Serial.println("RADIO CONNECTED");
  Serial.println("STARTING AS RECEIVER");

  radio.openWritingPipe(address[1]); // 00001
  radio.openReadingPipe(1, address[0]); // 00002
  radio.setPayloadSize(sizeof(RECEIVE_DATA_STRUCTURE));
  //radio.setChannel(112);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  //printf_begin();
  //radio.printPrettyDetails();

  Serial.println("STARTING MOTORS");

  motorLeft.attach(MOTOR_LEFT);
  motorRight.attach(MOTOR_RIGHT);

  motorLeft.write(30);
  motorRight.write(30);

  Serial.println("STARTUP COMPLETE");

}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event

  if (radio.available()) { //try to receive data
    remoteMillis = currentMillis;
     data_prev = data_remote;
     radio.read(&data_remote, sizeof(RECEIVE_DATA_STRUCTURE));
     remoteMillis = currentMillis;
  }

   // is the remote disconnected for too long ?
    if (currentMillis - remoteMillis > 500) {
      remoteState = false;
    }
    else {
      remoteState = true;
    }

    // if disconnected
    if (!remoteState) {         
      Serial.println("Disconnected");
      motorLeft.write(MOTOR_OFF);
      motorRight.write(MOTOR_OFF);
    } else {

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
        motorR = map(motorR, -512, 512, MOTOR_MIN, MOTOR_MAX);

       if(abs((180/2)-motorL)<CENTER) {
          motorL = MOTOR_OFF;
        }
        if(abs((180/2)-motorR)<CENTER) {
          motorR = MOTOR_OFF;
        }
        
        motorLeft.write(motorL);
        motorRight.write(motorR);
      }

      // LEDS
      //if mode changed or toggle changed
      if((data_remote.encSw4 && !data_prev.encSw4 && ledModeDelta+100<=millis()) || (!data_remote.tgl1 && data_prev.tgl1)) {
        ledModeDelta = millis();
        ledRefresh = true;
        ledBrightness = 255;
        ledMode=(ledMode+1)%LED_MODES;
        Serial.print("LED_MODE  ");
        Serial.println(ledMode);
      }

      if(data_remote.tgl1&&!data_prev.tgl1 || ledRefresh) {
         for(int i= 0;i<LED_COUNT;i++) {
            strip.setPixelColor(i, strip.Color(0, 0, 0));
         }
         strip.show();
         Serial.println("LEDS OFF");
      } 
      if(!data_remote.tgl1) {
        int t = max(10, min(1000, (data_remote.encVal3*10)));
        switch(ledMode) {
          case 0:
            if(data_remote.encVal2 !=data_prev.encVal2 || ledRefresh) {
              for(int i= 0;i<LED_COUNT;i++) {
                strip.setPixelColor(i, Wheel((data_remote.encVal2*5)%255));
              }
              strip.show();
            }
          break;
          case 1:
            if(ledDelta+t<=millis()) {
              ledDelta = millis();
              strip.setPixelColor(ledCounter, 0);
              strip.setPixelColor((ledCounter+1)%LED_COUNT, 0);
              strip.setPixelColor(((ledCounter+(LED_COUNT/2))%LED_COUNT), 0);
              strip.setPixelColor(((ledCounter+1+(LED_COUNT/2))%LED_COUNT), 0);
              ledCounter= (ledCounter+1)%LED_COUNT;
              strip.setPixelColor(ledCounter, Wheel((data_remote.encVal2*5)%255));
              strip.setPixelColor((ledCounter+1)%LED_COUNT, Wheel((data_remote.encVal2*5)%255));
             strip.setPixelColor(((ledCounter+(LED_COUNT/2))%LED_COUNT), Wheel((data_remote.encVal2*5)%255));
              strip.setPixelColor(((ledCounter+1+(LED_COUNT/2))%LED_COUNT), Wheel((data_remote.encVal2*5)%255));
              strip.show();
            }
          break;
        }
        ledRefresh = false;
      }        
    }
  } //end of timed loop
  
} //end of main loop

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
