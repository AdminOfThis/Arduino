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

//************************** PINS *************************************

#define RADIO_CE 7
#define RADIO_CSN 8

#define MOTOR_LEFT 5
#define MOTOR_RIGHT 6

//************************ END PINS ***********************************

//*********************** CONSTANTS ***********************************

const int MOTOR_OFF = 91;
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


SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE mydata_remote;

RF24 radio(RADIO_CE, RADIO_CSN); // CE, CSN

Servo motorLeft;
Servo motorRight;

//********************** VARIABLES **********************************

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
unsigned long remoteMillis; //last time I heard from the remote

bool remoteState;
bool remoteStateOld;

//******************** END VARIABLES ********************************

void setup() {
  Serial.begin(115200);

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

     radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
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

      if(mydata_remote.tgl6) {
         motorLeft.write(MOTOR_OFF);
         motorRight.write(MOTOR_OFF);
      } else {

        //int motorL = mydata_remote.joy1Y -(mydata_remote.joy1X-512)*2;

        long x1 = mydata_remote.joy1X-512;
        long y1 = mydata_remote.joy1Y-512;
        long z1 = mydata_remote.joy1Z-512;
        if(abs(z1)<CENTER) {
          z1 = 0;
        }

        double motorL = max(-512, min(512, y1 - min(0, -x1) + z1));
        double motorR = max(-512, min(512, y1 - min(0, x1) - z1));
         
        motorL = map(motorL, -512, 512, 0, 179);
        motorR = map(motorR, -512, 512, 0, 179);

        Serial.print( motorL);
        Serial.print(",");
        Serial.println(motorR);

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
