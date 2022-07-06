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

struct RECEIVE_DATA_STRUCTURE{
//struct __attribute__((__packed__)) SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO

  int16_t menuDown;
  int16_t Select;
  int16_t menuUp;
  int16_t toggleBottom;
  int16_t toggleTop;
  int16_t toggle1;
  int16_t toggle2;
  int16_t mode;
  int16_t RLR;
  int16_t RFB;
  int16_t RT;
  int16_t LLR;
  int16_t LFB;
  int16_t LT;

};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE mydata_remote;

RF24 radio(7, 8); // CE, CSN

uint8_t address[][6] = {"1Node", "2Node"};

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
unsigned long remoteMillis; //last time I heard from the remote

bool remoteState;
bool remoteStateOld;

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
  printf_begin();
  radio.printPrettyDetails();

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
      //Serial.println("Disconnected");
    } else {

      //DO STUFF HERE
      
    }

  } //end of timed loop
  
} //end of main loop
