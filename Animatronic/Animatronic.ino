#include <EEPROM.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#include <SPI.h>
#include <U8x8lib.h>

#include <printf.h>

#include <nRF24L01.h>
#include <RF24.h>

#define ENCODER_DO_NOT_USE_INTERRUPTS

#define PIN_1X 2
#define PIN_1Y 3
#define PIN_1Z 4
#define PIN_1LED 5

#define PIN_2X A3
#define PIN_2Y A4
#define PIN_2Z A5

#define PIN_NECK_L A0
#define PIN_NECK_R A1

#define PIN_ENC1 A6
#define PIN_ENC2 A7
#define PIN_ENC_BUT 16

#define DISPLAY_DC 8
#define DISPLAY_RST 9
#define DISPLAY_CS 10

#define RADIO_CSN 7
#define RADIO_CE 6

#define LED_COUNT 8

struct RECEIVE_DATA_STRUCTURE {
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


Servo eye1X;  
Servo eye1Y;  
Servo eye1Z;

Servo eye2X;  
Servo eye2Y;  
Servo eye2Z;

Servo neckL;
Servo neckR;

Adafruit_NeoPixel eyeLED(8, PIN_1LED, NEO_GRB + NEO_KHZ800);

U8X8_SSD1306_128X64_NONAME_4W_HW_SPI screen( DISPLAY_CS, DISPLAY_DC, DISPLAY_RST);

RF24 radio(RADIO_CE, RADIO_CSN); // CE, CSN

uint8_t address[][6] = {"1Node", "2Node"};

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE mydata_remote;

int minimum[8];
int maximum[8];

const double smoothing = .95; 

double pos1X =90;
double pos1Y =90;
double pos1Z =135;

double pos2X =90;
double pos2Y =90;
double pos2Z =135;

double posNeckL = 90;
double posNeckR = 90;

int valEncoder=0;
bool valAPrev;

bool remoteState;
bool remoteStateOld;

bool prevBtnState = false;

unsigned long previousMillis;
unsigned long remoteMillis; //last time I heard from the remote

void setup() {

  readIntArrayFromEEPROM(0, minimum,8 );
  readIntArrayFromEEPROM(16, maximum,8 );
  
  Serial.begin(115200);

  pinMode(PIN_ENC_BUT, INPUT_PULLUP);
  
  eye1X.attach(PIN_1X); 
  eye1Y.attach(PIN_1Y); 
  eye1Z.attach(PIN_1Z); 

  eye2X.attach(PIN_2X); 
  eye2Y.attach(PIN_2Y); 
  eye2Z.attach(PIN_2Z);

  neckL.attach(PIN_NECK_L);
  neckR.attach(PIN_NECK_R);

  eyeLED.begin();
  eyeLED.show();

  eye1X.write((minimum[0]+maximum[0])/2);
  eye1Y.write((minimum[1]+maximum[1])/2);
  eye1Z.write((minimum[2]+maximum[2])/2);
  
  eye2X.write((minimum[3]+maximum[3])/2);
  eye2Y.write((minimum[4]+maximum[4])/2);
  eye2Z.write((minimum[5]+maximum[5])/2);

  neckL.write((minimum[6]+maximum[6])/2);
  neckR.write((minimum[7]+maximum[7])/2);

  screen.begin();
  screen.setFont(u8x8_font_chroma48medium8_r );  

  screen.drawString(0,0,"Starting radio");

  delay(2000);

  setupRadio();
 
  previousMillis = millis();
  
  bool setupFlag = false;

  screen.clear();
  screen.drawString(0,0,"Press button");
  screen.drawString(0,1,"for setup");
  while(millis() - previousMillis <= 2000) {
    if(!digitalRead(PIN_ENC_BUT)) {
      setupFlag = true;
      break;
    }
  }
   screen.clear();
  if(setupFlag) {
    screen.drawString(0,0, "Going to setup");
    delay(1000);
    updateEEPROM();
  }
   
  screen.drawString(0,0,"Setup complete");
  delay(2000);
  screen.clear();
  delay(500);
}

void loop() {

  long currentMillis = millis();

  readEncoder(valEncoder, readPseudoDigital(analogRead(PIN_ENC1)), readPseudoDigital(analogRead(PIN_ENC2)), valAPrev);

  if (currentMillis - previousMillis >= 10) {

    if (radio.available()) { //try to receive data
      remoteMillis = currentMillis;

      radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
      remoteMillis = currentMillis;
    }

    // is the remote disconnected for too long ?
    if (currentMillis - remoteMillis > 500) {
      remoteState = false;
      screen.drawString(0,14, "N");
    }
    else {
      remoteState = true;
      screen.drawString(0,14, "C");
    }

    if(radio.failureDetected) {

      Serial.println("FAILURE");
      screen.drawString(0,0, "FAILURE");

      setupRadio();
    }

   if(remoteState) {
     pos1X = smoothing*pos1X + (1-smoothing)*map(mydata_remote.joy1X, 0, 1023, maximum[0], minimum[0]);
     pos1Y = smoothing*pos1Y + (1-smoothing)*map(mydata_remote.joy1Y, 0, 1023, minimum[1], maximum[1]);
     pos1Z = smoothing*pos1Z + (1-smoothing)*map(mydata_remote.joy1Z, 0, 1023, maximum[2], minimum[2]);

     pos2X = smoothing*pos2X + (1-smoothing)*map(mydata_remote.joy1X, 0, 1023, maximum[3], minimum[3]);
     pos2Y = smoothing*pos2Y + (1-smoothing)*map(mydata_remote.joy1Y, 0, 1023, minimum[4], maximum[4]);
     pos2Z = smoothing*pos2Z + (1-smoothing)*map(mydata_remote.joy1Z, 0, 1023, maximum[5], minimum[5]);

     posNeckL = smoothing*posNeckL + (1-smoothing)*(map(mydata_remote.joy2X + (mydata_remote.joy2Y-512), 0, 1023, maximum[6], minimum[6]));
     posNeckR = smoothing*posNeckR + (1-smoothing)*(map(mydata_remote.joy2X - (mydata_remote.joy2Y-512), 0, 1023, maximum[7], minimum[7]));
   }
 
   eye1X.write(pos1X);
   eye1Y.write(pos1Y);
   eye1Z.write(pos1Z);

   eye2X.write(pos2X);
   eye2Y.write(pos2Y);
   eye2Z.write(pos2Z);

   neckL.write(posNeckL);
   neckR.write(posNeckR);

   previousMillis = currentMillis;
     
  }
}

void updateEEPROM() {
 
 eye1X.write((minimum[0]+maximum[0])/2);
 eye1Y.write((minimum[1]+maximum[1])/2);
 eye1Z.write((minimum[2]+maximum[2])/2);
 eye2X.write((minimum[3]+maximum[3])/2);
 eye2Y.write((minimum[4]+maximum[4])/2);
 eye2Z.write((minimum[5]+maximum[5])/2);
 neckL.write((minimum[6]+maximum[6])/2);
 neckR.write((minimum[7]+maximum[7])/2);

 updateThisNumber(minimum[0], eye1X, "x1 minimum");
 updateThisNumber(maximum[0], eye1X, "x1 maximum");
 eye1X.write((minimum[0]+maximum[0])/2);
 
 updateThisNumber(minimum[1], eye1Y, "y1 minimum");
 updateThisNumber(maximum[1], eye1Y, "y1 maximum");
 eye1Y.write((minimum[1]+maximum[1])/2);
 
 updateThisNumber(minimum[2], eye1Z, "z1 minimum");
 updateThisNumber(maximum[2], eye1Z, "z1 maximum");
 eye1Z.write((minimum[2]+maximum[2])/2);
 
 updateThisNumber(minimum[3], eye2X, "x2 minimum");
 updateThisNumber(maximum[3], eye2X, "x2 maximum");
 eye2X.write((minimum[3]+maximum[3])/2);
 
 updateThisNumber(minimum[4], eye2Y, "y2 minimum");
 updateThisNumber(maximum[4], eye2Y, "y2 maximum");
 eye2Y.write((minimum[4]+maximum[4])/2);
 
 updateThisNumber(minimum[5], eye2Z, "z2 minimum");
 updateThisNumber(maximum[5], eye2Z, "z2 maximum");
 eye2Z.write((minimum[5]+maximum[5])/2);

 updateThisNumber(minimum[6], neckL, "neckL minimum");
 updateThisNumber(maximum[6], neckL, "neckL maximum");
 neckL.write((minimum[6]+maximum[6])/2);

 updateThisNumber(minimum[7], neckR, "neckR minimum");
 updateThisNumber(maximum[7], neckR, "neckR maximum");
 neckR.write((minimum[7]+maximum[7])/2);

 screen.clear();
 writeIntArrayIntoEEPROM(0, minimum, 8);
 writeIntArrayIntoEEPROM(16, maximum, 8);
}

void printInt(int v) {
  char cstr[16];
  itoa(v, cstr, 10);
  screen.drawString(0,3,"                ");
  screen.drawString(0,3,cstr);
}

void updateThisNumber(int& val, Servo& servo, char* s) {

  valEncoder = val;
  
  screen.clear();
  screen.drawString(0,0,"Turn until");
  screen.drawString(0,1, s);
  printInt(val);
  servo.write(val);

  while(digitalRead(PIN_ENC_BUT)) {

    readEncoder(valEncoder, readPseudoDigital(analogRead(PIN_ENC1)), readPseudoDigital(analogRead(PIN_ENC2)), valAPrev);

    if(valEncoder!=val) {
      val=max( min(valEncoder, 180), 0);
      valEncoder = val;
      printInt(val);
      servo.write(val);
    }
  }
  
  screen.clear();
  screen.drawString(0,0,"DONE");
  delay(500);
}

void readEncoder(int& val, bool val1, bool val2, bool& valAPrev) {

  if ((valAPrev == HIGH) && (val1 == LOW)) {
    if (val2 == HIGH) {
      val--;
    } else {
      val++;
    }
    //Serial.print (val);
    //Serial.print ("/");
  }
  valAPrev = val1;
}

bool readPseudoDigital(int val) {
  return (val>=512);
}

void setupRadio() {
  while (!radio.begin()) {
    //Serial.println("Radio hardware not responding!");
    screen.drawString(0,0,"Starting radio");
    delay(500);
  }

  radio.failureDetected=0;
  
  radio.openWritingPipe(address[1]); // 00001
  radio.openReadingPipe(1, address[0]); // 00002
  radio.setPayloadSize(sizeof(RECEIVE_DATA_STRUCTURE));
  //radio.setChannel(112);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  printf_begin();
  radio.printPrettyDetails();
}

void writeIntArrayIntoEEPROM(int address, int numbers[], int arraySize)
{
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++) 
  {
    EEPROM.update(addressIndex, numbers[i] >> 8);
    EEPROM.update(addressIndex + 1, numbers[i] & 0xFF);
    addressIndex += 2;
  }
}
void readIntArrayFromEEPROM(int address, int numbers[], int arraySize)
{
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++)
  {
    int val = (EEPROM.read(addressIndex) << 8) + EEPROM.read(addressIndex + 1);
    if(val<0) {
      numbers[i] = 90;
    } else {
      numbers[i] = val;
    }
    addressIndex += 2;
  }
}
