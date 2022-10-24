#include "MIDIUSB.h"
#include <Adafruit_NeoPixel.h>

#define REC_BUTTON 4 // Recording Button is on D4
#define PLAY_BUTTON 5 

#define REC_RING 6 //Pin of the LED REC_RING
#define PLAY_RING 7

#define LEDS 8

#define DEBOUNCE 10 //Debounce of 50ms

// MIDI SIGNALS
#define REC 1
#define PLAY_STOP 2

bool record = false;
bool play = false;

Adafruit_NeoPixel REC_STRIP(LEDS, REC_RING, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel PLAY_STRIP(LEDS, PLAY_RING, NEO_GRB + NEO_KHZ800);
uint8_t pixel = 0;
long t = 0;
int color[] ={0,0,0};

bool REC_prevState = LOW;
long REC_lastChange = 0;

bool PLAY_prevState = LOW;
long PLAY_lastChange = 0;

void setup() {
   
  //initialize Buttons
  
  pinMode(REC_BUTTON, INPUT_PULLUP);
  pinMode(PLAY_BUTTON, INPUT_PULLUP);
  
  REC_STRIP.begin();
  REC_STRIP.show();

  PLAY_STRIP.begin();
  PLAY_STRIP.show();
  
  Serial.begin(115200);

  delay(1000);

  Serial.println("Starting LoopPedal v1.0");
}

int i=0;

void loop() {

  i=(i+1)%8;
  
  for(int j=0;j<40;j++) {
    Serial.print("SEND ");
    Serial.println(i);
    noteOn(0x90,j,i);
  }
    MidiUSB.flush();
    delay(5000);    
}

void checkButtons() {
  bool REC_state = !digitalRead(REC_BUTTON);
  
  if(REC_state != REC_prevState && REC_lastChange+DEBOUNCE<=millis()) {

      if(REC_state) {
        noteOn(0, 93, 127);
      } else {
        noteOff(0, 93, 127);
      }
       MidiUSB.flush();
      
      REC_prevState = REC_state;
      REC_lastChange = millis();

      Serial.println("REC_BUTTON");
    }

    bool PLAY_state = !digitalRead(PLAY_BUTTON);
    if(PLAY_state != PLAY_prevState && PLAY_lastChange+DEBOUNCE<=millis()) {
  if(play) {
      if(PLAY_state) {
        noteOn(0, 91, 127);
      } else {
        noteOff(0, 91, 127);
      }

      Serial.println("PLAY_BUTTON");
  } else {
    if(PLAY_state) {
        noteOn(0, 92, 127);
      } else {
        noteOff(0, 92, 127);
      }
  }
       MidiUSB.flush();
      
      PLAY_prevState = PLAY_state;
      PLAY_lastChange = millis();

      Serial.println("PLAY/STOP_BUTTON");
    }
}

void manageLEDs() {
  if(t+150<millis()) {  
    
    t = millis();

    turnOffLED(REC_STRIP);
    turnOffLED(PLAY_STRIP);
    
    if(record) {
      REC_STRIP.setPixelColor((pixel)%LEDS, 255,0,0);
      REC_STRIP.setPixelColor((pixel+LEDS/2)%LEDS, 255,0,0);
      REC_STRIP.show();
    }

    if(play) {
      PLAY_STRIP.setPixelColor(pixel, 0, 255, 0);
      PLAY_STRIP.setPixelColor((pixel+LEDS/2)%LEDS, 0, 255, 0);
      PLAY_STRIP.show();
    }
 
    pixel = (pixel+1)%LEDS;
    
    }
}

void turnOffLED(Adafruit_NeoPixel& strip) {
  for(int i=0;i<LEDS;i++) {
     strip.setPixelColor(i, 0, 0, 0);
  }
  strip.show();
}

void turnOffLED(Adafruit_NeoPixel& strip, int i) {
  strip.setPixelColor(i, 0, 0, 0);
  strip.show();
}

void readMIDI() {
  midiEventPacket_t rx = MidiUSB.read();
  
    if (rx.header != 0) {
      
      Serial.print(rx.header);
      Serial.print(" , ");
      Serial.print(rx.byte1);
      Serial.print(" , ");
      Serial.print(rx.byte2);
      Serial.print(" , ");
      Serial.print(rx.byte3);
      Serial.println(" , ");
      

      
      if(rx.header = 0xB && rx.byte1 == 0xB0) {
        switch(rx.byte2) {
          case REC:
            record = rx.byte3 == 0x7F;
            //Serial.println("RECORD " + record);
          break;
          case PLAY_STOP:
            play = rx.byte3 == 0x7F;
          break;
          default:
            Serial.print("Received unknown: ");
            Serial.print(rx.header, HEX);
            Serial.print("-");
            Serial.print(rx.byte1, HEX);
            Serial.print("-");
            Serial.print(rx.byte2, HEX);
            Serial.print("-");
            Serial.println(rx.byte3, HEX);
        }
      }
    }
}

bool equal (midiEventPacket_t t1, midiEventPacket_t t2) {
  return t1.header == t2.header && t1.byte1 == t2.byte1 && t1.byte2 == t2.byte2 && t1.byte3 == t2.byte3;
}

// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

void noteOn(byte channel, byte pitch, byte velocity) {

  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};

  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {

  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};

  MidiUSB.sendMIDI(noteOff);
}
