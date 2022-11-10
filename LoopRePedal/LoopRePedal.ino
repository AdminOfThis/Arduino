#include <SPI.h>
#include <Wire.h>

#include "MIDIUSB.h"
#include <Adafruit_NeoPixel.h>

#include <U8x8lib.h>

#define PIN_LED 4 // Pin of the LED REC_RING

#define PIN_BANK 5
#define PIN_CLR 6

#define PIN_REC 7
#define PIN_STOP 8

#define PIN_UNDO 9
#define PIN_MODE 10

#define PIN_CH1 11
#define PIN_CH2 12
#define PIN_CH3 A0
#define PIN_CH4 A1

#define PIN_FX1 A2
#define PIN_FX2 A3
#define PIN_FX3 A4
#define PIN_FX4 A5

const uint8_t BUTTONS_PINS[] = {PIN_BANK, PIN_CLR, PIN_REC, PIN_STOP, PIN_UNDO, PIN_MODE, PIN_CH1, PIN_CH2, PIN_CH3, PIN_CH4, PIN_FX1, PIN_FX2, PIN_FX3, PIN_FX4};

#define LED_RINGS 14 // Number of LED rings
#define LED_COUNT 8  // Number of LEDs per ring

#define DEBOUNCE 10 // Debounce of 10ms
#define LED_DELTATIME 200

#define TIMED_LOOP_DELTA 100

// MIDI SIGNALS
#define REC 1
#define PLAY_STOP 2

#define BRIGHTNESS 63

int COLOR_OFF[3] = {0, 0, 0};
int COLOR_GREEN[3] = {0, 255, 0};
int COLOR_RED[3] = {255, 0, 0};

enum LED_MODE
{
  OFF,
  ON,
  SPINNING
};

bool record = false;
bool play = false;

// U8X8_SSD1309_128X64_NONAME0_4W_HW_SPI screen(OLED_CS, OLED_DC, OLED_RESET);

Adafruit_NeoPixel LED(LED_COUNT *LED_RINGS, PIN_LED, NEO_GRB + NEO_KHZ800);
int color[LED_RINGS][3];
LED_MODE ledmode[LED_RINGS];

uint8_t pixel = 0;
long lastPixelChange = 0;

bool buttonState[LED_RINGS];
bool prevButtonState[LED_RINGS];
long lastButtonChange[LED_RINGS];

long time = 0;
long lastTimedChange = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting LoopPedal v1.0");

  // initialize Buttons
  for (int i = 0; i < LED_RINGS; i++)
  {
    pinMode(BUTTONS_PINS[i], INPUT_PULLUP);
  }

  // initialize LED
  LED.begin();
  LED.setBrightness(BRIGHTNESS);
  for (int i = 0; i < LED_COUNT; i++)
  {
    for (int j = 0; j < LED_RINGS; j++)
    {
      for (int k = 0; k <= i; k++)
      {
        LED.setPixelColor((j * LED_COUNT) + k, LED.Color(255 / LED_COUNT * i, 164 / LED_COUNT * i, 255 - (255 / LED_COUNT * i)));
      }
    }
    LED.show();
    delay(1000 / LED_COUNT);
  }
  delay(1000);

  for (int i = 0; i < LED_RINGS; i++)
  {
    ledmode[i] = OFF;
    color[i][0] = COLOR_GREEN[0];
    color[i][1] = COLOR_GREEN[1];
    color[i][2] = COLOR_GREEN[2];
  }

  // screen.begin(); // initialite display
  // screen.setContrast(255);
  // screen.setFont(u8x8_font_8x13_1x2_f); // select font
  // screen.drawString(4,4, "Hello, World");
}

void loop()
{

  time = millis();

  checkButtons();

  // ********************************************* Timed Loop ********************************************************************
  if ((time - (TIMED_LOOP_DELTA)) > lastTimedChange)
  {
    lastTimedChange = time; // reset timed loop

    for (int i = 0; i < LED_RINGS; i++) // DEBUG CODE
    {
      ledmode[i] = buttonState[i] ? ON : OFF;
    }

    manageLEDs(); // update LED pattern
    readMIDI();   // read incoming MIDI signals
  }
  // ********************************************* End Timed Loop ********************************************************************

  // LED delay counter
  if ((time - (LED_DELTATIME)) > lastPixelChange)
  {
    lastPixelChange = time;                // reset led rollover timer
    pixel = (pixel + 1) % (LED_COUNT / 2); // rollover LED index
  }
}

void checkButtons()
{
  for (int i = 0; i < LED_RINGS; i++)
  {

    // read the state of the switch into a local variable:
    bool state = !digitalRead(BUTTONS_PINS[i]); // read rawSignal

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (state != prevButtonState[i])
    {
      // reset the debouncing timer
      lastButtonChange[i] = time;
    }

    if ((time - lastButtonChange[i]) > DEBOUNCE)
    {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (state != buttonState[i])
      {
        buttonState[i] = state;
      }
    }
    prevButtonState[i] = state;
  }
}

void manageLEDs()
{
  for (int i = 0; i < LED_RINGS; i++)
  {
    switch (ledmode[i])
    {
    case OFF:
      showColor(i, COLOR_OFF, false);
      break;
    case ON:
      showColor(i, color[i], false);
      break;
    case SPINNING:
      spin(i);
      break;
    }
  }
  LED.show();
}

void spin(int ringIndex)
{
  for (int i = 0; i < LED_COUNT; i++)
  {

    if (i == pixel || i == pixel + 4) // main pixel
    {
      LED.setPixelColor(ringIndex * LED_COUNT + i, LED.Color(color[ringIndex][0], color[ringIndex][1], color[ringIndex][2]));
    }
    else if (i == ((pixel + LED_COUNT) - 1) % LED_COUNT || i == ((pixel + 4 + LED_COUNT) - 1) % LED_COUNT) // tail pixel
    {
      LED.setPixelColor(ringIndex * LED_COUNT + i, LED.Color(color[ringIndex][0] / 6, color[ringIndex][1] / 6, color[ringIndex][2] / 6));
    }
    else
    {
      LED.setPixelColor(ringIndex * LED_COUNT + i, LED.Color(0, 0, 0));
    }
  }
}

void showColor(int ringIndex, int color[3], bool show)
{
  if (ringIndex < 0 || ringIndex >= LED_RINGS)
  {
    return;
  }
  for (int i = ringIndex * LED_COUNT; i < (ringIndex + 1) * LED_COUNT; i++)
  {
    LED.setPixelColor(i, LED.Color(color[0], color[1], color[2]));
  }
  if (show)
  {
    LED.show();
  }
}

// void checkButtons()
// {
//   bool REC_state = !digitalRead(REC_BUTTON);

//   if (REC_state != REC_prevState && REC_lastChange + DEBOUNCE <= millis())
//   {

//     if (REC_state)
//     {
//       noteOn(0, 93, 127);
//     }
//     else
//     {
//       noteOff(0, 93, 127);
//     }
//     MidiUSB.flush();

//     REC_prevState = REC_state;
//     REC_lastChange = millis();

//     Serial.println("REC_BUTTON");
//   }

//   bool PLAY_state = !digitalRead(PLAY_BUTTON);
//   if (PLAY_state != PLAY_prevState && PLAY_lastChange + DEBOUNCE <= millis())
//   {
//     if (play)
//     {
//       if (PLAY_state)
//       {
//         noteOn(0, 91, 127);
//       }
//       else
//       {
//         noteOff(0, 91, 127);
//       }

//       Serial.println("PLAY_BUTTON");
//     }
//     else
//     {
//       if (PLAY_state)
//       {
//         noteOn(0, 92, 127);
//       }
//       else
//       {
//         noteOff(0, 92, 127);
//       }
//     }
//     MidiUSB.flush();

//     PLAY_prevState = PLAY_state;
//     PLAY_lastChange = millis();

//     Serial.println("PLAY/STOP_BUTTON");
//   }
// }

void readMIDI()
{
  midiEventPacket_t rx = MidiUSB.read();

  if (rx.header != 0)
  {

    Serial.print(rx.header);
    Serial.print(" , ");
    Serial.print(rx.byte1);
    Serial.print(" , ");
    Serial.print(rx.byte2);
    Serial.print(" , ");
    Serial.print(rx.byte3);
    Serial.println(" , ");

    if (rx.header = 0xB && rx.byte1 == 0xB0)
    {
      switch (rx.byte2)
      {
      case REC:
        record = rx.byte3 == 0x7F;
        // Serial.println("RECORD " + record);
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

bool equal(midiEventPacket_t t1, midiEventPacket_t t2)
{
  return t1.header == t2.header && t1.byte1 == t2.byte1 && t1.byte2 == t2.byte2 && t1.byte3 == t2.byte3;
}

// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

void noteOn(byte channel, byte pitch, byte velocity)
{

  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};

  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity)
{

  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};

  MidiUSB.sendMIDI(noteOff);
}
