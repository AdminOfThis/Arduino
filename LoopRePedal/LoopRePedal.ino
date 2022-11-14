#include <SPI.h>
#include <Wire.h>

#include "MIDIUSB.h"
#include <Adafruit_NeoPixel.h>

#include <U8x8lib.h>

#define PIN_LED 4 // Pin of the LED REC_RING

#define PIN_ENC1 MOSI
#define PIN_ENC2 MISO
#define PIN_ENC_BUT SCK

#define PIN_DIS_SDA SDA
#define PIN_DIS_SCK SCL

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

const uint8_t BUTTONS_PINS[] = {PIN_BANK, PIN_CLR, PIN_REC, PIN_STOP, PIN_UNDO, PIN_MODE, PIN_CH1, PIN_CH2, PIN_CH3, PIN_CH4, PIN_FX1, PIN_FX2, PIN_FX3, PIN_FX4, PIN_ENC_BUT};

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
int COLOR_BLUE[3] = {0, 0, 255};
int COLOR_YELLOW[3] = {255, 255, 0};
int COLOR_ORANGE[3] = {255, 127, 0};

enum LED_MODE
{
  OFF,
  ON,
  SPINNING,
  BLINK
};

bool record = false;
bool play = true;

U8X8_SSD1306_128X64_NONAME_HW_I2C screen(/* reset=*/U8X8_PIN_NONE);

Adafruit_NeoPixel LED(LED_COUNT *LED_RINGS, PIN_LED, NEO_GRB + NEO_KHZ800);
int color[LED_RINGS][3];
LED_MODE ledMode[LED_RINGS];

uint8_t pixel = 0;
long lastPixelChange = 0;

bool buttonState[LED_RINGS+1];
bool buttonPushed[LED_RINGS+1];
bool prevButtonState[LED_RINGS+1];
long lastButtonChange[LED_RINGS+1];

long time = 0;
long lastTimedChange = 0;

int menuIndex = 0;
bool encValPrev = LOW;

// *************************************** MIDI VARIABLES ***********************************************

bool modeRec = true;
bool stopped = true;
bool recording = false;
int selectedChannel = 0;

bool chnState[4] = {HIGH, HIGH, HIGH, HIGH};
bool fxState[4];

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting LoopPedal v1.0");

  // initialize Buttons
  for (int i = 0; i < LED_RINGS; i++)
  {
    pinMode(BUTTONS_PINS[i], INPUT_PULLUP);
  }
  pinMode(PIN_ENC_BUT, INPUT_PULLUP);
  pinMode(PIN_ENC1, INPUT_PULLUP);
  pinMode(PIN_ENC2, INPUT_PULLUP);

  screen.begin(); // initialite display
  screen.setPowerSave(0);
  screen.setFont(u8x8_font_8x13_1x2_f); // select font
  printCentered("LoopPedal v 0.2", 3);

  // initialize LED
  LED.begin();
  LED.setBrightness(BRIGHTNESS);
  startupLEDs();

  // BANK Button
  fillColor(0, COLOR_YELLOW);
  ledMode[0] = ON;
  // CLR button
  fillColor(1, COLOR_RED);
  ledMode[1] = BLINK;
  // REC/PLAY Button
  fillColor(2, COLOR_GREEN);
  ledMode[2] = SPINNING;
  // STOP button
  fillColor(3, COLOR_RED);
  ledMode[3] = ON;
  // UNDO button
  fillColor(4, COLOR_YELLOW);
  ledMode[4] = ON;
  // MODE button
  fillColor(5, COLOR_RED);
  ledMode[5] = ON;

  // FX Buttons
  fillColor(10, COLOR_BLUE);
  fillColor(11, COLOR_BLUE);
  fillColor(12, COLOR_BLUE);
  fillColor(13, COLOR_BLUE);

  // screen.clear();
}

void loop()
{
  time = millis();

  // Buttons
  checkButtons();

  // encoder
  menuIndex = readEncoder(menuIndex, 1);

  // clr Button
  if (buttonPushed[1])
  {
    stopped = true;
    play = true;
    for (int i = 0; i < 4; i++)
    {
      fxState[i] = LOW;
      chnState[i] = HIGH;
    }
    modeRec = true;
    selectedChannel = 0;
  }

  // rec/play Button
  if (buttonPushed[2])
  {
    stopped = false;
    play = !play;
  }
  // stop button
  if (buttonPushed[3])
  {
    stopped = true;
    play = true; // restarts with playing, but is stopped for now
  }

  // mode Button
  if (buttonPushed[5])
  {
    modeRec = !modeRec;
  }

  // chn Buttons
  for (int i = 0; i < 4; i++)
  {
    if (buttonPushed[i + 6])
    {
      if (modeRec)
      {
        selectedChannel = i;
      }
      else
      {
        chnState[i] = !chnState[i];
      }
    }
  }

  // fx buttons
  for (int i = 0; i < 4; i++)
  {
    if (buttonPushed[i + 10])
    {
      fxState[i] = !fxState[i];
    }
  }

  // ********************************** Lighting handling *******************************************

  ledMode[3] = stopped ? OFF : ON;
  if (play)
  {
    fillColor(2, COLOR_GREEN);
  }

  ledMode[2] = stopped ? ON : SPINNING; // stop LED

  if (modeRec) // mode rec
  {
    fillColor(5, COLOR_RED);
    for (int i = 6; i < 10; i++)
    {
      fillColor(i, COLOR_RED);
      ledMode[i] = (selectedChannel + 6 == i) ? ON : OFF;
    }
  }
  else // mode play
  {
    fillColor(5, COLOR_GREEN);
    for (int i = 6; i < 10; i++)
    {
      fillColor(i, COLOR_GREEN);
      ledMode[i] = (chnState[i - 6]) ? ON : OFF;
    }
  }

  if (play)
  {
    fillColor(2, COLOR_GREEN);
  }
  else
  {
    fillColor(2, COLOR_ORANGE);
  }

  // fx buttons
  for (int i = 0; i < 4; i++)
  {
    ledMode[i + 10] = fxState[i] ? ON : OFF;
  }

  // ********************************************* Timed Loop ********************************************************************
  if ((time - (TIMED_LOOP_DELTA)) > lastTimedChange)
  {
    lastTimedChange = time; // reset timed loop

    manageLEDs(); // update LED pattern
    readMIDI();   // read incoming MIDI signals
  }
  // ********************************************* End Timed Loop ********************************************************************

  if ((time - (LED_DELTATIME)) > lastPixelChange) // LED delay counter
  {
    lastPixelChange = time;                // reset led rollover timer
    pixel = (pixel + 1) % (LED_COUNT / 2); // rollover LED index
  }
}

void fillColor(int target, int source[3])
{
  for (int i = 0; i < 3; i++)
  {
    color[target][i] = source[i];
  }
}

void checkButtons()
{
  for (int i = 0; i < LED_RINGS+1; i++)
  {
    buttonPushed[i] = LOW;
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
        if (buttonState[i])
        {
          buttonPushed[i] = HIGH;
        }
      }
    }
    prevButtonState[i] = state;
  }
}

void manageLEDs()
{
  for (int i = 0; i < LED_RINGS; i++)
  {
    switch (ledMode[i])
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
    case BLINK:
      showColor(i, ((pixel < 2) ? COLOR_OFF : color[i]), false);
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

void startupLEDs()
{
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
  for (int i = 0; i < LED_RINGS; i++) // just as backup, fill with default green color
  {
    fillColor(i, COLOR_GREEN);
  }
}

void printCentered(char *s, const int line)
{
  screen.drawString(8 - (strlen(s) * .5), (line), s); // put string of display at position X,
}

int readEncoder(int val, int stepSize)
{
  bool val1 = digitalRead(PIN_ENC1);
  bool val2 = digitalRead(PIN_ENC2);
  int valNew = val;
  if ((encValPrev == HIGH) && (val1 == LOW))
  {
    if (val2 == HIGH)
    {
      valNew = max(-9999, val - stepSize);
    }
    else
    {
      valNew = min(9999, val + stepSize);
    }
  }
  encValPrev = val1;
  return valNew;
}
