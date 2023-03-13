#include "Menu.h"

#include <Wire.h>

#include <MIDIUSB.h>
#include <Adafruit_NeoPixel.h>

#include <EEPROM.h>
#include <PCF8574.h>
#include <Servo.h>

#include <U8x8lib.h>

#define VERSION "LOOPPEDAL v0.4"
#define VERSION_LINE2 "by F.Hild"

#define PIN_LED 4 // Pin of the LED REC_RING

#define PIN_ENC1 MOSI
#define PIN_ENC2 MISO
#define PIN_ENC_BUT SCK

#define PIN_DIS_SDA SDA
#define PIN_DIS_SCK SCL

#define PIN_POTI A7
#define PIN_SERVO 5

// PINS are wired to the PCF IO expander
#define PIN_BANK 1
#define PIN_CLR 0

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

const uint8_t BUTTONS_PINS[] = {PIN_REC, PIN_STOP, PIN_UNDO, PIN_MODE, PIN_CH1, PIN_CH2, PIN_CH3, PIN_CH4, PIN_FX1, PIN_FX2, PIN_FX3, PIN_FX4, PIN_ENC_BUT};

#define LED_RINGS 14 // Number of LED rings
#define LED_COUNT 8  // Number of LEDs per ring
#define LED_PEDAL 9

#define CHANNEL_COUNT 8

#define DEBOUNCE 10 // Debounce of 10ms
#define LED_DELTATIME 1000

#define TIMED_LOOP_DELTA 10

#define TIME_UNTIL_MENU 5000

// RECEIVING MIDI SIGNALS
#define REC 1
#define PLAY_STOP 2

const int SERVO_MIN = 43;
const int SERVO_MAX = 148;
const int POTI_DEADZONE = 5;

int COLOR_OFF[3] = {0, 0, 0};
int COLOR_GREEN[3] = {0, 255, 0};
int COLOR_LIGHT_GREEN[3] = {127, 255, 0};
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

enum INPUT_MODE
{
  MENU,
  INFO,
  INPUT_INTEGER,
  INPUT_BOOLEAN
};

// I2C devices
U8X8_SSD1306_128X64_NONAME_HW_I2C screen(/* reset=*/U8X8_PIN_NONE);
PCF8574 pcfSwitch(0x21);

Servo servo;

INPUT_MODE inputMode = MENU;
int *inputIntegerVal;
int inputIntegerMin;
int inputIntegerMax;
int inputIntegerStepsize;

bool *inputBool;

long lastMenuAction = -TIME_UNTIL_MENU;
char *topStrings[] = {"LED Brightness", "Clear FX", "OD after REC", "Version", "Reboot"};
Menu topMenu(topStrings, 5);

Menu *activeMenu = &topMenu;

Adafruit_NeoPixel LED(LED_COUNT *LED_RINGS + LED_PEDAL, PIN_LED, NEO_GRB + NEO_KHZ800);
int color[LED_RINGS][3];
LED_MODE ledMode[LED_RINGS];
int ledBrightness = 127;

uint8_t pixel = 0;
long lastPixelChange = 0;

bool buttonState[LED_RINGS + 1];
bool buttonPushed[LED_RINGS + 1];
bool prevButtonState[LED_RINGS + 1];
long lastButtonChange[LED_RINGS + 1];

long time = 0;
long lastTimedChange = 0;

bool encValPrev = LOW;

bool clearFXOnClr = true;
bool overdubAfterRec = true;

int potiMin;
int potiMax;

int pedalValue = 127;
long lastServoTime;
bool pedalDetached = false;
int pedalFXChannel = -1;

bool longPush = false;

// *************************************** MIDI VARIABLES ***********************************************

const int CLR = 0x00;
const int STOP = 0x08;
const int START_RECORD = 0x09;

const int ZERO_DB_MIDI = 108;

// fx specific
const int FX = 0x60;
const int FX_PARAMETER = 0x68;

// channel specific
const int UNDO = 0x10;
const int ARM = 0x20;
const int OVERDUB = 0x30;
const int PLAY = 0x40;
const int ENABLE = 0x50;
const int VOLUME = 0x70;

int firstChannelIndex = 0;

bool firstTimeRec = true; // is true if there is no loop with a defined length set
bool modeRec = true;      // is true if the 4 channel leds show record Arm
bool play = false;        // is true if the play button is in play mode (not recording/overdubbing)
bool stopped = true;      // is true if audio is stopped

int selectedChannel = 0;

bool chnState[8] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
int chnVolume[8] = {ZERO_DB_MIDI, ZERO_DB_MIDI, ZERO_DB_MIDI, ZERO_DB_MIDI, ZERO_DB_MIDI, ZERO_DB_MIDI, ZERO_DB_MIDI, ZERO_DB_MIDI};
bool fxState[4];
int fxValue[4];

double bpm = 120;
int bpmFlag = 0;
long lastTempoHeartBeat = 0;

void setup()
{
  Serial.begin(115200);
  // delay(500);
  Serial.println("Starting LoopPedal v1.0");

  ledBrightness = EEPROM.read(0);
  clearFXOnClr = EEPROM.read(1);
  overdubAfterRec = EEPROM.read(2);
  for (int i = 0; i < 4; i++)
  {
    fxState[i] = bitRead(EEPROM.read(2), i);
  }
  for (int i = 0; i < 4; i++)
  {
    int val = EEPROM.read(3 + i);
    fxValue[i] = constrain(val, 0, 127);
  }

  // initialize LED
  LED.begin();
  LED.setBrightness(ledBrightness);
  for (int i = 0; i < LED_COUNT * LED_RINGS + LED_PEDAL; i++)
  {
    LED.setPixelColor(i, LED.Color(0, 0, 0));
  }
  LED.show();

  servo.attach(PIN_SERVO);
  servo.write(SERVO_MIN);
  delay(1000);
  potiMin = analogRead(PIN_POTI) - POTI_DEADZONE;
  servo.write(SERVO_MAX);
  delay(1000);
  potiMax = analogRead(PIN_POTI) + POTI_DEADZONE;
  servo.detach();
  pedalDetached = true;

  // initialize Buttons
  for (int i = 0; i < LED_RINGS; i++)
  {
    pinMode(BUTTONS_PINS[i], INPUT_PULLUP);
  }
  pinMode(PIN_ENC_BUT, INPUT_PULLUP);
  pinMode(PIN_ENC1, INPUT_PULLUP);
  pinMode(PIN_ENC2, INPUT_PULLUP);

  pcfSwitch.begin();
  pcfSwitch.write8(255); // Set all pins to high, so the switches can pull them low

  screen.begin(); // initialize display
  screen.setPowerSave(0);
  screen.setFont(u8x8_font_8x13_1x2_f); // select font
  showInfo();

  startupLEDs();

  for (int i = 0; i < LED_PEDAL; i++)
  {
    LED.setPixelColor(LED_COUNT * LED_RINGS + i, LED.Color(0, 0, 255));
  }

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

  clrAll();
  setupMenues();
  // topMenu.draw(screen);
}

void loop()
{
  time = millis();

  handlePedal(time);

  handlePedalLED();

  handleMenuIO(time);

  // Buttons
  checkButtons();

  // bankButton
  if (buttonPushed[0])
  {
    firstChannelIndex = (firstChannelIndex == 0) ? 4 : 0;
  }

  // clr Button
  if (buttonPushed[1])
  {
    clrAll();
  }

  // rec/play Button
  if (buttonPushed[2])
  {
    if (stopped && firstTimeRec)
    {
      controlMIDI(START_RECORD, 127);
    }
    if (!stopped && !firstTimeRec)
    {
      play = !play;
      if (play)
      {
        controlMIDI(PLAY + selectedChannel, 127);
      }
      else
      {
        controlMIDI(OVERDUB + selectedChannel, 127);
      }
    }
    if (!stopped && firstTimeRec)
    {
      for (int i = 0; i < CHANNEL_COUNT; i++) // After initial record
      {
        controlMIDI(PLAY + i, 127); // stop recording, start playing
      }
      if (overdubAfterRec)
      {
        controlMIDI(OVERDUB + selectedChannel, 127); // stop recording, start playing
        play = false;
      }
      firstTimeRec = false;
    }

    if (stopped && !firstTimeRec)
    {
      play = true;
      for (int i = 0; i < CHANNEL_COUNT; i++) // After initial record
      {
        controlMIDI(PLAY + i, 127); // stop recording, start playing
      }
    }
    stopped = false;
  }
  // stop button
  if (buttonPushed[3])
  {
    stopped = true;
    play = false; // restarts with playing, but is stopped for now
    controlMIDI(STOP, 127);
    Serial.println("Stop");
  }

  // undo
  if (buttonPushed[4])
  {
    controlMIDI(UNDO + selectedChannel, 127);
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
        if (!play && !stopped) // if currently recording
        {
          controlMIDI(PLAY + selectedChannel, 127); // stop recording on the current channel
        }
        pedalFXChannel = -1;
        selectedChannel = i + firstChannelIndex;
        setPedal(chnVolume[selectedChannel]);
        controlMIDI(ARM + i + firstChannelIndex, 127);
        if (!play && !stopped) // if currently recording
        {
          controlMIDI(OVERDUB + i + firstChannelIndex, 127); // start overdubbing on the new channel
        }
      }
      else
      {
        chnState[i + firstChannelIndex] = !chnState[i + firstChannelIndex];
        controlMIDI(ENABLE + i + firstChannelIndex, chnState[i + firstChannelIndex] ? 127 : 0);
      }
    }
  }

  // fx buttons
  for (int i = 0; i < 4; i++)
  {
    if (buttonPushed[i + 10])
    {
      pedalFXChannel = i;
      setPedal(fxValue[i]);
      fxState[i] = !fxState[i];
      controlMIDI(FX + i, fxState[i] ? 127 : 0);
      updateFXEEPROM();
    }
  }

  MidiUSB.flush(); // flush all aquired MIDI signals

  readMIDI();

  // ********************************** Lighting handling *******************************************

  ledMode[0] = (firstChannelIndex == 0) ? ON : BLINK;
  ledMode[1] = (firstTimeRec && stopped) ? OFF : BLINK;

  if (!stopped)
  {
    if (firstTimeRec)
    {
      fillColor(2, COLOR_RED);
    }
    else if (!play)
    {
      fillColor(2, COLOR_ORANGE);
    }
    else
    {
      fillColor(2, COLOR_GREEN);
    }
  }
  else
  {
    fillColor(2, COLOR_GREEN);
  }

  ledMode[2] = stopped ? ON : SPINNING; // stop LED
  ledMode[3] = stopped ? OFF : ON;

  ledMode[4] = firstTimeRec ? OFF : ON;

  if (modeRec) // mode rec
  {
    fillColor(5, COLOR_RED);
    for (int i = 6; i < 10; i++)
    {
      fillColor(i, COLOR_RED);
      ledMode[i] = (selectedChannel - firstChannelIndex + 6 == i) ? ON : OFF;
    }
  }
  else // mode play
  {
    fillColor(5, COLOR_GREEN);

    // channel buttons
    for (int i = 6; i < 10; i++)
    {
      fillColor(i, COLOR_GREEN);
      if ((chnState[i - 6 + firstChannelIndex]))
      {
        // channel is on
        if (pedalFXChannel < 0 && (selectedChannel - firstChannelIndex + 6 == i))
        {
          ledMode[i] = SPINNING;
        }
        else
        {
          ledMode[i] = ON;
        }
      }
      else
      {
        ledMode[i] = OFF;
      }
    }
  }

  // fx buttons
  for (int i = 0; i < 4; i++)
  {
    if (pedalFXChannel == i)
    {
      ledMode[i + 10] = SPINNING;
    }
    else
    {
      ledMode[i + 10] = fxState[i] ? ON : OFF;
    }
  }

  // ********************************************* Timed Loop ********************************************************************
  if ((time - (TIMED_LOOP_DELTA)) > lastTimedChange)
  {
    lastTimedChange = time; // reset timed loop

    handleMenuDisplay();

    manageLEDs(); // update LED pattern

    // readMIDI();   // read incoming MIDI signals
  }
  // ********************************************* End Timed Loop ********************************************************************

  if (bpmFlag >= 12 || (time - LED_DELTATIME > lastPixelChange)) // LED delay counter
  {
    lastPixelChange = time;                // reset led rollover timer
    pixel = (pixel + 1) % (LED_COUNT / 2); // rollover LED index
    bpmFlag = 0;
  }
}

void clrAll()
{
  Serial.println("Clr all");
  stopped = true;
  firstTimeRec = true;
  play = true;
  modeRec = true;
  for (int i = 0; i < 4; i++)
  {
    if (clearFXOnClr) // only clear fx if option is enabled
    {
      fxState[i] = LOW;
      controlMIDI(FX + i, 0);
      controlMIDI(FX_PARAMETER + i, fxValue[i]);
      updateFXEEPROM();
    }
  }
  for (int i = 0; i < CHANNEL_COUNT; i++)
  {

    chnState[i] = HIGH;           // set channel active
    controlMIDI(ENABLE + i, 127); // enable all channels
  }

  for (int i = 0; i < 8; i++)
  {
    chnVolume[i] = ZERO_DB_MIDI;
    controlMIDI(VOLUME + i, chnVolume[i]);
  }

  setPedal(chnVolume[0]);

  firstChannelIndex = 0;
  selectedChannel = 0;
  pedalFXChannel = -1;
  controlMIDI(ARM + selectedChannel + firstChannelIndex, 127); // reset Arm to first channel

  controlMIDI(STOP, 127);
  controlMIDI(CLR, 127);
}

void handleMenuIO(long currentMillis)
{
  if (buttonPushed[14])
  {
    switch (inputMode)
    {
    case MENU:
      (*activeMenu).action();
      break;
    case INPUT_INTEGER:
    case INPUT_BOOLEAN:
      EEPROM.update(0, ledBrightness);
      EEPROM.update(1, clearFXOnClr);
      EEPROM.update(2, overdubAfterRec);
      updateFXEEPROM();

    case INFO:
      inputMode = MENU;
      (*activeMenu).draw(screen);
      break;
    }
    lastMenuAction = currentMillis;
  }
  if (lastMenuAction + TIME_UNTIL_MENU < currentMillis)
  {
    showInfo();
  }
}

void updateFXEEPROM()
{
  int fxInt = 0;
  for (int i = 0; i < 4; i++)
  {
    bitWrite(fxInt, i, fxState[i]);
    EEPROM.update(3 + i, fxValue[i]);
  }
  EEPROM.update(2, fxInt);
}

void handleMenuDisplay()
{
  // dont dare replace this with a switch, for some reason it breaks and doesnt enter the INPUT_BOOLEAN case!
  if (inputMode == MENU)
  {
    (*activeMenu).updateMenu(screen, readEncoder((*activeMenu).getIndex(), 1));
  }
  if (inputMode == INPUT_INTEGER)
  {
    int reading = readEncoder(*inputIntegerVal, inputIntegerStepsize); // read new value
    int valNew = constrain(reading, inputIntegerMin, inputIntegerMax); // sanitize value

    if (reading != *inputIntegerVal) // check against old value
    {
      *inputIntegerVal = valNew; // if changed, change value
      drawInputInteger(valNew);  // display change
    }
  }
  if (inputMode == INPUT_BOOLEAN)
  {
    int readingBool = readEncoder(*inputBool, 1);
    bool valNewBool = readingBool % 2;

    if (readingBool != *inputBool)
    {
      *inputBool = valNewBool;
      drawInputBool(valNewBool);
    }
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
  for (int i = 0; i < LED_RINGS + 1; i++)
  {
    buttonPushed[i] = LOW;
    // read the state of the switch into a local variable:

    bool state;

    if (i == 0)
    {
      state = !pcfSwitch.read(PIN_BANK);
    }
    else if (i == 1)
    {
      state = !pcfSwitch.read(PIN_CLR);
    }
    else
    {
      state = !digitalRead(BUTTONS_PINS[i - 2]); // read rawSignal
    }

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (state != prevButtonState[i])
    {
      // reset the debouncing timer
      lastButtonChange[i] = time;
    }

    if (!longPush && state && (time - lastButtonChange[i]) > DEBOUNCE && time - lastButtonChange[i] > 500)
    {
      if (i >= 10 && i < 14)
      {
        pedalFXChannel = i - 10;
        setPedal(fxValue[pedalFXChannel]);
      }
      longPush = true;
    }

    if (!longPush && (time - lastButtonChange[i]) > DEBOUNCE)
    {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (state != buttonState[i])
      {

        buttonState[i] = state;
        if (!buttonState[i])
        {
          buttonPushed[i] = HIGH;
        }
      }
    }

    if (state != prevButtonState[i] && longPush)
    {
      longPush = false;
      lastButtonChange[i] = time + 100;
      buttonState[i] = state;
    }

    prevButtonState[i] = state;
  }
}

void manageLEDs()
{
  LED.setBrightness(ledBrightness);
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

void setPedal(int val)
{
  pedalValue = val;
  int a = map(val, 0, 127, SERVO_MIN, SERVO_MAX);
  servo.attach(PIN_SERVO);
  servo.write(a);
  pedalDetached = false;
  lastServoTime = millis();
}

void handlePedal(long time)
{
  if (time > lastServoTime + 500)
  {
    if (!pedalDetached)
    {
      servo.detach();
      pedalDetached = true;
    }
    else
    {
      int newPedalValue = map(analogRead(PIN_POTI), potiMin, potiMax, 0, 127);
      if (abs(newPedalValue - pedalValue) >= 3)
      {
        pedalValue = newPedalValue;
        if (pedalFXChannel < 0)
        {
          controlMIDI(VOLUME + selectedChannel, pedalValue);
          chnVolume[selectedChannel] = pedalValue;
        }
        else
        {
          fxValue[pedalFXChannel] = pedalValue;
          controlMIDI(FX_PARAMETER + pedalFXChannel, pedalValue);
        }
      }
    }
  }
}

void handlePedalLED()
{
  int valuePoti = analogRead(PIN_POTI);

  float step = abs(float(potiMax - potiMin)) / float(LED_PEDAL + 1);
  int valLED = LED_PEDAL - (valuePoti - min(potiMin, potiMax)) / round(step);

  double rest = step - (valuePoti - min(potiMin, potiMax)) % round(step);

  double brightnessNextLED = constrain((double(map(rest, 0, step, 0, 255)) / 255.0), 0.0, 1.0);

  uint32_t c;
  if (pedalFXChannel >= 0)
  {
    c = LED.Color(0, 0, 255);
  }
  else
  {
    if (modeRec)
    {
      c = LED.Color(255, 0, 0);
    }
    else
    {
      c = LED.Color(0, 255, 0);
    }
  }

  for (int i = 0; i < LED_PEDAL; i++)
  {
    if (LED_PEDAL - i > valLED)
    {
      LED.setPixelColor(LED_RINGS * LED_COUNT + i, LED.Color(0, 0, 0));
    }
    else if (LED_PEDAL - i == valLED)
    {
      LED.setPixelColor(LED_RINGS * LED_COUNT + i, c * brightnessNextLED);
    }
    else
    {
      LED.setPixelColor(LED_RINGS * LED_COUNT + i, c);
    }
  }
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

    // Serial.print(rx.header);
    // Serial.print(" , ");
    // Serial.print(rx.byte1);
    // Serial.print(" , ");
    // Serial.print(rx.byte2);
    // Serial.print(" , ");
    // Serial.println(rx.byte3);
    if (rx.header = 0xB && rx.byte1 == 0xB0 && rx.byte2 == 0x02 && rx.byte3 == 0x7F)
    {
      Serial.println("Play");
      bpmFlag = 2;
    }
    if (rx.header = 0xF && rx.byte1 == 0xF8 /*&& rx.byte2 == 0x2 && rx.byte3 == 0x7F*/)
    {
      bpmFlag++;
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

// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).
void controlChange(byte channel, byte control, byte value)
{
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

void controlMIDI(byte channel, byte val)
{
  controlChange(0, channel, val);
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
    // PEDAL
    LED.setPixelColor(LED_COUNT * LED_RINGS + LED_PEDAL - i - 1, LED.Color(255 / LED_COUNT * i, 164 / LED_COUNT * i, 255 - (255 / LED_COUNT * i)));

    LED.show();
    delay(1000 / LED_COUNT);
  }
  // PEDAL extrawurst
  delay(1000 / LED_COUNT);
  LED.setPixelColor(LED_COUNT * LED_RINGS, LED.Color(255 / LED_COUNT * LED_COUNT, 164 / LED_COUNT * LED_COUNT, 255 - (255 / LED_COUNT * LED_COUNT)));
  LED.show();

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
  bool val1 = digitalRead(PIN_ENC2);
  bool val2 = digitalRead(PIN_ENC1);
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

void inputInteger(char *title, int min, int max, int *val, int stepSize)
{
  inputMode = INPUT_INTEGER;
  inputIntegerVal = val;
  inputIntegerMin = min;
  inputIntegerMax = max;
  inputIntegerStepsize = stepSize;
  screen.clear();
  screen.drawString(0, 0, title);
  drawInputInteger(*val);
}

void drawInputInteger(int valNew)
{
  char c[16];
  itoa(*inputIntegerVal, c, 10);
  screen.drawString(6, 4, "    ");
  screen.drawString(6, 4, c);
  screen.drawString(0, 6, "|              |");
  double percent = ((double)valNew / ((double)inputIntegerMax));
  for (int i = 0; i < (round(percent * 14)); i++)
  {
    screen.drawString(i + 1, 6, "=");
  }
}

void inputBoolean(char *title, bool *val)
{
  inputMode = INPUT_BOOLEAN;
  inputBool = val;
  screen.clear();
  screen.drawString(0, 0, title);
  drawInputBool(*val);
}

void drawInputBool(bool valNew)
{
  if (valNew)
  {
    screen.drawString(6, 4, "YES");
  }
  else
  {
    screen.drawString(6, 4, "NO "); // added a whitespace to remove the 's' from YES
  }
}

void setupMenues()
{
  topMenu.actions[0] = []() // LED Brightness
  {
    inputInteger("LED Brightness", 0, 255, &ledBrightness, 5);
  };
  topMenu.actions[1] = []() // ClearFX on Clear Button Pressed
  {
    inputBoolean("Clear FX on Clr", &clearFXOnClr);
  };
  topMenu.actions[2] = []() // Overdub afer REC
  {
    inputBoolean("Overd. after REC", &overdubAfterRec);
  };
  topMenu.actions[3] = []()
  {
    showInfo();
  };
}

void showInfo()
{
  if (!inputMode == INFO)
  {
    inputMode = INFO;
    screen.clear();
    printCentered(VERSION, 2);
    printCentered(VERSION_LINE2, 5);
  }
}
