#include "Menu.h"

#include <printf.h>

#include <SPI.h>
#include <Wire.h>

#include <EEPROM.h>

#include <nRF24L01.h>
#include <RF24.h>

#include <U8x8lib.h>

#include <Adafruit_NeoPixel.h>

#include <PCF8574.h>

//************************** PINS *************************************

#define LED_PIN 2

#define RADIO_CE 7
#define RADIO_CSN 8

#define DISPLAY_RST 6
#define DISPLAY_DC 9
#define DISPLAY_CS 10

// Switches are wired to Enxpander 0
#define SWITCH1 0
#define SWITCH2 1
#define SWITCH3 2
#define SWITCH4 3
#define SWITCH5 4
#define SWITCH6 5

// Encoder 1 is directly soldered to the Arduino on PIN D3-D5
#define ENC1_SW 3
#define ENC1_1 4
#define ENC1_2 5
// Encoder 2 is wired to IO Expander 0 and 1 (SW on Expander 0)
#define ENC2_SW 7
#define ENC2_1 0
#define ENC2_2 1
// Encoder 3 is wired to IO Expander 1
#define ENC3_SW 2
#define ENC3_1 3
#define ENC3_2 4
// Encoder 4 is wired to IO Expander 1
#define ENC4_SW 5
#define ENC4_1 6
#define ENC4_2 7

// JOYSTICK 1
#define JOY1X A0
#define JOY1Y A7
#define JOY1Z A6
#define JOY1BUT 0 // Button is wired to Expander 2
// JOYSTICK 2
#define JOY2X A1
#define JOY2Y A3
#define JOY2Z A2
#define JOY2BUT 1 // Button is wired to Expander 2

//************************ END PINS ***********************************

//*********************** CONSTANTS ***********************************

char *VERSION = "0.1.2";

uint8_t address[][6] = {"1Node", "2Node"};

const byte LED_COUNT = 6;

enum INPUT_MODE
{
  MENU,
  DATA,
  INFO,
  INPUT_INTEGER
};

//********************* END CONSTANTS *********************************

struct SEND_DATA_STRUCTURE
{
  // struct __attribute__((__packed__)) SEND_DATA_STRUCTURE{
  // put your variable definitions here for the data you want to send
  // THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO

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

struct RECEIVE_DATA_STRUCTURE
{
  // put your variable definitions here for the data you want to receive
  // THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE mydata_remote;

//*********************** OBJECTS ***********************************

U8X8_SSD1309_128X64_NONAME0_4W_HW_SPI screen(DISPLAY_CS, DISPLAY_DC, DISPLAY_RST);

RF24 radio(RADIO_CE, RADIO_CSN); // CE, CSN

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

PCF8574 pcfSwitch(0x20);
PCF8574 pcfEncoder(0x21);
PCF8574 divEncoder(0x22);

char *topStrings[] = {"Data", "Setup", "Version", "Reboot"};
char *setupStrings[] = {"Back ...", "LED Brightness", "RF-Strength", "Startup-Delay"};
Menu topMenu(topStrings, 4);
Menu setupMenu(setupStrings, 4);

Menu *activeMenu = &topMenu;

//********************* END OBJECTS *********************************

//********************** VARIABLES **********************************

int startupDelay = 5;

int inputMode = MENU;
long lastAction = 0;

int *inputIntegerVal;
int inputIntegerMin;
int inputIntegerMax;
int inputIntegerStepsize;

bool connectionState = false;
bool prevConnectionState = false;

unsigned long previousMillis = 0;

int ledBrightness = 64;

// Encoder
int posEncoder[] = {0, 0, 0, 0};
int posEncoderPrev[] = {0, 0, 0, 0};
bool prevEnVal[] = {HIGH, HIGH, HIGH, HIGH};
bool buttonPressed[] = {LOW, LOW, LOW, LOW};
bool prevButtonPressed[] = {LOW, LOW, LOW, LOW};

// Switch
bool switches[] = {LOW, LOW, LOW, LOW, LOW, LOW};

// Joystick
bool joy1Btn = LOW;
int joy1X = 0;
int joy1Y = 0;
int joy1Z = 0;

bool joy2Btn = LOW;
int joy2X = 0;
int joy2Y = 0;
int joy2Z = 0;

int prevJoy1X = 0;
int prevJoy1Y = 0;
int prevJoy1Z = 0;

int prevJoy2X = 0;
int prevJoy2Y = 0;
int prevJoy2Z = 0;

//******************** END VARIABLES ********************************

void setup()
{
  Serial.begin(115200);
  Serial.println("=== Remote v0.1 ===");

  pinMode(ENC1_SW, INPUT_PULLUP);

  ledBrightness = EEPROM.read(0);
  startupDelay =  EEPROM.read(8);

  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();  // Turn OFF all pixels ASAP

  Serial.println("Starting display");

  screen.begin(); // initialite display

  strip.setPixelColor(5, strip.Color(0, 0, ledBrightness));
  screen.setFont(u8x8_font_8x13_1x2_f); // select font
  printCentered("Remote v0.1", 2);

  long t = millis();
  while (t + (startupDelay * 100) / 2 > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  strip.setPixelColor(5, strip.Color(0, ledBrightness, 0));
  while (t + (startupDelay * 100) / 2 > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  while (!digitalRead(ENC1_SW))
  {
  }

  printCentered("Starting display", 2);
  strip.setPixelColor(4, strip.Color(0, 0, ledBrightness));
  strip.show();

  t = millis();
  while (t + (startupDelay * 100) > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  while (!digitalRead(ENC1_SW))
  {
  }
  printCentered("Done", 4);
  strip.setPixelColor(4, strip.Color(0, ledBrightness, 0));
  strip.show();
  t = millis();
  while (t + (startupDelay * 100) > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  while (!digitalRead(ENC1_SW))
  {
  }
  screen.clear();

  Serial.println("Starting radio as Sender");
  printCentered("Starting radio", 2);
  strip.setPixelColor(3, strip.Color(0, 0, ledBrightness));
  strip.show();
  t = millis();
  while (t + (startupDelay * 100) > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  while (!digitalRead(ENC1_SW))
  {
  }

  while (!radio.begin())
  {
    Serial.println("ERROR: Radio hardware not responding!");
    delay(1000);
  }

  radio.openWritingPipe(address[0]);    // 00001
  radio.openReadingPipe(1, address[1]); // 00002
  radio.setPayloadSize(sizeof(SEND_DATA_STRUCTURE));
  radio.setRetries(0, 0);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  // printf_begin();
  // radio.printPrettyDetails();

  printCentered("Done", 4);
  strip.setPixelColor(3, strip.Color(0, ledBrightness, 0));
  strip.show();
  t = millis();
  while (t + (startupDelay * 100) > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  while (!digitalRead(ENC1_SW))
  {
  }
  screen.clear();

  Serial.println("Starting I/O");
  printCentered("Starting I/O", 2);
  strip.setPixelColor(2, strip.Color(0, 0, ledBrightness));
  strip.show();
  t = millis();
  while (t + (startupDelay * 100) > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  while (!digitalRead(ENC1_SW))
  {
  }

  while (!pcfSwitch.begin())
  { // initialize IO-Expander for Switches
    Serial.println("ERROR: IO Expander not responding!");
    delay(1000);
  }

  printCentered("Done", 4);
  strip.setPixelColor(2, strip.Color(0, ledBrightness, 0));
  strip.show();
  t = millis();
  while (t + (startupDelay * 100) > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  while (!digitalRead(ENC1_SW))
  {
  }

  pcfSwitch.write8(255);  // Set all pins to high, so the switches can pull them low
  pcfEncoder.write8(255); // Set all pins to high, so the encoders can pull them low
  divEncoder.write8(255); // Set all pins to high, so the encoders can pull them low

  pinMode(ENC1_1, INPUT_PULLUP);
  pinMode(ENC1_2, INPUT_PULLUP);

  for (int i = 1; i >= 0; i--)
  {
    strip.setPixelColor(i, strip.Color(0, ledBrightness, 0));
    t = millis();
    while (t + (startupDelay * 100) / 4 > millis() && digitalRead(ENC1_SW))
    {
    } // delay, but shortcut with button
    strip.show();
  }

  screen.clear();
  Serial.println("Startup complete");
  printCentered("Startup complete", 2);
  t = millis();
  while (t + (startupDelay * 100) > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  while (!digitalRead(ENC1_SW))
  {
  }
  printCentered("Enjoy!", 4);
  while (t + (startupDelay * 100) > millis() && digitalRead(ENC1_SW))
  {
  } // delay, but shortcut with button
  while (!digitalRead(ENC1_SW))
  {
  }

  strip.clear();
  strip.show();
  screen.clear();

  screen.setFont(u8x8_font_8x13_1x2_f); // select font
  screen.drawString(15, 0, (connectionState ? "C" : "N"));

  activeMenu = &topMenu;
  setupMenues();
  topMenu.draw(screen);
}

//********************** MAIN LOOP **********************************

void loop()
{

  unsigned long currentMillis = millis();

  handleInputs();

  handleMenuIO(currentMillis);

  
  //********************* TIMED LOOP **********************************

  if (currentMillis - previousMillis >= 10)
  {
    previousMillis = currentMillis;

    handleRF(currentMillis);

    handleMenuDisplay();

    handleLEDs();
  }
}

void handleMenuDisplay()
{
  switch (inputMode)
  {
  case MENU:
    (*activeMenu).updateMenu(screen, readEncoder(0, (*activeMenu).getIndex()));
    break;
  case DATA:
    showJoystickData(false);
    break;
  case INPUT_INTEGER:
  int reading = readEncoder(0, *inputIntegerVal, inputIntegerStepsize);

    int valNew = min(inputIntegerMax, max(inputIntegerMin, reading));

    if (reading != *inputIntegerVal)
    {
      *inputIntegerVal = valNew;
      drawInputInteger(valNew);
    }
    break;
  }
}

void handleRF(long currentMillis)
{
  //*********************** SEND DATA TO SENDER OBJECT *****************************
  // Switches
  mydata_send.tgl1 = switches[0];
  mydata_send.tgl2 = switches[1];
  mydata_send.tgl3 = switches[2];
  mydata_send.tgl4 = switches[3];
  mydata_send.tgl5 = switches[4];
  mydata_send.tgl6 = switches[5];

  // Encoder
  mydata_send.encVal1 = posEncoder[0];
  mydata_send.encVal2 = posEncoder[1];
  mydata_send.encVal3 = posEncoder[2];
  mydata_send.encVal4 = posEncoder[3];

  // Encoder Buttons
  mydata_send.encSw1 = buttonPressed[0];
  mydata_send.encSw2 = buttonPressed[1];
  mydata_send.encSw3 = buttonPressed[2];
  mydata_send.encSw4 = buttonPressed[3];

  // AXIS
  mydata_send.joy1Btn = joy1Btn;
  mydata_send.joy1X = joy1X;
  mydata_send.joy1Y = joy1Y;
  mydata_send.joy1Z = joy1Z;

  mydata_send.joy2Btn = joy2Btn;
  mydata_send.joy2X = joy2X;
  mydata_send.joy2Y = joy2Y;
  mydata_send.joy2Z = joy2Z;

  // ************************* SEND DATA *********************************

  connectionState = radio.write(&mydata_send, sizeof(SEND_DATA_STRUCTURE));

  if (connectionState != prevConnectionState)
  {
    screen.drawString(15, 0, (connectionState ? "C" : "N"));
    prevConnectionState = connectionState;
  }
}

void handleLEDs()
{
  for (int i = 0; i < 6; i++)
  {
    if (switches[5 - i])
    {
      strip.setPixelColor(i, strip.Color(ledBrightness, 0, 0));
    }
    else
    {
      strip.setPixelColor(i, strip.Color(0, ledBrightness, 0));
    }
  }

  for (int i = 0; i < 4; i++)
  {
    if (buttonPressed[i])
    {
      strip.setPixelColor(4 - i, strip.Color(0, 0, ledBrightness));
    }
  }
  strip.show();
}

void printIfNumChanged(int &val, int &prevVal, int x, int y, bool centerOnRight)
{
  printIfNumChanged(val, prevVal, x, y, centerOnRight, false);
}

void printIfNumChanged(int &val, int &prevVal, int x, int y, bool centerOnRight, bool forceDisplay)
{
  if (val != prevVal || forceDisplay)
  {
    prevVal = val;
    char buf[4];
    itoa(val, buf, 10);
    if (centerOnRight)
    {
      screen.drawString(x - 4, y, "    ");
      screen.drawString(x - strlen(buf), y, buf);
    }
    else
    {
      screen.drawString(x, y, "    ");
      screen.drawString(x, y, buf);
    }
  }
}

void printCentered(char *s, const int line)
{
  screen.drawString(8 - (strlen(s) * .5), (line), s); // put string of display at position X,
}

int readEncoder(int numEncoder, int val)
{
  return readEncoder(numEncoder, val, 1);
}

int readEncoder(int numEncoder, int val, int stepSize)
{

  switch (numEncoder)
  {
  case 0:
    return readEncoder(val, digitalRead(ENC1_2), digitalRead(ENC1_1), prevEnVal[0], stepSize);
    break;
  case 1:
    return readEncoder(val, pcfEncoder.read(ENC2_1), pcfEncoder.read(ENC2_2), prevEnVal[1], stepSize);
    break;
  case 2:
    return readEncoder(val, pcfEncoder.read(ENC3_1), pcfEncoder.read(ENC3_2), prevEnVal[2], stepSize);
    break;
  case 3:
    return readEncoder(val, pcfEncoder.read(ENC4_1), pcfEncoder.read(ENC4_2), prevEnVal[3], stepSize);
    break;
  }
  return -999;
}

int readEncoder(int val, bool val1, bool val2, bool &valAPrev, int stepSize)
{

  int valNew = val;
  if ((valAPrev == HIGH) && (val1 == LOW))
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
  valAPrev = val1;
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

void showJoystickData(bool force)
{
  printIfNumChanged(joy1X, prevJoy1X, 0, 0, false, force);
  printIfNumChanged(joy1Y, prevJoy1Y, 0, 3, false, force);
  printIfNumChanged(joy1Z, prevJoy1Z, 0, 6, false, force);
  printIfNumChanged(joy2X, prevJoy1X, 16, 0, true, force);
  printIfNumChanged(joy2Y, prevJoy1Y, 16, 3, true, force);
  printIfNumChanged(joy2Z, prevJoy1Z, 16, 6, true, force);
}



void handleInputs()
{
  prevButtonPressed[0] = buttonPressed[0];
  buttonPressed[0] = !digitalRead(ENC1_SW);
  prevButtonPressed[1] = buttonPressed[0];
  buttonPressed[1] = !pcfSwitch.read(ENC2_SW);
  prevButtonPressed[2] = buttonPressed[2];
  buttonPressed[2] = !pcfEncoder.read(ENC3_SW);
  prevButtonPressed[3] = buttonPressed[3];
  buttonPressed[3] = !pcfEncoder.read(ENC4_SW);

  for (int i = 0; i < sizeof(switches); i++)
  {
    switches[i] = pcfSwitch.read(i);
  }

  joy1Btn = divEncoder.read(JOY1BUT);
  joy2Btn = divEncoder.read(JOY2BUT);

  joy1X = analogRead(JOY1X);
  joy1Y = analogRead(JOY1Y);
  joy1Z = analogRead(JOY1Z);

  joy2X = analogRead(JOY2X);
  joy2Y = analogRead(JOY2Y);
  joy2Z = analogRead(JOY2Z);
}

void handleMenuIO(long currentMillis)
{
  if (buttonPressed[0] && !prevButtonPressed[0] && currentMillis >= lastAction + 500)
  {
    lastAction = currentMillis;
    switch (inputMode)
    {
    case MENU:
      (*activeMenu).action();
      break;
    case INPUT_INTEGER:
      EEPROM.write(0, ledBrightness);
      EEPROM.write(8, startupDelay);
    case DATA:
    case INFO:
      inputMode = MENU;
      (*activeMenu).draw(screen);
      break;
    }
  }
}

void setupMenues()
{
  topMenu.actions[0] = []()
  {
    inputMode = DATA;
    showJoystickData(true);
    screen.clear();
  };
  topMenu.actions[1] = []()
  {
    activeMenu = &setupMenu;
    (*activeMenu).draw(screen);
  };
  topMenu.actions[2] = []()
  {
    inputMode = INFO;
    screen.clear();
    printCentered(VERSION, 2);
  };
  setupMenu.actions[0] = []() // Back
  {
    activeMenu = &topMenu;
    (*activeMenu).draw(screen);
  };
  setupMenu.actions[1] = []() // LED Brightness
  {
    inputInteger("LED Brightness", 0, 255, &ledBrightness, 5);
  };
  setupMenu.actions[3] = []() // LED Brightness
  {
    inputInteger("Startup-Delay", 0, 10, &startupDelay, 1);
  };
}