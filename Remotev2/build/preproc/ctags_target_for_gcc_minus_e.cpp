# 1 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino"
# 2 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2

# 4 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2

# 6 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2
# 7 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2

# 9 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2

# 11 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2
# 12 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2

# 14 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2

# 16 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2

# 18 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino" 2

//************************** PINS *************************************
# 30 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Remotev2.ino"
// Switches are wired to Enxpander 0







// Encoder 1 is directly soldered to the Arduino on PIN D3-D5



// Encoder 2 is wired to IO Expander 0 and 1 (SW on Expander 0)



// Encoder 3 is wired to IO Expander 1



// Encoder 4 is wired to IO Expander 1




// JOYSTICK 1




// JOYSTICK 2





//************************ END PINS ***********************************

//*********************** CONSTANTS ***********************************

uint8_t address[][6] = {"1Node", "2Node"};

const byte LED_COUNT = 6;

const int MODE_SHOW_TIME = 1000;

const int NUM_SETTINGS = 2;

enum DISPLAY_MODE
{
  NONE = 0,
  SETUP,
  JOYSTICK,
  ENCODER
};

enum INPUT_MODE
{
  MENU,
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

U8X8_SSD1309_128X64_NONAME0_4W_HW_SPI screen(10, 9, 6);

RF24 radio(7, 8); // CE, CSN

Adafruit_NeoPixel strip(LED_COUNT, 2, ((1 << 6) | (1 << 4) | (0 << 2) | (2)) /*|< Transmit as G,R,B*/ + 0x0000 /*|< 800 KHz data transmission*/);

PCF8574 pcfSwitch(0x20);
PCF8574 pcfEncoder(0x21);
PCF8574 divEncoder(0x22);

char *topStrings[] = {"Data", "Setup", "Model", "Version", "Reboot"};
char *setupStrings[] = {"Back ...", "LED Brightness", "RF-Strength", "Startup-Delay"};
Menu topMenu(topStrings, 5);
Menu setupMenu(setupStrings, 4);

Menu *activeMenu = &topMenu;

//********************* END OBJECTS *********************************

//********************** VARIABLES **********************************

int startupDelay = 500;

int inputMode = MENU;

int *inputIntegerVal;
int inputIntegerMin;
int inputIntegerMax;
int inputIntegerStepsize;

bool connectionState = false;
bool prevConnectionState = false;

unsigned long previousMillis = 0;

bool modeSignClear = false;

bool modeChange = false;

int setupSetting = 0;
int prevSetupSetting = 0;

int mode = NONE;
int prevMode;

int ledBrightness = 64;
int prevLedBrightness;

// Encoder
int posEncoder[] = {0, 0, 0, 0};
int posEncoderPrev[] = {0, 0, 0, 0};
bool prevEnVal[] = {0x1, 0x1, 0x1, 0x1};
bool buttonPressed[] = {0x0, 0x0, 0x0, 0x0};
bool prevButtonPressed[] = {0x0, 0x0, 0x0, 0x0};

// Switch
bool switches[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// Joystick
bool joy1Btn = 0x0;
int joy1X = 0;
int joy1Y = 0;
int joy1Z = 0;

bool joy2Btn = 0x0;
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

  pinMode(3, 0x2);

  ledBrightness = EEPROM.read(0);
  startupDelay = EEPROM.read(16);

  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show(); // Turn OFF all pixels ASAP

  Serial.println("Starting display");

  screen.begin(); // initialite display

  strip.setPixelColor(5, strip.Color(0, 0, ledBrightness));
  screen.setFont(u8x8_font_8x13_1x2_f); // select font
  printCentered("Remote v0.1", 2);

  long t = millis();
  while (t + startupDelay / 2 > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  strip.setPixelColor(5, strip.Color(0, ledBrightness, 0));
  while (t + startupDelay / 2 > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  while (!digitalRead(3))
  {
  }

  printCentered("Starting display", 2);
  strip.setPixelColor(4, strip.Color(0, 0, ledBrightness));
  strip.show();

  t = millis();
  while (t + startupDelay > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  while (!digitalRead(3))
  {
  }
  printCentered("Done", 4);
  strip.setPixelColor(4, strip.Color(0, ledBrightness, 0));
  strip.show();
  t = millis();
  while (t + startupDelay > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  while (!digitalRead(3))
  {
  }
  screen.clear();

  Serial.println("Starting radio as Sender");
  printCentered("Starting radio", 2);
  strip.setPixelColor(3, strip.Color(0, 0, ledBrightness));
  strip.show();
  t = millis();
  while (t + startupDelay > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  while (!digitalRead(3))
  {
  }

  while (!radio.begin())
  {
    Serial.println("ERROR: Radio hardware not responding!");
    delay(1000);
  }

  radio.openWritingPipe(address[0]); // 00001
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
  while (t + startupDelay > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  while (!digitalRead(3))
  {
  }
  screen.clear();

  Serial.println("Starting I/O");
  printCentered("Starting I/O", 2);
  strip.setPixelColor(2, strip.Color(0, 0, ledBrightness));
  strip.show();
  t = millis();
  while (t + startupDelay > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  while (!digitalRead(3))
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
  while (t + startupDelay > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  while (!digitalRead(3))
  {
  }

  pcfSwitch.write8(255); // Set all pins to high, so the switches can pull them low
  pcfEncoder.write8(255); // Set all pins to high, so the encoders can pull them low
  divEncoder.write8(255); // Set all pins to high, so the encoders can pull them low

  pinMode(4, 0x2);
  pinMode(5, 0x2);

  for (int i = 1; i >= 0; i--)
  {
    strip.setPixelColor(i, strip.Color(0, ledBrightness, 0));
    t = millis();
    while (t + startupDelay / 4 > millis() && digitalRead(3))
    {
    } // delay, but shortcut with button
    strip.show();
  }

  screen.clear();
  Serial.println("Startup complete");
  printCentered("Startup complete", 2);
  t = millis();
  while (t + startupDelay > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  while (!digitalRead(3))
  {
  }
  printCentered("Enjoy!", 4);
  while (t + startupDelay > millis() && digitalRead(3))
  {
  } // delay, but shortcut with button
  while (!digitalRead(3))
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
int menuPos = 0;

long lastAction = 0;

void loop()
{

  unsigned long currentMillis = millis();

  switch (inputMode)
  {
  case MENU:
    (*activeMenu).updateMenu(screen, readEncoder(0, (*activeMenu).getIndex()));
    break;
  case INPUT_INTEGER:
    int valNew = ((inputIntegerMax)<(((inputIntegerMin)>(readEncoder(0, *inputIntegerVal, inputIntegerStepsize))?(inputIntegerMin):(readEncoder(0, *inputIntegerVal, inputIntegerStepsize))))?(inputIntegerMax):(((inputIntegerMin)>(readEncoder(0, *inputIntegerVal, inputIntegerStepsize))?(inputIntegerMin):(readEncoder(0, *inputIntegerVal, inputIntegerStepsize)))));

    if (valNew != *inputIntegerVal)
    {
      drawInputInteger(valNew);
    }
    break;
  }

  prevButtonPressed[0] = buttonPressed[0];
  buttonPressed[0] = !digitalRead(3);

  if (buttonPressed[0] && !prevButtonPressed[0] && currentMillis >= lastAction + 500)
  {
    lastAction = currentMillis;
    switch (inputMode)
    {
    case MENU:
      (*activeMenu).action();
      break;
    case INPUT_INTEGER:
      inputMode = MENU;
      EEPROM.write(0, ledBrightness);
      EEPROM.write(16, startupDelay);
      (*activeMenu).draw(screen);
      break;
    }
  }
}

void loop2()
{

  unsigned long currentMillis = millis();

  // Encoder 1
  if (modeChange)
  {
    // readEncoder(mode, digitalRead(ENC1_2), digitalRead(ENC1_1), prevEnVal[0]);
    mode = ((mode)>0?(mode):-(mode)) % (ENCODER - NONE + 1);
  }
  else
  {
    if (mode == SETUP)
    {
      if (setupSetting == 0)
      {
        // change LED_BRIGHTNESS
        // readEncoder(ledBrightness, digitalRead(ENC1_2), digitalRead(ENC1_1), prevEnVal[0]);
        ledBrightness = ((0)>(((255)<(ledBrightness)?(255):(ledBrightness)))?(0):(((255)<(ledBrightness)?(255):(ledBrightness))));
      }
    }
    else
    {
      // readEncoder(posEncoder[0], digitalRead(ENC1_2), digitalRead(ENC1_1), prevEnVal[0]);
    }
  }

  // Encoder 2
  if (!modeChange && mode == SETUP)
  {

    // readEncoder(setupSetting, pcfEncoder.read(ENC2_1), pcfEncoder.read(ENC2_2), prevEnVal[1]);
    setupSetting = ((0)>(setupSetting % (NUM_SETTINGS + 1))?(0):(setupSetting % (NUM_SETTINGS + 1)));

    // Actual Setting
    if (setupSetting != prevSetupSetting)
    {

      // Setting Number in top left corner
      int bla = setupSetting + 1;
      printIfNumChanged(bla, prevSetupSetting, 0, 0, false);
      screen.drawString(2, 0, "/");
      int bla2 = NUM_SETTINGS + 1;
      int bla3 = NUM_SETTINGS;
      printIfNumChanged(bla2, bla3, 4, 0, false);
      prevSetupSetting = setupSetting;

      screen.drawString(0, 3, "               ");
      if (setupSetting == 0)
      {
        screen.drawString(0, 3, "BRIGHTNESS: ");
      }
      else if (setupSetting == 1)
      {
        screen.drawString(0, 3, "BULLSHIT: ");
      }
      else if (setupSetting == 2)
      {
        screen.drawString(0, 3, "SET 3: ");
      }
    }
  }
  else
  {
    // readEncoder(posEncoder[1], pcfEncoder.read(ENC2_1), pcfEncoder.read(ENC2_2), prevEnVal[1]);
  }

  // Encoder 3
  // readEncoder(posEncoder[2], pcfEncoder.read(ENC3_1), pcfEncoder.read(ENC3_2), prevEnVal[2]);
  // Encoder 4
  // readEncoder(posEncoder[3], pcfEncoder.read(ENC4_1), pcfEncoder.read(ENC4_2), prevEnVal[3]);

  prevButtonPressed[0] = buttonPressed[0];
  buttonPressed[0] = !digitalRead(3);
  prevButtonPressed[1] = buttonPressed[0];
  buttonPressed[1] = !pcfSwitch.read(7);
  prevButtonPressed[2] = buttonPressed[2];
  buttonPressed[2] = !pcfEncoder.read(2);
  prevButtonPressed[3] = buttonPressed[3];
  buttonPressed[3] = !pcfEncoder.read(5);

  // If ENC_1 pressed, toggle mode-Change mode
  if (buttonPressed[0] && !prevButtonPressed[0])
  {
    modeChange = !modeChange;
    if (!modeChange && mode == SETUP)
    {
      EEPROM.update(0, ledBrightness);
    }
  }

  for (int i = 0; i < sizeof(switches); i++)
  {
    switches[i] = pcfSwitch.read(i);
  }

  joy1Btn = divEncoder.read(0 /* Button is wired to Expander 2*/);
  joy2Btn = divEncoder.read(1 /* Button is wired to Expander 2*/);

  joy1X = analogRead(A0);
  joy1Y = analogRead(A7);
  joy1Z = analogRead(A6);

  joy2X = analogRead(A1);
  joy2Y = analogRead(A3);
  joy2Z = analogRead(A2);

  // ************************ DISPLAY **********************************

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

  // blink LEDs if modeChange is active
  if (modeChange && (millis() / 500) % 2 == 0)
  {
    for (int i = 0; i < 6; i++)
    {
      strip.setPixelColor(i, strip.Color(0, 0, ledBrightness));
    }
  }
  strip.show();

  // CHANGE MODE
  if (mode != prevMode)
  {
    prevMode = mode;
    // mode = abs(posEncoder[0]) % (ENCODER-NONE+1);
    modeSignClear = true;
    screen.clear();
    switch (mode)
    {
    case NONE:
      printCentered("DEFAULT", 3);
      break;
    case SETUP:
      printCentered("SETUP", 3);
      break;
    case JOYSTICK:
      printCentered("JOYSTICK", 3);
      break;
    case ENCODER:
      printCentered("ENCODER", 3);
      break;
    }
  }

  // Stop displaying mode in center display
  if (modeSignClear && !modeChange)
  {
    screen.clear();
    modeSignClear = false;
    if (mode == NONE)
    {
      screen.drawString(15, 0, (connectionState ? "C" : "N"));
    }
  }

  //********************* TIMED LOOP **********************************
  if (currentMillis - previousMillis >= 10)
  {

    previousMillis = currentMillis;

    if (mode != SETUP)
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

      if (connectionState != prevConnectionState && mode == NONE)
      {
        screen.drawString(15, 0, (connectionState ? "C" : "N"));
        prevConnectionState = connectionState;
      }
    }

    if (!modeChange)
    {
      // SHOW DATA
      if (mode == JOYSTICK)
      {
        printIfNumChanged(joy1X, prevJoy1X, 0, 0, false);
        printIfNumChanged(joy1Y, prevJoy1Y, 0, 3, false);
        printIfNumChanged(joy1Z, prevJoy1Z, 0, 6, false);
        printIfNumChanged(joy2X, prevJoy1X, 16, 0, true);
        printIfNumChanged(joy2Y, prevJoy1Y, 16, 3, true);
        printIfNumChanged(joy2Z, prevJoy1Z, 16, 6, true);
      }
      if (mode == ENCODER)
      {
        printIfNumChanged(posEncoder[0], posEncoderPrev[0], 0, 3, false);
        printIfNumChanged(posEncoder[1], posEncoderPrev[1], 0, 6, false);
        printIfNumChanged(posEncoder[2], posEncoderPrev[2], 16, 3, true);
        printIfNumChanged(posEncoder[3], posEncoderPrev[3], 16, 6, true);
      }
      if (mode == SETUP)
      {

        if (setupSetting == 0)
        {
          printIfNumChanged(ledBrightness, prevLedBrightness, 12, 3, false);
        }
      }
    }
  }
  //******************** END TIMED LOOP *******************************
}
//******************** END MAIN LOOP ********************************

void printIfNumChanged(int &val, int &prevVal, int x, int y, bool centerOnRight)
{
  if (val != prevVal)
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
    return readEncoder(val, digitalRead(5), digitalRead(4), prevEnVal[0], stepSize);
    break;
  case 1:
    return readEncoder(val, pcfEncoder.read(0), pcfEncoder.read(1), prevEnVal[1], stepSize);
    break;
  case 2:
    return readEncoder(val, pcfEncoder.read(3), pcfEncoder.read(4), prevEnVal[2], stepSize);
    break;
  case 3:
    return readEncoder(val, pcfEncoder.read(6), pcfEncoder.read(7), prevEnVal[3], stepSize);
    break;
  }
  return -999;
}

int readEncoder(int val, bool val1, bool val2, bool &valAPrev, int stepSize)
{

  int valNew = val;
  if ((valAPrev == 0x1) && (val1 == 0x0))
  {
    if (val2 == 0x1)
    {
      valNew = ((-9999)>(val - stepSize)?(-9999):(val - stepSize));
    }
    else
    {
      valNew = ((9999)<(val + stepSize)?(9999):(val + stepSize));
    }
  }
  valAPrev = val1;
  return valNew;
}

void setupMenues()
{

  topMenu.actions[1] = []()
  {
    activeMenu = &setupMenu;
    (*activeMenu).draw(screen);
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
    inputInteger("Startup-Delay", 0, 1000, &startupDelay, 100);
  };
}

void inputInteger(char *title, int min, int max, int *val, int stepSize)
{
  inputIntegerVal = val;
  inputIntegerMin = min;
  inputIntegerMax = max;
  inputIntegerStepsize = stepSize;
  screen.clear();
  screen.drawString(0, 0, title);
  inputMode = INPUT_INTEGER;
  drawInputInteger(*val);
}

void drawInputInteger(int valNew)
{
  *inputIntegerVal = valNew;
  char c[16];
  itoa(*inputIntegerVal, c, 10);
  screen.drawString(6, 4, "    ");
  screen.drawString(6, 4, c);
  screen.drawString(0, 6, "|              |");
  double percent = ((double)valNew / ((double)inputIntegerMax));
  for (int i = 0; i < (((percent * 14)>=0?(long)((percent * 14)+0.5):(long)((percent * 14)-0.5))); i++)
  {
    screen.drawString(i + 1, 6, "=");
  }
}
