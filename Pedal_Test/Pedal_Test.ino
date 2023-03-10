/*
MIT License
Copyright 2021 Michael Schoeffler (https://www.mschoeffler.com)
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
 * Example source code of an Arduino tutorial on how to control an MG 996R servo motor.
 */

#include <Servo.h>
#include <Adafruit_NeoPixel.h>



Servo servo; // servo object representing the MG 996R servo
int min = 251;
int max = 803;
int deadZone = 5;

int servoMin = 43;
int servoMax = 148;

Adafruit_NeoPixel LED(9, 4, NEO_GRB + NEO_KHZ800);

void setup()
{
  Serial.begin(19200);
  servo.attach(5); // servo is wired to Arduino on digital pin 3
  LED.begin();
  //int ser = random(servoMin, servoMax);
  //servo.write(ser);
  //delay(10000);
  //servo.detach();
}

void loop()
{
  int value = analogRead(A0);
  int newValue = map(value, max - deadZone, min + deadZone, 0, 127);
  newValue = min(max(newValue, 0), 127);
  Serial.print(value);
  Serial.print(" , ");
  Serial.print(newValue);
  int led = map(newValue, 0, 127, 0, 9);

  int serv = map(newValue, 0, 127, servoMin, servoMax);

  serv = (servoMin + servoMax) / 2 +5;

  servo.write(serv);

  Serial.print(" , ");
  Serial.println(serv);
  for (int i = 0; i < 9 - led; i++)
  {
    LED.setPixelColor(i, LED.Color(0, 0, 0));
  }
  for (int i = 9 - led; i < 9; i++)
  {
    LED.setPixelColor(i, LED.Color(255, 0, 0));
  }
  LED.show();
}