/**
 * This example waves the brightness of a row of LEDs
 */

// This #include statement was automatically added by the Spark IDE.
#include "Adafruit_PCA9685.h"

Adafruit_PCA9685 ledDriver = Adafruit_PCA9685(0x40, true);  // Use the default address, but also turn on debugging

void setup()
{
  ledDriver.begin();    // This calls Wire.begin()
  ledDriver.setPWMFreq(1000);     // Maximum PWM frequency is 1600
  Serial.begin(9600);
}

int dutyCycle = 500;
int lastLed = 8;  // Edit this to how many LEDs you have wired up

void loop()
{
  for (int led=0; led <= lastLed; led++) {
      ledDriver.setVal(led, dutyCycle);
      delay(50);
  }

  dutyCycle += 1000;    // Edit this if you want to edit the brightness step size
  if (dutyCycle > 4095) {
    dutyCycle = 500;
  }
  int pwm = ledDriver.readPWMOff(7);
  Serial.print("Pwm value at LED 7 is "); Serial.println(pwm);
  Serial.print("Increasing duty cycle to " ); Serial.println( dutyCycle );
}