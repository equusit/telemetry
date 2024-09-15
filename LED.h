#ifndef LED_H
#define LED_H

#include <Arduino.h>

// LED class with default constructor
class LED {
public:
    LED() : pinNumber(-1) {
        // Default constructor
    }

    LED(int pin) : pinNumber(pin) {
        pinMode(pinNumber, OUTPUT);
    }

    void turnOn() {
        if (pinNumber >= 0) {
            digitalWrite(pinNumber, HIGH);
        }
    }

    void turnOff() {
        if (pinNumber >= 0) {
            digitalWrite(pinNumber, LOW);
        }
    }

    void blink(int delayTime, int repeat) {
        if (pinNumber >= 0) {
           for (int i = repeat; i > 0; i--) { 
            turnOn();
            delay(delayTime);
            turnOff();
            delay(delayTime);
           }
        }
    }

private:
    int pinNumber;
};

// LEDGroup class
class LEDGroup {
public:
    LEDGroup() : count(0) {}

    void addLED(int pin) {
        if (count < MAX_LEDS) {
            leds[count] = LED(pin);  // Use parameterized constructor
            count++;
        }
    }

    void turnAllOn() {
        for (int i = 0; i < count; ++i) {
            leds[i].turnOn();
        }
    }

    void turnAllOff() {
        for (int i = 0; i < count; ++i) {
            leds[i].turnOff();
        }
    }

    void blinkAll(int delayTime, int repeat) {
      for (int r = repeat; r > 0; r--) {
        turnAllOn();
        delay(delayTime);
        turnAllOff();
        delay(delayTime);
      }
    }

private:
    static const int MAX_LEDS = 10;  // Adjust as needed
    LED leds[MAX_LEDS];
    int count;
};

#endif  // LED_H
