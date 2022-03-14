#include "Arduino.h"
#include "sensors.h"
#include <Wire.h>

#define MAX_PWM 255
#define DEBOUNCE_DELAY 50  // 50 ms (may need to change)

Motor::Motor(uint16_t speed_pin, uint16_t forward_pin, uint16_t backward_pin)
{
    speedPin = speed_pin;
    forwardPin = forward_pin;
    backwardPin = backward_pin;
    pinMode(speedPin, OUTPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
}

void Motor::forward(uint16_t pwm_signal = 100)
{
    uint16_t pwm = map(pwm_signal, 0, 100, 0, MAX_PWM);
    analogWrite(speedPin, pwm);
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backwardPin, LOW);
}

void Motor::backward(uint16_t pwm_signal = 100)
{
    uint16_t pwm = map(pwm_signal, 0, 100, 0, MAX_PWM);
    analogWrite(speedPin, pwm);
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, HIGH);
}

void Motor::stop()
{
    analogWrite(speedPin, MAX_PWM);
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
}

Encoder::Encoder(uint16_t clock_pin, uint16_t dt_pin)
{
    clockPin = clock_pin;
    dtPin = dt_pin;
    lastDebounceTime = 0;
    encoderPosCount = 0;
    pinMode(clockPin, INPUT);
    pinMode(dtPin, INPUT);
    lastValue = digitalRead(clockPin);
}

uint32_t Encoder::update()
{
    int clockVal = digitalRead(clockPin);

    if (clockVal != lastValue) {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    // check if reading is valid and if so check direction of encoder
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // if dtPin is different then clockVal we are rotating forward
        if (digitalRead(dtPin) != clockVal)) {
            ++encoderPosCount;
        } else {
            --encoderPosCount;
        }
    }

    lastValue = clockVal;
}

unsigned long Encoder::getTicks()
{
    return encoderPosCount;
}


 TOF::TOF()
 {
     if (!lox.begin())
     {
         Serial.println(F("Failed to boot VL53L0X"));
         while (1);
     }
 }

 void TOF::update()
 {
     // read sensor value
     // append to buffer
     // average values using buffer
     // update current reading based on filter
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

 }

 uint16_t TOF::getDistance()
 {
     return distance;
 }
 