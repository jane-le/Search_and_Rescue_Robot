#include "Arduino.h"
#include "sensors.h"
#include <Wire.h>

#define MAX_PWM 255

Motor::Motor(uint16_t speed_pin, uint16_t forward_pin, uint16_t backward_pin)
{
    speedPin = speed_pin;
    forwardPin = forward_pin;
    backwardPin = backward_pin;
    initialized = false;
}

void Motor::setup();
{
    pinMode(speedPin, OUTPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
    initialized = true;
}

void Motor::forward(motor_t motor, uint16_t pwm_signal = 100)
{
    if (!initialized) setup();
    uint16_t pwm = map(pwm_signal, 0, 100, 0, MAX_PWM);
    analogWrite(speedPin, pwm);
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backwardPin, LOW);
}

void Motor::backward(motor_t motor, uint16_t pwm_signal = 100)
{
    if (!initialized) setup();
    uint16_t pwm = map(pwm_signal, 0, 100, 0, MAX_PWM);
    analogWrite(speedPin, pwm);
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, HIGH);
}

void Motor::stop(motor_t motor)
{
    if (!initialized) setup();
    analogWrite(speedPin, MAX_PWM);
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
}

Encoder::Encoder(uint16_t clock_pin, uint16_t dt_pin)
{
    clockPin = clock_pin;
    dtPin = dt_pin;
    initialized = false;
}

void Encoder::setup()
{
    pinMode(clockPin, INPUT);
    pinMode(dtPin, INPUT);
    lastValue = digitalRead(clockPin);
    initialized = true;
}

uint32_t Encoder::getTicks()
{
    if (!initialized) setup();

}