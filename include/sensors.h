#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include "Adafruit_TCS34725.h"

// functions for motors

class Motor {
  public:
    Motor(uint16_t speed_pin, uint16_t forward_pin, uint16_t backward_pin);
    void setup();
    void forward(uint16_t pwm_signal);
    void backward(uint16_t pwm_signal);
    void stop(uint16_t pwm_signal);


  private:
    uint16_t speedPin;
    uint16_t forwardPin;
    uint16_t backwardPin;
    bool initialized;
};

// functions for encoders

class Encoder {
  public:
    Encoder(uint16_t clock_pin, uint16_t dt_pin);
    void setup();
    // gives absolute position of encoder
    uint32_t getTicks();

  private:
    uint16_t clockPin;
    uint16_t dtPin;
    uint32_t encoderPosCount;
    uint32_t lastValue;
    bool initialized;
};


// functions for color sensor



// functions for TOF sensor



#endif