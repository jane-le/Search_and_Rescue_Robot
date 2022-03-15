#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
// #include "Adafruit_TCS34725.h"
#include "Adafruit_VL53L0X.h"

// functions for motors

class Motor {
  public:
    Motor(uint16_t speed_pin, uint16_t forward_pin, uint16_t backward_pin);
    void forward(uint16_t pwm_signal);
    void backward(uint16_t pwm_signal);
    void stop();


  private:
    uint16_t speedPin;
    uint16_t forwardPin;
    uint16_t backwardPin;
};

// functions for encoders

class Encoder {
  public:
    Encoder(uint16_t clock_pin, uint16_t dt_pin);

    // this functions should get called on every loop to update the encoder values
    void update();

    // gives absolute position of encoder
    unsigned long getTicks();

  private:
    uint16_t clockPin;
    uint16_t dtPin;
    uint32_t encoderPosCount;
    uint32_t lastValue;
    unsigned long ticks;
    unsigned long lastDebounceTime;
};


// functions for imu sensor
class IMU {
  public:
    IMU();
    void calibrate();
    void update();
    float getYaw();
    float getPitch();
    float getRoll();

    bool getYPR(float* y, float* p, float* r);

  private:
    float yaw;
    float pitch;
    float roll;


// functions for TOF sensor
class TOF {
  public:
    TOF();
    void update();
    uint16_t getDistance();
    
  private:
    Adafruit_VL53L0X lox;
    uint32_t buffer[10];
    uint16_t distance;
}


#endif