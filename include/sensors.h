#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include "Adafruit_TCS34725.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

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
    void calibrate(float* mag_hardiron, float* mag_softiron, float mag_field, int num_points);
    void update();
    float getHeading();
    float getPitch();
    float getRoll();
    void getQuaternion(float* qw, float* qx, float* qy, float* qz);
    void getYPR(float* y, float* p, float* r);

  private:
    float heading;
    float pitch;
    float roll;

    float _qx, _qy, _qz, _qw;

    Adafruit_ICM20948 icm;
    Adafruit_Mahony filter;
    Adafruit_Sensor_Calibration_EEPROM cal;
    Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
};



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
};

// functions for colour sensor
class ColorSensor {
  public:
    ColorSensor();
    bool isSand();
    bool isTile();
  private:
    Adafruit_TCS34725 tcs;
};


#endif