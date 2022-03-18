#include "Arduino.h"
#include "sensors.h"
#include <Wire.h>

#define MAX_PWM 255
#define DEBOUNCE_DELAY 50  // 50 ms (may need to change)

// uncomment this for debugging
//#define DEBUG_ON

// uncomment this for calibration
//#define CALIBRATE_IMU

#define SAND_R_MIN 100
#define SAND_R_MAX 120
#define SAND_G_MIN 100
#define SAND_G_MAX 120
#define SAND_B_MIN 100
#define SAND_B_MAX 120

#define TILE_R_MIN 200
#define TILE_R_MAX 220
#define TILE_G_MIN 200
#define TILE_G_MAX 220
#define TILE_B_MIN 200
#define TILE_B_MAX 220

Motor::Motor(uint16_t speed_pin, uint16_t forward_pin, uint16_t backward_pin)
{
    speedPin = speed_pin;
    forwardPin = forward_pin;
    backwardPin = backward_pin;
}

void Motor::init()
{
    pinMode(speedPin, OUTPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
#ifdef DEBUG_ON
    Serial.println(F("Initialized Motor"));
#endif
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
}

void Encoder::init()
{
    pinMode(clockPin, INPUT);
    pinMode(dtPin, INPUT);
    lastValue = digitalRead(clockPin);
#ifdef DEBUG_ON
    Serial.println(F("Initialized Encoder"));
#endif
}

void Encoder::update()
{
    int clockVal = digitalRead(clockPin);

    if (clockVal != lastValue) {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    // check if reading is valid and if so check direction of encoder
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // if dtPin is different then clockVal we are rotating forward
        if (digitalRead(dtPin) != clockVal) {
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

TOF::TOF(uint16_t lox_address, uint16_t shutdown_pin)
{
    loxAddress = lox_address;
    shutdownPin = shutdown_pin;
}
void TOF::init()
{
    if (!lox.begin(loxAddress)) {
        Serial.println(F("Failed to boot VL53L0X"));
        while (1);
    }
#ifdef DEBUG_ON
    Serial.println(F("Found VL53L0X"));
#endif
}

int TOF::getDistance()
{
     // read sensor value
    VL53L0X_RangingMeasurementData_t rangingMeasurementData;
    lox.getSingleRangingMeasurement(&rangingMeasurementData);
    // make sure reading is valid and less than 1m
    if (rangingMeasurementData.RangeStatus != 4 && rangingMeasurementData.RangeMilliMeter < 1000) {
        return rangingMeasurementData.RangeMilliMeter;
    }
    return -1;
     // append to buffer
     // average values using buffer
     // update current reading based on filter
}

void setupTOF(TOF leftTOF) {
    pinMode(leftTOF.shutdownPin, OUTPUT);    
    // pinMode(frontTOF.shutdownPin, OUTPUT);
    // pinMode(backTOF.shutdownPin, OUTPUT);
    delay(10);
    // all reset
    digitalWrite(leftTOF.shutdownPin, LOW);    
    // digitalWrite(frontTOF.shutdownPin, LOW);
    // digitalWrite(backTOF.shutdownPin, LOW);
    delay(10);

    // all unreset
    digitalWrite(leftTOF.shutdownPin, HIGH);    
    // digitalWrite(frontTOF.shutdownPin, HIGH);
    // digitalWrite(backTOF.shutdownPin, HIGH);
    delay(10);

    // activating leftTOF and resetting other two
    digitalWrite(leftTOF.shutdownPin, HIGH);
    // digitalWrite(frontTOF.shutdownPin, LOW);
    // digitalWrite(backTOF.shutdownPin, LOW);

    leftTOF.init();
    delay(10);

    // activating frontTOF
    // digitalWrite(frontTOF.shutdownPin, HIGH);
    // delay(10);

    // frontTOF.init();
    // delay(10);

    // // activate backTOF
    // digitalWrite(backTOF.shutdownPin, HIGH);
    // delay(10);

    // backTOF.init();
    // delay(10);
}

void IMU::init()
{
    if (!icm.begin_I2C()) {
        Serial.println("Failed to find sensors");
        while (1) delay(10);
    }
    icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);

    // set slightly above refresh rate
    icm.setAccelRateDivisor(4095);
    icm.setGyroRateDivisor(255);
    icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

    accelerometer = icm.getAccelerometerSensor();
    gyroscope = icm.getGyroSensor();
    magnetometer = icm.getMagnetometerSensor();

    if (!cal.begin()) {
        Serial.println("Failed to initialize calibration helper");
        while (1) yield();
    }
    if (!cal.loadCalibration()) {
        Serial.println("No calibration loaded/found... will start with defaults");
    } else {
        Serial.println("Loaded existing calibration");
    }
}

void IMU::calibrate(float* mag_hardiron, float* mag_softiron, float mag_field, int num_points = 100)
{
#ifdef CALIBRATE_IMU
    sensors_event_t mag_event, gyro_event, accel_event;
    
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
    
    Serial.println(F("Adafruit AHRS - IMU Calibration!"));

    Serial.println("Calibration filesys test");
    if (!cal.begin()) {
        Serial.println("Failed to initialize calibration helper");
        while (1) yield();
    }
    if (! cal.loadCalibration()) {
        Serial.println("No calibration loaded/found... will start with defaults");
    } else {
        Serial.println("Loaded existing calibration");
    }

    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
    magnetometer->printSensorDetails();
    
    Wire.setClock(400000); // 400KHz

    if (num_points < 100)
        num_points = 100;

    
    float accelx_u = 0, accely_u = 0, accelz_u = 0;
    float gx_u = 0, gy_u = 0, gz_u = 0;

    // read num_points imu measurements and get the standard deviation then apply the offset
    for (int i = 0; i < num_points; ++i)
    {
        magnetometer->getEvent(&mag_event);
        gyroscope->getEvent(&gyro_event);
        accelerometer->getEvent(&accel_event);

        accelx_u += accel_event.acceleration.x;
        accely_u += accel_event.acceleration.y;
        accelz_u += accel_event.acceleration.z;

        gx_u += gyro_event.gyro.x;
        gy_u += gyro_event.gyro.y;
        gz_u += gyro_event.gyro.z;
        delay(20);
    }

    cal.accel_zerog[0] = accelx_u / num_points;
    cal.accel_zerog[1] = accely_u / num_points;
    cal.accel_zerog[2] = accelz_u / num_points - 9.81;
    
    cal.gyro_zerorate[0] = gx_u / num_points;
    cal.gyro_zerorate[1] = gy_u / num_points;
    cal.gyro_zerorate[2] = gz_u / num_points;
    
    cal.mag_hardiron[0] = mag_hardiron[0];
    cal.mag_hardiron[1] = mag_hardiron[1];
    cal.mag_hardiron[2] = mag_hardiron[2];

    cal.mag_field = mag_field;
    
    cal.mag_softiron[0] = mag_softiron[0];
    cal.mag_softiron[1] = mag_softiron[1];
    cal.mag_softiron[2] = mag_softiron[2];
    cal.mag_softiron[3] = mag_softiron[3];
    cal.mag_softiron[4] = mag_softiron[4];
    cal.mag_softiron[5] = mag_softiron[5];
    cal.mag_softiron[6] = mag_softiron[6];
    cal.mag_softiron[7] = mag_softiron[7];
    cal.mag_softiron[8] = mag_softiron[8];

    if (! cal.saveCalibration()) {
        Serial.println("**WARNING** Couldn't save calibration");
    } else {
        Serial.println("Wrote calibration");    
    }
    cal.printSavedCalibration()
#endif
}

void IMU::update()
{
    sensors_event_t accel, gyro, mag;

    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);

    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);

    float gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    float gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    float gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
    
#ifdef DEBUG_ON
    Serial.print("Raw: ");
    Serial.print(accel.acceleration.x, 4); Serial.print(", ");
    Serial.print(accel.acceleration.y, 4); Serial.print(", ");
    Serial.print(accel.acceleration.z, 4); Serial.print(", ");
    Serial.print(gx, 4); Serial.print(", ");
    Serial.print(gy, 4); Serial.print(", ");
    Serial.print(gz, 4); Serial.print(", ");
    Serial.print(mag.magnetic.x, 4); Serial.print(", ");
    Serial.print(mag.magnetic.y, 4); Serial.print(", ");
    Serial.print(mag.magnetic.z, 4); Serial.println("");
#endif

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    filter.getQuaternion(&_qw, &_qx, &_qy, &_qz);
}

float IMU::getPitch()
{
    return pitch;
}

float IMU::getHeading()
{
    return heading;
}

float IMU::getRoll()
{
    return roll;
}

void IMU::getQuaternion(float *qw, float *qx, float *qy, float *qz)
{
    *qw = _qw;
    *qx = _qx;
    *qy = _qy;
    *qz = _qz;
}

void IMU::getYPR(float *y, float *p, float *r)
{
    *y = heading;
    *p = pitch;
    *r = roll;
}

ColorSensor::ColorSensor()
{
    enabled = true;
}

void ColorSensor::init()
{
    tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
    if (tcs.begin()) {
        Serial.println("Found color sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1);
    }
}

void ColorSensor::disable()
{
    if (enabled)
        tcs.disable();
    enabled = false;
}

void ColorSensor::enable()
{
    if (!enabled)
        tcs.enable();
    enabled = true;
}

bool ColorSensor::isSand()
{
    float r, g, b;
    tcs.getRGB(&r, &g, &b);

    if (r > SAND_R_MIN && r < SAND_R_MAX && g > SAND_G_MIN && g < SAND_G_MAX && b < SAND_B_MIN && b > SAND_B_MAX)
    {
        return true;
    }
    return false;
}

bool ColorSensor::isTile()
{
    float r, g, b;
    tcs.getRGB(&r, &g, &b);

    if (r > TILE_R_MIN && r < TILE_R_MAX && g > TILE_G_MIN && g < TILE_G_MAX && b < TILE_B_MIN && b > TILE_B_MAX)
    {
        return true;
    }
    return false;
}
