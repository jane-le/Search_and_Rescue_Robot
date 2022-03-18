#define CALIBRATE_IMU

#include <sensors.h>

IMU imu;

float mag_hardiron[3];
float mag_softiron[9];

mag_hardiron[0] = 13.61;
mag_hardiron[1] = -22.93;
mag_hardiron[2] = -31.0;

float mag_field = 50.06;

mag_softiron[0] = 0.998;
mag_softiron[1] = -0.002;
mag_softiron[2] = 0.017;
mag_softiron[3] = 0.002;
mag_softiron[4] = 1.071;
mag_softiron[5] = 0.045;
mag_softiron[6] = 0.017;
mag_softiron[7] = 0.045;
mag_softiron[8] = 0.937;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  imu.setup();
  imu.calibrate(mag_hardiron, mag_softiron, mag_field, 300);
}

void loop() {

}