#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <ArduinoSTL.h>
#include <HashMap.h>
#include <sensors.h>
#include <microTuple.h>
#include <map>

// Digital pins connected to TOF sensors
#define SHT_LOX1 23
#define SHT_LOX2 25
#define SHT_LOX3 27

// I2C addressing of TOF sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// motor pins
#define L_MOTOR_PWM 2
#define L_MOTOR_PIN1 22
#define L_MOTOR_PIN2 24
#define R_MOTOR_PWM 3
#define R_MOTOR_PIN1 26
#define R_MOTOR_PIN2 28

typedef enum {
  INIT,
  TILE_FORWARD,
  PIT_FORWARD,
  TURN_RIGHT,
  LEFT_ADJUST,
  RIGHT_ADJUST,
  STOP
} robot_state_t;

typedef enum {
  LEFT,
  TOP,
  RIGHT,
  BOTTOM
} robot_orientation;

robot_state_t robot_state = INIT;
int button_state = 0;
const int button_pin = 2;
int current_tile = 0;
robot_orientation current_orientation;
std::pair<double, double> robot_position;
const double WIDTH = 1800;
const double TILE_WIDTH = 300;
int left_motor_power = 0;
int right_motor_power = 0;
int last_turned_tile = -1; 

double prev_pitch = 0;
const int PIT_MOTOR_LOW = 40;
const int PIT_MOTOR_MED = 50;
const int PIT_MOTOR_HIGH = 60;
const int PIT_INCREMENT = 2; 
const int TILE_MOTOR_VALUE = 60;
const int TURN_MOTOR_VALUE_LEFT = 50;
const int TURN_MOTOR_VALUE_RIGHT = 50; // either stop (speed 0) or 50 in the reverse direction
const int ROBOT_WIDTH = 150;
const int ROBOT_LENGTH = 190;
const int ROBOT_MOTOR_OFFSET = 5;
const int RIGHT_ADJUST_DIST_TOL = -10;
const int RIGHT_ADJUST_ANGLE_TOL = 10;
const int LEFT_ADJUST_DIST_TOL = 10;
const int LEFT_ADJUST_ANGLE_TOL = 10;

const double CENTER_TILE_TOL = 10;

const double PITCH_UPWARDS_VALUE = 40;
const double PITCH_DOWNWARDS_VALUE = -40;
const double ADJUST_VALUE = 10;

// Initialize a motor
Motor left_motor(L_MOTOR_PWM, L_MOTOR_PIN1, L_MOTOR_PIN2);
Motor right_motor(R_MOTOR_PWM, R_MOTOR_PIN1, R_MOTOR_PIN2);

TOF front_tof(LOX2_ADDRESS, SHT_LOX2);
TOF back_tof(LOX3_ADDRESS, SHT_LOX3);

IMU imu;

TOF left_tof(LOX1_ADDRESS, SHT_LOX1); 


int heading_offset = 0; 
int pitch_offset = 0; 
int left_tof_offset = 0;

// S = start, E = end, T = turn
std::map<std::pair<int, int>, char> course = {
  {{5, 4}, 'S'}, {{5, 0}, 'T'}, {{0, 0}, 'T'}, {{0, 5}, 'T'},
  {{4, 5}, 'T'}, {{4, 1}, 'T'}, {{1, 1}, 'T'}, {{1, 4}, 'T'},
  {{3, 4}, 'T'}, {{3, 2}, 'T'}, {{2, 2}, 'T'}, {{2, 3}, 'E'}
};

// F = front tof, B = back tof, x and y in mm, based on 1.8 m
std::vector<std::vector<MicroTuple<int, int, char>>> coords = {
  {{150, 150, 'F'}, {450, 150, 'B'}, {750, 150, 'B'}, {1050, 150, 'F'}, {1350, 150, 'F'}, {1650, 150, 'F'}},
  {{150, 450, 'F'}, {450, 450, 'F'}, {750, 450, 'B'}, {1050, 450, 'F'}, {1350, 450, 'F'}, {1650, 450, 'B'}},
  {{150, 750, 'F'}, {450, 750, 'F'}, {750, 750, 'F'}, {1050, 750, 'F'}, {1350, 750, 'B'}, {1650, 750, 'B'}},
  {{150, 1050, 'B'}, {450, 1050, 'B'}, {750, 1050, 'F'}, {1050, 1050, 'B'}, {1350, 1050, 'F'}, {1650, 1050, 'F'}},
  {{150, 1350, 'B'}, {450, 1350, 'F'}, {750, 1350, 'F'}, {1050, 1350, 'B'}, {1350, 1350, 'B'}, {1650, 1350, 'F'}},
  {{150, 1650, 'F'}, {450, 1650, 'F'}, {750, 1650, 'F'}, {1050, 1650, 'B'}, {1350, 1650, 'B'}, {1650, 1650, 'F'}}
};

std::vector<std::pair<int, int>> path = {
  {5,4}, {5, 3}, {5, 2}, {5, 1}, {5, 0}, {4, 0}, {3, 0},
  {2, 0}, {1, 0}, {0, 0}, {0, 1}, {0, 2}, {0, 3},
  {0, 4}, {0, 5}, {1, 5}, {2, 5}, {3, 5}, {4, 5},
  {4, 4}, {4, 3}, {4, 2}, {4, 1}, {3, 1}, {2, 1},
  {1, 1}, {1, 2}, {1, 3}, {1, 4}, {2, 4}, {3, 4},
  {3, 3}, {3, 2}, {2, 2}, {2, 3}
};


/* ------------ Functions --------------- */

void setState(robot_state_t new_state) {
  Serial.println("Set State");
  robot_state = new_state;
  Serial.println(robot_state);
}

void calculatePosition(robot_orientation current_orientation, std::pair<double, double>& position) {
  
  Serial.println("TOF values");

  int front_tof_value = front_tof.getDistance(); 
  int left_tof_value = left_tof.getDistance();
  int back_tof_value = back_tof.getDistance();
  Serial.println(front_tof_value);
  Serial.println(left_tof_value);
  Serial.println(back_tof_value);

  /*
  if(abs(imu.getPitch() - pitch_offset) > 30) {
    Serial.println("didn't calculate");
    return;
  }  */  
  MicroTuple<int, int, char> coord = coords[path[current_tile + 1].first][path[current_tile+1].second];
  char which_tof = coord.get<2>();

  if (left_tof_value == -1) {
    return;   
  }
  if(which_tof == 'B' && back_tof_value == -1) {
    return;
  }

  if(which_tof == 'F' && front_tof_value == -1) {
    return;
  }
  

  switch (current_orientation) {
    case LEFT:
      position.first = left_tof.getDistance();
      position.second = which_tof == 'B' ? WIDTH - back_tof.getDistance() : front_tof.getDistance();
      break;

    case RIGHT:
      position.first = WIDTH - left_tof.getDistance();
      position.second = which_tof == 'B' ? back_tof.getDistance() : WIDTH - front_tof.getDistance();
      break;

    case TOP:
      position.first = which_tof == 'B' ? back_tof.getDistance() : WIDTH - front_tof.getDistance();
      position.second = left_tof.getDistance();
      break;

    case BOTTOM:
      position.first = which_tof == 'B' ? WIDTH - back_tof.getDistance() : front_tof.getDistance();
      position.second = WIDTH - left_tof.getDistance();
      break;
  }  
}

void updateCurrentTile(const std::pair<double, double>& position, const int next_tile, int& current_tile) {
  /*
  if(abs(imu.getPitch() - pitch_offset) > 10) {
    return;
  }*/
  
  MicroTuple<int, int, char> coord = coords[path[next_tile].first][path[next_tile].second];
  double next_center_x = coord.get<0>();
  double next_center_y = coord.get<1>();
  double position_x = position.first;
  double position_y = position.second;

  if (position_x <= next_center_x + TILE_WIDTH / 2.0 && position_x > next_center_x - TILE_WIDTH / 2.0
      && position_y <= next_center_y + TILE_WIDTH / 2.0 && position_y > next_center_y - TILE_WIDTH / 2.0) { // this may need tolerances
    current_tile = next_tile;
  }
}

bool shouldAdjustRight() {
  if ((left_tof.getDistance() - left_tof_offset) < RIGHT_ADJUST_DIST_TOL || (imu.getHeading() - heading_offset) > RIGHT_ADJUST_ANGLE_TOL ) {
    return true;
  } else {
    return false;
  }
}

bool shouldAdjustLeft() {
  if ((left_tof.getDistance() - left_tof_offset) > LEFT_ADJUST_DIST_TOL || (heading_offset - imu.getHeading()) > LEFT_ADJUST_ANGLE_TOL ) {
    return true;
  } else {
    return false;
  }
}

// INIT state, robot waits for push button to be pressed before moving
void handleInit() {
  /*
  while (digitalRead(button_pin) == LOW) {
    imu.updateIMU();
  }
*/ 
  Serial.println("HandleInit");
  delay(5000);
  imu.updateIMU();
  heading_offset = imu.getHeading(); 
  pitch_offset = imu.getPitch(); 
  left_tof_offset = left_tof.getDistance();
  
  setState(TILE_FORWARD);
}

void handleTileForward() {
  //left_tof.addValue();
  
  if ((imu.getPitch() - pitch_offset) < PITCH_DOWNWARDS_VALUE) {
    setState(PIT_FORWARD);
    return;
  }

  // check for turn or stop
  // only change state when robot is center of tile
  std::map<std::pair<int, int>, char>::iterator it = course.find(path[current_tile]);
  if (it != course.end()) {
    char landmark = it->second;

    if (landmark == 'T' && last_turned_tile != current_tile) {
      setState(TURN_RIGHT);
      return;
    } else if (landmark == 'E') {
      setState(STOP);
      return;
    }
  }

  left_motor_power = TILE_MOTOR_VALUE;
  right_motor_power = TILE_MOTOR_VALUE;

  // drive straight
  if (shouldAdjustLeft()) {
    setState(LEFT_ADJUST);
    return;
  }

  if (shouldAdjustRight()) {
    setState(RIGHT_ADJUST);
    return;
  }

  left_motor.backward(left_motor_power);
  right_motor.backward(right_motor_power); 
}

void handlePitForward() {
  Serial.println("handlePitForward");

  if((imu.getPitch() + pitch_offset) < pitch_offset - 5) {
    left_motor_power = PIT_MOTOR_LOW;
    right_motor_power = PIT_MOTOR_LOW;
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);
  }

  if(imu.getPitch() > pitch_offset - 5 && imu.getPitch() < pitch_offset + 5) {
    left_motor_power = left_motor_power >= PIT_MOTOR_MED ? left_motor_power : left_motor_power + PIT_INCREMENT;
    right_motor_power = right_motor_power >= PIT_MOTOR_MED ? right_motor_power : right_motor_power + PIT_INCREMENT;
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);
  }

  if((imu.getPitch() + pitch_offset) > pitch_offset + 5) {
    
    left_motor_power = left_motor_power >= PIT_MOTOR_HIGH ? left_motor_power : left_motor_power + PIT_INCREMENT;
    right_motor_power = right_motor_power >=  PIT_MOTOR_HIGH ? right_motor_power : right_motor_power + PIT_INCREMENT;
    
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);
      
    // exit pit state
    while(!(imu.getPitch() > pitch_offset - 5 && imu.getPitch() < pitch_offset + 5)) {
      imu.updateIMU();   
    }
      
    setState(TILE_FORWARD);
    
  }
}

void handleTurnRight() {
  Serial.println("handleTurnRight");
  int target_heading = imu.getHeading() - 90 < 0 ? imu.getHeading() - 90 + 360 : imu.getHeading() - 90; // should we use heading offset here??

  left_motor.stop();
  right_motor.stop();
  
  while (abs(imu.getHeading() - target_heading) > 5) {
    imu.updateIMU();
    left_motor_power = TURN_MOTOR_VALUE_LEFT;
    right_motor_power = TURN_MOTOR_VALUE_RIGHT;
    left_motor.backward(left_motor_power);
    right_motor.forward(right_motor_power);
  }

  left_motor.stop();
  right_motor.stop();
  
  switch (current_orientation) {
    case (LEFT):
      current_orientation = TOP;
      break;
    case (TOP):
      current_orientation = RIGHT;
      break;
    case (RIGHT):
      current_orientation = BOTTOM;
      break;
    case (BOTTOM):
      current_orientation = LEFT;
      break;
  }

  left_tof_offset = left_tof.getDistance(); // get new correct distance to wall (the distance to keep straight at)
  heading_offset =  heading_offset - 90 < 0 ? heading_offset - 90 + 360 : heading_offset - 90; // calculate new correct heading
  setState(TILE_FORWARD);
  last_turned_tile = current_tile;
  //left_tof.clearValues();
}

void handleLeftAdjust() {
    Serial.println("handleLeftAdjust");
    right_motor_power = ADJUST_VALUE + TILE_MOTOR_VALUE;
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);

    if(abs(imu.getHeading() - heading_offset) < 10) {
      setState(TILE_FORWARD);
    }
}

void handleRightAdjust() {
    Serial.println("handleRightAdjust");
    left_motor_power = ADJUST_VALUE + TILE_MOTOR_VALUE;
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);

    if(abs(imu.getHeading() - heading_offset) < 10) {
      setState(TILE_FORWARD);
    }
}

void handleStop() {
  Serial.println("handleStop");
  left_motor.stop();
  right_motor.stop();
  Serial.println("STOP!!");
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  Serial.println(F("Setting up TOF")); 
  pinMode(left_tof.shutdownPin, OUTPUT);
  pinMode(front_tof.shutdownPin, OUTPUT);
  pinMode(back_tof.shutdownPin, OUTPUT);
  delay(10);
  
  // all reset   
  digitalWrite(left_tof.shutdownPin, LOW);
  digitalWrite(front_tof.shutdownPin, LOW);
  digitalWrite(back_tof.shutdownPin, LOW);
  delay(10);

  // all unreset   
  digitalWrite(left_tof.shutdownPin, HIGH);
  digitalWrite(front_tof.shutdownPin, HIGH);
  digitalWrite(back_tof.shutdownPin, HIGH);
  delay(10);

  // activating leftTOF and resetting other two
  digitalWrite(left_tof.shutdownPin, HIGH);
  digitalWrite(front_tof.shutdownPin, LOW);
  digitalWrite(back_tof.shutdownPin, LOW);
  Serial.println(F("Here"));
  left_tof.init();

  delay(10);
  Serial.println(F("Set up TOF"));

  digitalWrite(front_tof.shutdownPin, HIGH);
  front_tof.init();
  Serial.println(F("Set up front TOF"));

  delay(10);
  
  digitalWrite(back_tof.shutdownPin, HIGH);
  back_tof.init();
  Serial.println(F("Set up all TOF"));

  

  // setup motors and encoders
  left_motor.init();
  right_motor.init();
  
  // then initialize imu
  imu.init();

  pinMode(button_pin, INPUT);
  robot_state = INIT;
  current_orientation = BOTTOM;
  current_tile = 0;
  robot_position.first = 0;
  robot_position.second = 0;

  Serial.println(F("Set up all sensors"));
}

void loop() {
  Serial.println("State");
  Serial.println(robot_state);
  Serial.println("Current Tile");
  Serial.println(current_tile);
  Serial.println("Current Orientation");
  Serial.println(current_orientation);
  Serial.println("Position");
  Serial.println(robot_position.first);
  Serial.println(robot_position.second);

  imu.updateIMU();
  // calculate position and localize (match with map)
  
  calculatePosition(current_orientation, robot_position);
  int next_tile = current_tile + 1;

  if (next_tile >= path.size())  {
    setState(STOP);
  } else {
    // update current tile to next tile if position is in the bounds, update robot path
    updateCurrentTile(robot_position, next_tile, current_tile);
  }

  prev_pitch = imu.getPitch(); 

  switch (robot_state) {
    case INIT:
      handleInit();
      break;
    case TILE_FORWARD:
      handleTileForward();
      break;
    case PIT_FORWARD:
      handlePitForward();
      break;
    case TURN_RIGHT:
      handleTurnRight();
      break;
    case LEFT_ADJUST:
     handleLeftAdjust();
      break;
    case RIGHT_ADJUST:
     handleRightAdjust();
     break;
    case STOP:
     handleStop();
     break;
  }

  delay(200); 
}
