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
#define SHT_LOX1 25
#define SHT_LOX2 27

// I2C addressing of TOF sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

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
  TURN_ADJUST,
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

const int PIT_MOTOR_LOW = 40;
const int PIT_MOTOR_MED = 50;
const int PIT_MOTOR_HIGH = 60;
const int PIT_INCREMENT = 2; 
const int TILE_MOTOR_VALUE = 50;
const int TURN_MOTOR_VALUE = 50;
const int SLOW_MOTOR_VALUE = 40;
const int TILE_ADJUST_VALUE = 50;

const int ROBOT_WIDTH = 150;
const int ROBOT_LENGTH = 190;
const int LEFT_DIST_TOL = 20;
const int LEFT_DIST_ADJUST_TOL = 30;

const double CENTER_TILE_TOL = 10;

const double PITCH_UPWARDS_VALUE = 40;
const double PITCH_DOWNWARDS_VALUE = -40;
const double ADJUST_VALUE = 10;

const int RIGHT_ADJUST_DIST_TOL = -10;
const int RIGHT_ADJUST_ANGLE_TOL = 10;
const int LEFT_ADJUST_DIST_TOL = 10;
const int LEFT_ADJUST_ANGLE_TOL = 10;

// Initialize a motor
Motor left_motor(L_MOTOR_PWM, L_MOTOR_PIN1, L_MOTOR_PIN2);
Motor right_motor(R_MOTOR_PWM, R_MOTOR_PIN1, R_MOTOR_PIN2);

TOF front_tof(LOX2_ADDRESS, SHT_LOX2, false);
TOF left_tof(LOX1_ADDRESS, SHT_LOX1, true); 

IMU imu;


int heading_offset = 0; 
int pitch_offset = 0; 
int des_left_offset = 0;

// S = start, E = end, T = turn
std::map<std::pair<int, int>, char> course = {
  {{5, 4}, 'S'}, {{5, 0}, 'T'}, {{0, 0}, 'T'}, {{0, 5}, 'T'},
  {{4, 5}, 'T'}, {{4, 1}, 'T'}, {{1, 1}, 'T'}, {{1, 4}, 'T'},
  {{3, 4}, 'T'}, {{3, 2}, 'T'}, {{2, 2}, 'T'}, {{2, 3}, 'E'}
};

std::map<std::pair<int, int>, robot_orientation> turn_orientation = {
  {{5, 0}, BOTTOM}, {{0, 0}, LEFT}, {{0, 5}, TOP},
  {{4, 5}, RIGHT}, {{4, 1}, BOTTOM}, {{1, 1}, LEFT}, {{1, 4}, TOP},
  {{3, 4}, RIGHT}, {{3, 2}, BOTTOM}, {{2, 2}, TOP}
};


// F = front tof, B = back tof, x and y in mm, based on 1.8 m
std::vector<std::vector<std::pair<int, int>>> coords = {
  {{150, 150}, {450, 150}, {750, 150}, {1050, 150}, {1350, 150}, {1650, 150}},
  {{150, 450}, {450, 450}, {750, 450}, {1050, 450}, {1350, 450}, {1650, 450}},
  {{150, 750}, {450, 750}, {750, 750}, {1050, 750}, {1350, 750}, {1650, 750}},
  {{150, 1050}, {450, 1050}, {750, 1050}, {1050, 1050}, {1350, 1050}, {1650, 1050}},
  {{150, 1350}, {450, 1350}, {750, 1350}, {1050, 1350}, {1350, 1350}, {1650, 1350}},
  {{150, 1650}, {450, 1650}, {750, 1650}, {1050, 1650}, {1350, 1650}, {1650, 1650}}
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
  robot_state = new_state;
  Serial.println(robot_state);
}

void calculatePosition(robot_orientation current_orientation, std::pair<double, double>& position) {
  int left_tof_value = left_tof.getDistance();
  int front_tof_value = front_tof.getDistance();
  Serial.println(" "); 
  Serial.print("TOF VALUES "); 
  Serial.print(left_tof_value); 
  Serial.print("   "); 
  Serial.print(front_tof_value); 
  Serial.println(" "); 

  if (left_tof_value == -1 || front_tof_value == -1) {
    return;   
  }
  
  switch (current_orientation) {
    case LEFT:
      position.first = left_tof_value + ROBOT_WIDTH / 2.0;
      position.second = front_tof_value + ROBOT_LENGTH / 2.0;
      break;
      position.first = WIDTH - left_tof_value - ROBOT_WIDTH / 2.0;
      position.second = WIDTH - front_tof_value - ROBOT_LENGTH / 2.0; 
      break;
    case TOP:
      position.first = WIDTH - front_tof_value - ROBOT_LENGTH / 2.0; 
      position.second = front_tof_value + ROBOT_LENGTH / 2.0;
      break;
    case BOTTOM:
      position.first = front_tof_value + ROBOT_LENGTH / 2.0; 
      position.second = WIDTH - left_tof_value - ROBOT_WIDTH / 2.0;
      break;
  }  
}

void updateCurrentTile(const std::pair<double, double>& position, const int next_tile, int& current_tile) {
  /*
  if(abs(imu.getPitch() - pitch_offset) > 40) {
    return;
  }
*/
  if(left_tof.getDistance() == -1 || front_tof.getDistance() == -1) {
    return;
  }
  
  std::pair<int, int> coord = coords[path[next_tile].first][path[next_tile].second];
  double next_center_x = coord.first;
  double next_center_y = coord.second;
  double position_x = position.first;
  double position_y = position.second;

  // check if we're in the next tile
  if (position_x <= next_center_x + TILE_WIDTH / 2.0 && position_x > next_center_x - TILE_WIDTH / 2.0
      && position_y <= next_center_y + TILE_WIDTH / 2.0 && position_y > next_center_y - TILE_WIDTH / 2.0) {
    current_tile = next_tile;
    return;
  } 
  
  // check if we're not in the current tile and if we're not, where the hell are we?
  std::pair<int, int> curr_coord = coords[path[current_tile].first][path[current_tile].second];
  double curr_center_x = curr_coord.first;
  double curr_center_y = curr_coord.second;

  if (position_x > curr_center_x + TILE_WIDTH / 2.0 || position_x < curr_center_x - TILE_WIDTH / 2.0
      || position_y > curr_center_y + TILE_WIDTH / 2.0 || position_y < curr_center_y - TILE_WIDTH / 2.0) {
        Serial.println("FIND WHERE WE ARE..");
    
    // iterate through coords and find the tile we're on 
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            std::pair<int, int> ij_coord = coords[i][j];

            double ij_center_x = ij_coord.first;
            double ij_center_y = ij_coord.second;
            
            if (position_x <= ij_center_x + TILE_WIDTH / 2.0 && position_x > ij_center_x - TILE_WIDTH / 2.0
                || position_y <= ij_center_y + TILE_WIDTH / 2.0 && position_y > ij_center_y - TILE_WIDTH / 2.0) {
                // iterate through path to get current_tile
                for(int k = 0; k < path.size(); k++) {
                  if(path[k].first == i && path[k].second == j) {
                      current_tile = k; 
                      Serial.print("..FOUND where we are..");
                      Serial.print(current_tile);
                      Serial.println(" ");
                      return;
                  }
                }
             }
          }
      }
  } 
}

// returns desired left tof value 
int getDesLeftDist() {
  std::pair<int, int> coord = coords[path[current_tile].first][path[current_tile].second];
  
  switch (current_orientation) {
    case LEFT:
      return coord.first - ROBOT_WIDTH / 2 + LEFT_DIST_TOL;
    case RIGHT:
      return WIDTH - coord.first - ROBOT_WIDTH / 2 + LEFT_DIST_TOL;
    case TOP:
      return coord.second - ROBOT_WIDTH / 2 + LEFT_DIST_TOL;
    case BOTTOM:
      return WIDTH - coord.second - ROBOT_WIDTH / 2 + LEFT_DIST_TOL;
  }
  return 0;
}

// INIT state, robot waits for push button to be pressed before moving
void handleInit() {
  Serial.println("HandleInit");
  delay(5000);
  imu.updateIMU();
  heading_offset = imu.getHeading(); 
  pitch_offset = imu.getPitch(); 
  des_left_offset = getDesLeftDist(); 
  
  setState(TILE_FORWARD);
}

bool shouldAdjustRight() {
  int left_tof_value = left_tof.getDistance();
  return (left_tof_value < des_left_offset && abs(left_tof_value-des_left_offset) > LEFT_DIST_ADJUST_TOL);
}

bool shouldAdjustLeft() {
  int left_tof_value = left_tof.getDistance();
  return (left_tof_value > des_left_offset && abs(left_tof_value-des_left_offset) > LEFT_DIST_ADJUST_TOL);
}

void handleTileForward() {
  Serial.println("HandleTileForward");
  
  if ((imu.getPitch() - pitch_offset) < PITCH_DOWNWARDS_VALUE) {
    setState(PIT_FORWARD);
    return;
  }

  std::map<std::pair<int, int>, char>::iterator it = course.find(path[current_tile]);
  if (it != course.end()) {
    char landmark = it->second;

    if (landmark == 'T') {
      std::map<std::pair<int, int>, robot_orientation>::iterator it_turn = turn_orientation.find(path[current_tile]);
      if(it_turn != turn_orientation.end() && it_turn->second == current_orientation) {
          setState(TURN_RIGHT);
          return;
      }
    } else if (landmark == 'E') {
      setState(STOP);
      return;
    }
  }
  

  if(shouldAdjustRight()) {
    setState(RIGHT_ADJUST);
    return;
  }

  if(shouldAdjustLeft()) {
    setState(LEFT_ADJUST);
    return;
  }

  left_motor_power = TILE_MOTOR_VALUE;
  right_motor_power = TILE_MOTOR_VALUE;
  left_motor.backward(left_motor_power);
  right_motor.backward(right_motor_power); 
}

void handlePitForward() {
  Serial.println("handlePitForward");
  Serial.println(" ");
  Serial.print("Pitch: ");
  Serial.print(imu.getPitch());
  Serial.println(" ");
  
  if((imu.getPitch() + pitch_offset) < pitch_offset - 40) {
    left_motor_power = PIT_MOTOR_LOW;
    right_motor_power = PIT_MOTOR_LOW;
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);
    return;
  }

  if(imu.getPitch() > pitch_offset - 5 && imu.getPitch() < pitch_offset + 5) {
    left_motor_power = left_motor_power >= PIT_MOTOR_MED ? left_motor_power : left_motor_power + PIT_INCREMENT;
    right_motor_power = right_motor_power >= PIT_MOTOR_MED ? right_motor_power : right_motor_power + PIT_INCREMENT;
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);
    return;
  }

  if((imu.getPitch() + pitch_offset) > pitch_offset + 40) {
    
    left_motor_power = left_motor_power >= PIT_MOTOR_HIGH ? left_motor_power : left_motor_power + PIT_INCREMENT;
    right_motor_power = right_motor_power >=  PIT_MOTOR_HIGH ? right_motor_power : right_motor_power + PIT_INCREMENT;
    
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);
      
    // exit pit state
    while(!(imu.getPitch() > pitch_offset - 10 && imu.getPitch() < pitch_offset + 10)) {
      imu.updateIMU();   
    }
      
    setState(TILE_FORWARD);
    
  }
}

void handleTurnRight() {
  Serial.println("handleTurnRight");
  
  // slow down 
  left_motor_power = SLOW_MOTOR_VALUE;
  right_motor_power = SLOW_MOTOR_VALUE;
  left_motor.backward(left_motor_power);
  right_motor.backward(right_motor_power); 

  // is our front_tof value in the middle of the tile ? if not return; 
  std::pair<int, int> coord = coords[path[current_tile].first][path[current_tile].second];
  double front_dist_tolerance = 120;
    switch (current_orientation) {
    case LEFT:
      if(robot_position.second > coord.second + front_dist_tolerance) return; 
    case RIGHT:
      if(robot_position.second < coord.second - front_dist_tolerance) return;
    case TOP:
      if(robot_position.first < coord.first - front_dist_tolerance) return;
    case BOTTOM:
      if(robot_position.first > coord.first + front_dist_tolerance) return;
  }  
  
  // turn !! 
  double curr_heading = imu.getHeading();
  double target_heading = curr_heading - 90 < 0 ? curr_heading - 90 + 360 : curr_heading - 90; 

  left_motor.stop();
  right_motor.stop();
  
  while (abs(imu.getHeading() - target_heading) > 10) {
    imu.updateIMU();
    left_motor_power = TURN_MOTOR_VALUE;
    right_motor_power = TURN_MOTOR_VALUE;
    left_motor.backward(left_motor_power);
    right_motor.forward(right_motor_power);
    Serial.println(abs(imu.getHeading() - target_heading));
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

  des_left_offset = getDesLeftDist();

  heading_offset =  heading_offset - 90 < 0 ? heading_offset - 90 + 360 : heading_offset - 90; 
  setState(TILE_FORWARD);
}

void handleLeftAdjust() {
    
    if (abs(imu.getPitch() - pitch_offset) > PITCH_UPWARDS_VALUE) {
      setState(PIT_FORWARD);
      return;
    }
    std::map<std::pair<int, int>, char>::iterator it = course.find(path[current_tile]);
    if (it != course.end()) {
      char landmark = it->second;
  
      if (landmark == 'T') {
        std::map<std::pair<int, int>, robot_orientation>::iterator it_turn = turn_orientation.find(path[current_tile]);
        if(it_turn != turn_orientation.end() && it_turn->second == current_orientation) {
            setState(TURN_RIGHT);
            return;
        }
      } else if (landmark == 'E') {
        setState(STOP);
        return;
      }
    }
    

  
    Serial.println("handleLeftAdjust");
    right_motor_power = ADJUST_VALUE + TILE_ADJUST_VALUE;
    left_motor_power = TILE_ADJUST_VALUE;
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);

    if(abs(imu.getHeading() - heading_offset) < 10 || abs(left_tof.getDistance()-des_left_offset) < LEFT_DIST_TOL) {
      setState(TILE_FORWARD);
    }
}

void handleRightAdjust() {
    
    if (abs(imu.getPitch() - pitch_offset) > PITCH_UPWARDS_VALUE) {
      setState(PIT_FORWARD);
      return;
    }

    std::map<std::pair<int, int>, char>::iterator it = course.find(path[current_tile]);
    if (it != course.end()) {
      char landmark = it->second;
  
      if (landmark == 'T') {
        std::map<std::pair<int, int>, robot_orientation>::iterator it_turn = turn_orientation.find(path[current_tile]);
        if(it_turn != turn_orientation.end() && it_turn->second == current_orientation) {
            setState(TURN_RIGHT);
            return;
        }
      } else if (landmark == 'E') {
        setState(STOP);
        return;
      }
    }
  

    Serial.println("handleRightAdjust");
    left_motor_power = ADJUST_VALUE + TILE_ADJUST_VALUE;
    right_motor_power = TILE_ADJUST_VALUE;
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);

    if(abs(imu.getHeading() - heading_offset) < 10 || abs(left_tof.getDistance()-des_left_offset) < LEFT_DIST_TOL) {
      setState(TILE_FORWARD);
    }
}

void handleTurnAdjust() {
  int heading_err = imu.getHeading() - heading_offset; 


  // adjust left
  if(heading_err > 0) {
      while(abs(left_tof.getDistance() - des_left_offset) > 30 || abs(imu.getHeading() - heading_offset) < 10) {
        left_motor_power = TURN_MOTOR_VALUE;
        right_motor_power = TURN_MOTOR_VALUE;
        left_motor.forward(left_motor_power);
        right_motor.backward(right_motor_power);
      }
  } else {
      while(abs(left_tof.getDistance() - des_left_offset) > 30 ||  abs(imu.getHeading() - heading_offset) < 10 ) {
        left_motor_power = TURN_MOTOR_VALUE;
        right_motor_power = TURN_MOTOR_VALUE;
        left_motor.backward(left_motor_power);
        right_motor.forward(right_motor_power);
      } 
  }

  setState(TILE_FORWARD); 
  return;

    
}

void handleStop() {
  Serial.println("handleStop");


  // slow down 
 
  left_motor_power = SLOW_MOTOR_VALUE;
  right_motor_power = SLOW_MOTOR_VALUE;
  left_motor.backward(left_motor_power);
  right_motor.backward(right_motor_power); 
/*
  // is our front_tof value in the middle of the tile ? if not return; 
  std::pair<int, int> coord = coords[path[current_tile].first][path[current_tile].second];
  double front_dist_tolerance = 30;
    switch (current_orientation) {
    case LEFT:
      if(robot_position.second > coord.second + front_dist_tolerance) return; 
    case RIGHT:
      if(robot_position.second < coord.second - front_dist_tolerance) return;
    case TOP:
      if(robot_position.first < coord.first - front_dist_tolerance) return;
    case BOTTOM:
      if(robot_position.first > coord.first + front_dist_tolerance) return;
  }  */
  
  left_motor.stop();
  right_motor.stop();
  Serial.println("STOP!!!");
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
  delay(10);
  
  // all reset   
  digitalWrite(left_tof.shutdownPin, LOW);
  digitalWrite(front_tof.shutdownPin, LOW);
  delay(10);

  // all unreset   
  digitalWrite(left_tof.shutdownPin, HIGH);
  digitalWrite(front_tof.shutdownPin, HIGH);
  
  delay(10);

  // activating leftTOF and resetting other two
  digitalWrite(left_tof.shutdownPin, HIGH);
  digitalWrite(front_tof.shutdownPin, LOW);
  left_tof.init();

  delay(10);
  Serial.println(F("Set up Left TOF"));

  digitalWrite(front_tof.shutdownPin, HIGH);
  front_tof.init();
  Serial.println(F("Set up front TOF"));

  delay(10);
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

  // hardcode first position 
  robot_position.first = 1350;
  robot_position.second = 1650;

  Serial.println(F("Set up all sensors"));
}

void loop() {

  imu.updateIMU();
  Serial.print("Current Tile ");
  Serial.print(current_tile);

  Serial.println(" ");

  Serial.print("Position: "); 
  Serial.print(robot_position.first); 
  Serial.print("   "); 
  Serial.print(robot_position.second); 
  Serial.println(" "); 
  
  Serial.print("Expected left dist: ");
  Serial.print(des_left_offset);
  Serial.println(" ");

  Serial.print("IMU Heading: "); 
  Serial.print(imu.getHeading()); 
  Serial.println(" ");

  Serial.print("IMU heading offset: "); 
  Serial.print(heading_offset); 
  Serial.println(" ");

  Serial.print("IMU pitch offset: "); 
  Serial.print(pitch_offset); 
  Serial.println(" ");

  
  // calculate position and localize (match with map)
  calculatePosition(current_orientation, robot_position);
  int next_tile = current_tile + 1;

  if (next_tile >= path.size())  {
    setState(STOP);
  } else {
    // update current tile to next tile if position is in the bounds, update robot path
    updateCurrentTile(robot_position, next_tile, current_tile);
  }

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
    case TURN_ADJUST: 
     handleTurnAdjust(); 
     break;
    case STOP:
     handleStop();
     break;
  }

  delay(50); 
}
