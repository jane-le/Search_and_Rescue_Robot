// import standard libraries
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <iostream>
#include <map>
#include <utility>
#include <vector>
#include <tuple>

#define LMotorForwardPin
#define LMotorStopPin
#define LMotorPWMPin
#define RMotorForwardPin
#define RMotorStopPin
#define RMotorPWMPin

typedef enum {
  INIT,
  TILE_FORWARD,
  SAND_FORWARD,
  GRAVEL_FORWARD,
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

// IO
struct {
    // Left Motor connections
    const int ENA = 9;
    const int IN1 = 8;
    const int IN2 = 7;
    // Right Motor connections
    const int ENB = 3;
    const int IN3 = 5;
    const int IN4 = 4;
} IO;

robot_state_t robot_state;
int button_state = 0;
const int button_pin = 2;
int current_tile; 
robot_orientation current_orientation; 
std::pair<double, double> robot_position;
const double WIDTH = 1828.8; // 6 feet in mm (dimension of the course)
const double TILE_WIDTH = 300; 
int encoderL_tick = 0;
int encoderR_tick = 0;
int left_motor_power = 0;
int right_motor_power = 0;
double prev_pitch = 0;
Adafruit_TCS34725 color_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
const int SAND_MOTOR_VALUE = 255; 
const int GRAVEL_MOTOR_VALUE = 255; 
const int PIT_MOTOR_LOW = 40; 
const int PIT_MOTOR_HIGH = 255;
const int TILE_MOTOR_VALUE = 255; 
const int TURN_MOTOR_VALUE_LEFT = 255; 
const int TURN_MOTOR_VALUE_RIGHT = 50; // either stop (speed 0) or 50 in the reverse direction 
const int ROBOT_WIDTH = 200; 
const int ROBOT_LENGTH = 200;
const int ROBOT_MOTOR_OFFSET = 5;
const int TRAP_EXIT_TIME = 20000;
const double CENTER_TILE_TOL = 10;
const int TURN_DELAY = 20000;
const int PIT_DELAY_LOW = 30000;
const int PIT_DELAY_MED = 50000;
const int PIT_DELAY_HIGH = 30000;
const double PITCH_UPWARDS_VALUE = 40; 
const double PITCH_DOWNWARDS_VALUE = -40; 
const double ADJUST_VALUE = 40;
LeftTofSensor left_tof = new LeftTofSensor();
ColorSensor color_sensor = new ColorSensor();


// S = start, E = end, T = turn 
std::map<std::pair<int, int>, char> course = {
  {(5, 3), 'S'}, {(5,0), 'T'}, {(0,0), 'T'}, {(0,5), 'T'}, 
  {(4,5), 'T'}, {(4,1), 'T'}, {(1,1), 'T'}, {(1,4), 'T'}, 
  {(3,4), 'T'}, {(3,2), 'T'}, {(2,2), 'T'}, {(2,3), 'E'}
}; 

// F = front tof, B = back tof, x and y in mm, based on 1.8 m
std::vector<std::vector<std::tuple<double,double, char>>> coords = {
  {(150,150,'F'), (450,150,'B'), (750,150,'B'), (1050,150,'F'), (1350,150,'F'), (1650,150,'F')},
  {(150,450,'F'), (450,450,'F'), (750,450,'B'), (1050,450,'F'), (1350,450,'F'), (1650,450,'B')},
  {(150,750,'F'), (450,750,'F'), (750,750,'F'), (1050,750,'F'), (1350,750,'B'), (1650,750,'B')},
  {(150,1050,'B'), (450,1050,'B'), (750,1050,'F'), (1050,1050,'B'), (1350,1050,'F'), (1650,1050,'F')},
  {(150,1350,'B'), (450,1350,'F'), (750,1350,'F'), (1050,1350,'B'), (1350,1350,'B'), (1650,1350,'F')},
  {(150,1650,'F'), (450,1650,'F'), (750,1650,'F'), (1050,1650,'B'), (1350,1650,'B'), (1650,1650,'F')}
};

std::vector<std::pair<double, double>> path = {
  (5, 3), (5, 2), (5, 1), (5, 0), (4, 0), (3, 0), 
  (2, 0), (1, 0), (0, 0), (0, 1), (0, 2), (0, 3), 
  (0, 4), (0, 5), (1, 5), (2, 5), (3, 5), (4, 5), 
  (4, 4), (4, 3), (4, 2), (4, 1), (3, 1), (2, 1), 
  (1, 1), (1, 2), (1, 3), (1, 4), (2, 4), (3, 4), 
  (3, 3), (3, 2), (2, 2), (2, 3)
}


/* ------------ Functions --------------- */

void setState(robot_state_t new_state) {
  robot_state = new_state;
}

void calculatePosition(robot_orientation current_orientation, std::pair<double, double>& position) {
  // obtain TOF data, front_tof, back_tof, left_tof

  char which_tof = std::get<2>(coords[path[current_tile][0]][path[current_tile][1]]); 

  switch(current_orientation) {
    case LEFT:
        position.first = left_tof.getValue();
        position.second = which_tof == 'B' ? WIDTH - front_tof : front_tof;
        break;
        
    case RIGHT:
        position.first = WIDTH - left_tof.getValue(); 
        position.second = which_tof == 'B' ? back_tof : WIDTH - front_tof;
        break;
        
    case TOP:
        position.second = left_tof.getValue();
        position.first = which_tof == 'B' ? back_tof : WIDTH - front_tof;
        break; 

    case BOTTOM:
        position.second = WIDTH - left_tof.getValue();
        position.first = which_tof == 'B' ? WIDTH - back_tof : front_tof; 
        break;
  }
}

void updateCurrentTile(const std::pair<double, double>& position, const int next_tile, int& current_tile) {
    double next_center_x = std::get<0>(coords[path[next_tile][0]][path[next_tile][1]]);
    double next_center_y = std::get<1>(coords[path[next_tile][0]][path[next_tile][1]]); 
    double position_x = position.first;
    double position_y = position.second;

    if (position_x <= next_center_x + TILE_WIDTH/2.0 && position_x > next_center_x - TILE_WIDTH/2.0 
        && position_y <= next_center_y + TILE_WIDTH/2.0 && position_y > next_center_y - TILE_WIDTH/2.0) { // this may need tolerances 
            current_tile = next_tile; 
    } 
    /* else {
        // talk to AZUM
        // left adjust or right adjust
        double error_x = position_x - next_center_x;
        double error_y = position_y - next_center_y;
    } */
}

void driveStraight() { 
    left_motor_power = TILE_MOTOR_VALUE;
    right_motor_power = TILE_MOTOR_VALUE;    

    left_count = encoderL.getTicks(LEFT);
    right_count = encoderR.getTicks(RIGHT);

    long left_diff = left_count - encoderL_tick;
    long right_diff = right_count - encoderR_tick;
    // adjust left & right motor powers to keep counts similar (drive straight)
    // if left rotated more than right, slow down left & speed up right
    if (left_diff > right_diff) {
        left_motor_power -= OFFSET;
        right_motor_power += OFFSET;
    } else if (left_diff < right_diff) {
        left_motor_power += OFFSET;
        right_motor_power -= OFFSET;
    }
    left_motor.forward(left_motor_power);
    right_motor.forward(right_motor_power);
}

// INIT state, robot waits for push button to be pressed before moving
void handleInit() {
  while(digitalRead(button_pin) == LOW) {}
  setState(TILE_FORWARD);
}

void handleTileForward() {
    left_tof.addValue(); 

    driveStraight();

    // check for trap
    if (color_sensor.checkSand()) {
        setState(SAND_FORWARD);
        break;
    } else if (color_sensor.checkTile()) {
        setState(TILE_FORWARD);
        break;
    } else if ((imu.getPitch() - prev_pitch) < PITCH_DOWNWARDS_VALUE) {
        setState(PIT_FORWARD);
        break;
    } else {
        setState(GRAVEL_FORWARD);
        break;
    }
    
    // check for turn or stop 
    // only change state when robot is center of tile 
    std::map<std::pair<int, int>, char>::iterator it = course.find(path[current_tile]); 
    if(it != course.end()) {
        char landmark = it->second;
        
        if(landmark == 'T') {
            // only change state when robot is center of tile
            // double center_x =  std::get<0>(coords[path[current_tile][0]][path[current_tile][1]]);
            // double center_y =  std::get<1>(coords[path[current_tile][0]][path[current_tile][1]]);
            // if (robot_position.first > center_x - CENTER_TILE_TOL && robot_position.first < center_x + CENTER_TILE_TOL)
            delay(TURN_DELAY); // if doesnt work use front tof -- do math
            setState(TURN_RIGHT);
     

    if(left_tof.shouldAdjustLeft()) {
        setState(LEFT_ADJUST);
    }

    if(left_tof.shouldAdjustRight()) {
        setState(RIGHT_ADJUST);
    }   } else if (landmark == 'E') {
            delay(TURN_DELAY);
            setState(STOP); 
        }
    }
}

void handleSandForward() {
    // EXIT STATES: TRAPS & TILE_FORWARD
    left_motor.forward(SAND_MOTOR_VALUE);
    right_motor.forward(SAND_MOTOR_VALUE);
    exitTraps(SAND_FORWARD);
    // set left power right power variables 
    left_motor_power = SAND_MOTOR_VALUE; 
    right_motor_power = SAND_MOTOR_VALUE;

    if(color_sensor.isSand()) return; 

    // imu check 
    if(imu.getPitch() - prev_pitch < PITCH_DOWNWARDS_VALUE) {
        setState(PIT_FORWARD);
    }

    if(color_sensor.isTile()) {
        // add a delay? or something to check if robot fully overcame trap
        setState(TILE_FORWARD); 
    } else {
        // add a delay? or something to check if robot fully overcame trap
        setState(GRAVEL_FORWARD);
    }

}

void handleGravelForward() {
    left_motor_power = GRAVEL_MOTOR_VALUE;
    right_motor_power = GRAVEL_MOTOR_VALUE;
    left_motor.forward(left_motor_power);
    right_motor.forward(right_motor_power);

    // 7. in gravel, exit to sand
    // 8. in gravel, exit to pit
    // 9. in gravel, exit to tile
    
    if (color_sensor.checkSand()) {
        setState(SAND_FORWARD);
    } else if (color_sensor.checkTile()) {
        setState(TILE_FORWARD);
    } else if ((imu.getPitch() - prev_pitch) < PITCH_DOWNWARDS_VALUE) {
        setState(PIT_FORWARD);
    }
}

void handlePitForward() {
    left_motor_power = PIT_MOTOR_LOW;
    right_motor_power = PIT_MOTOR_LOW;
    left_motor.forward(left_motor_power);
    right_motor.forward(right_motor_power); 
    delay(PIT_DELAY_LOW);
    
    left_motor_power = PIT_MOTOR_MED;
    right_motor_power = PIT_MOTOR_MED;
    left_motor.forward(left_motor_power);
    right_motor.forward(right_motor_power); 
    delay(PIT_DELAY_MED);

    left_motor_power = PIT_MOTOR_HIGH;
    right_motor_power = PIT_MOTOR_HIGH;
    left_motor.forward(left_motor_power);
    right_motor.forward(right_motor_power); 
    delay(PIT_DELAY_HIGH);

    // exit pit state
    setState(TILE_FORWARD);
        /*
        if(color_sensor.checkTile()) {
            setState(TILE_FORWARD);
        } else if (color_sensor.checkSand()) {
            setState(SAND_FORWARD);
        } else {
            setState(GRAVEL_FORWARD);
        } */
}

void handleTurnRight() {
    //EXIT STATE: TILE FORWARD
    while(imu.getYaw() < 90) {
        left_motor_power = TURN_MOTOR_VALUE_LEFT;
        right_motor_power = TURN_MOTOR_VALUE_RIGHT;
        left_motor.forward(left_motor_power);
        right_motor.backward(right_motor_power);
    }

    // may need to 0 imu or store yaw imu value
    
    // update direction of robot 
    switch(current_orientation) {
        case(LEFT):
            current_orientation = TOP;
            break;
        case(TOP):
            current_orientation = RIGHT;
            break;
        case(RIGHT):
            current_orientation = BOTTOM;
            break;
        case(BOTTOM):
            current_orientation = LEFT;
            break;
    }

    // zero imu heading
    // adjust position with a set value

    setState(TILE_FORWARD);

    left_tof.clearValues();
}

void handleLeftAdjust() {
    right_motor_power += ADJUST_VALUE;
    right_motor.forward(right_motor_power);
    delay(1000);
    setState(TILE_FORWARD);

}

void handleRightAdjust() {
    left_motor_power += ADJUST_VALUE;
    left_motor.forward(left_motor_power);
    delay(1000);
    setState(TILE_FORWARD);
}

void handleStop() {
    left_motor.stop();
    right_motor.stop();
    setState(INIT); 
}

void setup() {
    pinMode(button_pin, INPUT);
    robot_state = INIT;
    current_orientation = BOTTOM; 
    current_tile = 0;
    // calibrate IMU (set to 0)
    // set TOF's to zero
}

void loop() {
    // calculate position and localize (match with map)
    calculatePosition(current_orientation, robot_position);
    int next_tile = current_tile + 1; 

    if(next_tile >= path.size())  {
        setState(STOP); 
    } else {
        // update current tile to next tile if position is in the bounds, update robot path 
        updateCurrentTile(robot_position, next_tile, current_tile); 
    }
    
    // get and update prev motor ticks value 
    encoderL_tick = getTicks(left); // update this later with sensor apis 
    encoderR_tick = getTicks(right); // update this later with sensor apis 

    prev_pitch = imu.getPitch();
    
    switch(robot_state) {
        case INIT:
            handleInit();
            break;
        case TILE_FORWARD:
            handleTileForward();
            break; 
        case SAND_FORWARD: 
            handleSandForward();
            break; 
        case GRAVEL_FORWARD: 
            handleGravelForward(); 
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
}