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
Adafruit_TCS34725 color_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
const int SAND_MOTOR_VALUE = 255; 
const int GRAVEL_MOTOR_VALUE = 255; 
const int PIT_MOTOR_VALUE = 255; 
const int TILE_MOTOR_VALUE = 255; 
const int TURN_MOTOR_VALUE = 255; 


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
void drive(float speed, float multiplier = 1.0) {
  speed *= multiplier;
  motor_setSpeed(motor_left, speed);
  motor_setSpeed(motor_right, speed);
}

void motorControl(bool turn_on, bool forward, bool motor1) {
    if (turn_on && forward) {
        // motor forward
        if (motor1) {
            digitalWrite(IO.IN1, HIGH);
            digitalWrite(IO.IN2, LOW);    
        } else {
            digitalWrite(IO.IN3, HIGH);
            digitalWrite(IO.IN4, LOW);
        }
    } else if (turn_on && !forward) {
        // motor reverse
        if (motor1) {
            digitalWrite(IO.IN1, LOW);
            digitalWrite(IO.IN2, HIGH);    
        } else {
            digitalWrite(IO.IN3, LOW);
            digitalWrite(IO.IN4, HIGH);
        }
    } else {
        // motor stop
        if (motor1) {
            digitalWrite(IO.IN1, LOW);
            digitalWrite(IO.IN2, LOW);    
        } else {
            digitalWrite(IO.IN3, LOW);
            digitalWrite(IO.IN4, LOW);
        }
    }
}

void setState(robot_state_t new_state) {
  robot_state = new_state;
}

void calculatePosition(robot_orientation current_orientation, std::pair<double, double>& position) {
  // obtain TOF data, front_tof, back_tof, left_tof

  char which_tof = std::get<2>(coords[path[current_tile][0]][path[current_tile][1]]); 

  switch(current_orientation) {
    case LEFT:
        position.first = left_tof;
        position.second = which_tof == 'B' ? WIDTH - front_tof : front_tof;
        break;
        
    case RIGHT:
        position.first = WIDTH - left_tof; 
        position.second = which_tof == 'B' ? back_tof : WIDTH - front_tof;
        break;
        
    case TOP:
        position.second = left_tof;
        position.first = which_tof == 'B' ? back_tof : WIDTH - front_tof;
        break; 

    case BOTTOM:
        position.second = WIDTH - left_tof;
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
        && position_y <= next_center_y + TILE_WIDTH/2.0 && position_y > next_center_y - TILE_WIDTH/2.0) {
            current_tile = next_tile; 
    } 
    /* else {
        // left adjust or right adjust
        double error_x = position_x - next_center_x;
        double error_y = position_y - next_center_y;
    } */
}

// INIT state, robot waits for push button to be pressed before moving
void handleInit() {
  while(digitalRead(button_pin) == LOW) {}
  setState(TILE_FORWARD);
}

void handleTileForward() {
    motorControl(true, true, true); // motor 1 forward
    motorControl(true, true, false); // motor 2 forward
    analogWrite(enA, TILE_MOTOR_VALUE);
	analogWrite(enB, TILE_MOTOR_VALUE);
    
    
    uint16_t r, g, b, c, colorTemp, lux;

    color_sensor.getRawData(&r, &g, &b, &c);
    // colorTemp = color_sensor.calculateColorTemperature(r, g, b);
    colorTemp = color_sensor.calculateColorTemperature_dn40(r, g, b, c);
    lux = color_sensor.calculateLux(r, g, b);
    // check for trap
    // if color_sensor == black
        // setState(SAND_FORWARD);
        // break;
    // else if color_sensor >= x && color_sensor <= y
        // setState(GRAVEL_FORWARD);
        // break;
    // else if color_sensor == yellow
        // setState(TILE_FORWARD);
        // break;
    // else
        // invalid color, assume it is gravel
        // setState(GRAVEL_FORWARD)
        // break;
    
    // check for turn or stop 
    std::map<std::pair<int, int>, char>::iterator it = course.find(path[current_tile]); 
    if(it != course.end()) {
        char value = it->second;
        
        if(value == 'T') {
            setState(TURN_RIGHT);
        } else if (value == 'E') {
            setState(STOP); 
        }
    }
}

void handleSandForward() {
    // EXIT STATES: TRAPS & TILE_FORWARD
    motorControl(true, true, true);
    motorControl(true, true, false);
    analogWrite(IO.ENA, SAND_MOTOR_VALUE);
	analogWrite(IO.ENB, SAND_MOTOR_VALUE);
}

void handleGravelForward() {
    // EXIT STATES: TRAPS & TILE_FORWARD
    motorControl(true, true, true);
    motorControl(true, true, false);
    analogWrite(IO.ENA, GRAVEL_MOTOR_VALUE);
	analogWrite(IO.ENB, GRAVEL_MOTOR_VALUE);
}
void handlePitForward() {
    // EXIT STATES: TRAPS & TILE_FORWARD
    motorControl(true, true, true);
    motorControl(true, true, false);
    analogWrite(IO.ENA, PIT_MOTOR_VALUE);
	analogWrite(IO.ENB, PIT_MOTOR_VALUE);
    
}

void handleTurnRight() {

    //EXIT STATE: TILE FORWARD

    // assumes motorA is left, motorB is right
    // motorA forward
    digitalWrite(IO.IN1, HIGH);
    digitalWrite(IO.IN2, LOW);
    
    //motorB forward  
    digitalWrite(IO.IN3, LOW);
    digitalWrite(IO.IN4, HIGH);

    analogWrite(IO.ENA, TURN_MOTOR_VALUE);
	analogWrite(IO.ENB, TURN_MOTOR_VALUE);

    // while(angle < 90 degrees) {}

    // turn motors off 
    motorControl(false, true, true);
    motorControl(false, true, false);

    setState(TILE_FORWARD);
}

void handleLeftAdjust() {}
void handleRightAdjust() {}

void handleStop() {
    motorControl(false, true, true);
    motorControl(false, true, false);
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