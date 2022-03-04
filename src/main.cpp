// import standard libraries
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

typedef enum {
    INIT,
    TILE_FORWARD,
    SAND_FORWARD,
    GRAVEL_FORWARD,
    TURN_RIGHT,
    LEFT_ADJUST,
    RIGHT_ADJUST,
    STOP
} robot_state_t;

robot_state_t robot_state;
int button_state = 0;
const int button_pin = 2;



/* ------------ Functions --------------- */
void drive(float speed, float multiplier = 1.0) {
    speed *= multiplier;
    motor_setSpeed(motor_left, speed);
    motor_setSpeed(motor_right, speed);
}

void setState(robot_state_t new_state) {
    robot_state = new_state;
}

void setup() {
    pinMode(button_pin, INPUT);
    robot_state = INIT;
    // calibrate IMU (set to 0)
}

void loop() {
    // calculate position and localize (match with map)
    
    switch(robot_state) {
        // INIT state, robot waits for push button to be pressed before moving
        case INIT:
            button_state = digitalRead(button_pin);
            if (button_state == LOW) {
                setState(INIT);
            } else {
                setState(TILE_FORWARD);
            }
            break;
        
        // 
        case TILE_FORWARD:
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
            // check for turn


    }
}