#ifndef HELPERS_H
#define HELPERS_H

#include <queue>

double TOLERANCE = 10; 

class LeftTofSensor {
    public: 
        double value; 
        TOF tofSensor;
        std::queue<double> buffer;

        LeftTofSensor(TOF tofSensor);

        void addValue();

        int getValue();

        void clearValues();

        bool shouldAdjustRight();

        bool shouldAdjustLeft();
}
#endif