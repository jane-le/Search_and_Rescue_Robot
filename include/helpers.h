#ifndef HELPERS_H
#define HELPERS_H

#include <queue>

double TOLERANCE = 10; 

class LeftTofSensor {
    public: 
        double value_; 
        TOF tof_sensor_;
        std::queue<double> buffer_;

        LeftTofSensor(TOF tofSensor);

        void addValue();

        int getValue();

        void clearValues();

        bool shouldAdjustRight();

        bool shouldAdjustLeft();
}
#endif // HELPERS_H