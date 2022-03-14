#include <helpers.h>


class LeftTofSensor {
    public:
        LeftTofSensor() {
            TOF tof_sensor;
            tofSensor = tof_sensor;
        }

        void addValue() {
            // get left sensor reading 
            double sensor_value = tofSensor.getValue(); // subject to change cus azum
            if(sensor_value == -1) return;
    
            // add sensor reading to queue
            while (buffer.size() >= 10) {
                queue.pop();
            }
            buffer.push(sensor_value);
        }

        int getValue() {
            return queue.back();
        }

        void clearValues() {
            std::queue<int> empty;
            std::swap(queue, empty);
        }

        bool shouldAdjustRight() {
            if (queue.size() < 4) {
                return false;
            }
            return (queue.back() - queue.front() > TOLERANCE);
        }

        bool shouldAdjustLeft() {
            if(queue.size() < 4) {
                return false;
            }
            return (queue.front() - queue.back() > TOLERANCE); 
        }
}