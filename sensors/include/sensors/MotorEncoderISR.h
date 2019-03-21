#include <ros/ros.h>
namespace MotorEncoderISR {
    volatile int rising_edges_1 = 0;  // Rising edge count for encoder 1
    volatile int rising_edges_2 = 0;  // Rising edge count for encoder 2

    void ISR1 () {
        ++rising_edges_1;
    }

    void ISR2 () {
        ++rising_edges_2;
    }
}
