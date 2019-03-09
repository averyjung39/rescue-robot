#include <ros/ros.h>

#include "external/wiringPi/wiringPi.h"
#include "sensors/MotorEncoder.h"

MotorEncoder::MotorEncoder(const int &pin_a, const int &pin_b, void (*isr_function)(void), volatile int *global_rising_edge_count) {
    _count = 0;
    _global_rising_edge_count = global_rising_edge_count;

    // set up ISR for reading encoder
    // http://www.science.smith.edu/dftwiki/index.php/Tutorial:_Interrupt-Driven_Event-Counter_on_the_Raspberry_Pi
    if (wiringPiSetup() < 0) {
        ROS_ERROR("Unable to set up wiringPi");
        throw std::runtime_error("");
    }
    _pin_a = pin_a;
    _pin_b = pin_b;
    pinMode(_pin_a, INPUT);
    pinMode(_pin_b, INPUT);
    if (wiringPiISR(_pin_a, INT_EDGE_RISING, isr_function) < 0) {
        ROS_ERROR("Unable to set up ISR");
        throw std::runtime_error("");
    }
}

float MotorEncoder::positionInDegrees() const {
    return 360.0 * _count / COUNTS_PER_REV;
}

bool MotorEncoder::updateCount() {
    // Make local copy of volatile variable
    int rising_edges = *_global_rising_edge_count;

    if (rising_edges > 0) {
        // There is new data
        bool b_val = digitalRead(_pin_b);

        // Assume motor has been turning in the same direction
        // for each observed rising edge
        if (b_val) {
            // TODO check if increment or decrement
        } else {

        }
        _count %= COUNTS_PER_REV;
    }

    // Update rising edge count
    *_global_rising_edge_count -= rising_edges;

    return true;
}