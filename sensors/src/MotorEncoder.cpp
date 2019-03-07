#include <ros/ros.h>

#include "sensors/MotorEncoder.h"
#include "sensors/GPIOClass.h"
#include "controls/wiringPi.h"

MotorEncoder::MotorEncoder(const char *pin_a, const char *pin_b, void (*isr_function)(void), volatile int *global_rising_edge_count) {
    _gpio_a = new GPIOClass(pin_a);
    _gpio_a->export_gpio();
    _gpio_a->setdir_gpio("in");
    
    _gpio_b = new GPIOClass(pin_b);
    _gpio_b->export_gpio();
    _gpio_b->setdir_gpio("in");
    
    _count = 0;
    _global_rising_edge_count = global_rising_edge_count;

    // set up ISR for reading encoder
    // http://www.science.smith.edu/dftwiki/index.php/Tutorial:_Interrupt-Driven_Event-Counter_on_the_Raspberry_Pi
    if (wiringPiSetup() < 0) {
        ROS_ERROR("Unable to set up wiringPi");
    }
    if (wiringPiISR(atoi(pin_a), INT_EDGE_RISING, isr_function) < 0) {
        ROS_ERROR("Unable to set up ISR");
    }
}

MotorEncoder::~MotorEncoder() {
    if (_gpio_a) {
        delete _gpio_a;
        _gpio_a = NULL;
    }
    if (_gpio_b) {
        delete _gpio_b;
        _gpio_b = NULL;
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
        bool b_val;
        if (!_gpio_b->read_gpio(b_val)) {
            ROS_ERROR("Unable to read B pin on encoder.");
            return false;
        }

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