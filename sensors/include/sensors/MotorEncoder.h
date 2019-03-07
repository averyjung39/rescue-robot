#ifndef MOTOR_ENCODER
#define MOTOR_ENCODER

#include <ros/duration.h>

#include "sensors/GPIOClass.h"

// Class for getting values from a motor encoder
class MotorEncoder {
public:
    MotorEncoder(const char *pin_a, const char *pin_b, void (*isr_function)(void), volatile int *global_rising_edge_count);
    ~MotorEncoder();
    float positionInDegrees() const;
    bool updateCount();
private:
    static const int COUNTS_PER_REV = 64*30;

    GPIOClass *_gpio_a;
    GPIOClass *_gpio_b;
    int _count;
    volatile int *_global_rising_edge_count;
};

#endif  // MOTOR_ENCODER