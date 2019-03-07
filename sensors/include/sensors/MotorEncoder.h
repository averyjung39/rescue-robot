#ifndef MOTOR_ENCODER
#define MOTOR_ENCODER

#include <ros/duration.h>

#include "sensors/GPIOClass.h"

// Class for getting values from a motor encoder
class MotorEncoder {
public:
    /**
     * @param pin_a, pin_b: pin numbers for motor encoder at A and B
     * @param isr_function: pointer to ISR to run when a rising edge of A is detected
     * @param global_rising_edge_count: pointer to integer counter for encoder values
     */
    MotorEncoder(const char *pin_a,
        const char *pin_b,
        void (*isr_function)(void),
        volatile int *global_rising_edge_count);
    ~MotorEncoder();

    /**
     * @brief Returns motor position in degrees, with respect to wherever it was on startup
     */
    float positionInDegrees() const;

    /**
     * @brief  Updates encoder count for motor
     * @return false if unable to update count
     */
    bool updateCount();
private:
    static const int COUNTS_PER_REV = 64*30;

    GPIOClass *_gpio_a;
    GPIOClass *_gpio_b;
    int _count;
    volatile int *_global_rising_edge_count;
};

#endif  // MOTOR_ENCODER