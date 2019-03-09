#ifndef MOTOR_ENCODER
#define MOTOR_ENCODER

#include <ros/duration.h>

// Class for getting values from a motor encoder
class MotorEncoder {
public:
    /**
     * @param pin_a, pin_b: pin numbers for motor encoder at A and B
     * @param isr_function: pointer to ISR to run when a rising edge of A is detected
     * @param global_rising_edge_count: pointer to integer counter for encoder values
     */
    MotorEncoder(const int &pin_a,
        const int &pin_b,
        void (*isr_function)(void),
        volatile int *global_rising_edge_count);

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

    int _count;
    int _pin_a;
    int _pin_b;
    volatile int *_global_rising_edge_count;
};

#endif  // MOTOR_ENCODER