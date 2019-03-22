#ifndef GPIO_PINS
#define GPIO_PINS

#define MOTOR_RIGHT_1 20
#define MOTOR_RIGHT_2 21
#define MOTOR_ENABLE_1 18

#define MOTOR_LEFT_1 22
#define MOTOR_LEFT_2 23
#define MOTOR_ENABLE_2 19

// Photodiode on robot left = 1 robot right = 5
#define PHOTODIODE_1 4
#define PHOTODIODE_2 5
#define PHOTODIODE_3 6
#define PHOTODIODE_4 7
#define PHOTODIODE_5 8

#define HALL_EFFECT_1 9
#define HALL_EFFECT_2 10

#define MOTOR_R_ENCODER_A_PIN 16
#define MOTOR_R_ENCODER_B_PIN 17
#define MOTOR_L_ENCODER_A_PIN 24
#define MOTOR_L_ENCODER_B_PIN 25

#define TOF_BOTTOM_LEFT 34
#define TOF_BOTTOM_RIGHT 35
#define TOF_TOP_FRONT 37
#define TOF_TOP_LEFT 36
#define TOF_TOP_RIGHT 33

// TODO: define these with actual pin numbers used
#define FOOD_INDICATOR 40
#define PERSON_INDICATOR 41
#define SURVIVORS_INDICATOR 42
#define HOME_INDICATOR 43

#define FAN 44

#define ULTRASONIC_LOW_ECHO 31
#define ULTRASONIC_LOW_TRIG 28
#define ULTRASONIC_HIGH_ECHO 32
#define ULTRASONIC_HIGH_TRIG 29

#define BOTTOM_LEFT_ADDR 0x30
#define BOTTOM_RIGHT_ADDR 0x31
#define TOP_FRONT_ADDR 0x32
#define TOP_LEFT_ADDR 0x33
#define TOP_RIGHT_ADDR 0x34

#endif // GPIO_PINS
