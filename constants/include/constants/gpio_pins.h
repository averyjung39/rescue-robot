#ifndef GPIO_PINS
#define GPIO_PINS

#define MOTOR_RIGHT_1 20
#define MOTOR_RIGHT_2 21
#define MOTOR_ENABLE_1 18

#define MOTOR_LEFT_1 22
#define MOTOR_LEFT_2 23
#define MOTOR_ENABLE_2 19

#define ULTRASONIC_R_TRIG 31
#define ULTRASONIC_R_ECHO 28
#define ULTRASONIC_L_TRIG 29
#define ULTRASONIC_L_ECHO 32
#define ULTRASONIC_B_TRIG 30
#define ULTRASONIC_B_ECHO 33

// Photodiode on robot left = 1 robot right = 5
#define PHOTODIODE_1 4
#define PHOTODIODE_2 5
#define PHOTODIODE_3 6
#define PHOTODIODE_4 7
#define PHOTODIODE_5 8

#define HALL_EFFECT_1_H 8
#define HALL_EFFECT_1_L 9
#define HALL_EFFECT_2_H 10
#define HALL_EFFECT_2_L 11

#define MOTOR_R_ENCODER_A_PIN 16
#define MOTOR_R_ENCODER_B_PIN 17
#define MOTOR_L_ENCODER_A_PIN 24
#define MOTOR_L_ENCODER_B_PIN 25

#define TOF_XSHUT_1 34
#define TOF_XSHUT_2 35
#define TOF_XSHUT_3 37
//#define TOF_XSHUT_4 37
//#define TOF_XSHUT_5 38

#define TOF_ADDR_1 0x30
#define TOF_ADDR_2 0x31
#define TOF_ADDR_3 0x32
#define TOF_ADDR_4 0x33
#define TOF_ADDR_5 0x34

// TODO: define these with actual pin numbers used
#define FOOD_INDICATOR 40
#define PERSON_INDICATOR 41
#define SURVIVORS_INDICATOR 42
#define HOME_INDICATOR 43

#define FAN 44


#endif // GPIO_PINS
