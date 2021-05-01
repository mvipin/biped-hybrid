#ifndef UTILS_H
#define UTILS_H

#define DEG_TO_RAD(d) (((d)/180.0)*M_PI)
#define RAD_TO_DEG(r) (((r)/M_PI)*180.0)

#define NUM_DATA_POINTS 25 // For offline trajectory computation

// Pin assignment for the left leg servos
#define LEFT_SERVO_PIN_S11 5
#define LEFT_SERVO_PIN_S21 6
#define LEFT_SERVO_PIN_S41 7
#define LEFT_SERVO_PIN_S22 8
#define LEFT_SERVO_PIN_S42 9

// Pin assignment for the right leg servos
#define RIGHT_SERVO_PIN_S11 0
#define RIGHT_SERVO_PIN_S21 1
#define RIGHT_SERVO_PIN_S41 2
#define RIGHT_SERVO_PIN_S22 3
#define RIGHT_SERVO_PIN_S42 4

// Calibrated params for the left leg servos
#define LEFT_SERVO_INIT_POS_S11 1470
#define LEFT_SERVO_INIT_POS_S21 1450
#define LEFT_SERVO_INIT_POS_S41 1500
#define LEFT_SERVO_INIT_POS_S22 1520
#define LEFT_SERVO_INIT_POS_S42 1460

// Calibrated params for the right leg servos
#define RIGHT_SERVO_INIT_POS_S11 1470
#define RIGHT_SERVO_INIT_POS_S21 1450
#define RIGHT_SERVO_INIT_POS_S41 1500
#define RIGHT_SERVO_INIT_POS_S22 1520
#define RIGHT_SERVO_INIT_POS_S42 1460

// Gait Parameters
#define STEPS_DURATION 2000
#define X_SWING_REST 10.0 //13.42
#define X_SWING_AMP 96.28
#define Y_MEAN_LEFT 0.5
#define Y_SIGMA 0.14
#define Y_MEAN_RIGHT 1.5
#define Y_AMP 18.0
#define Y_OFFSET 200 //230.0
#define Z_AMP1 70.0
#define Z_AMP2 35.0
#define Z_PHASE_OFFSET 4.7

// Physical Parameters
#define LINK_L1_SIZE 35.087
#define LINK_L2_SIZE 110.0
#define LINK_L3_SIZE 110.0
#define LINK_L4_SIZE 48.132
#define LINK_L5_SIZE 126.669
#define LINK_L6_SIZE 48.795
#define LINK_PHI DEG_TO_RAD(34.55)
#define LINK_GAMMA DEG_TO_RAD(135.8)
#define LINK_A_SIZE 0.0

// Change these to #define for enabling
#undef OFFLINE_TRAJECTORY_SUPPORT
#undef UNIT_TEST_SUPPORT

extern Adafruit_PWMServoDriver pwm;
#endif // UTILS_H
