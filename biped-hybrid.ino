#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Geometry.h>
#include "Utils.h"
#include "Link.h"
#include "KinematicChain.h"
#include "JointServo.h"
#include "Gait.h"
#include "ParallelChain.h"

// Change it to #define for enabling unit test support
#undef UNIT_TEST_SUPPORT

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define OSC_FREQ 27000000
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define USMIN 500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2470 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define DEGREE_TO_PULSE(deg) (USMIN + (float(deg)/180)*(USMAX-USMIN))

#define TEST_LEFT_RIGHT() do { \
    unit_test_1(-30, 30, -190, -190, 0, 0); \
} while (0)

#define TEST_UP_DOWN() do { \
    unit_test_1(0, 0, -220, -120, 0, 0); \
} while (0)

#define TEST_FRONT_BACK() do { \
    unit_test_1(0, 0, -190, -190, -50, 50); \
} while (0)

#ifdef UNIT_TEST_SUPPORT
struct _trajectory {
    float t11;
    float t21;
    float t41;
    float t22;
    float t42;
} trajectory[NUM_DATA_POINTS];
#endif // UNIT_TEST_SUPPORT

const int T = 2000; // in ms

unsigned long init_ts;

// Kinematic chains for upper and end leg

#ifdef UNIT_TEST_SUPPORT
void unit_test_1(int xmin, int xmax, int ymin, int ymax, int zmin, int zmax) {
    Point PE;
    int index;
    int xmid = (xmin + xmax) >> 1;
    int xamp = (xmax - xmin) >> 1;
    int ymid = (ymin + ymax) >> 1;
    int yamp = (ymax - ymin) >> 1;
    int zmid = (zmin + zmax) >> 1;
    int zamp = (zmax - zmin) >> 1;

    // Generate the trajectory
    int t = (millis() - init_ts) % T;
    float tmp = sin(2*M_PI*t/T);
    PE.X() = xmid + xamp * tmp;
    PE.Y() = ymid + yamp * tmp;
    PE.Z() = zmid + zamp * tmp;

    // Compute the joint variables
    active.InverseKinematics(PE);
    s11.Move(RAD_TO_DEG(L1_0.theta));
    s21.Move(RAD_TO_DEG(L2.theta));
    s41.Move(RAD_TO_DEG(L4.theta));
    s22.Move(RAD_TO_DEG(L2.theta));
    s42.Move(RAD_TO_DEG(L4.theta));
}

void unit_test_2(int xmin, int xmax, int ymin, int ymax, int zmin, int zmax, bool compute) {
    Point PE;
    int index;
    int xmid = (xmin + xmax) >> 1;
    int xamp = (xmax - xmin) >> 1;
    int ymid = (ymin + ymax) >> 1;
    int yamp = (ymax - ymin) >> 1;
    int zmid = (zmin + zmax) >> 1;
    int zamp = (zmax - zmin) >> 1;
    int interval = T/NUM_DATA_POINTS;

    // Precompute or execute
    if (compute) {
        for (int t=0; t<T; t+=interval) {
            // Generate the trajectory
            float tmp = sin(2*M_PI*t/T);
            PE.X() = xmid + xamp * tmp;
            PE.Y() = ymid + yamp * tmp;
            PE.Z() = zmid + zamp * tmp;
    
            // Precompute the joint variables
            index = t/interval;
            active.InverseKinematics(PE);
            trajectory[index].t11 = RAD_TO_DEG(L1_0.theta);
            trajectory[index].t21 = RAD_TO_DEG(L2.theta);
            trajectory[index].t41 = RAD_TO_DEG(L4.theta);
            trajectory[index].t22 = RAD_TO_DEG(L2.theta);
            trajectory[index].t42 = RAD_TO_DEG(L4.theta);
        }
    } else {
        index = ((millis() - init_ts) % T)/interval;
        s11.Move(trajectory[index].t11);
        s21.Move(trajectory[index].t21);
        s41.Move(trajectory[index].t41);
        s22.Move(trajectory[index].t22);
        s42.Move(trajectory[index].t42);      
    }
}
#else
Gait gait;
#endif // UNIT_TEST_SUPPORT

void setup() {
    Serial.begin(115200);

    // Setup the servo controller
    pwm.begin();
    pwm.setOscillatorFrequency(OSC_FREQ);  // The int.osc. is closer to 27MHz  
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

#ifdef UNIT_TEST_SUPPORT
    //unit_test_2(-30, 30, -190, -120, -10, 10, true);
#else
#ifdef OFFLINE_TRAJECTORY_SUPPORT
    gait.Compute();
#endif // OFFLINE_TRAJECTORY_SUPPORT
#endif // UNIT_TEST_SUPPORT

    init_ts = millis();
}

void loop() {
#ifdef UNIT_TEST_SUPPORT
    TEST_UP_DOWN();
    //unit_test_2(-30, 30, -190, -120, -10, 10, false);
#else
#ifdef OFFLINE_TRAJECTORY_SUPPORT
    gait.Execute(init_ts);
#endif // OFFLINE_TRAJECTORY_SUPPORT
    gait.ComputeExecute(init_ts);
#endif // UNIT_TEST_SUPPORT
}
