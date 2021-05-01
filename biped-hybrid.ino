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

#define NUM_DATA_POINTS 10

#define TEST_LEFT_RIGHT() do { \
    unit_test_1(-30, 30, -190, -190, 0, 0); \
} while (0)

#define TEST_UP_DOWN() do { \
    unit_test_1(0, 0, -220, -120, 0, 0); \
} while (0)

#define TEST_FRONT_BACK() do { \
    unit_test_1(0, 0, -190, -190, -50, 50); \
} while (0)

enum {
    LEFT_LEG,
    RIGHT_LEG,
    LEG_NUM_MAX,
};

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
const float phi = DEG_TO_RAD(34.55);
const float gamma = DEG_TO_RAD(135.8);
const float a = 0; //overlapping joint in mm
const float l1 = 35.087; // mm
const float l2 = 110.0; // mm
const float l3 = 110.0; // mm
const float l4 = 48.132; // mm
const float l5 = 126.669; // mm
const float l6 = 48.795; // mm

unsigned long init_ts;

// Define the joints
RevoluteJoint L1_0(a, M_PI_2, 0, M_PI_2); // JV = theta1
RevoluteJoint L1_1(0, phi, l1, 0);
RevoluteJoint L2(0, 0, -l2, 0); // JV = theta2
RevoluteJoint L3(0, 0, -l3, 0); // JV = theta3
RevoluteJoint L4(0, M_PI_2-phi, l4, 0); // JV = theta4
RevoluteJoint L6(0, -gamma, -l6, 0); // JV = theta3

JointServo s11(pwm, 0, 1470, -7.5, -90.0, 30.0); // Hip servo
JointServo s21(pwm, 1, 1450, 7.5, -50.0, 20.0); // Lower servo - 1
JointServo s41(pwm, 2, 1500, 7.5, -30.0, 30.0); // Upper servo - 1
JointServo s22(pwm, 3, 1520, -7.5, -50.0, 20.0); // Lower servo - 2
JointServo s42(pwm, 4, 1460, -7.5, -30.0, 30.0); // Upper servo - 2

// Kinematic chains for upper and end leg
ParallelChain active(phi, gamma, a, l1, l2, l3, l4, l5, l6, L1_0, L1_1, L2, L3, L4, L6);

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
Gait gait(s11, s21, s41, s22, s42, L1_0, L1_1, L2, L3, L4, L6, T);
#endif // UNIT_TEST_SUPPORT

void setup() {
    Serial.begin(115200);

    // Setup the servo controller
    pwm.begin();
    pwm.setOscillatorFrequency(OSC_FREQ);  // The int.osc. is closer to 27MHz  
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    // Add Kinematic chains and servos for active side of the parallel leg 
    active.AddServo(SERVO_T11, s11);
    active.AddServo(SERVO_T41, s41);
    active.AddServo(SERVO_T21, s21);
    active.AddServo(SERVO_T42, s42);
    active.AddServo(SERVO_T22, s22);

    delay(2000);

    active.AddToChain(KCHAIN_UPPER, L1_0);  
    active.AddToChain(KCHAIN_UPPER, L1_1);  
    active.AddToChain(KCHAIN_UPPER, L4);  

    active.AddToChain(KCHAIN_MIDDLE, L1_0);
    active.AddToChain(KCHAIN_MIDDLE, L2);
    active.AddToChain(KCHAIN_MIDDLE, L3);
    L3.Move(active.Theta3()); // TODO: specify theta3 while declaring the link 

    active.AddToChain(KCHAIN_LOWER, L1_0);
    active.AddToChain(KCHAIN_LOWER, L2);
    active.AddToChain(KCHAIN_LOWER, L6);
    L6.Move(active.Theta3()); // TODO: specify theta3 while declaring the link

    s11.SetOffset(RAD_TO_DEG(L1_0.theta));
    s21.SetOffset(RAD_TO_DEG(L2.theta));
    s41.SetOffset(RAD_TO_DEG(L4.theta));
    s22.SetOffset(RAD_TO_DEG(L2.theta));
    s42.SetOffset(RAD_TO_DEG(L4.theta));

    init_ts = millis();
#ifdef UNIT_TEST_SUPPORT
    //unit_test_2(-30, 30, -190, -120, -10, 10, true);
#else
#ifdef OFFLINE_TRAJECTORY_SUPPORT
    gait.Compute(init_ts);
#endif // OFFLINE_TRAJECTORY_SUPPORT
#endif // UNIT_TEST_SUPPORT
}

void loop() {
#ifdef UNIT_TEST_SUPPORT
    TEST_UP_DOWN();
    //unit_test_2(-30, 30, -190, -120, -10, 10, false);
#else
#ifdef OFFLINE_TRAJECTORY_SUPPORT
    gait.Execute();
#endif // OFFLINE_TRAJECTORY_SUPPORT
    gait.ComputeExecute(init_ts);
#endif // UNIT_TEST_SUPPORT
}
