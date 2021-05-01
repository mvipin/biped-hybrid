#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Geometry.h>
#include "Link.h"
#include "KinematicChain.h"
#include "Config.h"
#include "JointServo.h"
#include "ParallelChain.h"
#include "Gait.h"
#include "UnitTest.h"

#define OSC_FREQ 27000000
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long init_ts; // used to establish the time origin

#ifdef UNIT_TEST_SUPPORT
UnitTest test(UP_DOWN);
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
    test.Init();
#ifdef OFFLINE_TRAJECTORY_SUPPORT
    test.Compute();
#endif // OFFLINE_TRAJECTORY_SUPPORT
#else // UNIT_TEST_SUPPORT
    gait.Init();
#ifdef OFFLINE_TRAJECTORY_SUPPORT
    gait.Compute();
#endif // OFFLINE_TRAJECTORY_SUPPORT
#endif // UNIT_TEST_SUPPORT

    init_ts = millis();
}

void loop() {
#ifdef OFFLINE_TRAJECTORY_SUPPORT
#ifdef UNIT_TEST_SUPPORT
    test.Execute(init_ts);
#else
    gait.Execute(init_ts);
#endif // UNIT_TEST_SUPPORT
#else // OFFLINE_TRAJECTORY_SUPPORT
#ifdef UNIT_TEST_SUPPORT
    test.ComputeExecute(init_ts);
#else
    gait.ComputeExecute(init_ts);
#endif // UNIT_TEST_SUPPORT
#endif // OFFLINE_TRAJECTORY_SUPPORT
}
