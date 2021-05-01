#ifndef JOINT_SERVO_H
#define JOINT_SERVO_H
#include <Adafruit_PWMServoDriver.h>

#define USMIN 500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2470 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define DEGREE_TO_PULSE(deg) (USMIN + (float(deg)/180)*(USMAX-USMIN))

enum {
    SERVO_T11,
    SERVO_T21,
    SERVO_T41,
    SERVO_T22,
    SERVO_T42,
    SERVO_9,
    SERVO_NUM_MAX,
};

class JointServo {
    int id;
    int uinit;
    float dmin;
    float dmax;
    float slope;
    float offset;
    Adafruit_PWMServoDriver *pwm;
    
    public:
    JointServo(int _id, int _uinit, float _slope, float _dmin, float _dmax) {
        id = _id;
        uinit = _uinit;
        dmin = _dmin;
        dmax = _dmax;
        slope = _slope;
        offset = 0;
//        pwm = &_pwm;
    }

    void Move(float deg) {
        //Serial << "sid = " << id << " usec = " << (deg-offset)*slope + uinit << "\n";
        pwm->writeMicroseconds(id, (deg-offset)*slope + uinit);
    }

    void SetOffset(float _offset) {
        offset = _offset;
    }
};

#endif //JOINT_SERVO_H
