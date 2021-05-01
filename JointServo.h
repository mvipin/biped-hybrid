#ifndef JOINT_SERVO_H
#define JOINT_SERVO_H

#define USMIN 500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2470 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define DEGREE_TO_PULSE(deg) (USMIN + (float(deg)/180)*(USMAX-USMIN))

class JointServo {
    int id;
    int uinit;
    float dmin;
    float dmax;
    float slope;
    float offset;

    public:
    JointServo(float _slope, float _dmin, float _dmax) :
        dmin(_dmin), dmax(_dmax), slope(_slope) {}

    void Move(float deg) {
        //Serial << "sid = " << id << " usec = " << (deg-offset)*slope + uinit << "\n";
        pwm.writeMicroseconds(id, (deg-offset)*slope + uinit);
    }

    void SetOffset(float _offset) {
        offset = _offset;
    }

    void SetId(int _id) {
        id = _id;
    }

    void SetUinit(int _uinit) {
        uinit = _uinit;
    }
};

#endif //JOINT_SERVO_H
