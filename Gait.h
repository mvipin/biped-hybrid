#ifndef GAIT_H
#define GAIT_H

#include "Utils.h"
#include "JointServo.h"
#include "Link.h"

#undef OFFLINE_TRAJECTORY_SUPPORT

class Gait {
    const int time_period = 2000;
    const float x_swing_rest = 13.42;
    const float x_swing_amp = 96.28;
    const float y_mean_left = 0.5;
    const float y_sigma = 0.14;
    const float y_mean_right = 1.5;
    const float y_amp = 18;
    const float y_offset = 230;
    const float z_amp1 = 70;
    const float z_amp2 = 35;
    const float z_phase_offset = 4.7;

#ifdef OFFLINE_TRAJECTORY_SUPPORT
    struct {
        float t11;
        float t21;
        float t41;
        float t22;
        float t42;
    } trajectory[LEG_NUM_MAX][NUM_DATA_POINTS];
#endif // OFFLINE_TRAJECTORY_SUPPORT
    JointServo *s11, *s21, *s41, *s22, *s42;
    RevoluteJoint *L1_0, *L1_1, *L2, *L3, *L4, *L6;

    public:
    Gait(JointServo &_s11, JointServo &_s21, JointServo &_s41, JointServo &_s22, JointServo &_s42,
          RevoluteJoint &_L1_0, RevoluteJoint &_L1_1, RevoluteJoint &_L2, RevoluteJoint &_L3,
          RevoluteJoint &_L4, RevoluteJoint &_L6,
          int _time_period) {
        s11 = &_s11;
        s21 = &_s21;
        s41 = &_s41;
        s22 = &_s22;
        s42 = &_s42;
        L1_0 = &_L1_0;
        L1_1 = &_L1_1;
        L2 = &_L2;
        L3 = &_L3;
        L4 = &_L4;
        L6 = &_L6;
    }

#ifdef OFFLINE_TRAJECTORY_SUPPORT
    Compute(int init_ts) {
        Point PE;
        int interval = T/NUM_DATA_POINTS;

        for (int leg=LEFT_LEG; leg<=RIGHT_LEG; leg++) {
            float x_rest = x_swing_rest;
            float y_mean = y_mean_left;
            float phase_offset = 0;

            if (leg == RIGHT_LEG) {
                phase_offset = M_PI;
                x_rest *= -1;
                y_mean = y_mean_right;
            }

            for (int t=0; t<T; t+=interval) {
                // Generate the trajectory
                float tmp = 2*M_PI*t/time_period;
                PE.X() = (x_swing_amp/2)*sin(tmp) + x_rest;
                PE.Y() = (y_amp/(y_sigma*sqrt(2*M_PI))) * exp(-0.5 * (((t/1000)-y_mean)/y_sigma) * (((t/1000)-y_mean)/y_sigma)) - y_offset;
                PE.Z() = z_amp1*sin(tmp+z_phase_offset) + z_amp2*sin(2*(tmp+z_phase_offset));
        
                // Precompute the joint variables
                int index = t/interval;
                active.InverseKinematics(PE);
                trajectory[leg][index].t11 = RAD_TO_DEG(L1_0->theta);
                trajectory[leg][index].t21 = RAD_TO_DEG(L2->theta);
                trajectory[leg][index].t41 = RAD_TO_DEG(L4->theta);
                trajectory[leg][index].t22 = RAD_TO_DEG(L2->theta);
                trajectory[leg][index].t42 = RAD_TO_DEG(L4->theta);
            }
        }
    }

    Execute(void) {        
        int interval = time_period/NUM_DATA_POINTS;
        int index = ((millis() - init_ts) % time_period)/interval;
        s11->Move(trajectory[LEFT_LEG][index].t11);
        s21->Move(trajectory[LEFT_LEG][index].t21);
        s41->Move(trajectory[LEFT_LEG][index].t41);
        s22->Move(trajectory[LEFT_LEG][index].t22);
        s42->Move(trajectory[LEFT_LEG][index].t42);
    }
#endif // OFFLINE_TRAJECTORY_SUPPORT

    void ComputeExecute(int init_ts) {
        Point PE;
        float x_rest = x_swing_rest;
        float y_mean = y_mean_left;
        float phase_offset = 0.0;

        // Generate the trajectory for Left Leg
        int t = (millis() - init_ts) % time_period;
        float tmp1 = 2*M_PI*t/time_period;
        PE.X() = (x_swing_amp/2)*sin(tmp1) + x_rest;
        float tmp2 = ((t/1000.0)-y_mean)/y_sigma;
        PE.Y() = (y_amp/(y_sigma*sqrt(2*M_PI))) * exp(-0.5*tmp2*tmp2) - y_offset;
        PE.Z() = z_amp1*sin(tmp1+z_phase_offset+phase_offset) + z_amp2*sin(2*(tmp1+z_phase_offset+phase_offset));

        // Precompute the joint variables
        s11->Move(RAD_TO_DEG(L1_0->theta));
        s21->Move(RAD_TO_DEG(L2->theta));
        s41->Move(RAD_TO_DEG(L4->theta));
        s22->Move(RAD_TO_DEG(L2->theta));
        s42->Move(RAD_TO_DEG(L4->theta));

        // Generate the trajectory for Right Leg
        phase_offset = M_PI;
        x_rest *= -1;
        y_mean = y_mean_right;
        PE.X() = (x_swing_amp/2)*sin(tmp1) + x_rest;
        tmp2 = ((t/1000.0)-y_mean)/y_sigma;
        PE.Y() = (y_amp/(y_sigma*sqrt(2*M_PI))) * exp(-0.5*tmp2*tmp2) - y_offset;
        PE.Z() = z_amp1*sin(tmp1+z_phase_offset+phase_offset) + z_amp2*sin(2*(tmp1+z_phase_offset+phase_offset));

        // Precompute the joint variables        
    }
};

#endif //GAIT_H
