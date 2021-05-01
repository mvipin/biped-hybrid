#ifndef GAIT_H
#define GAIT_H

#include "Utils.h"
#include "JointServo.h"
#include "Link.h"
#include "ParallelChain.h"

enum {
    LEFT_LEG,
    RIGHT_LEG,
    LEG_NUM_MAX,
};

class Gait {
    private:
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

    float jv[5];
    ParallelChain left, right;

#ifdef OFFLINE_TRAJECTORY_SUPPORT
    float trajectory[LEG_NUM_MAX][NUM_DATA_POINTS][5];
#endif // OFFLINE_TRAJECTORY_SUPPORT

    public:
#ifdef OFFLINE_TRAJECTORY_SUPPORT
    Compute() {
        Point PE;
        int interval = time_period/NUM_DATA_POINTS;

        for (int leg=LEFT_LEG; leg<=RIGHT_LEG; leg++) {
            float x_rest = x_swing_rest;
            float y_mean = y_mean_left;
            float phase_offset = 0;

            if (leg == RIGHT_LEG) {
                phase_offset = M_PI;
                x_rest *= -1;
                y_mean = y_mean_right;
            }

            for (int t=0; t<time_period; t+=interval) {
                // Generate the trajectory
                float tmp = 2*M_PI*t/time_period;
                PE.X() = (x_swing_amp/2)*sin(tmp) + x_rest;
                PE.Y() = (y_amp/(y_sigma*sqrt(2*M_PI))) * exp(-0.5 * (((t/1000)-y_mean)/y_sigma) * (((t/1000)-y_mean)/y_sigma)) - y_offset;
                PE.Z() = z_amp1*sin(tmp+z_phase_offset) + z_amp2*sin(2*(tmp+z_phase_offset));
        
                // Precompute the joint variables
                int index = t/interval;
                left.InverseKinematics(PE, trajectory[leg][index]);
            }
        }
    }

    Execute(int init_ts) {
        int interval = time_period/NUM_DATA_POINTS;
        int index = ((millis() - init_ts) % time_period)/interval;
        left.MoveServos(trajectory[LEFT_LEG][index]);
        right.MoveServos(trajectory[RIGHT_LEG][index]);
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

        // Compute the joint variables
        left.InverseKinematics(PE, jv);
        left.MoveServos(jv);

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
