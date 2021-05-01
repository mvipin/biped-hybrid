#ifndef GAIT_H
#define GAIT_H

enum {
    LEFT_LEG,
    RIGHT_LEG,
    LEG_NUM_MAX,
};

class Gait {
    private:
    const int time_period = STEPS_DURATION;
    const float x_swing_rest = X_SWING_REST;
    const float x_swing_amp = X_SWING_AMP;
    const float y_mean_left = Y_MEAN_LEFT;
    const float y_sigma = Y_SIGMA;
    const float y_mean_right = Y_MEAN_RIGHT;
    const float y_amp = Y_AMP;
    const float y_offset = Y_OFFSET;
    const float z_amp1 = Z_AMP1;
    const float z_amp2 = Z_AMP2;
    const float z_phase_offset = Z_PHASE_OFFSET;

    int ruinit[ACTIVE_JOINTS_PER_PCHAIN] = {
        RIGHT_SERVO_INIT_POS_S11, RIGHT_SERVO_INIT_POS_S21, RIGHT_SERVO_INIT_POS_S41,
        RIGHT_SERVO_INIT_POS_S22, RIGHT_SERVO_INIT_POS_S42};
    int rid[ACTIVE_JOINTS_PER_PCHAIN] = {
        RIGHT_SERVO_PIN_S11, RIGHT_SERVO_PIN_S21, RIGHT_SERVO_PIN_S41,
        RIGHT_SERVO_PIN_S22, RIGHT_SERVO_PIN_S42};
    int luinit[ACTIVE_JOINTS_PER_PCHAIN] = {
        LEFT_SERVO_INIT_POS_S11, LEFT_SERVO_INIT_POS_S21, LEFT_SERVO_INIT_POS_S41,
        LEFT_SERVO_INIT_POS_S22, LEFT_SERVO_INIT_POS_S42};
    int lid[ACTIVE_JOINTS_PER_PCHAIN] = {
        LEFT_SERVO_PIN_S11, LEFT_SERVO_PIN_S21, LEFT_SERVO_PIN_S41,
        LEFT_SERVO_PIN_S22, LEFT_SERVO_PIN_S42};

    ParallelChain leg[LEG_NUM_MAX] = {ParallelChain(lid, luinit), ParallelChain(rid, ruinit)};

#ifdef OFFLINE_TRAJECTORY_SUPPORT
    float trajectory[LEG_NUM_MAX][NUM_DATA_POINTS][ACTIVE_JOINTS_PER_PCHAIN];
#endif // OFFLINE_TRAJECTORY_SUPPORT

    Point Trajectory(int t, float _x_rest, float _y_mean, float _z_phase_offset) {
        Point PE;
        float tmp = 2*M_PI*t/time_period;

        PE.X() = (x_swing_amp/2)*sin(tmp) + _x_rest;
        PE.Y() = (y_amp/(y_sigma*sqrt(2*M_PI))) * exp(-0.5 * (((t/1000.0)-_y_mean)/y_sigma) * (((t/1000.0)-_y_mean)/y_sigma)) - y_offset;
        PE.Z() = z_amp1*sin(tmp+z_phase_offset+_z_phase_offset) + z_amp2*sin(2*(tmp+z_phase_offset+_z_phase_offset));

        return PE;
    }

    public:
    void Init() {
        leg[LEFT_LEG].Init();
        leg[RIGHT_LEG].Init();
    }

#ifdef OFFLINE_TRAJECTORY_SUPPORT
    Compute() {
        Point PE;
        int interval = time_period/NUM_DATA_POINTS;

        for (int index=LEFT_LEG; index<=RIGHT_LEG; index++) {
            float x_rest = x_swing_rest;
            float y_mean = y_mean_left;
            float phase_offset = 0;

            if (index == RIGHT_LEG) {
                phase_offset = M_PI;
                x_rest *= -1;
                y_mean = y_mean_right;
            }

            for (int t=0; t<time_period; t+=interval) {
                // Generate the trajectory
                PE = Trajectory(t, x_rest, y_mean, phase_offset);
        
                // Precompute the joint variables
                leg[index].InverseKinematics(PE, trajectory[index][t/interval]);
            }
        }
    }

    Execute(int init_ts) {
        int interval = time_period/NUM_DATA_POINTS;
        int index = ((millis() - init_ts) % time_period)/interval;
        leg[LEFT_LEG].MoveServos(trajectory[LEFT_LEG][index]);
        leg[RIGHT_LEG].MoveServos(trajectory[RIGHT_LEG][index]);
    }
#endif // OFFLINE_TRAJECTORY_SUPPORT

    void ComputeExecute(int init_ts) {
        float x_rest = x_swing_rest;
        float y_mean = y_mean_left;
        float phase_offset = 0.0;
        float jv[ACTIVE_JOINTS_PER_PCHAIN];

        // Generate the trajectory for Left Leg
        int t = (millis() - init_ts) % time_period;
        Point PE = Trajectory(t, x_rest, y_mean, phase_offset);
        //Serial << t << " " << PE.X() << " " << PE.Y() << " " << PE.Z() << " ";

        // Compute and actuate the joint variables
        leg[LEFT_LEG].InverseKinematics(PE, jv);
        leg[LEFT_LEG].MoveServos(jv);
        //Serial << t << " " << jv[ACTIVE_JOINT_11] << " " << jv[ACTIVE_JOINT_21] << " " << jv[ACTIVE_JOINT_41] << " " << jv[ACTIVE_JOINT_22] << " " << jv[ACTIVE_JOINT_42] << "\n";

        // Generate the trajectory for Right Leg
        phase_offset = M_PI;
        x_rest *= -1;
        y_mean = y_mean_right;
        t = (millis() - init_ts) % time_period;
        PE = Trajectory(t, x_rest, y_mean, phase_offset);
        //Serial << t << " " << PE.X() << " " << PE.Y() << " " << PE.Z() << " " << "\n";
 
        // Compute and actuate the joint variables
        leg[RIGHT_LEG].InverseKinematics(PE, jv);
        leg[RIGHT_LEG].MoveServos(jv);
    }
};
#endif //GAIT_H
