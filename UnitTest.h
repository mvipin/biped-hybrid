#ifndef UNIT_TEST_H
#define UNIT_TEST_H

#define UP_DOWN 0, 0, -220, -120, 0, 0
#define LEFT_RIGHT -30, 30, -190, -190, 0, 0
#define FRONT_REAR 0, 0, -190, -190, -50, 50

class UnitTest {
    private:
    const int time_period = STEPS_DURATION;

    int interval;
    int xmin, xmax, xmid, xamp;
    int ymin, ymax, ymid, yamp;
    int zmin, zmax, zmid, zamp;

#ifdef OFFLINE_TRAJECTORY_SUPPORT
    float trajectory[NUM_DATA_POINTS][ACTIVE_JOINTS_PER_PCHAIN];
#endif // OFFLINE_TRAJECTORY_SUPPORT

    int uinit[ACTIVE_JOINTS_PER_PCHAIN] = {RIGHT_SERVO_INIT_POS_S11,
                                           RIGHT_SERVO_INIT_POS_S21,
                                           RIGHT_SERVO_INIT_POS_S41,
                                           RIGHT_SERVO_INIT_POS_S22,
                                           RIGHT_SERVO_INIT_POS_S42,
                                           RIGHT_SERVO_INIT_POS_S9};
    int id[ACTIVE_JOINTS_PER_PCHAIN] = {RIGHT_SERVO_PIN_S11,
                                        RIGHT_SERVO_PIN_S21,
                                        RIGHT_SERVO_PIN_S41,
                                        RIGHT_SERVO_PIN_S22,
                                        RIGHT_SERVO_PIN_S42,
                                        RIGHT_SERVO_PIN_S9};
    ParallelChain leg{id, uinit};

    public:
    UnitTest(int _xmin, int _xmax, int _ymin, int _ymax, int _zmin, int _zmax) :
        xmin(_xmin), xmax(_xmax), ymin(_ymin), ymax(_ymax), zmin(_zmin), zmax(_zmax) {
        xmid = (xmin + xmax) >> 1;
        xamp = (xmax - xmin) >> 1;
        ymid = (ymin + ymax) >> 1;
        yamp = (ymax - ymin) >> 1;
        zmid = (zmin + zmax) >> 1;
        zamp = (zmax - zmin) >> 1;
        interval = time_period/NUM_DATA_POINTS;
    }

    void Init(void) {
        leg.Init();
    }

    void ComputeExecute(int init_ts) {
        float jv[ACTIVE_JOINTS_PER_PCHAIN];
        Point PE;

        // Generate the trajectory
        int t = (millis() - init_ts) % time_period;
        float tmp = sin(2*M_PI*t/time_period);
        PE.X() = xmid + xamp * tmp;
        PE.Y() = ymid + yamp * tmp;
        PE.Z() = zmid + zamp * tmp;
        //Serial << "X = " << PE.X() << " Y = " << PE.Y() << " Z = " << PE.Z() << "\n";

        // Compute the joint variables
        leg.InverseKinematics(PE, jv);
        leg.MoveServos(jv);
        //Serial << t << " " << jv[ACTIVE_JOINT_11] << " " << jv[ACTIVE_JOINT_21] << " " << jv[ACTIVE_JOINT_41] << " " << jv[ACTIVE_JOINT_22] << " " << jv[ACTIVE_JOINT_42] << "\n";
    }

#ifdef OFFLINE_TRAJECTORY_SUPPORT
    void Compute() {
        int index;
        Point PE;
        for (int t=0; t<time_period; t+=interval) {
            // Generate the trajectory
            float tmp = sin(2*M_PI*t/time_period);
            PE.X() = xmid + xamp * tmp;
            PE.Y() = ymid + yamp * tmp;
            PE.Z() = zmid + zamp * tmp;
        
            // Precompute the joint variables
            index = t/interval;
            leg.InverseKinematics(PE, trajectory[index]);
        }
    }

    void Execute(int init_ts) {
        int index = ((millis() - init_ts) % time_period)/interval;
        leg.MoveServos(trajectory[index]);
    }
#endif // OFFLINE_TRAJECTORY_SUPPORT
};
#endif //UNIT_TEST_H
