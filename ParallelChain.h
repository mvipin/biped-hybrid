#ifndef PARALLEL_CHAIN_H
#define PARALLEL_CHAIN_H

// Position of a link within a kinematic chain
enum {
    LINK_POS_1 = 1,
    LINK_POS_2,
    LINK_POS_3,
    LINKS_PER_KCHAIN = LINK_POS_3,
};

// Joints with servos in a prallel chain
enum {
    ACTIVE_JOINT_11,
    ACTIVE_JOINT_21,
    ACTIVE_JOINT_41,
    ACTIVE_JOINT_22,
    ACTIVE_JOINT_42,
    ACTIVE_JOINT_9,
    ACTIVE_JOINTS_PER_PCHAIN,
};

// Kinematic chains in a parallel chain
enum {
    KCHAIN_UPPER,
    KCHAIN_MIDDLE,
    KCHAIN_LOWER,
    KCHAIN_NUM_MAX,
};

class ParallelChain {
    private:
    const float phi = LINK_PHI;
    const float gamma = LINK_GAMMA;
    const float a = LINK_A_SIZE; //overlapping joint in mm
    const float l1 = LINK_L1_SIZE; // mm
    const float l2 = LINK_L2_SIZE; // mm
    const float l3 = LINK_L3_SIZE; // mm
    const float l4 = LINK_L4_SIZE; // mm
    const float l5 = LINK_L5_SIZE; // mm
    const float l6 = LINK_L6_SIZE; // mm

    // Define the joints
    RevoluteJoint L1_0{a, M_PI_2, 0, M_PI_2}; // JV = theta1
    RevoluteJoint L1_1{0, phi, l1, 0};
    RevoluteJoint L2{0, 0, -l2, 0}; // JV = theta2
    RevoluteJoint L3{0, 0, -l3, 0}; // JV = theta3
    RevoluteJoint L4{0, M_PI_2-phi, l4, 0}; // JV = theta4
    RevoluteJoint L6{0, -gamma, -l6, 0}; // JV = theta3

    // Define the servos
    JointServo servo[ACTIVE_JOINTS_PER_PCHAIN] = {JointServo(-7.5, -90.0, 30.0), // Hip servo
                                                  JointServo(7.5, -50.0, 20.0), // Lower servo - 1
                                                  JointServo(7.5, -30.0, 30.0), // Upper servo - 1
                                                  JointServo(-7.5, -50.0, 20.0), // Lower servo - 2
                                                  JointServo(-7.5, -30.0, 30.0), // Upper servo - 2
                                                  JointServo(7.5, -30.0, 30.0)}; // Foot servo

    KinematicChain<LINKS_PER_KCHAIN> kchain[KCHAIN_NUM_MAX];

    float theta3(void) {
        Point P4 = kchain[KCHAIN_UPPER].ForwardKinematics(LINK_POS_2).p;
        Point P5 = kchain[KCHAIN_UPPER].ForwardKinematics(LINK_POS_3).p;
        Point P3 = kchain[KCHAIN_MIDDLE].ForwardKinematics(LINK_POS_2).p;

        // Compute required vectors
        Point V1 = P5 - P3;
        float v1mag = V1.Magnitude();
     
        Point V2 = P4 - P3;
        float v2mag = V2.Magnitude();
     
        // Calculate intermediate angles required for calculating initial value for joint variable at P3
        float alpha1 = acos((l6*l6 + v1mag*v1mag - l5*l5)/(2*l6*v1mag));
        float alpha2 = acos((v1mag*v1mag + v2mag*v2mag - l4*l4)/(2*v1mag*v2mag));
        float alpha3 = acos((v2mag*v2mag + l2*l2 - l1*l1)/(2*l2*v2mag));
        float t3 = alpha1 + alpha2 + alpha3 + gamma - M_PI;
        
        return t3;
    }

    public:
    ParallelChain(int id[], int uinit[]) {
        // Initialize the kinematic chains and joint servos
        kchain[KCHAIN_UPPER].AddLink(L1_0);  
        kchain[KCHAIN_UPPER].AddLink(L1_1);  
        kchain[KCHAIN_UPPER].AddLink(L4);  
    
        kchain[KCHAIN_MIDDLE].AddLink(L1_0);  
        kchain[KCHAIN_MIDDLE].AddLink(L2);  
        kchain[KCHAIN_MIDDLE].AddLink(L3);  
        L3.Move(theta3()); // TODO: specify theta3 while declaring the link 
    
        kchain[KCHAIN_LOWER].AddLink(L1_0);  
        kchain[KCHAIN_LOWER].AddLink(L2);  
        kchain[KCHAIN_LOWER].AddLink(L6);  
        L6.Move(theta3()); // TODO: specify theta3 while declaring the link

        for (int i=ACTIVE_JOINT_11; i<ACTIVE_JOINTS_PER_PCHAIN; i++) {
            servo[i].SetId(id[i]);
            servo[i].SetUinit(uinit[i]);
        }
    }

    void Init(void) {
        for (int i=ACTIVE_JOINT_11; i<ACTIVE_JOINTS_PER_PCHAIN; i++) {
            servo[i].Move(0);
        }
        delay(1000);

        servo[ACTIVE_JOINT_11].SetOffset(RAD_TO_DEG(L1_0.theta));
        servo[ACTIVE_JOINT_21].SetOffset(RAD_TO_DEG(L2.theta));
        servo[ACTIVE_JOINT_41].SetOffset(RAD_TO_DEG(L4.theta));
        servo[ACTIVE_JOINT_22].SetOffset(RAD_TO_DEG(L2.theta));
        servo[ACTIVE_JOINT_42].SetOffset(RAD_TO_DEG(L4.theta));
        servo[ACTIVE_JOINT_9].SetOffset(0);
    }

    void MoveServos(float jv[]) {
        for (int i=ACTIVE_JOINT_11; i<ACTIVE_JOINTS_PER_PCHAIN; i++) {
            servo[i].Move(jv[i]);
        }
    }

    void InverseKinematics(Point PE, float rjv[]) {
        float jv;
    
        // Calculate and update 'theta1' and associated link(s)
        jv = M_PI_2 + atan(-PE.X()/PE.Y());
        //Serial << "Delta Theta1 = " << jv - L1_0.theta << '\n';
        L1_0.Move(jv - L1_0.theta);
        rjv[0] = RAD_TO_DEG(L1_0.theta);
        //Serial.print("Theta1 = ");
        //Serial.println(RAD_TO_DEG(L1_0.theta));
    
        // Calculate and update 'theta2' and associated link(s)
        float r1 = PE.Magnitude();
        jv = atan(PE.Z()/PE.Y()) - acos((l2*l2 + r1*r1 - l3*l3)/(2*l2*r1));
        //Serial << "Delta Theta2 = " << jv - L2.theta << '\n';
        L2.Move(jv - L2.theta);
        rjv[1] = rjv[3] = RAD_TO_DEG(L2.theta);
        //Serial.print("Theta2 = ");
        //Serial.println(RAD_TO_DEG(L2.theta));
        
        // Calculate and update 'theta3' and associated link(s)
        jv = M_PI - acos((l2*l2 + l3*l3 - r1*r1)/(2*l2*l3));
        //Serial << "Delta Theta3 = " << jv - L3.theta << '\n';
        L3.Move(jv - L3.theta);
        L6.Move(jv - L6.theta - gamma);
    
        // Calculate and update 'theta4' and associated link(s)
        Point P3 = kchain[KCHAIN_MIDDLE].ForwardKinematics(LINK_POS_2).p;
        Point P4 = kchain[KCHAIN_UPPER].ForwardKinematics(LINK_POS_2).p;
        Point P6 = kchain[KCHAIN_LOWER].ForwardKinematics(LINK_POS_3).p;
        //Serial << "P3 = " << P3.X() << ", " << P3.Y() << ", " << P3.Z() << "\n";   
        //Serial << "P4 = " << P4.X() << ", " << P4.Y() << ", " << P4.Z() << "\n";   
        //Serial << "P6 = " << P6.X() << ", " << P6.Y() << ", " << P6.Z() << "\n";   
    
        // Compute required vectors
        Point V2 = P4 - P3;
        float v2mag = V2.Magnitude();
     
        Point V3 = P4 - P6;
        float v3mag = V3.Magnitude();
    
        float beta1 = acos((l4*l4 + v3mag*v3mag - l5*l5)/(2*l4*v3mag));
        float beta2 = acos((v2mag*v2mag + v3mag*v3mag - l6*l6)/(2*v2mag*v3mag));
        float beta3 = acos((v2mag*v2mag + l1*l1 - l2*l2)/(2*l1*v2mag));
        jv = M_PI - (beta1 + beta2 + beta3);
        //Serial << "Delta Theta4 = " << jv - L4.theta << '\n';
        L4.Move(jv - L4.theta);
        rjv[2] = rjv[4] = RAD_TO_DEG(L4.theta);
        rjv[5] = rjv[1] - (-RAD_TO_DEG(theta3()));
    }
};
#endif // PARALLEL_CHAIN_H
