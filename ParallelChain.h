#ifndef PARALLEL_CHAIN_H
#define PARALLEL_CHAIN_H

#include "Link.h"
#include "KinematicChain.h"
#include "JointServo.h"
#include "Utils.h"

enum {
    KCHAIN_UPPER,
    KCHAIN_MIDDLE,
    KCHAIN_LOWER,
    KCHAIN_NUM_MAX,
};

#define LINKS_PER_KCHAIN 3

class ParallelChain {
    float phi;
    float gamma;
    float a;
    float l1;
    float l2;
    float l3;
    float l4;
    float l5;
    float l6;
    KinematicChain<LINKS_PER_KCHAIN> kchain[KCHAIN_NUM_MAX];
    JointServo *servo[SERVO_NUM_MAX];
    RevoluteJoint *L1_0, *L1_1, *L2, *L3, *L4, *L6;

    public:
    ParallelChain(float _phi, float _gamma, float _a, float _l1, float _l2, float _l3, float _l4, float _l5, float _l6,
          RevoluteJoint &_L1_0, RevoluteJoint &_L1_1, RevoluteJoint &_L2, RevoluteJoint &_L3,
          RevoluteJoint &_L4, RevoluteJoint &_L6) {
        phi = _phi;
        gamma = _gamma;
        a = _a;
        l1 = _l1;
        l2 = _l2;
        l3 = _l3;
        l4 = _l4;
        l5 = _l5;
        l6 = _l6;
        L1_0 = &_L1_0;
        L1_1 = &_L1_1;
        L2 = &_L2;
        L3 = &_L3;
        L4 = &_L4;
        L6 = &_L6;
    }

    AddToChain(int cid, RevoluteJoint &j) {
        kchain[cid].AddLink(j);
    }

    AddServo(int sid, JointServo &s) {
        servo[sid] = &s;
        servo[sid]->Move(0);
    }

    float Theta3(void) {
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

    void InverseKinematics(Point PE) {
        float jv;
    
        // Calculate and update 'theta1' and associated link(s)
        jv = M_PI_2 + atan(-PE.X()/PE.Y());
        //Serial << "Delta Theta1 = " << jv - L1_0.theta << '\n';
        L1_0->Move(jv - L1_0->theta);
        //Serial.print("Theta1 = ");
        //Serial.println(RAD_TO_DEG(L1_0.theta));
    
        // Calculate and update 'theta2' and associated link(s)
        float r1 = PE.Magnitude();
        jv = atan(PE.Z()/PE.Y()) - acos((l2*l2 + r1*r1 - l3*l3)/(2*l2*r1));
        //Serial << "Delta Theta2 = " << jv - L2.theta << '\n';
        L2->Move(jv - L2->theta);
        //Serial.print("Theta2 = ");
        //Serial.println(RAD_TO_DEG(L2.theta));
        
        // Calculate and update 'theta3' and associated link(s)
        jv = M_PI - acos((l2*l2 + l3*l3 - r1*r1)/(2*l2*l3));
        //Serial << "Delta Theta3 = " << jv - L3.theta << '\n';
        L3->Move(jv - L3->theta);
        L6->Move(jv - L6->theta - gamma);
    
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
        L4->Move(jv - L4->theta);
        //Serial.print("Theta4 = ");
        //Serial.println(RAD_TO_DEG(L4.theta));
    }
};
#endif // PARALLEL_CHAIN_H
