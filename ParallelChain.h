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
#define phi DEG_TO_RAD(34.55)
#define gamma DEG_TO_RAD(135.8)
#define a 0 //overlapping joint in mm
#define l1 35.087 // mm
#define l2 110.0 // mm
#define l3 110.0 // mm
#define l4 48.132 // mm
#define l5 126.669 // mm
#define l6 48.795 // mm

class ParallelChain {
    private:
    // Define the joints
    RevoluteJoint L1_0{a, M_PI_2, 0, M_PI_2}; // JV = theta1
    RevoluteJoint L1_1{0, phi, l1, 0};
    RevoluteJoint L2{0, 0, -l2, 0}; // JV = theta2
    RevoluteJoint L3{0, 0, -l3, 0}; // JV = theta3
    RevoluteJoint L4{0, M_PI_2-phi, l4, 0}; // JV = theta4
    RevoluteJoint L6{0, -gamma, -l6, 0}; // JV = theta3

    // Define the servos
    JointServo s11{0, 1470, -7.5, -90.0, 30.0}; // Hip servo
    JointServo s21{1, 1450, 7.5, -50.0, 20.0}; // Lower servo - 1
    JointServo s41{2, 1500, 7.5, -30.0, 30.0}; // Upper servo - 1
    JointServo s22{3, 1520, -7.5, -50.0, 20.0}; // Lower servo - 2
    JointServo s42{4, 1460, -7.5, -30.0, 30.0}; // Upper servo - 2
    
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
    ParallelChain() {
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

        s11.Move(0);
        s21.Move(0);
        s41.Move(0);
        s22.Move(0);
        s42.Move(0);
        delay(2000);

        s11.SetOffset(RAD_TO_DEG(L1_0.theta));
        s21.SetOffset(RAD_TO_DEG(L2.theta));
        s41.SetOffset(RAD_TO_DEG(L4.theta));
        s22.SetOffset(RAD_TO_DEG(L2.theta));
        s42.SetOffset(RAD_TO_DEG(L4.theta));
    }

    void MoveServos(float jv[]) {
        s11.Move(jv[0]);
        s21.Move(jv[1]);
        s41.Move(jv[2]);
        s22.Move(jv[3]);
        s42.Move(jv[4]);
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
        //Serial.print("Theta4 = ");
        //Serial.println(RAD_TO_DEG(L4.theta));
    }
};
#endif // PARALLEL_CHAIN_H
