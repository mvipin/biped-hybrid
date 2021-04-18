#include <Geometry.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define USMIN 500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2470 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define DEGREE_TO_PULSE(deg) (USMIN + (float(deg)/180)*(USMAX-USMIN))

#define DEG_TO_RAD(d) (((d)/180.0)*M_PI)
#define RAD_TO_DEG(r) (((r)/M_PI)*180.0)

enum {
    LINK1 = 1,
    LINK2,
    LINK3,
};

enum {
    SERVO_T11,
    SERVO_T21,
    SERVO_T41,
    SERVO_T22,
    SERVO_T42,
    SERVO_9,
    SERVO_NUM_MAX,
};

struct servo_params {
    int uinit;
    float dmin;
    float dmax;
    float slope;
    float offset;
} servo[SERVO_NUM_MAX];

const float phi = DEG_TO_RAD(34.55);
const float gamma = DEG_TO_RAD(135.8);
const float a = 0; //overlapping joint in mm
const float l1 = 35.087; // mm
const float l2 = 110.0; // mm
const float l3 = 110.0; // mm
const float l4 = 48.132; // mm
const float l5 = 126.669; // mm
const float l6 = 48.795; // mm

// Link stores the D-H parameters for one link in the chain. It's an abstract base class so to use it you have to subclass it and define the Move function, more on that later though
class Link {
    public:
    float d, theta, r, alpha;

    Link(float _d, float _theta, float _r, float _alpha) : d(_d), theta(_theta), r(_r), alpha(_alpha) { }
    virtual void Move(float amount) = 0;
};

// KinematicChain manages the links and implements the forward and inverse kinematics
template<int maxLinks> class KinematicChain {
    // A few variables used in the inverse kinematics defined here to save re-allocating them every time inverse kinematics is called
    Point deltaPose;
    Matrix<3,1> jacobian;
    Matrix<maxLinks> deltaAngles;
    Transformation currentPose, perturbedPose;

    // The number of links addedto the chain via AddLink
    unsigned int noOfLinks;

    public:
    // An array containing all the D-H parameters for the chain
    Link *chain[maxLinks];

    KinematicChain() {
        noOfLinks = 0;
    }

    // Add a link - it's D-H parameters as well as a function pointer describing it's joint movement
    void AddLink(Link &l) { //float d, float theta, float r, float alpha, void (*move)(Link&, float))
        if (noOfLinks == maxLinks)
          return;

        chain[noOfLinks++] = &l;
    }

    int NoOfLinks() {
        return noOfLinks;
    }

    // Transforms pose from the end effector coordinate frame to the base coordinate frame.
    Transformation &ForwardKinematics(Transformation &pose) {
        for (int i = noOfLinks - 1; i >= 0; i--) {
            // These four operations will convert between two coordinate frames defined by D-H parameters, it's pretty standard stuff
            pose.RotateX(chain[i]->alpha);
            pose.Translate(chain[i]->r,0,0);
            pose.Translate(0,0,chain[i]->d);
            pose.RotateZ(chain[i]->theta);
        }

        return pose;
    }

    // Transforms pose from the nth coordinate frame to the base coordinate frame.
    Transformation &ForwardKinematics(Transformation &pose, int n) {
        for (int i = n - 1; i >= 0; i--) {
            // These four operations will convert between two coordinate frames defined by D-H parameters, it's pretty standard stuff
            pose.RotateX(chain[i]->alpha);
            pose.Translate(chain[i]->r,0,0);
            pose.Translate(0,0,chain[i]->d);
            pose.RotateZ(chain[i]->theta);
        }

        return pose;
    }
   
    // Handy overload to save having to feed in a fresh Transformation every time
    Transformation ForwardKinematics() {
        currentPose = Identity<4,4>();
        return ForwardKinematics(currentPose);
    }

    // Handy overload to save having to feed in a fresh Transformation every time
    Transformation ForwardKinematics(int n) {
        currentPose = Identity<4,4>();
        return ForwardKinematics(currentPose, n);
    }
   
    // Calculates the joints angles Transforms targetPose to the R^chainsize domain of link angles and sets it on the internal chain array
    virtual Transformation &InverseKinematics(Transformation &targetPose, int maxIterations = 1000, float convergenceSpeed = 0.00001, float closeEnough = 0.1, float perturbance = 0.001) {
        // Inverse kinematics doesn't have a general solution so we have to solve it iteratively
        for (int it = 0; it < maxIterations; it++) {
            // First find the current end effector position
            currentPose = Identity<4,4>();
            ForwardKinematics(currentPose);

            // And find the error between the target and the current pose
            deltaPose = currentPose.p - targetPose.p;

            // If the search gets close enough then just break
            if (deltaPose.Magnitude() < closeEnough)
                break;

            // Now we move each joint a little bit and see how it affects the position of the end effector.
            for (unsigned int link = 0; link < noOfLinks; link++) {
                // Move the link a little bit
                perturbedPose = Identity<4,4>();
                chain[link]->Move(perturbance);

                // Find the change in end effector position
                ForwardKinematics(perturbedPose);

                // And set the link back to where it was for now
                chain[link]->Move(-perturbance);

                // Now divide the change in x/y/z position by the amount by which we moved the joint to find a jacobian in the form of {dx/dtheta, dy/dtheta, dz/dtheta}
                jacobian = (currentPose.p - perturbedPose.p) * (1 / perturbance);

                // Now calculate a change in joint angle to bring the chain towards the target position. The joint angle / position relatioship is really non-linear so the
                // jacobian won't be valid very far from the operating point and it's better to move only a little bit at a time; this is handled with the convergeSpeed parameter
                // Ideally we'd find the pseudoinverse of the jacobian here, but that's quite a bit of doing. For this example we'll just use the transpose as an inverse of sorts.
                deltaAngles(link) = (~jacobian * deltaPose)(0) * convergenceSpeed;
            }

            // Update the link angles
            for (unsigned int link = 0; link < noOfLinks; link++) {
                chain[link]->Move(deltaAngles(link));
            }
        }

        return currentPose;
    }
};

// In addition to the D-H parameters, the joint also needs to specify how the D-H parameters change in response to it's movement. This let's the inverse kinematics algorithm know how useful the joints movement is in reaching
// the goal position. You can define how the joint moves by subclassing Link and overriding the Move function. The function should modify the link's D-H parameters in some way proportional to the 'amount' parameter.

// For example, we can define a revolute joint which changes the theta D-H parameter when it moves
class RevoluteJoint : public Link {
    public:
    RevoluteJoint(float d, float theta, float r, float alpha) : Link(d, theta, r, alpha) { }
    void Move(float amount) {
        theta += amount;
    }
};

// We can define a prismatic joint which changes the r parameter. We migh also throw in a parameter 'stiffness' to make the joint more reluctant move in the IK
class PrismaticJoint : public Link {
    float stiffness;
    
    public:
    PrismaticJoint(float d, float theta, float r, float alpha, float _stiffness = 1) : Link(d, theta, r, alpha), stiffness(_stiffness) { }
    void Move(float amount) {
        r += (amount * stiffness);
    }
};

// We could even define a joint that doesn't move at all. The IK will effectively ignore this one
class ImmobileJoint : public Link {
    public:
    ImmobileJoint(float d, float theta, float r, float alpha) : Link(d, theta, r, alpha) { }
    void Move(float amount) { }
};

// Kinematic chains for upper and end leg
KinematicChain<3> upper;
KinematicChain<3> end;
KinematicChain<3> lower;

// Define the joints
RevoluteJoint L1_0(a, M_PI_2, 0, M_PI_2); // JV = theta1
RevoluteJoint L1_1(0, phi, l1, 0);
RevoluteJoint L2(0, 0, -l2, 0); // JV = theta2
RevoluteJoint L3(0, 0, -l3, 0); // JV = theta3
RevoluteJoint L4(0, M_PI_2-phi, l4, 0); // JV = theta4
RevoluteJoint L6(0, -gamma, -l6, 0); // JV = theta3

float theta3(void) {
    Point P4 = upper.ForwardKinematics(LINK2).p;
    Point P5 = upper.ForwardKinematics(LINK3).p;
    Point P3 = end.ForwardKinematics(LINK2).p;
 
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
    L1_0.Move(jv - L1_0.theta);
//    Serial.print("Theta1 = ");
//    Serial.println(RAD_TO_DEG(L1_0.theta));

    // Calculate and update 'theta2' and associated link(s)
    float r1 = PE.Magnitude();
    jv = atan(PE.Z()/PE.Y()) - acos((l2*l2 + r1*r1 - l3*l3)/(2*l2*r1));
    //Serial << "Delta Theta2 = " << jv - L2.theta << '\n';
    L2.Move(jv - L2.theta);
//    Serial.print("Theta2 = ");
//    Serial.println(RAD_TO_DEG(L2.theta));
    
    // Calculate and update 'theta3' and associated link(s)
    jv = M_PI - acos((l2*l2 + l3*l3 - r1*r1)/(2*l2*l3));
    //Serial << "Delta Theta3 = " << jv - L3.theta << '\n';
    L3.Move(jv - L3.theta);
    L6.Move(jv - L6.theta - gamma);

    // Calculate and update 'theta4' and associated link(s)
    Point P3 = end.ForwardKinematics(LINK2).p;
    Point P4 = upper.ForwardKinematics(LINK2).p;
    Point P6 = lower.ForwardKinematics(LINK3).p;
    
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
//    Serial.print("Theta4 = ");
//    Serial.println(RAD_TO_DEG(L4.theta));
}

void MoveServo(int sid, float deg) {
    pwm.writeMicroseconds(sid, deg*servo[sid].slope + servo[sid].uinit);
}

void setup() {
    Serial.begin(115200);
    
    // In theory the internal oscillator is 25MHz but it really isn't
    // that precise. You can 'calibrate' by tweaking this number till
    // you get the frequency you're expecting!
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    // Hip servo
    servo[SERVO_T11].slope = -7.5;
    servo[SERVO_T11].dmin = -90.0;
    servo[SERVO_T11].dmax = 30.0;
    servo[SERVO_T11].uinit = 1470;
    MoveServo(SERVO_T11, 0);
    
    // Upper servo - 1
    servo[SERVO_T41].slope = 7.5;
    servo[SERVO_T41].dmin = -30.0;
    servo[SERVO_T41].dmax = 30.0;
    servo[SERVO_T41].uinit = 1500;
    MoveServo(SERVO_T41, 0);

    // Lower servo - 1
    servo[SERVO_T21].slope = 7.5;
    servo[SERVO_T21].dmin = -50.0;
    servo[SERVO_T21].dmax = 20.0;
    servo[SERVO_T21].uinit = 1450;
    MoveServo(SERVO_T21, 0);

    // Upper servo - 2
    servo[SERVO_T42].slope = 7.5;
    servo[SERVO_T42].dmin = -30.0;
    servo[SERVO_T42].dmax = 30.0;
    servo[SERVO_T42].uinit = 1460;
    MoveServo(SERVO_T42, 0);
    
    // Lower servo - 2
    servo[SERVO_T22].slope = 7.5;
    servo[SERVO_T22].dmin = -50.0;
    servo[SERVO_T22].dmax = 20.0;
    servo[SERVO_T22].uinit = 1520;
    MoveServo(SERVO_T22, 0);

    delay(2000);
    
    // Add links to the upper kinematic chain
    upper.AddLink(L1_0);
    upper.AddLink(L1_1);
    upper.AddLink(L4);

    // Add links to the end effector kinematic chain
    end.AddLink(L1_0);
    end.AddLink(L2);
    end.AddLink(L3);
    L3.Move(theta3()); // TODO: specify theta3 while declaring the link 
    
    // Add links to the lower kinematic chain
    lower.AddLink(L1_0);
    lower.AddLink(L2);
    lower.AddLink(L6);
    L6.Move(theta3()); // TODO: specify theta3 while declaring the link 

    servo[SERVO_T11].offset = RAD_TO_DEG(L1_0.theta);
    servo[SERVO_T21].offset = RAD_TO_DEG(L2.theta);
    servo[SERVO_T41].offset = RAD_TO_DEG(L4.theta);
    servo[SERVO_T22].offset = RAD_TO_DEG(L2.theta);
    servo[SERVO_T42].offset = RAD_TO_DEG(L4.theta);
#if 0
    Serial << "Initial values of JVs" << '\n';
    Serial.print("Theta 11 = ");
    Serial.println(RAD_TO_DEG(L1_0.theta));
    Serial.print("Theta 21 = ");
    Serial.println(RAD_TO_DEG(L2.theta));
    Serial.print("Theta 41 = ");
    Serial.println(RAD_TO_DEG(L4.theta));
    Serial.print("Theta 22 = ");
    Serial.println(RAD_TO_DEG(L2.theta));
    Serial.print("Theta 42 = ");
    Serial.println(RAD_TO_DEG(L4.theta));
#endif
}

void testFrontBack(void) {
    Point PE;
    PE.X() = 0.0;
    PE.Y() = -190.0;
    PE.Z() = 0.0;

    int min = -20;
    int max = 20;
    int delayms = 5;

    // Movement in Z direction
    for (int i=0; i<=max; i+=1) {
        PE.Z() = i;
        InverseKinematics(PE);
        MoveServo(SERVO_T11, RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        MoveServo(SERVO_T21, RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        MoveServo(SERVO_T41, RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        delay(delayms);
//        Serial << "PE = " << end.ForwardKinematics().p << "\n\n";
    }

    for (int i=max; i>=min; i-=1) {
        PE.Z() = i;
        InverseKinematics(PE);
        MoveServo(SERVO_T11, RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        MoveServo(SERVO_T21, RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        MoveServo(SERVO_T41, RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        delay(delayms);
#if 0
        Serial.print("Servo T11 = ");
        Serial.println(RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        Serial.print("Servo T21 = ");
        Serial.println(RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        Serial.print("Servo T41 = ");
        Serial.println(RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
#endif
//        Serial << "PE = " << end.ForwardKinematics().p << "\n\n";
    }

    for (int i=min; i<=0; i+=10) {
        PE.Z() = i;
        InverseKinematics(PE);
        MoveServo(SERVO_T11, RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        MoveServo(SERVO_T21, RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        MoveServo(SERVO_T41, RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        delay(delayms);
//        Serial << "PE = " << end.ForwardKinematics().p << "\n\n";
    }
}

void testLeftRight(void) {
    Point PE;
    PE.X() = 0.0;
    PE.Y() = -190.0;
    PE.Z() = 0.0;

    int min = -10;
    int max = 10;
    int delayms = 5;
    
    // Movement in X direction
    for (int i=0; i<=max; i+=1) {
        PE.X() = i;
        InverseKinematics(PE);
        MoveServo(SERVO_T11, RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        MoveServo(SERVO_T21, RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        MoveServo(SERVO_T41, RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        delay(delayms);
//        Serial << "PE = " << end.ForwardKinematics().p << "\n\n";
    }

    for (int i=max; i>=min; i-=1) {
        PE.X() = i;
        InverseKinematics(PE);
        MoveServo(SERVO_T11, RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        MoveServo(SERVO_T21, RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        MoveServo(SERVO_T41, RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        delay(delayms);
#if 0
        Serial.print("Servo T11 = ");
        Serial.println(RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        Serial.print("Servo T21 = ");
        Serial.println(RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        Serial.print("Servo T41 = ");
        Serial.println(RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
#endif
//        Serial << "PE = " << end.ForwardKinematics().p << "\n\n";
    }

    for (int i=min; i<=0; i+=10) {
        PE.X() = i;
        InverseKinematics(PE);
        MoveServo(SERVO_T11, RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        MoveServo(SERVO_T21, RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        MoveServo(SERVO_T41, RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        delay(delayms);
//        Serial << "PE = " << end.ForwardKinematics().p << "\n\n";
    }
}

void testUpDown(void) {
    Point PE;
    PE.X() = 0.0;
    PE.Y() = -220.0;
    PE.Z() = 0.0;

    int min = -220;
    int max = -120;
    int delayms = 5;

    // Movement in Y direction
    for (int i=min; i<=max; i+=1) {
        PE.Y() = i;
        InverseKinematics(PE);
        MoveServo(SERVO_T11, RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        MoveServo(SERVO_T21, RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        MoveServo(SERVO_T41, RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        MoveServo(SERVO_T22, -(RAD_TO_DEG(L2.theta) - servo[SERVO_T22].offset));
        MoveServo(SERVO_T42, -(RAD_TO_DEG(L4.theta) - servo[SERVO_T42].offset));
        delay(delayms);
#if 0
        Serial.print("Servo T11 = ");
        Serial.println(RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        Serial.print("Servo T21 = ");
        Serial.println(RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        Serial.print("Servo T41 = ");
        Serial.println(RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        Serial << "PE = " << end.ForwardKinematics().p << "\n\n";
#endif
    }
    
    for (int i=max; i>=min; i-=1) {
        PE.Y() = i;
        InverseKinematics(PE);
        MoveServo(SERVO_T11, RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        MoveServo(SERVO_T21, RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        MoveServo(SERVO_T41, RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        MoveServo(SERVO_T22, -(RAD_TO_DEG(L2.theta) - servo[SERVO_T22].offset));
        MoveServo(SERVO_T42, -(RAD_TO_DEG(L4.theta) - servo[SERVO_T42].offset));
        delay(delayms);
#if 0
        Serial.print("Servo T11 = ");
        Serial.println(RAD_TO_DEG(L1_0.theta) - servo[SERVO_T11].offset);
        Serial.print("Servo T21 = ");
        Serial.println(RAD_TO_DEG(L2.theta) - servo[SERVO_T21].offset);
        Serial.print("Servo T41 = ");
        Serial.println(RAD_TO_DEG(L4.theta) - servo[SERVO_T41].offset);
        Serial << "PE = " << end.ForwardKinematics().p << "\n\n";
#endif
    }
}

void loop() {
    testUpDown();
}
