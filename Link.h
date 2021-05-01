#ifndef LINK_H
#define LINK_H

enum {
    LINK_POS_1 = 1,
    LINK_POS_2,
    LINK_POS_3,
};

// Link stores the D-H parameters for one link in the chain. It's an abstract base class so to use it you have to subclass it and define the Move function, more on that later though
class Link {
    public:
    float d, theta, r, alpha;

    Link(float _d, float _theta, float _r, float _alpha) : d(_d), theta(_theta), r(_r), alpha(_alpha) { }
    virtual void Move(float amount) = 0;
};

// In addition to the D-H parameters, the joint also needs to specify how the D-H parameters change in response to it's movement. This let's the inverse kinematics algorithm know how useful the joints movement is in reaching
// the goal position.

// A revolute joint which changes the theta D-H parameter when it moves
class RevoluteJoint : public Link {
    public:
    RevoluteJoint(float d, float theta, float r, float alpha) : Link(d, theta, r, alpha) { }
    void Move(float amount) {
        theta += amount;
    }
};

// A prismatic joint which changes the r parameter. We migh also throw in a parameter 'stiffness' to make the joint more reluctant move in the IK
class PrismaticJoint : public Link {
    float stiffness;
    
    public:
    PrismaticJoint(float d, float theta, float r, float alpha, float _stiffness = 1) : Link(d, theta, r, alpha), stiffness(_stiffness) { }
    void Move(float amount) {
        r += (amount * stiffness);
    }
};

// A joint that doesn't move at all. The IK will effectively ignore this one
class ImmobileJoint : public Link {
    public:
    ImmobileJoint(float d, float theta, float r, float alpha) : Link(d, theta, r, alpha) { }
    void Move(float amount) {}
};
#endif // LINK_H
