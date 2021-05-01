#ifndef KINEMATIC_CHAIN_H
#define KINEMATIC_CHAIN_H

#include <Geometry.h>
#include "Link.h"

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
#endif //KINEMATIC_CHAIN_H
