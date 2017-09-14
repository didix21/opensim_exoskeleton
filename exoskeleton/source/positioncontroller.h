#ifndef POSITIONCONTROLLER_H
#define POSITIONCONTROLLER_H

#include<OpenSim/OpenSim.h>
#include <math.h>
#include "printOpenSimInformation.h"
#include "configuration.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

class PositionController : public Controller
{
OpenSim_DECLARE_CONCRETE_OBJECT(PositionController, Controller);

public:
    // This section contains methods that can be called in this controller class.
    public:
        /**
         * Constructor
         *
         * @param[in] aModel
         * Model to be controlled
         * @param[in] aKp
         * Position gain by which the position error will be multiplied
         */
        PositionController(double hipJointRefAngle, double kneeJointRefAngle,
                           double ankleJointRefAngle, Model* aModel);

        void getComponentsOfTwoVec3(SimTK::Vec3 initialPoint, SimTK::Vec3 finalPoint, SimTK::Vec3* vectorResult) const;
        double getVectorModule(SimTK::Vec3 &vector) const;
        double computeDesiredAngularAccelerationHipJoint(const SimTK::State &s,
                                                         double* getErr,
                                                         double* getPos) const;
        double computeDesiredAngularAccelerationKneeJoint(const SimTK::State &s,
                                                          double* getErr,
                                                          double* getPos) const;
        double computeDesiredAngularAccelerationAnkleJoint(const SimTK::State &s,
                                                           double* getErr,
                                                           double* getPos) const;
        SimTK::Vector computeDynamicsTorque(const SimTK::State &s,
                                            double* getErrs,
                                            double* getPoses) const;
        /**
        * @brief computeGravityCompensation obtain all the
        * gravity value that affects each joint
        * @param[in] s
        *  actual state
        * @return Return all the values of each body affected by the gravity
         */
        SimTK::Vector computeGravityCompensation(const SimTK::State &s) const;
        SimTK::Vector computeCoriolisCompensation(const SimTK::State &s) const;


        void computeControls(const SimTK::State& s, SimTK::Vector &controls) const;


    // This section contains the member variables of this controller class.
    private:
        /** Position gain for this controller */
        double k_exohip[3], k_exoknee[3], k_exoankle[3];
        /** Velocity gain for this controller */
        double hipJointRefAngle, kneeJointRefAngle, ankleJointRefAngle;

        // Model
        Model* aModel;

        // Mass
        double rFemurMass, rTibiaMass, rPatellaMass, rTalusMass, rCalcnMass,
               rToesMass, rFootMass, rShinMass, rThighMass, exoHipMass;

        double massFemThigh_r, massTibShin_r, massToeFoot_r;

};

#endif
