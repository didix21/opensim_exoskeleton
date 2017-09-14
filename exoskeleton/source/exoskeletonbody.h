#ifndef EXOSKELETONBODY_H
#define EXOSKELETONBODY_H

#include <OpenSim/OpenSim.h>
#include "configuration.h"
#include <OpenSim/Common/PiecewiseConstantFunction.h>
using namespace OpenSim;
using namespace SimTK;

class ExoskeletonBody
{
public:
    ExoskeletonBody(Model* osimModel, OpenSim::Set<OpenSim::Body>* bodySet,
                    OpenSim::Set<OpenSim::Joint>* jointSet);
    void createBodies();
    void createJoints();
    void createExoskeleton();
    void defineExoCoordinates();
    void defineInitialPosition(SimTK::State& s);

#ifdef ENABLE_BUSHINGFORCES

    void defineBushingForces(OpenSim::Set<OpenSim::Force>* bushingForces);

#endif

    void defineFrictionForces();
    void defineActuators(OpenSim::Set<OpenSim::Actuator>* actuatorSet);

private:
    Model* osimModel;
    OpenSim::Body rFoot, rShin, rThigh, exoHip;
    OpenSim::Set<OpenSim::Body>* bodySet;
    OpenSim::Set<OpenSim::Joint>* jointSet;

    // Body variables
    double rFootMass, rShinMass, rThighMass, hipMass;
    const Vec3 rFootMassCenter, rShinMassCenter, rThighMassCenter, hipMassCenter;
    const Vec3 scaleRFoot, scaleRShin, scaleRThigh, sclaeHip;
    const Inertia rFootInertia, rShinInertia, rThighInertia, hipInertia;

};

#endif // EXOSKELETONBODY_H
