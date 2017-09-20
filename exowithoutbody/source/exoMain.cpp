#include <OpenSim/OpenSim.h>
#include "printOpenSimInformation.h"
#include "exoskeletonbody.h"
#include "configuration.h"
#include <OpenSim/Common/PiecewiseConstantFunction.h>
#include "positioncontroller.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

int main()
{
    double initTime = 0.0, finalTime = 15.0;
    Model osimModel;
    osimModel.setName("exowithoutbody");

    // Set gravity value
#ifdef ENABLE_GRAVITY

    const Vec3 gravity(GRAVITY_VALUE);

#else

    const Vec3 gravity(0);

#endif

    osimModel.setGravity(gravity);

    // Set visualization mode
    osimModel.setUseVisualizer(true);
    OpenSim::Set<OpenSim::Body> bodySet;
    OpenSim::Set<OpenSim::Joint> jointSet;
    OpenSim::Set<OpenSim::Force> bushingForces;
    OpenSim::Set<OpenSim::Actuator> actuatorSet;

    ExoskeletonBody exoskeletonBody(&osimModel, &bodySet, &jointSet);
    exoskeletonBody.createExoskeleton();
    exoskeletonBody.defineExoCoordinates();
    exoskeletonBody.defineActuators(&actuatorSet);

#ifdef ENABLE_CONTROLLER

    double hipRef = degreesToRadians(50),
           kneeRef = degreesToRadians(-60) - hipRef/1.5,
           ankleRef = degreesToRadians(20);

    PositionController* posController = new PositionController(hipRef,
                                                               kneeRef,
                                                               ankleRef,
                                                               &osimModel);
    posController->setActuators(osimModel.updActuators());
    osimModel.addController(posController);

#endif


    //////////////////////////
    // PERFORM A SIMULATION //
    //////////////////////////
    // Initialize System
    State& si = osimModel.initSystem();
    osimModel.printDetailedInfo(si, std::cout);
    exoskeletonBody.defineFrictionForces();
    osimModel.print("exowithoutbody.osim");

    si = osimModel.initializeState();
    exoskeletonBody.defineInitialPosition(si);

    ////////////////////////
    // DISABLE ACTUACTORS //
    ////////////////////////
    osimModel.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = osimModel.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundColor(Black);

    // Simulate
    RungeKuttaMersonIntegrator integrator (osimModel.updMultibodySystem());
    Manager manager(osimModel, integrator);
    manager.setInitialTime(initTime); manager.setFinalTime(finalTime);
    manager.integrate(si);

    return 0;
}
