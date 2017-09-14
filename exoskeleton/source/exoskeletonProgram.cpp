// Explain All the code

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
    try
    {
        double initTime = 0.0, finalTime = 30.0;
        Model osimModel("skeleton.osim");
        osimModel.setName("exoskeleton");

        // Set gravity value
#ifdef ENABLE_GRAVITY

        const Vec3 gravity(GRAVITY_VALUE);

#else

        const Vec3 gravity(0,0,0);

#endif

        osimModel.setGravity(gravity);

        // Set visualization mode
        osimModel.setUseVisualizer(true);
        OpenSim::Set<OpenSim::Body> bodySet;
        OpenSim::Set<OpenSim::Joint> jointSet;
        OpenSim::Set<OpenSim::Force> bushingForces;
        OpenSim::Set<OpenSim::Actuator> actuatorSet;

        const Vec3 locInParent(0), orInParent(0), locInBody(0), orInBody(0);
        WeldJoint* pelvisGround = new WeldJoint("pelvisground",
                                              osimModel.getGroundBody(),
                                              locInParent,
                                              orInParent,
                                              osimModel.updBodySet().get(1),
                                              locInBody,
                                              orInBody);

        ExoskeletonBody exoskeletonBody(&osimModel, &bodySet, &jointSet);
        exoskeletonBody.createExoskeleton();
        exoskeletonBody.defineExoCoordinates();

#ifdef ENABLE_BUSHINGFORCES

        exoskeletonBody.defineBushingForces(&bushingForces);

#endif

        exoskeletonBody.defineActuators(&actuatorSet);

#ifdef ENABLE_CONTROLLER

        double hipRef = degreesToRadians(-60),
               kneeRef = degreesToRadians(40),
               ankleRef = degreesToRadians(0);

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

        /////////////////////
        exoskeletonBody.defineFrictionForces();


        ForceReporter *forces = new ForceReporter(&osimModel);
        osimModel.updAnalysisSet().adoptAndAppend(forces);
        //PrintForceToFile printBushingForce("forces.txt","tibiaBushingForce1.tibia_r",&osimModel);

        si = osimModel.initializeState();
        exoskeletonBody.defineInitialPosition(si);
        //osimModel.getMultibodySystem().realize(si,Stage::Acceleration);
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
        // Save results
        forces->getForceStorage().print("allForces.csv");
        //printBushingForce.printForces();

    }
    catch(OpenSim::Exception ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }

    return 0;
}
