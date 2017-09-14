/**
* This Project consist in control a double pendulum with OpenSim API.
*
*
*/

#include <OpenSim/OpenSim.h>
#include "printOpenSimInformation.h"
using namespace OpenSim;
using namespace std;

class PendulumController : public Controller
{
    OpenSim_DECLARE_CONCRETE_OBJECT(PendulumController, Controller);
public:
    PendulumController(double firstJointAngle, double secondJointAngle) : Controller(),
        r1(firstJointAngle), r2(secondJointAngle)
        {

        }

    double getAngleFromJoints(SimTK::String joint, const SimTK::State &s) const
    {
        double z = _model->getCoordinateSet().get(joint).getValue(s);
        return z;
    }

    double getError(double ref, double aCtAngle) const
    {
        return ref - aCtAngle ;
    }

    void getAlphaAndK(double Kp, double Kv, double* getAlphaK) const
    {
        double alpha = (Kv + sqrt(pow(Kv,2) - 4*Kp)/2);
        double k = 2*Kp/alpha;
        getAlphaK[0] = alpha;
        getAlphaK[1] = k;
    }


    double controlFirstJoint(SimTK::State s,double ref,double Kp,double Kv, double *getErr=nullptr) const
    {
       //double k = Kp, alpha = Kv;
       double xt_0 = getAngleFromJoints("q1",s); // Get actual joint state x(t)
       static double xt_1 = 0; // x(t-1)
       double err = getError(ref, xt_0); // Compute the error e = x_r - x(t)
       if(getErr != nullptr) // Return the error in a pointer, In case of needed it
           *getErr = err;
       double dx = (xt_0 - xt_1)/0.033; // Compute the derivative dx = (x(t)-x(t-1))/timeSample
       xt_1 = xt_0;
       //double torque = alpha*k/2.0*err - (k/2 + alpha)*dx; // Control law with k and alpha
       double torque = Kp*err - Kv*dx; // Control law
       return torque;
    }



    double controlSecondtJoint(SimTK::State s,double ref,double Kp,double Kv,double* getErr=nullptr) const
    {
       //double k = Kp, alpha = Kv;
       double xt_0 = getAngleFromJoints("q2",s);
       static double xt_1 = 0;
       double err = getError(ref, xt_0);
       if(getErr != nullptr)
        *getErr = err;
       double dx = (xt_0 - xt_1)/0.033;
       xt_1 = xt_0;
       //double desAcc = alpha*k/2.0*(err) - (k/2 + alpha)*dx;
       double torque = Kp*err - Kv*dx; // Control law
       return torque;
    }

    double getDistFromMassCtoFirstJointC(string joint,const SimTK::State &s) const
    {
        SimTK::MobilizedBodyIndex mIndex(0);
        if(joint == "q1")
            mIndex++;
        else if(joint == "q2")
            mIndex+=2;
        else if(joint == "block")
            mIndex+=3;
        SimTK::Vec3 massCenter(_model->getMatterSubsystem().getMobilizedBody(mIndex).findMassCenterLocationInGround(s));
        return sqrt(pow(massCenter[0],2) + pow(massCenter[1],2) + pow(massCenter[2],2));
    }

    double getDistFromMassCtoSecondJointC(string joint,const SimTK::State &s) const
    {
        SimTK::MobilizedBodyIndex mIndex(0);
        if(joint == "q1")
            mIndex++;
        else if(joint == "q2")
            mIndex+=2;
        else if(joint == "block")
            mIndex+=3;
        SimTK::MobilizedBody secondCylinderMob(_model->getMatterSubsystem().
                                             getMobilizedBody(SimTK::MobilizedBodyIndex(2)));
        SimTK::Vec3 massCenter(_model->getMatterSubsystem().
                             getMobilizedBody(mIndex).
                             findMassCenterLocationInAnotherBody(s,
                                                                 secondCylinderMob));
        return sqrt(pow(massCenter[0],2) + pow(massCenter[1],2) + pow(massCenter[2],2));
    }

    double getDesFirstJointTorque(double desAcc, const SimTK::State &s) const
    {
        double distJoint1ToBody1MassC = getDistFromMassCtoFirstJointC("q1",s);
        double distJoint1ToBody2MassC = getDistFromMassCtoFirstJointC("q2",s);
        double distJoint1ToBody3MassC = getDistFromMassCtoFirstJointC("block",s);
        double massArray[3];
        for(int i=1; i < _model->getBodySet().getSize(); i++)
            massArray[i-1] = _model->getBodySet().get(i).getMass();
        double desTorque = (distJoint1ToBody1MassC*massArray[0] + distJoint1ToBody2MassC*massArray[1] +
                   distJoint1ToBody3MassC*massArray[2])*desAcc;
        return desTorque;

    }

    double getDesSecondJointTorque(double desAcc, const SimTK::State &s) const
    {
        double distJoint2ToBody2MassC = getDistFromMassCtoSecondJointC("q2",s);
        double distJoint2ToBody3MassC = getDistFromMassCtoSecondJointC("block",s);
        double massArray[2];
        for(int i=2; i < _model->getBodySet().getSize(); i++)
            massArray[i-2] = _model->getBodySet().get(i).getMass();
        double desTorque = (distJoint2ToBody2MassC*massArray[0]
                + distJoint2ToBody3MassC*massArray[1])*desAcc;
        return desTorque;

    }

    SimTK::Vector calcCoriolis(const SimTK::State &s) const
    {
        SimTK::Vector c;
        _model->getMatterSubsystem().
                calcResidualForceIgnoringConstraints(
                    s, SimTK::Vector(0), SimTK::Vector_<SimTK::SpatialVec>(0),
                    SimTK::Vector(0), c);
        return c;
    }

    SimTK::Vector calcGravityCompensation(const SimTK::State &s) const
    {
        SimTK::Vector g;
        _model->getMatterSubsystem().
                multiplyBySystemJacobianTranspose(s,
                                                  _model->getGravityForce().getBodyForces(s),
                                                  g);
        return g;
    }

    void computeControls(const SimTK::State& s, SimTK::Vector &controls) const
    {
        double KP1 = 0.1, KV1 = 2;
        double KP2 = 0.08, KV2 = 2;


        double err1, err2;
        double desAcc1 = controlFirstJoint(s, r1, KP1, KV1, &err1);
        double desAcc2 = controlSecondtJoint(s, r2, KP2, KV2, &err2);
//        double desTorque1 = getDesFirstJointTorque(desAcc1+desAcc2, s);
//        double desTorque2 = getDesSecondJointTorque(desAcc2, s);
        _model->getMultibodySystem().realize(s,SimTK::Stage::Velocity);
        SimTK::Vector coriolisForce(calcCoriolis(s));

        SimTK::Vector desAcc(2);
        desAcc[0] = desAcc1;
        desAcc[1] = desAcc2;
        SimTK::Matrix M;

        SimTK::Vector desiredTorque;

        _model->getMatterSubsystem().multiplyByM(s,desAcc,desiredTorque);
        SimTK::Vector grav(calcGravityCompensation(s));
        double torque1 = (desiredTorque[0] + coriolisForce.get(0)
                - grav.get(0))/_model->getActuators().get("firstJointActuator").getOptimalForce();
        double torque2 = (desiredTorque[1] + coriolisForce.get(1)
                - grav.get(1))/_model->getActuators().get("secondJointActuator").getOptimalForce();

        SimTK::Vector torqueControl_1(1, torque1);
        _model->updActuators().get("firstJointActuator").addInControls(torqueControl_1, controls);

        SimTK::Vector torqueControl_2(1, torque2);
        _model->updActuators().get("secondJointActuator").addInControls(torqueControl_2, controls);

        // Save data to a file
        vector<string> nameData = {"Error J1", "Error J2",
                                   "Position J1", "Position J2",
                                   "desTorque1","desTorque2",
                                   "coriolis1","coriolis2",
                                   "grav1","grav2",
                                   "time"};
        vector<double> dataToPrint(11);// ={0.0, 0.0, 0.0, 0.0, 0.0};

        dataToPrint[0] = err1;
        dataToPrint[1] = err2;
        dataToPrint[2] = getAngleFromJoints("q1",s);
        dataToPrint[3] = getAngleFromJoints("q2",s);
        dataToPrint[4] = desiredTorque[0];
        dataToPrint[5] = desiredTorque[1];
        dataToPrint[6] = coriolisForce.get(0);
        dataToPrint[7] = coriolisForce.get(1);
        dataToPrint[8] = grav.get(0);
        dataToPrint[9] = grav.get(1);
        dataToPrint[10] = s.getTime();
        // Print all data to file
        static PrintToFile printFile("DataToPlot.csv");
        printFile.PrintDataToFile(nameData, dataToPrint);

    }

private:
    double r1, r2;
    double errorJoint1, errorJoint2;


};

int main()
{
    try
    {
        //////////////////////
        // DEFINE VARIABLES //
        //////////////////////
        SimTK::Vec3 gravity(0,-9.8065, 0); // Model'ls gravity value
        //SimTK::Vec3 gravity(0); // Model'ls gravity value
        // Time simulation
        double initTime(0), endTime(40); // In seconds

        //////////////////
        // CREATE MODEL //
        //////////////////
        Model osimPendulum;
        osimPendulum.setName("ControlDoublePendulum"); // Set the name of the model
        osimPendulum.setAuthors("Dídac Coll");         // Set the name of the author
        osimPendulum.setUseVisualizer(true);
        osimPendulum.setGravity(gravity);

        // Get ground
        OpenSim::Body& ground = osimPendulum.getGroundBody();
        ground.addDisplayGeometry("checkered_floor.vtp");

        ///////////////////
        // CREATE BODIES //
        ///////////////////
        // Define Mass properties, dimensions and inertia
        double cylinderMass = 0.1, cylinderLength = 0.5, cylinderDiameter = 0.06;
        SimTK::Vec3 cylinderDimensions(cylinderDiameter, cylinderLength, cylinderDiameter);
        SimTK::Vec3 cylinderMassCenter(0, cylinderLength/2, 0);
        SimTK::Inertia cylinderInertia = SimTK::Inertia::cylinderAlongY(cylinderDiameter/2.0,
                                                                        cylinderLength/2.0);

        OpenSim::Body *firstCylinder = new OpenSim::Body("firstCylinder", cylinderMass, cylinderMassCenter,
                                       cylinderMass*cylinderInertia);
        // Set a graphical representation of the cylinder
        firstCylinder->addDisplayGeometry("cylinder.vtp");
        // Scale the grahical cylinder(1 m tall, 1 m diameter) to match with body's dimensions
        GeometrySet& geometrySet = firstCylinder->updDisplayer()->updGeometrySet();
        DisplayGeometry& newDimCylinder = geometrySet[0];
        newDimCylinder.setScaleFactors(cylinderDimensions);
        newDimCylinder.setTransform(Transform(SimTK::Vec3(0.0, cylinderLength/2.0, 0.0)));
        // Add Sphere in Joint place
        firstCylinder->addDisplayGeometry("sphere.vtp");
        // Scale sphere
        geometrySet[1].setScaleFactors(SimTK::Vec3(0.1));



        OpenSim::Body *secondCylinder = new OpenSim::Body(*firstCylinder);
        secondCylinder->setName("secondCylinder");

        //Create block mass
        double blockMass = 20.0, blockSideLength = 0.2;
        SimTK::Vec3 blockMassCenter(0);
        SimTK::Inertia blockInertia = blockMass*SimTK::Inertia::brick(blockSideLength,
                                                                      blockSideLength,
                                                                      blockSideLength);

        OpenSim::Body *block = new OpenSim::Body("block", blockMass, blockMassCenter, blockInertia);
        block->addDisplayGeometry("block.vtp");
        block->updDisplayer()->updGeometrySet()[0].setScaleFactors(SimTK::Vec3(2.0));

        // Create 1 degree-of-freedom pin joints between ground, first cylinder, and second cyilinder
        // , weldjoint to attach block to second cyilinder
        SimTK::Vec3 orientationInGround(0), locationInGround(0),
                locationInParent(0.0, cylinderLength, 0.0),
            orientationInChild(0), locationInChild(0);

        PinJoint *firstJoint = new PinJoint("firstJoint", ground, locationInGround,
                                            orientationInGround, *firstCylinder,
                                            locationInChild, orientationInChild);
        PinJoint *secondJoint = new PinJoint("secondJoint", *firstCylinder, locationInParent,
                                             orientationInChild, *secondCylinder,
                                             locationInChild, orientationInChild);
        WeldJoint *endPendulum = new WeldJoint("endPendulum", *secondCylinder,
                                               locationInParent, orientationInChild,
                                               *block, locationInChild, orientationInChild);

        // Set range of joints coordinates
        double range[2] = {-SimTK::Pi*2, SimTK::Pi*2};
        // Get coordinate set of firstJoint
        CoordinateSet& fJointCoordinates = firstJoint->upd_CoordinateSet();
        fJointCoordinates[0].setName("q1");
        fJointCoordinates[0].setRange(range);
        // Get coordinate set of secondJoint
        CoordinateSet& sJointCoordinates = secondJoint->upd_CoordinateSet();
        sJointCoordinates[0].setName("q2");
        sJointCoordinates[0].setRange(range);

        // Add Bodies
        osimPendulum.addBody(firstCylinder);
        osimPendulum.addBody(secondCylinder);
        osimPendulum.addBody(block);

        // Creaate Actuators
        TorqueActuator* firstJointActuator = new TorqueActuator();
        firstJointActuator->setName("firstJointActuator");
        firstJointActuator->setBodyA(osimPendulum.getBodySet().get("firstCylinder"));
        firstJointActuator->setBodyB(osimPendulum.getGroundBody());
        firstJointActuator->setOptimalForce(20);
        firstJointActuator->setTorqueIsGlobal(false);
        TorqueActuator* secondJointActuator = new TorqueActuator();
        secondJointActuator->setName("secondJointActuator");
        secondJointActuator->setBodyA(osimPendulum.getBodySet().get("secondCylinder"));
        secondJointActuator->setBodyB(osimPendulum.getBodySet().get("firstCylinder"));
        secondJointActuator->setOptimalForce(20);
        secondJointActuator->setTorqueIsGlobal(false);
        // Add actuators as forces
        osimPendulum.addForce(firstJointActuator);
        osimPendulum.addForce(secondJointActuator);

        // Add a controller
        PendulumController *pendControl = new PendulumController(-SimTK::Pi/1.5, 0.0);
        pendControl->setActuators(osimPendulum.updActuators());
        osimPendulum.addController(pendControl);

        // Initialize system
        SimTK::State &si = osimPendulum.initSystem();

        // Add frictions
        SimTK::MultibodySystem &system = osimPendulum.updMultibodySystem();
        SimTK::GeneralForceSubsystem &gForces = osimPendulum.updForceSubsystem();
        SimTK::Force::MobilityLinearDamper fricFirstJoint (gForces,
                                                           osimPendulum.updMatterSubsystem().
                                                           updMobilizedBody(SimTK::MobilizedBodyIndex(1)),
                                                           SimTK::MobilizerUIndex(0),
                                                           0);

        SimTK::Force::MobilityLinearDamper fricSecondJoint (gForces,
                                                           osimPendulum.updMatterSubsystem().
                                                           updMobilizedBody(SimTK::MobilizedBodyIndex(2)),
                                                           SimTK::MobilizerUIndex(0),
                                                           0);

        // Set Up Visualizer
        osimPendulum.updMatterSubsystem().setShowDefaultGeometry(true);
        SimTK::Visualizer& viz = osimPendulum.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundColor(SimTK::Black);

        si = osimPendulum.initializeState();
        osimPendulum.printDetailedInfo(si, cout);
        // Define initial position of each joint
        double q1 = -SimTK::Pi;
        double q2 = -q1 + SimTK::Pi;

        fJointCoordinates[0].setValue(si,q1);
        sJointCoordinates[0].setValue(si,q2);

        //osimPendulum.updMultibodySystem().realize(si, SimTK::Stage::Acceleration);
        // Set integrator
        SimTK::RungeKuttaMersonIntegrator integrator(osimPendulum.getMultibodySystem());
        Manager manager(osimPendulum, integrator);
        manager.setInitialTime(initTime); manager.setFinalTime(endTime);
        manager.integrate(si);

    }
    catch(const OpenSim::Exception& exception)
    {
        cout << "Exception occured: " << exception.getMessage() << endl;
        return 1;
    }

return 0;
}
