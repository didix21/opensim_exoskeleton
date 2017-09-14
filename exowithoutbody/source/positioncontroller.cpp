#include "positioncontroller.h"


PositionController::PositionController(double hipJointRefAngle, double kneeJointRefAngle,
                                       double ankleJointRefAngle, Model* aModel) :
    Controller(),k_exohip K_EXOHIP,
    k_exoknee K_EXOKNEE,
    k_exoankle K_EXOANKLE,
    hipJointRefAngle(hipJointRefAngle),
    kneeJointRefAngle(kneeJointRefAngle),
    ankleJointRefAngle(ankleJointRefAngle)
{

}


/**
 * @brief PositionController::getComponentsOfTwoVec3: compute the main components between two points
 * @param initialPoint
 * @param finalPoint
 * @param vectorResult
 */
void PositionController::getComponentsOfTwoVec3(SimTK::Vec3 initialPoint, SimTK::Vec3 finalPoint, SimTK::Vec3* vectorResult) const
{

    SimTK::Vec3 vectResult(finalPoint[0]-initialPoint[0],finalPoint[1]-initialPoint[1],finalPoint[2]-initialPoint[2]);
    *vectorResult = vectResult;

}

double PositionController::getVectorModule(SimTK::Vec3& vector) const
{
    double module = sqrt(pow(vector[0],2) + pow(vector[1],2) + pow(vector[2],2));
    return module;
}

//double PositionController::computeDesiredAngularAccelerationHipJoint(const State &s) const
//{
//    // Obtain the coordinateset of the joint desired
//    const Coordinate& coord = _model->getCoordinateSet().get("exoHipJoint_coord");
//    double xt = coord.getValue(s); // Actual angle x(t)
//    static double xt1 = 0; // Angle position one previous step x(t-1)
//    static double errt1 = 0;
//    double iX = (xt + xt1)*0.033; // Compute integration
//    double dx = (xt-xt1)/0.033; // Compute the derivative of xt
//    xt1 = xt; // save the actual value of xt to compute the derivative
//    double err = hipJointRefAngle - xt; // Error
//    cout << "Error ExoHipJoint = " << err << endl;
//    double  kp = k_exohip[0], kv = k_exohip[1], ki = k_exohip[2];
//    double desAcc = kp*err - kv*dx + ki*iX;
//    return desAcc;
//}


double PositionController::computeDesiredAngularAccelerationHipJoint(const State &s,
                                                                     double* getErr = nullptr,
                                                                     double* getPos = nullptr) const
{
    // Obtain the coordinateset of the joint desired
    const Coordinate& coord = _model->getCoordinateSet().get("exoHipJoint_coord");
    double xt = coord.getValue(s); // Actual angle x(t)
    *getPos = xt;
    static double errt1 = hipJointRefAngle;
    double err = hipJointRefAngle - xt; // Error
    double iErr = (err + errt1)*0.033; // Compute integration
    double dErr = (err - errt1)/0.033; // Compute the derivative of xt
    errt1 = err;
    *getErr = err;
   // cout << "Error ExoHipJoint = " << err << endl;
    double  kp = k_exohip[0], kv = k_exohip[1], ki = k_exohip[2];
    double desAcc = kp*err + kv*dErr + ki*iErr;
    return desAcc;
}


//double PositionController::computeDesiredAngularAccelerationKneeJoint(const State &s) const
//{
//    const Coordinate& coord = _model->getCoordinateSet().get("kneeJoint_r_coord");
//    double xt = coord.getValue(s);
//    static double xt1 = 0;
//    double iX = (xt + xt1)*0.033;
//    double dx = (xt - xt1)/0.033;
//    xt1 = xt;
//    double err = kneeJointRefAngle - xt;

//    double  kp = k_exoknee[0], kv = k_exoknee[1], ki = k_exoknee[2];
//    double desAcc = kp*err - kv*dx + ki*iX;
//    cout << "Error ExoKneeJoint = " << err << endl;
//    return desAcc;
//}

double PositionController::computeDesiredAngularAccelerationKneeJoint(const State &s,
                                                                      double* getErr = nullptr,
                                                                      double* getPos = nullptr) const
{
    const Coordinate& coord = _model->getCoordinateSet().get("kneeJoint_r_coord");
    double xt = coord.getValue(s);
    *getPos = xt;
    static double errt1 = kneeJointRefAngle;
    double err = kneeJointRefAngle - xt; // Error
    double iErr = (err + errt1)*0.033; // Compute integration
    double dErr = (err - errt1)/0.033; // Compute the derivative of xt
    errt1 = err;
    *getErr = err;
    //cout << "Error ExoKneeJoint = " << err << endl;
    double  kp = k_exoknee[0], kv = k_exoknee[1], ki = k_exoknee[2];
    double desAcc = kp*err + kv*dErr + ki*iErr;
    return desAcc;
}


//double PositionController::computeDesiredAngularAccelerationAnkleJoint(const State &s) const
//{
//    const Coordinate& coord = _model->getCoordinateSet().get("ankleJoint_r_coord");
//    double xt = coord.getValue(s);
//    static double xt1 = 0;
//    double iX = (xt + xt1)*0.033;
//    double dx = (xt - xt1)/0.033;
//    xt1 = xt;
//    double err = ankleJointRefAngle - xt;
//    double  kp = k_exoankle[0], kv = k_exoankle[1], ki = k_exoankle[2];
//    double desAcc = kp*err - kv*dx + ki*iX;
//    cout << "Error ExoAnkleJoint = " << err << endl;
//    return desAcc;
//}

double PositionController::computeDesiredAngularAccelerationAnkleJoint(const State &s,
                                                                       double* getErr = nullptr,
                                                                       double* getPos = nullptr) const
{
    const Coordinate& coord = _model->getCoordinateSet().get("ankleJoint_r_coord");
    double xt = coord.getValue(s);
    *getPos = xt;
    static double xt1 = 0; // Angle position one previous step x(t-1)
    static double errt1 = ankleJointRefAngle;
    double err = ankleJointRefAngle - xt; // Error
    double iErr = (err + errt1)*0.033; // Compute integration
    double dErr = (err - errt1)/0.033; // Compute the derivative of xt
    errt1 = err;
    *getErr = err;
   // cout << "Error ExoAnkleJoint = " << err << endl;
    double  kp = k_exoankle[0], kv = k_exoankle[1], ki = k_exoankle[2];
    double desAcc = kp*err + kv*dErr + ki*iErr;
    return desAcc;
}

SimTK::Vector PositionController::computeDynamicsTorque(const State &s,
                                                        double* getErrs = nullptr,
                                                        double* getPoses = nullptr) const
{
    double desAccExoHip = computeDesiredAngularAccelerationHipJoint(s, &getErrs[0], &getPoses[0]);
    double desAccExoKnee = computeDesiredAngularAccelerationKneeJoint(s, &getErrs[1], &getPoses[1]);
    double desAccExoAnkle = computeDesiredAngularAccelerationAnkleJoint(s, &getErrs[2], &getPoses[2]);
    SimTK::Vector_<double> desAcc(3);
    desAcc[0] = desAccExoHip;
    desAcc[1] = desAccExoKnee;
    desAcc[2] = desAccExoAnkle;
    SimTK::Vector torques;
    _model->getMatterSubsystem().multiplyByM(s, desAcc, torques);

    return torques;
}

SimTK::Vector PositionController::computeGravityCompensation(const SimTK::State &s) const
{
    SimTK::Vector g;
    _model->getMatterSubsystem().
            multiplyBySystemJacobianTranspose(s,
                                              _model->getGravityForce().getBodyForces(s),
                                              g);
    return g;
}

SimTK::Vector PositionController::computeCoriolisCompensation(const SimTK::State &s) const
{
    SimTK::Vector c;
    _model->getMatterSubsystem().
            calcResidualForceIgnoringConstraints(s,
                                                 SimTK::Vector(0),
                                                 SimTK::Vector_<SpatialVec>(0),
                                                 SimTK::Vector(0), c);
    return c;
}



void PositionController::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
   double getErrs[3];
   double getPoses[3];
   SimTK::Vector dynTorque(computeDynamicsTorque(s, getErrs, getPoses));
   SimTK::Vector grav(computeGravityCompensation(s));
   SimTK::Vector coriolis(computeCoriolisCompensation(s));



   double nTorqueExoHip = (dynTorque.get(0)
                           + coriolis.get(0)
                           - (grav.get(0))
                           )/_model->getActuators().get("exoHipActuator_r").getOptimalForce();
   double nTorqueExoKnee = (dynTorque.get(1)
                            + coriolis.get(1)
                            - grav.get(1)
                            )/_model->getActuators().get("exoKneeActuator_r").getOptimalForce();
   double nTorqueExoAnkle = (//desAnkleJointTorq
                             dynTorque.get(2)
                             + coriolis.get(2)
                             - grav.get(2)
                             )/_model->getActuators().get("exoAnkleActuator_r").getOptimalForce();

//   cout << "Torque 1 = " << nTorqueExoHip << endl;
//   cout << "Torque 2 = " << nTorqueExoKnee << endl;
//   cout << "Torque 3 = " << nTorqueExoAnkle << endl;

   vector<string> nameData = {"Error J1", "Error J2", "Error J3",
                              "Position J1", "Position J2", "Position J3",
                              "desTorque1","desTorque2", "desTorque3",
                              "coriolis1","coriolis2", "coriolis3",
                              "grav1","grav2", "grav3",
                              "time"};
   vector<double> dataToPrint(nameData.size());
   for(int i = 0; i < 3; i++)
   {
       dataToPrint[i] = getErrs[i]; // Save errors
       dataToPrint[3 + i] = getPoses[i]; // Save positions
       dataToPrint[6 + i] = dynTorque.get(i); // Save Dynamics Torques
       dataToPrint[9 + i] = coriolis.get(i); // Save coriolis and centrifugal torques
       dataToPrint[12 + i] = grav.get(i); // Save
       dataToPrint[15] = s.getTime();
   }
    // Print all data to file
    static PrintToFile printFile("DataToPlot.txt");
    printFile.PrintDataToFile(nameData, dataToPrint);

   Vector torqueControl(1, -nTorqueExoHip);
   _model->updActuators().get("exoHipActuator_r").addInControls(torqueControl, controls);
   Vector torqueControl2(1, -nTorqueExoKnee);
   _model->updActuators().get("exoKneeActuator_r").addInControls(torqueControl2, controls);
   Vector torqueControl3(1, nTorqueExoAnkle);
   _model->updActuators().get("exoAnkleActuator_r").addInControls(torqueControl3, controls);
}
