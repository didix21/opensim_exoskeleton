#include "exoskeletonbody.h"
/**
 * @brief ExoskeletonBody::ExoskeletonBody it is the default
 * constructor.
 * @param osimModel
 * @param bodySet
 * @param jointSet
 */
ExoskeletonBody::ExoskeletonBody(Model* osimModel, OpenSim::Set<OpenSim::Body>* bodySet,
                                 OpenSim::Set<OpenSim::Joint>* jointSet)
    : osimModel(osimModel), bodySet(bodySet), jointSet(jointSet),
    // Initialize Body Mass
    rFootMass(0.510668),    // kg
    rShinMass(0.19567251),  // kg
    rThighMass(0.2138295),  // kg
    hipMass(0.85308774),   // kg
    // Initialize Body Mass Centers
//    rFootMassCenter(0.0282501, -0.523347, 0.116142),  // m
//    rShinMassCenter(0.0649924, -0.80779, 0.0808896),  // m
//    rThighMassCenter(0.0805624, -0.191386, 0.117911), // m
//    hipMassCenter(0.0567373, 0.0433811, -0.00018222),// m
    rFootMassCenter(0.067394, -0.058410, -0.063129),  // m
    rShinMassCenter(-0.009493, 0.13976, 0.00998),  // m
    rThighMassCenter(0.057235, 0.142446, -0.009), // m
    hipMassCenter(0.0567373, 0.0433811, -0.00018222),// m
    // Initilize Inertia
    //                xx          yy          zz
    //                xy          xz          yz
    rFootInertia(0.33744544, 0.00852229, 0.33785791,
                 -0.02713561, 0.00250198,-0.03328571),
    rShinInertia(0.05890566, 0.00320362, 0.0565293,
                 0.00229535, 0.00058312, -0.01209693),
    rThighInertia(0.01556931, 0.004679, 0.01394983,
                  -0.00314352, 0.00203429, -0.00518597),
    hipInertia(0.01254384, 0.01446753, 0.00695939,
                0.00217966, 0.00011335, -4.28e-006)
{

}

/**
 * @brief ExoskeletonBody::createBodies: this method is in charge of creating
 * all the exoskeleton's bodies. First of all, the bodies has to be created,
 * then they the joints between them are created and finally the bodies are
 * added to the OpenSim Model.
 */
void ExoskeletonBody::createBodies()
{

#ifdef ENABLE_EXOFOOT

    // Create the main exoskeleton bodies
    OpenSim::Body rF  ("rFoot",         // Body Name
                       rFootMass,       // Body Mass
                       rFootMassCenter, // Body Mass Center
                       rFootInertia     // Body Inertia
                       );
    rF.scale(Vec3(1.1884260802,1.2782330916,1.3)); // Scale the body
    rF.addDisplayGeometry("rfoot.obj"); // Add visual geometry to the body
    rFoot = rF; // Save the object in a private object body

    #ifdef ENABLE_EXOSHIN

    OpenSim::Body rS ("rShin",
                   rShinMass,
                   rShinMassCenter,
                   rShinInertia);
    rS.scale(Vec3(1.1884260802,1.3434147124,1.3));
    rS.addDisplayGeometry("RightKnee.obj");
    rShin = rS;

        #ifdef ENABLE_EXOTHIGH

             OpenSim::Body rT ("rThigh",
                               rThighMass,
                               rThighMassCenter,
                               rThighInertia);
             rT.scale(Vec3(1.3884260802,1.2178638159,1.3));
             rT.addDisplayGeometry("RightHip.obj");
             rThigh = rT;

            #ifdef ENABLE_EXOHIP

                OpenSim::Body hip ("exoHip",
                                    hipMass,
                                    hipMassCenter,
                                    hipInertia);
                hip.scale(Vec3(1.38842608, 1.391052749, 1.3));
                hip.addDisplayGeometry("BackExo.obj");
                exoHip = hip;

            #endif // ENABLE_EXOHIP

        #endif // ENABLE_EXOTHIGH

    #endif // ENABLE_EXOSHIN

#endif // ENABLE_EXOFOOT

}

/**
 * @brief ExoskeletonBody::createJoints: thism ethod cal lfirst createBodies
 * and then all the joints are created. Finally the method adds the joints and
 * bodies in two different vectors of type Joint and Body, respectively.
 * Body vector is added in OpenSim Model.
 */
void ExoskeletonBody::createJoints()
{

#ifdef ENABLE_EXOFOOT

    createBodies();

    // Create a weldjoint behind skeleton and exoskeleton foot.
    const Vec3 exoBodyLocInParent(-0.15, 0.07, 0.1), exoBodyOrInParent(0),
             exoBodyLocInBody(0, 0, 0), exoBodyOrInBody(0);
    WeldJoint exoBodyJ_r ("exoBodyJoint_r",
                        osimModel->updBodySet().get(7), // toe_r
                        exoBodyLocInParent,
                        exoBodyOrInParent,
                        rFoot,
                        exoBodyLocInBody,
                        exoBodyOrInBody);
    jointSet->insert(jointSet->getSize(),exoBodyJ_r);
    bodySet->insert(bodySet->getSize(),rFoot);

    #ifdef ENABLE_EXOSHIN

        Vec3 ankLocInParent(0, 0, 0), ankOrInParent(0),
                ankLocInBody(0, 0, 0), ankOrInBody(0);
        /**
         * @brief exoAnkJ_r: This joint connects the tibia
         * exoskeleton part with the foot
         */
        PinJoint exoAnkJ_r ("exoAnkleJoint_r",
                              rFoot,
                              ankLocInParent,
                              ankOrInParent,
                              rShin,
                              ankLocInBody,
                              ankOrInBody);
        jointSet->insert(jointSet->getSize(),exoAnkJ_r);
        bodySet->insert(bodySet->getSize(),rShin);

        #ifdef ENABLE_EXOTHIGH

            Vec3 kneeLocInParent(72.5e-3, 0.445, 0), kneeOrInParent(0),
                    kneeLocInBody(0), kneeOrInBody(0);
            PinJoint exoKneeJ_r ("exoKneeJoint_r",
                                rShin,
                                kneeLocInParent,
                                kneeOrInParent,
                                rThigh,
                                kneeLocInBody,
                                kneeOrInBody);
            jointSet->insert(jointSet->getSize(),exoKneeJ_r);
            bodySet->insert(bodySet->getSize(),rThigh);

            #ifdef ENABLE_EXOHIP

                Vec3 hipLocInParent(38e-3, 0.4645, 0), hipOrInParent(0),
                        hipLocInBody(0), hipOrInBody(0);
                PinJoint exoHipJ ("exoHipJ",
                                  rThigh,
                                  hipLocInParent,
                                  hipOrInParent,
                                  exoHip,
                                  hipLocInBody,
                                  hipOrInBody);
                jointSet->insert(jointSet->getSize(),exoHipJ);
                bodySet->insert(bodySet->getSize(),exoHip);

           #endif // ENABLE_EXOHIP

        #endif // ENABLE_EXOTHIGH

    #endif // ENABLE_EXOSHIN

#endif // ENABLE_EXOFOOT

#ifdef ENABLE_EXOFOOT

        osimModel->addBody(&bodySet->get("rFoot"));

    #ifdef ENABLE_EXOSHIN

            osimModel->addBody(&bodySet->get("rShin"));

        #ifdef ENABLE_EXOTHIGH

                osimModel->addBody(&bodySet->get("rThigh"));

                #ifdef ENABLE_EXOHIP

                    osimModel->addBody(&bodySet->get("exoHip"));

                #endif

        #endif //ENABLE_EXOTHIGH

    #endif //ENABLE_EXOSHIN

#endif //ENABLE_EXOFOOT



}

/**
 * @brief ExoskeletonBody::createExoskeleton: calls createJoints method.
 */
void ExoskeletonBody::createExoskeleton()
{
    createJoints();
}

/**
 * @brief ExoskeletonBody::defineExoCoordinates: define the coordinates names
 * and ranges.
 */
void ExoskeletonBody::defineExoCoordinates()
{

    CoordinateSet& coordinates = osimModel->updCoordinateSet();

#if defined(ENABLE_EXOFOOT) && defined(ENABLE_EXOSHIN)

    double exoHipRange[2] = EXOHIP_RANGE;
    coordinates[4].setName("ankleJoint_r_coord");
    coordinates[4].setRange(exoHipRange);

    #ifdef ENABLE_EXOTHIGH

        double exoKneeRange[2] = EXOKNEE_RANGE;
        coordinates[5].setName("kneeJoint_r_coord");
        coordinates[5].setRange(exoKneeRange);

        #ifdef ENABLE_EXOHIP

            double exoAnkleRange[2] = EXOANKLE_RANGE;
            coordinates[6].setName("exoHipJoint_coord");
            coordinates[6].setRange(exoAnkleRange);

        #endif

    #endif // ENABLE_EXOTHIGH

#endif // ENABLE_EXOSHIN

}
/**
 * @brief ExoskeletonBody::defineInitialPosition: define the initial position of
 * the joints.
 * @param s: the state of the sistem.
 */

void ExoskeletonBody::defineInitialPosition(SimTK::State& s)
{

#if defined(ENABLE_EXOFOOT) && defined(ENABLE_EXOSHIN)

    CoordinateSet& coordinates = osimModel->updCoordinateSet();
    coordinates.get("ankleJoint_r_coord").setValue(s, EXOANKLE_INITIALPOS, true);

    #ifdef ENABLE_EXOTHIGH

        coordinates.get("kneeJoint_r_coord").setValue(s, EXOKNEE_INITIALPOS, true);

        #ifdef ENABLE_EXOHIP

            coordinates.get("exoHipJoint_coord").setValue(s, EXOHIP_INITIALPOS, true);

        #endif // ENABLE_EXOHIP

    #endif // ENABLE_EXOTHIGH

#endif // ENABLE_EXOFOOT && ENABLE_EXOSHIN

}


#ifdef ENABLE_BUSHINGFORCES // Enables all the bushing forces

/**
 * @brief ExoskeletonBody::defineBushingForces: create all the bushing forces are
 * going to interact between the exoskeleton and the body.
 * @param bushingForces: vector of type forces
 */

void ExoskeletonBody::defineBushingForces(OpenSim::Set<OpenSim::Force>* bushingForces)
{

#if defined(ENABLE_EXOFOOT) && defined(ENABLE_EXOSHIN)

    Vec3 transStiffness(100000), rotStiffness(10000), transDamping(0.1),
            rotDamping(0);
    Vec3 tibiaLoc1(-0.01, -0.45, 0.05);
    Vec3 tibiaOrient(0), ortAnklLoc1(0.04, 0.045, -0.04), ortAnkOrient(0);
    // Contact forces definitions
    BushingForce tibiaBushingForce1 ("tibia_r",      // Name of body part
                                     tibiaLoc1,      // Location
                                     tibiaOrient,    // Orientation
                                     "rShin",        // Name of exobody part
                                     ortAnklLoc1,    // Exobody's location
                                     ortAnkOrient,   // Exobody orientation
                                     transStiffness, // Translation stiffness
                                     rotStiffness,   // Rotation stiffness
                                     transDamping,   // Translation damping
                                     rotDamping);    // Orientation damping
    tibiaBushingForce1.setName("tibiaBushingForce1");// Set the name of bushing force

    Vec3 tibiaLoc2(-0.01, -0.11, 0.05), ortAnklLoc2(0.041, 0.389, -0.038);
    BushingForce tibiaBushingForce2 ("tibia_r",
                                      tibiaLoc2,
                                      tibiaOrient,
                                      "rShin",
                                      ortAnklLoc2,
                                      ortAnkOrient,
                                      transStiffness,
                                      rotStiffness,
                                      transDamping,
                                      rotDamping);
    tibiaBushingForce2.setName("tibiaBushingForce2");

    bushingForces->insert(bushingForces->getSize(),tibiaBushingForce1); // Save the force to the vector
    bushingForces->insert(bushingForces->getSize(),tibiaBushingForce2);

    osimModel->addForce(&bushingForces->get("tibiaBushingForce1")); // Add vector's force to the model
    osimModel->addForce(&bushingForces->get("tibiaBushingForce2"));

#ifdef ENABLE_EXOTHIGH

    Vec3 thighLoc1(0.005, -0.4, 0.065), thighOrient(0),
                    exoThighLoc1(0.025, 0.084, -0.04), exoThighOrient(0);
    BushingForce thighBushingForce1 ("femur_r",
                                   thighLoc1,
                                   thighOrient,
                                   "rThigh",
                                   exoThighLoc1,
                                   exoThighOrient,
                                   transStiffness,
                                   rotStiffness,
                                   transDamping,
                                   rotDamping);
    thighBushingForce1.setName("thighBushingForce1");

    Vec3 thighLoc2(0.005, -0.05, 0.065), exoThighLoc2(0.064, 0.431, -0.0345);
    BushingForce thighBushingForce2 ("femur_r",
                                   thighLoc2,
                                   thighOrient,
                                   "rThigh",
                                   exoThighLoc2,
                                   exoThighOrient,
                                   transStiffness,
                                   rotStiffness,
                                   transDamping,
                                   rotDamping);
    thighBushingForce2.setName("thighBushingForce2");

    bushingForces->insert(bushingForces->getSize(),thighBushingForce1);
    bushingForces->insert(bushingForces->getSize(),thighBushingForce2);

    osimModel->addForce(&bushingForces->get("thighBushingForce1"));
    osimModel->addForce(&bushingForces->get("thighBushingForce2"));

#ifdef ENABLE_EXOHIP

    Vec3 pelvisLoc1(-0.1, 0.03, 0.12), exoHipLoc1(-0.034, 0.108, -0.059),
        pelvisOr(0), exoHipOr(0);
    BushingForce hipBushingForce1("pelvis",
                                  pelvisLoc1,
                                  pelvisOr,
                                  "exoHip",
                                  exoHipLoc1,
                                  exoHipOr,
                                  transStiffness,
                                  rotStiffness,
                                  transDamping,
                                  rotDamping);
    hipBushingForce1.setName("hipBushingForce1");

    Vec3 pelvisLoc2(-0.08, 0.02, -0.1), exoHipLoc2(-0.034, 0.108, -0.28);
    BushingForce hipBushingForce2 ("pelvis",
                                   pelvisLoc2,
                                   pelvisOr,
                                   "exoHip",
                                   exoHipLoc1,
                                   exoHipOr,
                                   transStiffness,
                                   rotStiffness,
                                   transDamping,
                                   rotDamping);
    hipBushingForce2.setName("hipBushingForce2");

    bushingForces->insert(bushingForces->getSize(),hipBushingForce1);
    bushingForces->insert(bushingForces->getSize(),hipBushingForce2);

    osimModel->addForce(&bushingForces->get("hipBushingForce1"));
  //  osimModel->addForce(&bushingForces->get("hipBushingForce2"));

#endif // ENABLE_EXOHIP

#endif // ENABLE_EXOTHIGH

#endif // ENABLE_EXOFOOT && ENABLE_EXOSHIN

}

#endif // ENABLE_BUSHINGFORCES

/**
 * @brief ExoskeletonBody::defineFrictionForces: define joints frictions.
 */

void ExoskeletonBody::defineFrictionForces()
{

#if defined(ENABLE_EXOFOOT) && defined(ENABLE_EXOSHIN)

    MobilizedBodyIndex mobIndex(8);
    const MobilizedBody& mobExoAnkle =
            osimModel->getMatterSubsystem().getMobilizedBody(mobIndex);

    MultibodySystem& system = osimModel->updMultibodySystem();
    GeneralForceSubsystem& forces = osimModel->updForceSubsystem();

    SimTK::Force::MobilityLinearDamper frictionAnkle(forces,
                                                     mobExoAnkle,
                                                     SimTK::MobilizerUIndex(0),
                                                     EXOANKLE_FRICTIONFORCE_VALUE);

#ifdef ENABLE_EXOTHIGH

    mobIndex++;
    const MobilizedBody& mobExoKnee =
            osimModel->getMatterSubsystem().getMobilizedBody(mobIndex);

    SimTK::Force::MobilityLinearDamper frictionKnee (forces,
                                                     mobExoKnee,
                                                     SimTK::MobilizerUIndex(0),
                                                     EXOKNEE_FRICTIONFORCE_VALUE);
#ifdef ENABLE_EXOHIP

    mobIndex++;
    const MobilizedBody& mobExoHip =
            osimModel->getMatterSubsystem().getMobilizedBody(mobIndex);
    SimTK::Force::MobilityLinearDamper frictionHip (forces,
                                                    mobExoHip,
                                                    SimTK::MobilizerUIndex(0),
                                                    EXOHIP_FRICTIONFORCE_VALUE);

#endif

#endif // ENABLE_EXOTHIGH

#endif // ENABLE_EXOFOOT && ENABLE_EXOSHIN

}

/**
 * @brief ExoskeletonBody::defineActuators: define joint force actuators
 * @param actuatorSet
 */

void ExoskeletonBody::defineActuators(OpenSim::Set<OpenSim::Actuator>* actuatorSet)
{
#if (defined(ENABLE_EXOHIP) && defined(ENABLE_EXOTHIGH))

    TorqueActuator* exoHipActuator_r = new TorqueActuator();
    exoHipActuator_r->setName("exoHipActuator_r");
    exoHipActuator_r->setBodyA(bodySet->get("exoHip"));
    exoHipActuator_r->setBodyB(bodySet->get("rThigh"));
    exoHipActuator_r->setOptimalForce(OPTIMAL_EXOHIP_TORQUE);
    exoHipActuator_r->setTorqueIsGlobal(false);

    actuatorSet->insert(actuatorSet->getSize(), *exoHipActuator_r);
    osimModel->addForce(&actuatorSet->get("exoHipActuator_r"));

#endif

#if (defined(ENABLE_EXOTHIGH) && defined(ENABLE_EXOSHIN))

    TorqueActuator* exoKneeActuator_r = new TorqueActuator();
    exoKneeActuator_r->setName("exoKneeActuator_r");
    exoKneeActuator_r->setBodyA(bodySet->get("rThigh"));
    exoKneeActuator_r->setBodyB(bodySet->get("rShin"));
    exoKneeActuator_r->setOptimalForce(OPTIMAL_EXOKNEE_TORQUE);
    exoKneeActuator_r->setTorqueIsGlobal(false);

    actuatorSet->insert(actuatorSet->getSize(), *exoKneeActuator_r);
    osimModel->addForce(&actuatorSet->get("exoKneeActuator_r"));

#endif

#if defined(ENABLE_EXOFOOT) && defined(ENABLE_EXOSHIN)

    TorqueActuator* exoAnkleActuator_r = new TorqueActuator();
    exoAnkleActuator_r->setName("exoAnkleActuator_r");
    exoAnkleActuator_r->setBodyA(bodySet->get("rFoot"));
    exoAnkleActuator_r->setBodyB(bodySet->get("rShin"));
    exoAnkleActuator_r->setOptimalForce(OPTIMAL_EXOANKLE_TORQUE);
    exoAnkleActuator_r->setTorqueIsGlobal(false);

    actuatorSet->insert(actuatorSet->getSize(), *exoAnkleActuator_r);
    osimModel->addForce(&actuatorSet->get("exoAnkleActuator_r"));

#endif
}
