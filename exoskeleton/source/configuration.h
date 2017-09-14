/**
 * This file contain all the #defines that allow to the user configurate
 * the exoskeleton. Some of the features that can be modified easily are
 * disable or enable friction forces, set gravity to 0, and modify the
 * number of exoskeleton parts that acts in the body.
 *
 */
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/// Uncomment defines to disable the properties

#include <OpenSim/OpenSim.h>


////////////
// MACROS //
////////////

#define degreesToRadians(angleDegrees) (angleDegrees * SimTK::Pi / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / SimTK::Pi)


//////////////////////
// EXO'S BODY PARTS //
//////////////////////

#define ENABLE_EXOFOOT
#define ENABLE_EXOSHIN
#define ENABLE_EXOTHIGH
#define ENABLE_EXOHIP


////////////
// FORCES //
////////////

// Gravity
#define ENABLE_GRAVITY // Enable gravity
#define GRAVITY_VALUE SimTK::Vec3(0.0, -9.8065, 0.0) // Set the gravity's value

// Bushing forces
#define ENABLE_BUSHINGFORCES // Enable all bushing forces

// Friction forces
#define ENABLE_FRICTIONFORCES // Enable all friction forces

// Set friction forces value
#define EXOHIP_FRICTIONFORCE_VALUE 5
#define EXOKNEE_FRICTIONFORCE_VALUE 5
#define EXOANKLE_FRICTIONFORCE_VALUE 5

///////////////
// ACTUATORS //
///////////////

// Set Optimal Torque
#define OPTIMAL_EXOHIP_TORQUE 20
#define OPTIMAL_EXOKNEE_TORQUE 20
#define OPTIMAL_EXOANKLE_TORQUE 20

/////////////////
// CONTROLLERS //
/////////////////

#define ENABLE_CONTROLLER // Enable controller

    // Set control parameter values
    #define KP_EXOHIP 50.0
    #define KV_EXOHIP 40.0
    #define KI_EXOHIP 120.0

    #define KP_EXOKNEE 50.0
    #define KV_EXOKNEE 40.0
    #define KI_EXOKNEE 120.0

    #define KP_EXOANKLE 5.0
    #define KV_EXOANKLE 10.0
    #define KI_EXOANKLE 10.0

    #define K_EXOHIP {KP_EXOHIP,KV_EXOHIP,KI_EXOANKLE}
    #define K_EXOKNEE {KP_EXOKNEE,KV_EXOKNEE, KI_EXOKNEE}
    #define K_EXOANKLE {KP_EXOANKLE,KV_EXOANKLE,KI_EXOANKLE}

////////////////////////////
// EXOSKELETON PARAMETERS //
////////////////////////////

// Set Exo hip's ranges min and max in degrees
#define EXOHIP_MINRANGE -20 // degrees
#define EXOHIP_MAXRANGE 100 // degrees

// Set Exo Knees ranges min and max in degrees
#define EXOKNEE_MINRANGE -5
#define EXOKNEE_MAXRANGE 100

// Set Exo ankles ranges min and max in degrees
#define EXOANKLE_MINRANGE -15
#define EXOANKLE_MAXRANGE 20

#define EXOHIP_RANGE {degreesToRadians(EXOHIP_MINRANGE), degreesToRadians(EXOHIP_MAXRANGE)}
#define EXOKNEE_RANGE {degreesToRadians(EXOKNEE_MINRANGE), degreesToRadians(EXOKNEE_MAXRANGE)}
#define EXOANKLE_RANGE {degreesToRadians(EXOANKLE_MINRANGE), degreesToRadians(EXOANKLE_MAXRANGE)}

// ExoHip: Range -20 to 100 º, torque -+ 40(Nm)
// ExoKnee: Range -5 to 100 º, torque -+ 40(Nm)
// ExoAnkle: Range -15 to 20 º, torque -+ 40(Nm)

// Initial Position of each joint
#define EXOANKLE_INITIALPOS degreesToRadians(0)
#define EXOKNEE_INITIALPOS degreesToRadians(37.864)
#define EXOHIP_INITIALPOS degreesToRadians(-16.689)

#endif //CONFIGURATION_H
