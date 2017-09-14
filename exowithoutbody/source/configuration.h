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

/******************************************************************************/
//////////////////////
// EXO'S BODY PARTS //
//////////////////////

#define ENABLE_EXOFOOT
#define ENABLE_EXOSHIN
#define ENABLE_EXOTHIGH
#define ENABLE_EXOPELVIS

/******************************************************************************/
////////////////////////////
// EXOSKELETON PARAMETERS //
////////////////////////////

// Set Masses
#define EXOPELVIS_MASS 0.85308774   // kg
#define EXOTHIGH_MASS 0.2138295     // kg
#define EXOSHIN_MASS 0.19567251     // kg
#define EXOFOOT_MASS 0.510668       // kg

// Set Mass Center                             X          Y           Z
#define EXOPELVIS_MASS_CENTER SimTK::Vec3(0.0567373, 0.0433811, -0.00018222)// m
#define EXOTHIGH_MASS_CENTER SimTK::Vec3(0.057235, 0.142446, -0.009)        // m
#define EXOSHIN_MASS_CENTER SimTK::Vec3(-0.009493, 0.13976, 0.00998)        // m
#define EXOFOOT_MASS_CENTER SimTK::Vec3(0.067394, -0.058410, -0.063129)     // m

// Set Inertia
//                                            xx          yy          zz        \
//                                            xy          xz          yz
#define EXOPELVIS_INERTIA SimTK::Inertia(0.33744544, 0.00852229, 0.33785791,    \
                                        -0.02713561, 0.00250198,-0.03328571)
#define EXOTHIGH_INERTIA SimTK::Inertia(0.05890566, 0.00320362, 0.0565293,      \
                                        0.00229535, 0.00058312, -0.01209693)
#define EXOSHIN_INERTIA SimTK::Inertia(0.05890566, 0.00320362, 0.0565293,        \
                                       0.00229535, 0.00058312, -0.01209693)
#define EXOFOOT_INERTIA SimTK::Inertia(0.33744544, 0.00852229, 0.33785791,       \
                                       -0.02713561, 0.00250198,-0.03328571)

// Scale the Body parts
#define EXOPELVIS_SCALE SimTK::Vec3(1.38842608, 1.391052749, 1.3)
#define EXOTHIGH_SCALE SimTK::Vec3(1.3884260802, 1.2178638159, 1.3)
#define EXOSHIN_SCALE SimTK::Vec3(1.1884260802, 1.3434147124, 1.3)
#define EXOFOOT_SCALE SimTK::Vec3(1.1884260802, 1.2782330916, 1.3)

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

/******************************************************************************/
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
#define EXOHIP_FRICTIONFORCE_VALUE 0
#define EXOKNEE_FRICTIONFORCE_VALUE 0
#define EXOANKLE_FRICTIONFORCE_VALUE 0

/******************************************************************************/
/////////////////////////////
// INITIAL JOINT POSITIONS //
/////////////////////////////
#define EXOHIP_INITPOS_DEGREES 0
#define EXOKNEE_INITPOS_DEGREES 0
#define EXOANKLE_INITPOS_DEGREES 0

#define EXOHIP_IPOS degreesToRadians(EXOHIP_INITPOS_DEGREES)
#define EXOKNEE_IPOS degreesToRadians(EXOKNEE_INITPOS_DEGREES)
#define EXOANKLE_IPOS degreesToRadians(EXOANKLE_INITPOS_DEGREES)
/******************************************************************************/
///////////////
// ACTUATORS //
///////////////

// Set Optimal Torque
#define OPTIMAL_EXOHIP_TORQUE 20
#define OPTIMAL_EXOKNEE_TORQUE 20
#define OPTIMAL_EXOANKLE_TORQUE 20
/******************************************************************************/
/////////////////
// CONTROLLERS //
/////////////////

#define ENABLE_CONTROLLER // Enable controller

// Set control parameter values
#define KP_EXOHIP 1.25
#define KV_EXOHIP 5.0
#define KI_EXOHIP 4.0

#define KP_EXOKNEE 0.75
#define KV_EXOKNEE 5.0
#define KI_EXOKNEE 4.0

#define KP_EXOANKLE 0.5
#define KV_EXOANKLE 5.0
#define KI_EXOANKLE 4.0

#define K_EXOHIP {KP_EXOHIP,KV_EXOHIP,KI_EXOANKLE}
#define K_EXOKNEE {KP_EXOKNEE,KV_EXOKNEE, KI_EXOKNEE}
#define K_EXOANKLE {KP_EXOANKLE,KV_EXOANKLE,KI_EXOANKLE}
/******************************************************************************/


#endif //CONFIGURATION_H
