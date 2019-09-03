/* This file contains configuration values for the MobileManipulator program.
These values are used to control how the program executes and provide
default values for the program.
*/
#ifndef __INC_CONFIG_H__
#define __INC_CONFIG_H__

#include "posemath.h"

// software configuration parameters
const bool commThreadVerbose = true;
const bool useManualContinue = false;
const bool useBisection = false;      // on index fiducials
const bool useLargeTargets = true;    // bisect large reflectors
const bool useNewBisectionAlgorithm = true;  // use new faster bisection 

// learn and reuse docking position specific offsets
// Next pair work to use last parameters from registration
// If recordTargetOffsets is true, will reuse RAM parameters in same run
// Only one can be true
const bool recordTargetOffsets = false;     // record target offsets for future reference
const bool useTargetOffsets = false;        // load target offsets from file and use
// If within tDT and tAT then have been here before as docking location (AGV)
// New position for recording data. AGV usually good to 10 mm
const double targetDistanceThreshold = 50.0;  // distance in mm
const double targetAngleThreshold = 5.0;  // angle in degrees
#define targetOffsetsFile "targetOffsets.csv"

// average multiple agv positions  
// number to average from AGV navigation to compute AGV position
// Samples at 16 hz - can go up to 100 to 200 if needed, but is slow
const int agvPositionSamples = 10;  // number of samples to average from AGV

// Scan large reflector boundaries
// Don't change, for specific test
// Maps entire reflector, very slow
const bool scanTargets = false;
const double scanTargetStep = 0.5;

// AR toolkit
const bool useARToolkit = false;

// Spiral search parameters
const double searchStep = 0.5; // millimeters - small target
const double largeTargetStepSize = 0.5; // millimeters
const double searchRadius = 75.0; // millimeters
const double searchTimeout = 1400.0; // seconds
// Number of loops around target  
const int numIters = 2;
// Delay for data collection with Opitrack
const int bisectPauseMs = 100; // delay before performing bisection
const int robotSettleTime = 100; // delay in microseconds between move and measurement
const int agvSettleTime = 5000; // delay to let AGV nav settle, (milliseconds)

// Set sensor orientation
// Rotation first, translation second
// Base to tool transform  
const double sensorRot[3] = { 180.0, 0.0, 90.0 };
//double sensorHeight = 293;
// Height above base (close to RMMA)
const double camMount = 35;
const double sensorHeight = 293 + camMount; // Height with camera mount

// AR version config values
// Only used in AR version of code
// override standoff and orientation specified by AR system
const bool force_align = false;
 // use offset between initial points to find second point rather than AR
const bool use_offset = false; 

// AR camera offsets from laser
const double cameraOffsetX = 73.916;
const double cameraOffsetY = 21.175;
const double cameraOffSetZ = 0;
// RCS library coordinate transformation
//  RPY roll/pitch/yaw
// PM is Pose Map library 
// Conversion routines in main between robot and PM poses
const PM_POSE cam_offset( PM_CARTESIAN(cameraOffsetY, cameraOffsetX, 0), PM_RPY() );

// World position of first two fiducials
// World coordinate system is room
// constants for static test
const PM_POSE squareAgvPose = PM_POSE(PM_CARTESIAN(6308, 13000, 0), PM_RPY(0, 0, 90.08 * TO_RAD));
// X,Y position read from pendant when positioned by pendant over fiducial
const PM_CARTESIAN squarePoint1Robot = PM_CARTESIAN(-645.3, -192.1, sensorHeight );
const PM_CARTESIAN squarePoint2Robot = PM_CARTESIAN(-637.5, 267.4, sensorHeight );

const PM_POSE circleAgvPose = PM_POSE(PM_CARTESIAN(6313, 12493, 0), PM_RPY(0, 0, 90.09 * TO_RAD));
const PM_CARTESIAN circlePoint1Robot = PM_CARTESIAN(-993.6, 71.2, sensorHeight );
const PM_CARTESIAN circlePoint2Robot = PM_CARTESIAN(-688.4, 66.4, sensorHeight );

// robot base offset
//const PM_POSE agvToRobot = PM_POSE(PM_CARTESIAN(831.5, -7.5, 1240), PM_RPY(0, 0, 90.60 * TO_RAD));
//const PM_POSE agvToRobot = PM_POSE(PM_CARTESIAN(834.253, -8.16409, 1240), PM_RPY(0, 0, 90.3676 * TO_RAD)); // from 2/24 data
const PM_POSE agvToRobot = PM_POSE(PM_CARTESIAN(832.63, -10.6548, 1240), PM_RPY(0, 0, 90.5535 * TO_RAD)); // from 3/15 data

// constants for stowing the arm
const bool stowArm = true;
const PM_CARTESIAN stowPoint( 20, 670, sensorHeight );
const PM_CARTESIAN stagePoint1( 440, 0, sensorHeight );
const PM_CARTESIAN stagePoint2( 173, -670, sensorHeight );

//// large reflector positions
//const PM_POSE largeSquareAgvPose = PM_POSE(PM_CARTESIAN(8070, 12999, 0), PM_RPY(0, 0, 90.37 * TO_RAD));
//const PM_CARTESIAN largeSquarePoint1Robot = PM_CARTESIAN(900, -205, 293);
//const PM_CARTESIAN largeSquarePoint2Robot = PM_CARTESIAN(908, 250, 293);
//
//const PM_POSE largeCircleAgvPose = PM_POSE(PM_CARTESIAN(8070, 12999, 0), PM_RPY(0, 0, 90.37 * TO_RAD));
//const PM_CARTESIAN largeCirclePoint1Robot = PM_CARTESIAN(912,403, 293);
//const PM_CARTESIAN largeCirclePoint2Robot = PM_CARTESIAN(916, 708, 293);

// large reflector positions
const PM_POSE largeSquareAgvPose = PM_POSE(PM_CARTESIAN(6092,14917, 0), PM_RPY(0, 0, 314.65 * TO_RAD));
const PM_CARTESIAN largeSquarePoint1Robot = PM_CARTESIAN(136.72, -559.10, sensorHeight );
const PM_CARTESIAN largeSquarePoint2Robot = PM_CARTESIAN(-190.23, -879.53, sensorHeight );

const PM_POSE largeCircleAgvPose = PM_POSE(PM_CARTESIAN(7151, 11809, 0), PM_RPY(0, 0, 90.14 * TO_RAD));
const PM_CARTESIAN largeCirclePoint1Robot = PM_CARTESIAN(-10.90, -768.67, sensorHeight );
const PM_CARTESIAN largeCirclePoint2Robot = PM_CARTESIAN(-6.14, -463.91, sensorHeight );

#endif // __INC_CONFIG_H__
