/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

ur5_control.cpp

Description
===========

Interface for controlling the ur5 arm. Declares configuration and pose constants for performance test, 
pose conversion helper functions, and control functions.

Code Citations
==============

Note: some comments that appear in this file were made by the original author of the copied or adapted functions, S. Legowik.

setPosition(), cart2rob(), pm2robotConvert(), robot2pmConvert(), rob2cart(), time_since(int seconds), get_seconds(),
copied from mobmanmain.cpp by S. Legowik

bisect(), accSearch(), runTargetScan() copied from scanUtils.cpp by S. Legowik

compute_start(), spiral_search(), square(), circle(), square_bisect(), circle_bisect(), square_bisect_short() are adapted
from scanUtils.cpp by S. Legowik

Adapted from:
mobmanmain.cpp by S. Legowik
scanUtils.cpp by S. Legowik

References
==========


*/

#ifndef UR5_CONTROL_H
#define UR5_CONTROL_H

#include "AssemblyPrims.h"
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "lynx_msg.h"
#include "posemath.h"

using namespace std;
using namespace crpi_robot;
using namespace MotionPrims;

//ARCL MESSAGE CONSTANTS
//The following constants are messages sent from Lynx Core. Used to determine when to start robot arm searches.
	const char square_goal1[] = "Arrived at Goal3";
	const char square_goal2[] = "Arrived at Goal5";

	const char square_route1[] = "Finished patrolling route Route-Goal3";
	const char square_route2[] = "Finished patrolling route Route-Goal5";
	const char stage_route[] = "Finished patrolling route Route-Stage";
	const char dock_route[] = "Finished patrolling route Route-DockHelp";
	
//CONFIGURATION AND POSE CONSTANTs
	const bool use_new_bisection_algorithm = false;//Disabled to improve stability of laser signal.
	
	const double sensor_rot[3] = { 180, 0, 0 };//Rotation for TCP
	const double sensor_height = 297;
	const PM_POSE lynx_to_ur5 = PM_POSE(PM_CARTESIAN(0, 0, 0), PM_RPY(0, 0, 0));//Vehicle to Manipulator base offset, currently set to zero

////////////////////////////////////////////////////////////////////////////////////////
//BISECTION PERFORMANCE TEST CONSTANTS
//Commanded lynx position for each goal point
	const PM_POSE square_lynx_goal5 = PM_POSE(PM_CARTESIAN(7645, -1500, 0), PM_RPY(0, 0, -90 * TO_RAD));
	const PM_POSE square_lynx_goal3 = PM_POSE(PM_CARTESIAN(6120, -1400, 0), PM_RPY(0, 0, 90 * TO_RAD));
// X,Y position read from pendant when positioned by pendant over fiducial
//Used for initial bisect reflector search position
//Goal 3
	const PM_CARTESIAN large_square_point1_goal3(260.617, -627.419, sensor_height);
	const PM_CARTESIAN large_square_point2_goal3(-196.697, -636.896, sensor_height);

//Goal 5
	const PM_CARTESIAN large_square_point1_goal5(280.777, -575.722, sensor_height);
	const PM_CARTESIAN large_square_point2_goal5(-176.629, -582.186, sensor_height);
///////////////////////////////////////////////////////////////////////////////////////////

//Test Constants
	const PM_CARTESIAN large_square_point1(200.557, -612.383, sensor_height);
	const PM_CARTESIAN large_square_point2(-245.635, -555.005, sensor_height);
	const PM_POSE lynx_pose_test = PM_POSE(PM_CARTESIAN(1350, -418, 0), PM_RPY(0, 0, 0 * TO_RAD));
	const PM_POSE circle_lynx = PM_POSE(PM_CARTESIAN(1000, 0, 0), PM_RPY(0, 0, 0));//90*TO_RAD));
	const PM_CARTESIAN square_point1_ur5_performance(265.958, -625.264, sensor_height);
	const PM_CARTESIAN square_point2_ur5_performance(-193.242, -625.264, sensor_height);
	// X,Y position read from pendant when positioned by pendant over fiducial
	const PM_CARTESIAN square_point1_ur5(296.389, -658.664, sensor_height);
	const PM_CARTESIAN square_point2_ur5(-162.028, -658.641, sensor_height);
	const PM_CARTESIAN circle_point1_ur5 = PM_CARTESIAN(-501.241, -420.128, sensor_height);
	const PM_CARTESIAN circle_point2_ur5 = PM_CARTESIAN(0, -671.179, sensor_height);
	//Offset points used to test spiral search
	const PM_CARTESIAN square_point1_ur5_offset(252.588, -362.225, sensor_height);
	const PM_CARTESIAN square_point2_ur5_offset(-218.187, -370.521, sensor_height);
	const PM_CARTESIAN circle_point1_ur5_offset(-307.997, -402.743, sensor_height);
	const PM_CARTESIAN circle_point2_ur5_offset(-331.737, -705.825, sensor_height);

//ARM STOW AND STAGE CONSTANTS
	const PM_CARTESIAN stow(48.7, 175, 100); //stow point

	//Stage points
	//These constants are used as a path to safely guide the robot arm to the stow pose
	//1. If move straight to commands are applied in numerical order, robot is assumed to be at stow pose and executing the moves will 
	//provide a safe path to a point whereby the arm can access the RMMA
	//2. If move straight to commands are executed in reverse order, plus the stage point at the end, robot is assumed to be at some arbitrary point over the RMMA and executing the moves will safely stow the robot. 

	const PM_CARTESIAN stage_1(97.4, 350, 100);
	const PM_CARTESIAN stage_2(380, 350, sensor_height);
	const PM_CARTESIAN stage_3(380, -350, sensor_height);
	const PM_CARTESIAN stage_4(0, -350, sensor_height);

// Spiral search parameters
	const double searchStep = 0.5; // millimeters - small target
	const double largeTargetStepSize = 0.5; // millimeters
	const double searchRadius = 75.0; // millimeters
	const double searchTimeout = 1400.0; // seconds
	const int numIters = 2; // Number of loops around target  

// Delays
	const int bisectPauseMs = 1000; // delay before performing bisection
	const int robot_settle_time = 100; // delay in microseconds between move and measurement
	const int large_robot_settle_time = 250;
	const int lynx_settle_time = 5000; // delay to let AGV nav settle, (milliseconds)


//CONVERSION FUNCTIONS
	void setPosition(robotPose &pose, const PM_CARTESIAN point, const double rot[]);//Conversion function from PM_CARTESIAN to CRPI RobotPose
	robotPose cart2rob(PM_CARTESIAN position, robotPose orientation);//Conversion function from PM_CARTESIAN to CRPI RobotPose
	robotPose pm2robotConvert(PM_POSE pm);//Conversion function from PM_POSE to CRPI RobotPose
	PM_POSE robot2pmConvert(robotPose robot);//Conversion function from CRPI robotPose to PM_POSE
	PM_CARTESIAN rob2cart(robotPose pose);//Conversion function from CRPI robotPose to PM_CARTESIAN
	void compute_start(PM_CARTESIAN& point1_world, PM_CARTESIAN& point2_world, PM_CARTESIAN point1_ur5, PM_CARTESIAN point2_ur5, PM_POSE rmma_to_lynx, PM_POSE lynx_pose);//Compute location of reflectors based on actual lynx position
	double time_since(int seconds);//Compute the time elapsed since a given start time.
	int get_seconds();//Get UNIX timestamp from system clock

//CONTROL FUNCTIONS
	CanonReturn stage_arm(CrpiRobot<CrpiUniversal> * ur_robot);//Stage arm so it can access the RMMA
	CanonReturn stow_arm(CrpiRobot<CrpiUniversal> * ur_robot);//Stow arm on vehicle when Lynx is in motion.
	CanonReturn spiral_search_large(CrpiRobot<CrpiUniversal> * ur_robot, robotPose rmma_point, robotPose& pose_me, ofstream& out_file, int mytime_seconds);//Coarse Spiral Search used to localize bisect reflectors.
	CanonReturn spiral_search(CrpiRobot<CrpiUniversal> * ur_robot, robotPose rmma_point, robotPose& pose_me, ofstream& out_file, int mytime_seconds);//Fine Spiral Search used to localize small reflectors
	CanonReturn square(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1, PM_CARTESIAN start_point2, int num_iters, int mytime_seconds);//Executes multiple spiral searches to make a square pattern of multiple reflectors
	CanonReturn circle(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1, PM_CARTESIAN start_point2, int num_iters, int mytime_seconds);//Executes multiple spiral searches to make a circle pattern of multiple reflectors
	CanonReturn square_bisect(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds);//Executes multiple spiral searches to make a square pattern of multiple reflectors, uses bisection registration first to locate other reflectors
	CanonReturn square_bisect_short(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds, int goal);//Executes multiple spiral searches to make a circle pattern of multiple reflectors, uses bisection registration first to locate other reflectors
	bool bisect(CrpiRobot<CrpiUniversal> *arm, robotPose &pose, double stepSize, int *stepCount, double *searchTime);//Old Bisect registration algorithm
	void runTargetScan(CrpiRobot<CrpiUniversal> *arm, string filename, double stepSize, robotPose center);//Raster scan, ported from Steve Legowik's code, but is unused
	bool accSearch(CrpiRobot<CrpiUniversal> *arm, robotPose &pose, bool up, bool xAxis, double stepSize, int &count);//New bisect algorithm, not used as it resulted in laser signal stability problems
#endif