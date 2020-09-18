/**
*\file ur5_control.h
*\brief Interface for controlling the ur5 arm. Declares configuration and pose constants for performance test, 
*pose conversion helper functions, and control functions. \n\n
*
*Note: some comments that appear in this file were made by the original author of the copied or adapted functions, S. Legowik. \n
*
*setPosition(), cart2rob(), pm2robotConvert(), robot2pmConvert(), rob2cart(), time_since(int seconds), get_seconds(),
*copied from mobmanmain.cpp by S. Legowik \n
*
*bisect(), accSearch(), runTargetScan() copied from scanUtils.cpp by S. Legowik \n
*
*compute_start(), spiral_search(), square(), circle(), square_bisect(), circle_bisect(), square_bisect_short() are adapted \n
*from scanUtils.cpp by S. Legowik \n\n
*
*Adapted from:\n
*mobmanmain.cpp by S. Legowik\n
*scanUtils.cpp by S. Legowik\n
*
\author Omar Aboul-Enein
\date 2017-05-05
*/

#ifndef UR5_CONTROL_H
#define UR5_CONTROL_H

#include "AssemblyPrims.h"
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ld_msg.h"
#include "posemath.h"

using namespace std;
using namespace crpi_robot;
using namespace MotionPrims;


//ARCL Message Constants: 

	/**
	*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
	*/
	const char square_goal1[] = "Arrived at Goal3";

	/**
	*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
	*/
	const char square_goal2[] = "Arrived at Goal5";

	/**
	*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
	*/
	const char square_route1[] = "Finished patrolling route Route-Goal3";

	/**
	*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
	*/
	const char square_route2[] = "Finished patrolling route Route-Goal5";

	/**
	*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
	*/
	const char stage_route[] = "Finished patrolling route Route-Stage";

	/**
	*Message sent from ld Core. Used to determine when to start robot arm search. From old mobile robot code and is not currently used.
	*/
	const char dock_route[] = "Finished patrolling route Route-DockHelp";
	
//Configuration and pose constants: 

	/**
	*Boolean used to enable the use of the faster bisect search algorithm. Disabled to improve stability of laser signal.
	*/
	const bool use_new_bisection_algorithm = false;
	
	/**
	*Rotation for TCP of manipulator.
	*/
	const double sensor_rot[3] = { 180, 0, 0 };

	/**
	*Constant sensor height used for all commanded poses. Adjusted to allow laser sensor to access RMMA.
	*/
	const double sensor_height = 297;
	//const double sensor_height = 100;

	/**
	*Vehicle to Manipulator base offset, currently set to zero
	*/
	const PM_POSE ld_to_ur5 = PM_POSE(PM_CARTESIAN(0, 0, 0), PM_RPY(0, 0, 0));

//Bisection test constants:
	/**
	*Commanded ld position for goal point
	*/
	const PM_POSE square_ld_goal5 = PM_POSE(PM_CARTESIAN(7645, -1500, 0), PM_RPY(0, 0, -90 * TO_RAD));
	
	/**
	*Commanded ld position for goal point
	*/
	const PM_POSE square_ld_goal3 = PM_POSE(PM_CARTESIAN(6120, -1400, 0), PM_RPY(0, 0, 90 * TO_RAD));

	/**
	*X,Y position read from pendant when positioned by pendant over fiducial
	*Used for initial bisect reflector search position for Goal 3
	*/
	const PM_CARTESIAN large_square_point1_goal3(464.375, 280.525, sensor_height);

	/**
	*X,Y position read from pendant when positioned by pendant over fiducial
	*Used for initial bisect reflector search position for Goal 3
	*/
	const PM_CARTESIAN large_square_point2_goal3(458.988, -178.663, sensor_height);

	/**
	*X,Y position read from pendant when positioned by pendant over fiducial
	*Used for initial bisect reflector search position for Goal 5
	*/
	const PM_CARTESIAN large_square_point1_goal5(280.777, -575.722, sensor_height);

	/**
	*X,Y position read from pendant when positioned by pendant over fiducial
	*Used for initial bisect reflector search position for Goal 5
	*/
	const PM_CARTESIAN large_square_point2_goal5(-176.629, -582.186, sensor_height);

//Additional test constants used for control code debugging. From old mobile robot code.
	const PM_CARTESIAN large_square_point1(464.375, 280.525, sensor_height);
	const PM_CARTESIAN large_square_point2(458.988, -178.663, sensor_height);
	const PM_POSE ld_pose_test = PM_POSE(PM_CARTESIAN(1350, -418, 0), PM_RPY(0, 0, 0 * TO_RAD));
	const PM_POSE circle_ld = PM_POSE(PM_CARTESIAN(1000, 0, 0), PM_RPY(0, 0, 0));//90*TO_RAD));
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

//Manipulator arm stage and stow constants:

	/**
	*Stow point for arm on payload structure.
	*/
	const PM_CARTESIAN stow(48.7, 175, 100);

	/**
	*Stage point\n
	*This constant is used as part of a path to safely guide the robot arm to the stow pose: \n
	*1. If move straight to commands are applied in numerical order, robot is assumed to be at stow pose and executing the moves will 
	*provide a safe path to a point whereby the arm can access the RMMA \n
	*2. If move straight to commands are executed in reverse order, plus the stage point at the end, robot is assumed to be at some arbitrary point over the RMMA and executing the moves will safely stow the robot. 
	*/
	const PM_CARTESIAN stage_1(97.4, 350, 100);

	/**
	*Stage point\n
	*This constant is used as part of a path to safely guide the robot arm to the stow pose: \n
	*1. If move straight to commands are applied in numerical order, robot is assumed to be at stow pose and executing the moves will
	*provide a safe path to a point whereby the arm can access the RMMA \n
	*2. If move straight to commands are executed in reverse order, plus the stage point at the end, robot is assumed to be at some arbitrary point over the RMMA and executing the moves will safely stow the robot.
	*/
	const PM_CARTESIAN stage_2(380, 350, sensor_height);

	/**
	*Stage point\n
	*This constant is used as part of a path to safely guide the robot arm to the stow pose: \n
	*1. If move straight to commands are applied in numerical order, robot is assumed to be at stow pose and executing the moves will
	*provide a safe path to a point whereby the arm can access the RMMA \n
	*2. If move straight to commands are executed in reverse order, plus the stage point at the end, robot is assumed to be at some arbitrary point over the RMMA and executing the moves will safely stow the robot.
	*/
	const PM_CARTESIAN stage_3(380, 0, sensor_height);

	//OLD Stage Poses for Lynx
	//const PM_CARTESIAN stage_3(380, -350, sensor_height);
	//const PM_CARTESIAN stage_4(0, -350, sensor_height);

	const PM_CARTESIAN pan_limit(249.078234, 0, sensor_height);

// Spiral search parameters:

	/**
	*Step size for square spiral pattern in millimeters. For small targets.
	*/
	const double searchStep = 0.5;

	/**
	*Step size for bisect pattern in millieters. For large targets.
	*/
	const double largeTargetStepSize = 0.5;

	/**
	*Radius of square spiral pattern in millimeters.
	*/
	const double searchRadius = 75.0;

	/**
	*Time elapsed before aborting spiral search in seconds.
	*/
	const double searchTimeout = 1400.0;

	/**
	*Number of verification steps (or loops around target).
	*/
	const int numIters = 1; 

// Delay constants:

	/**
	*Delay before performing bisection
	*/
	const int bisectPauseMs = 1000;

	/**
	*Delay in microseconds between move and measurement.
	*/
	const int robot_settle_time = 100;

	/**
	*Delay to allow robot motion to settle before attempting to position again.
	*/
	const int large_robot_settle_time = 250;

	/**
	*Delay to let AGV nav settle in milliseconds
	*/
	const int ld_settle_time = 5000;


//Conversion functions:

	/**
	*Conversion function from PM_CARTESIAN to CRPI RobotPose. From from mobmanmain by S. Legowik.
	*\param[out] pose	Pointer to a CRPI RobotPose object to hold the converted pose.
	*\param[in] point	X, y, z position of point to be converted expressed in NIST RCS 
	*\param[in] rot		Rotation of point to be converted
	*/
	void setPosition(robotPose &pose, const PM_CARTESIAN point, const double rot[]);
	
	/**
	*Conversion function from PM_CARTESIAN to CRPI RobotPose. From mobmanmain by S. Legowik.
	*\param[in] position		NIST RCS Library point to be converted. "Position is used to set the x and y coordinates (S. Legowik)".
	*\param[in] orientation		Orientation of TCP at point. Should be specified so that the TCP is facing towards the RMMA. "Orientation is used to set the z value and the rotation (S. Legowik)".
	*\returns					A CRPI RobotPose that contains the same position and oriantation as input parameters.
	*/
	robotPose cart2rob(PM_CARTESIAN position, robotPose orientation);

	/**
	*Conversion function from PM_POSE to CRPI RobotPose. From mobmanmain by S. Legowik
	*\param[in] pm		NIST RCS pose to be converted.
	*\return			Converted CRPI Robot pose.
	*/
	robotPose pm2robotConvert(PM_POSE pm);

	/**
	*Conversion function from CRPI robotPose to PM_POSE. From mobmanmain by S. Legowik
	*\param[in] robot	CRPI Robot Pose to be converted.
	*\return			NIST RCS pose.
	*/
	PM_POSE robot2pmConvert(robotPose robot);

	/**
	*Conversion function from CRPI robotPose to PM_CARTESIAN. From mobmanmain by S. Legowik
	*\param[in] pose	CRPI Robot pose to be converted
	*\return			Converted NIST RCS position.
	*/
	PM_CARTESIAN rob2cart(robotPose pose);
	
	/**
	*Transforms trained initial reflector points given in robot base coordinates \n
	*to ld coordinates using the commanded ld position, \n
	*then performs the inverse transformation using the actual ld position. \n
	*The final result corrects the expected reflector position based on position and orientation \n
	*readings from the lync controller. Adapted from mobmanmain by S. Legowik.
	*Note that ld_to_ur5 constant is used to factor in the mounting offset of the robot base, currently this pose is set to zero.
	*\param[out] point1_world	Computed position of first reflector. 
	*\param[out] point2_world	Computed position of second reflector.
	*\param[in] point1_ur5		Expected position of first reflector relative to manipulator base.
	*\param[in] point2_ur5		Expected position of second reflector relative to manipulator base
	*\param[in] rmma_to_ld		Expected position of RMMA dock point in vehicle map.
	*\param[in]	ld_pose			Actual position of ld in vehicle map.
	*/
	void compute_start(PM_CARTESIAN& point1_world, PM_CARTESIAN& point2_world, PM_CARTESIAN point1_ur5, PM_CARTESIAN point2_ur5, PM_POSE rmma_to_ld, PM_POSE ld_pose);

	/**
	*Transforms expected reflector points given in robot base coordinates \n
	*to ld coordinates using the commanded ld position, \n
	*then performs the inverse transformation using the actual ld position. \n
	*The final result corrects the expected reflector position based on position and orientation \n
	*readings from the lync controller. Adapted from mobmanmain by S. Legowik.
	*Note that ld_to_ur5 constant is used to factor in the mounting offset of the robot base, currently this pose is set to zero.
	*\param[out] point_world	Computed position of point in ur base coordinates.
	*\param[in] point_ur5		Expected position of first reflector relative to manipulator base.
	*\param[in] rmma_to_ld		Expected position of RMMA dock point in vehicle map.
	*\param[in]	ld_pose			Actual position of ld in vehicle map.
	*/
	void compute_start(PM_CARTESIAN& point_world, PM_CARTESIAN point_ur5, PM_POSE rmma_to_ld, PM_POSE ld_pose);

	/**
	*Compute the time elapsed since a given start time. From mobmanmain by S. Legowik
	*\param[in] seconds		Start time
	*\return				Elapsed time since start time.
	*/
	double time_since(int seconds);

	/**
	*Get seconds since January 1, 1970 (UNIX TIME) based on Windows System Libraries. From mobmanmainby S. Legowik
	*\return	Timestamp from system clock
	*/
	int get_seconds();

//Control functions:
	/**
	*Lowers the actuated feet mounted on the cart. Feet cannot be lowered again if they were lowered previously to avoid tipping the cart.
	*\param[in] ur_robot	CRPI object that stores configuration and connection parameters for robot arm.
	*\return				CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn lower_feet(CrpiRobot<CrpiUniversal> * ur_robot);

	/**
	*Raises the actuated feet mounted on the cart.
	*\param[in] ur_robot	CRPI object that stores configuration and connection parameters for robot arm.
	*\return				CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn raise_feet(CrpiRobot<CrpiUniversal> * ur_robot);

	/**
	*Moves the arm into a position whereby it can safely access the RMMA. \n
	*Uses a series of intermediate points to stage the arm. Stage points are stored \n
	*in ur5_control.h \n
	*Note: Robot arm must be within 5 mm of stow point to be safely staged.
	*\param[in] ur_robot	CRPI object that stores configuration and connection parameters for robot arm.
	*\return				CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn stage_arm(CrpiRobot<CrpiUniversal> * ur_robot);

	/**
	*Moves the robot arm into a position whereby the TCP is over the ld payload structure and \n
	*the ld can safely move when robot is placed in this position. Series of intermediate points used to establish\n
	*stow position.\n
	*Constants defined in ur5_control.h\n
	*Arm must not already be in stow position.\n
	*If the robot arm drifts from the stow position, and thus cannot be staged, manually jog the robot arm as if it is accessing the RMMA\n
	*Then execute this function to properly send the robot arm to the stow point within the tolerances needed for this control program.
	*\param[in] ur_robot	CRPI robot object that allows control of the manipulator over TCP/IP socket.
	*\return				CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn stow_arm(CrpiRobot<CrpiUniversal> * ur_robot);

	/**
	*This function implements the coarse spiral search used to localize the large reflectors. This code is identical to the spiral_search function,\n
	*but uses a larger step size for square spiral path.
	*\param[in] ur_robot			CRPI object that stores configuration and connection parameters for robot arm.
	*\param[in] rmma_point			CRPI robot pose that holds the initial search position of the robot arm.
	*\param[in] pose_me				CRPI pose that holds the result of the search after localizing the bisect reflector.
	*\param[in] out_file			Output file for logging the reseults of the search.
	*\param[in] mytime_seconds		Start time used for computing the time it took to complete the search.
	*\return						CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn spiral_search_large(CrpiRobot<CrpiUniversal> * ur_robot, robotPose rmma_point, robotPose& pose_me, ofstream& out_file, int mytime_seconds);

	/**
	*This function implements the fine spiral search used to localize the small reflectors. \n
	*This function is called multiple times in other functions to perform a full test around the square or circle reflector patterns.
	*\param[in] ur_robot			CRPI object that stores configuration and connection parameters for robot arm.
	*\param[in] pose_me				CRPI pose that holds the result of the search after localizing the bisect reflector.
	*\param[in] out_file			Output file for logging the reseults of the search
	*\param[in] mytime_seconds		Time seed used for computing the time it took to complete the search.
	*\return						CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn spiral_search(CrpiRobot<CrpiUniversal> * ur_robot, robotPose rmma_point, robotPose& pose_me, ofstream& out_file, int mytime_seconds);

	/**
	*This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a square shape.
	*After the localization of each reflector, the location of the other reflectors are updated.\n
	*NOTE: This function was intended for use on AGV/UR10 mobile manipulator and should not be used on the ld. This is because the ur5 arm cannot reach. \n
	*all of the reflectors that are specified in this search patttern. This code is useful, however to help develop new search patterns.
	*\param[in] ur_robot			CRPI object that stores configuration and connection parameters for robot arm.
	*\param[in] start_point1		Expected position of the first reflector used to compute the location of the rest of the square patern reflectors.
	*\param[in] start_point2		Expected position of the second reflector used to compute the location of the rest of the square patern reflectors.
	*\param[in] num_iters			Number of times to go around the square pattern
	*\param[in] mytime_seconds		Timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
	*\return						CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn square(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1, PM_CARTESIAN start_point2, int num_iters, int mytime_seconds);


	/**
	*This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a circular shape. \n
	*After the localization of each reflector, the location of the other reflectors are updated. \n
	*NOTE: This function was intended for use on AGV/UR10 mobile manipulator and should not be used on the ld. This is because the ur5 arm cannot reach \n
	*all of the reflectors that are specified in this search patttern. This code is useful, however to help develop new search patterns.
	*\param[in] ur_robot				CRPI object that stores configuration and connection parameters for robot arm.
	*\param[in] start_point1_large		Expected position of the first bisect reflectors used to compute the location of the rest of the square patern reflectors.
	*\param[in] start_point2_large		Expected position of the second bisect reflectors used to compute the location of the rest of the square patern reflectors.
	*\param[in] num_iters				Number of times to go around the square pattern
	*\param[in] mytime_seconds			Timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
	*/
	CanonReturn circle(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1, PM_CARTESIAN start_point2, int num_iters, int mytime_seconds);

	/**
	*This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a square shape.\n
	*After the localization of each reflector, the location of the other reflectors are updated. Bisection registration is also used to\n
	*further localize the start points of the square pattern.\n
	*NOTE: This function was intended for use on AGV/UR10 mobile manipulator and should not be used on the ld. This is because the ur5 arm cannot reach\n
	*all of the reflectors that are specified in this search patttern. This code is useful, however to help develop new search patterns.
	*\param[in] ur_robot				CRPI object that stores configuration and connection parameters for robot arm.
	*\param[in] start_point1_large		Expected position of the first bisect reflector used to compute the location of the rest of the square patern reflectors.
	*\param[in] start_point2_large		Expected position of the second bisect reflector used to compute the location of the rest of the square patern reflectors.
	*\param[in] num_iters				Number of times to go around the square pattern
	*\param[in] mytime_seconds			Timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
	*\return							CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn square_bisect(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds);


	/**
	*This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a square shape.\n
	*After the localization of each reflector, the location of the other reflectors are updated. Bisection registration is also used to\n
	*further localize the start points of the square pattern.\n
	*This function is modified for the ld/ur5 mobile manipulator, adjusting the location of the bisect reflectors to account for the limited reach of the ur5.\n
	*In addition, the function implements spiral_search_large to account for the larger uncertainty in the ld vehicle docking.\n
	*This is the recommended function to use when testing on the ld.\n
	*This overloaded version of the function also returns the actual position of the large reflectors so they can be updated in the configuration file.
	*\param[in] ur_robot				CRPI object that stores configuration and connection parameters for robot arm.
	*\param[in] start_point1_large		Expected position of the first bisect reflector used to compute the location of the rest of the square patern reflectors.
	*\param[in] start_point2_large		Expected position of the second bisect reflector used to compute the location of the rest of the square patern reflectors.
	*\param[in] num_iters				Number of times to go around the square pattern
	*\param[in] mytime_seconds			Timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
	*\param[in] goal					Goal name associated with the position that the ld is parked at. Primarily used for logging purposes.
	*\param[in] ots_pause				Flag to enable extra pauses for capturing OTS data.
	*\param[out] start_point1_large		Actual position of the first bisect reflector after registering.
	*\param[out] start_point2_large		Actual position of the second bisect reflector after registering.
	*\return							CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn square_bisect_short(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds, char* goal, bool ots_pause, PM_CARTESIAN* updated_point1, PM_CARTESIAN* updated_point2);

	/**
	*This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a square shape.\n
	*After the localization of each reflector, the location of the other reflectors are updated. Bisection registration is also used to\n
	*further localize the start points of the square pattern.\n
	*This function is modified for the ld/ur5 mobile manipulator, adjusting the location of the bisect reflectors to account for the limited reach of the ur5.\n
	*In addition, the function implements spiral_search_large to account for the larger uncertainty in the ld vehicle docking.\n
	*This is the recommended function to use when testing on the ld.
	*\param[in] ur_robot				CRPI object that stores configuration and connection parameters for robot arm.
	*\param[in] start_point1_large		Expected position of the first bisect reflector used to compute the location of the rest of the square patern reflectors.
	*\param[in] start_point2_large		Expected position of the second bisect reflector used to compute the location of the rest of the square patern reflectors.
	*\param[in] num_iters				Number of times to go around the square pattern
	*\param[in] mytime_seconds			Timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
	*\param[in] goal					Goal name associated with the position that the ld is parked at. Primarily used for logging purposes.
	*\param[in] ots_pause				Flag to enable extra pauses for capturing OTS data.
	*\return							CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn square_bisect_short(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds, char* goal, bool ots_pause);

	/**
	*This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a square shape.\n
	*After the localization of each reflector, the location of the other reflectors are updated. Edge registration is also used to\n
	*further localize the start points of the square pattern by searching for table corner.\n
	*This function is modified for the ld/ur5 mobile manipulator, adjusting the location of the bisect reflectors to account for the limited reach of the ur5.\n
	*In addition, the function implements spiral_search_large to account for the larger uncertainty in the ld vehicle docking.\n
	*This is the recommended function to use when testing on the ld.
	*\param[in] ur_robot				CRPI object that stores configuration and connection parameters for robot arm.
	*\param[in] start_point				Current pose of robot from which the registration begins.
	*\param[in] r3_start				Start position to search for third reference point (on
	*\param[in] max_dist				Threshold search distance used for panning.
	*\param[in] large_step			    Initial step size used for panning.
	*\param[in] r2_offset				Horizontal offset used when localizing the second reference point (on the long table edge).
	*\param[in] r3_offset				Horizontal offset used when localizing the third reference point (on the short table edge). Ununused in this function, will be removed.
	*\param[in] num_iters				Number of times to go around the square pattern
	*\param[in] mytime_seconds			Timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
	*\param[in] goal					Goal name associated with the position that the ld is parked at. Primarily used for logging purposes.
	*\param[in] ots_pause				Flag to enable extra pauses for capturing OTS data.
	*\return							CRPI canonical return that indicates if an error occurred while attempting to move the robot.
	*/
	CanonReturn square_edge_short(CrpiRobot<CrpiUniversal>* ur_robot, PM_CARTESIAN start_point, double r1_start, double r3_start, double max_dist, double large_step, double r2_offset, int num_iters, int mytime_seconds, char* goal, bool ots_pause);

	CanonReturn square_edge_short_cont(CrpiRobot<CrpiUniversal>* ur_robot, PM_CARTESIAN start_point, double r1_start, double r3_start, double max_dist, double large_step, double r2_offset, int num_iters, int mytime_seconds, char* goal, bool ots_pause);

	CanonReturn square_edge_short_cont2(CrpiRobot<CrpiUniversal>* ur_robot, PM_CARTESIAN start_point, double r3_start, double max_dist, double large_step, double r2_offset, int num_iters, int mytime_seconds, char* goal, bool ots_pause);

	/**
	*Stage function with non-blocking moves. Allows for detecting the tape as the robot arm is unstowing.
	*\param[in] ur_robot				CRPI object that stores configuration and connection parameters for robot arm.
	*\param[out] edgePose				Returns the pose of the robot upon detection of the edge tape if detected. Otherwise, returns the final robot stage position if tape not detected.
	*/
	CanonReturn stage_arm_edge(CrpiRobot<CrpiUniversal> * ur_robot, robotPose& edgePose);

	/**
	*Code and comments from scanUtils.cpp by S. Legowik:
	*Bisect the current target to find the center more accurately.
	*Detects target when dio[8] is false;
	*\param[in]  arm			Pointer to robot control data structure
	*\param[in]  pose			Returns pose of long table edge. Use X component to get the X component of table corner.
	*\param[in]  stepSize1		Initial large step size to used when incrementing along vertical pan. Used to speed up search.
	*\param[out] stepSize2		Initial smaller step size to used when incrementing along vertical pan. Used to more precisely detect location of table edge.
	*\param[out] stepCount		Stores number of steps taken in vertical pan.
	*\param[out] searchTime		Stores time it took to complete the vertical pan.
	*/
	bool vertical_pan(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_x, int* stepCount, double* searchTime);

	bool vertical_pan_cont(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_x, int* stepCount, double* searchTime);

	bool vertical_pan_cont(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_x, int* stepCount1, int* stepCount2, double* searchTime);

	bool vertical_pan_cont2(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_x, int* stepCount1, int* stepCount2, double* searchTime);
	bool horizontal_pan_cont2(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, robotPose r1, robotPose r2, double offset, double stepSize1, double stepSize2, double max_y, bool reverse, int* stepCount1, int* stepCount2, double* searchTime);
	/**
	*Code and comments from scanUtils.cpp by S. Legowik:
	*Bisect the current target to find the center more accurately.
	*Detects target when dio[8] is false;
	*\param[in]  arm			Pointer to robot control data structure
	*\param[in]  pose			Returns pose of short table edge. Use Y component to get the Y component of table corner.
	*\param[in]  stepSize1		Initial large step size to used when incrementing along horizontal pan. Used to speed up search.
	*\param[out] stepSize2		Initial smaller step size to used when incrementing along horiozontal pan. Used to more precisely detect location of table edge.
	*\param[out] stepCount		Stores number of steps taken in horizontal pan.
	*\param[out] searchTime		Stores time it took to complete the horizontal pan.
	*/
	bool horizontal_pan(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_y, bool reverse, int* stepCount, double* searchTime);

	bool horizontal_pan_cont(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_y, bool reverse, int* stepCount, double* searchTime);

	bool horizontal_pan_cont2(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double offset, double stepSize1, double stepSize2, double max_y, bool reverse, int* stepCount1, int* stepCount2, double* searchTime);
	bool horizontal_pan_cont2(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, robotPose r1, robotPose r2, double offset, double stepSize1, double stepSize2, double max_y, bool reverse, int* stepCount1, int* stepCount2, double* searchTime);

	/**
	*Code and comments from scanUtils.cpp by S. Legowik:
	*Bisect the current target to find the center more accurately.
	*Detects target when dio[8] is false;
	*\param[in]  arm				Pointer to robot control data structure
	*\param[in]  pose			Returns center pose of reflector.
	*\param[in]  stepSize		Step size to used when incrementing along bisect path.
	*\param[out] stepCount		Stores number of steps taken in bisect.
	*\param[out] searchTime		Stores time it took to complete the bisect.
	*/
	bool bisect(CrpiRobot<CrpiUniversal> *arm, robotPose &pose, double stepSize, int *stepCount, double *searchTime);

	/**
	*Raster scan function, not currently used, but ported from Steve Legowik's code for reference.
	*/
	void runTargetScan(CrpiRobot<CrpiUniversal> *arm, string filename, double stepSize, robotPose center);

	/**
	*Code and comments From scanUtils.cpp by S. Legowik: \n
	* Fast search bisection search.  Starts moving at step size \n
	*doubling the distance each time till the transition is detected. \n
	*Then it starts searching with decreasing step size. \n
	*Subroutine assumes that robot has been paused long enough \n
	*prior to call to allow for a different pause at beginning and \n
	*end of search. \n
	*Routine increments count by 1 per step.  Count should be reset \n
	*externally.

	*\param[in] arm		Pointer to robot control data structure
	*\param[in] pose	Returns center pose of reflector.
	*\param[in] up		True if serching in positive direction.
	*\param[in] xAxis	True if motion is along x-axis, false for y-axis.
	*\param[in] count	Incremented for each step taken by the robot.
	*/
	bool accSearch(CrpiRobot<CrpiUniversal> *arm, robotPose &pose, bool up, bool xAxis, double stepSize, int &count);

#endif