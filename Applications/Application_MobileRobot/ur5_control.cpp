/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

ur5_control.cpp

Description
===========

Function implementation for controlling the ur5 arm. Includes pose conversion helper functions, spiral searches, and laser registration methods.

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

#include <iostream>
#include "ur5_control.h"
#include "AssemblyPrims.h"
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "lynx_msg.h"

using namespace std;
using namespace crpi_robot;
using namespace MotionPrims;

//CONVERSION FUNCTIONS

/*
	setPosition

	Description
	===============================================================
	Convenience functions brought in from mobmanmain by S. Legowik
	Convenience function for copying cartesian point into robotPose

	Parameters
	===============================================================
	robotPose -- Used to hold the converted pose
	point -- Holds position vector
	rot -- Array that holds the TCP rotation
 */
void setPosition(robotPose &pose, const PM_CARTESIAN point, const double rot[])
{
	pose.x = point.x;
	pose.y = point.y;
	pose.z = point.z;
	pose.xrot = rot[0];
	pose.yrot = rot[1];
	pose.zrot = rot[2];
}//end setPosition

/*
	cart2rob:

	Description
	====================================================================
	Function directly ported from mobmanmain by S. Legowik
	convenience function for converting from PM_CARTESIAN to robotPose
	@param position is used to set the x and y coordinates
	@param orientation is used to set the z value and the rotation
*/
robotPose cart2rob(PM_CARTESIAN position, robotPose orientation)
{
	robotPose pose = orientation;
	pose.x = position.x;
	pose.y = position.y;

	return pose;
}//end cart2rob

/*
	pm2robotConvert:
	
	Description
	=====================================
	convert from a PM_POSE to a robotPose
	Directly ported from mobmanmain by S. Legowik.
*/
robotPose pm2robotConvert(PM_POSE pm)
{
	robotPose pose;
	pose.x = pm.tran.x;
	pose.y = pm.tran.y;
	pose.z = pm.tran.z;
	PM_RPY rpy = pm.rot;
	pose.xrot = rpy.r * 180 / PM_PI;
	pose.yrot = rpy.p * 180 / PM_PI;
	pose.zrot = rpy.y * 180 / PM_PI;

	return pose;
}//end pm2robotConvert

/*
	robot2pmConvert:

	Description
	=================================
	convert from a robotPose to a PM_POSE
	Directly ported from mobmanmain by S. Legowik

*/
PM_POSE robot2pmConvert(robotPose robot)
{
	PM_POSE pose;
	pose.tran.x = robot.x;
	pose.tran.y = robot.y;
	pose.tran.z = robot.z;
	PM_RPY rpy(robot.xrot*PM_PI / 180, robot.yrot*PM_PI / 180, robot.zrot*PM_PI / 180);
	pose.rot = rpy;

	return pose;
}//end robot2pmConvert

/*
 rob2cart:

 Description
 ====================================================================
 convenience function for converting from robotPose to PM_CARTESIAN
 @param pose is used to set the x and y values, z is set to 0
 NOTE: Modified for lynx so z is not set to zero.
*/
PM_CARTESIAN rob2cart(robotPose pose)
{
	return PM_CARTESIAN(pose.x, pose.y, sensor_height);
}//end rob2cart

/*compute_start

	Description
	====================================================================
	Transforms expected reflector points given in robot base coordinates
	to lynx coordinates using the commanded lynx position, 
	then performs the inverse transformation using the actual lynx position.
	The final result corrects the expected reflector position based on position and orientation
	readings from the lync controller.

	Parameters
	====================================================================
	point1_world, point2_world -- Holds the corrected position for the start points of two reflectors.
	
	point1_ur5, point2_ur5 -- Expected position of the start points in robot coordinates, reflector position constants
	passed from ur5_control.h file.
	
	rmma_to_lynx -- Commanded pose of lynx.

	lynx_pose -- Actual lynx pose

	Note that lynx_to_ur5 constant is used to factor in the mounting offset of the robot base, currently this pose is set to zero.

*/
void compute_start(PM_CARTESIAN& point1_world, PM_CARTESIAN& point2_world, PM_CARTESIAN point1_ur5, PM_CARTESIAN point2_ur5, PM_POSE rmma_to_lynx, PM_POSE lynx_pose)
{

	cout << "Point 1: (" << point1_ur5.x << ", " << point1_ur5.y << ", " << point1_ur5.z << ")" << endl;
	cout << "Point 2: (" << point2_ur5.x << ", " << point2_ur5.y << ", " << point1_ur5.z << ")" << endl;

//Compute world coordinates of expected reflector points based on commanded lynx pose.
	point1_world = rmma_to_lynx * lynx_to_ur5 * point1_ur5;
	point2_world = rmma_to_lynx * lynx_to_ur5 * point2_ur5;

	
	cout << "Point 1: (" << point1_world.x << ", " << point1_world.y <<", "<<point1_world.z<<")" << endl;
	cout << "Point 2: (" << point2_world.x << ", " << point2_world.y <<", "<<point1_world.z<<")"<< endl;

//Compute inverse using actual position
	point1_world = inv(lynx_to_ur5) * inv(lynx_pose) * point1_world;
	point2_world = inv(lynx_to_ur5) * inv(lynx_pose) * point2_world;

	
	cout << "Point 1: (" << point1_world.x << ", " << point1_world.y <<", "<<point1_world.z<<")"<<endl;
	cout << "Point 2: (" << point2_world.x << ", " << point2_world.y <<", "<<point1_world.z<<")"<< endl;
	

	return;

}//end compute_start

/*
	time_since:

	Description
	========================================================
	Function to compute the elapsed time since a starting timestamp.
	Directly ported from mobmanmain by S. Legowik

*/
double time_since(int seconds)
{
	SYSTEMTIME sysTime;
	GetSystemTime(&sysTime);

	double timestamp =
		sysTime.wHour * 60 * 60 +
		sysTime.wMinute * 60 +
		sysTime.wSecond +
		sysTime.wMilliseconds / 1000.0
		- seconds;
	return timestamp;
}//end time_since

/*
	get_seconds:

	Description
	=================================
	Get seconds since January 1, 1970 (UNIX TIME) based on Windows System Libraries
	Ported directly from mobmanmain by S. Legowik
*/
int get_seconds()
{
	SYSTEMTIME sysTime;
	GetSystemTime(&sysTime);

	return
		sysTime.wHour * 60 * 60 +
		sysTime.wMinute * 60 +
		sysTime.wSecond;
}//end get_seconds

//CONTROL FUNCTIONS
/*
	stage_arm:

	Description
	===============================================
	Moves the arm into a position whereby it can safely access the RMMA.
	Uses a series of intermediate points to stage the arm. Stage points are stored
	in ur5_control.h

	Robot arm must be within 5 mm of stow point to be safely staged.

	Parameters
	===========================================
	ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
*/
CanonReturn stage_arm(CrpiRobot<CrpiUniversal> * ur_robot)
{
	CanonReturn retval = CANON_FAILURE;
	robotPose robot_stow, robot_stage_1, robot_stage_2, robot_stage_3, robot_stage_4;
	robotPose curPose;

	//Copy pose constants from ur5_control.h into a CRPI pose used to command robot arm.
	setPosition(robot_stow, stow, sensor_rot);
	setPosition(robot_stage_1, stage_1, sensor_rot);
	setPosition(robot_stage_2, stage_2, sensor_rot);
	setPosition(robot_stage_3, stage_3, sensor_rot);
	setPosition(robot_stage_4, stage_4, sensor_rot);

	ur_robot->GetRobotPose(&curPose);

	if (curPose.distance(robot_stow) > 5) //Check that the robot is within 5 mm of stow position
	{
		cout << "Cannot Move Robot: Robot not in stow position" << endl;
		return retval;
	}//end if

	cout << "Moving to ";
	robot_stage_1.print();
	cout << endl;
	if ((retval = ur_robot->MoveStraightTo(robot_stage_1)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
		return retval;
	}//end if

	//This while loop prevents the next move command from being sent to the robot arm until the current command has finsihed executing.
	while (curPose.distance(robot_stage_1) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	//Rest of this function repeats the logic for moving to the first stage points but used to move to the rest of the stage points.

	cout << "Moving to ";
	robot_stage_2.print();
	cout << endl;

	if ((retval = ur_robot->MoveStraightTo(robot_stage_2)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
		return retval;
	}//end if

	while (curPose.distance(robot_stage_2) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	cout << "Moving to ";
	robot_stage_3.print();
	cout << endl;

	if ((retval = ur_robot->MoveStraightTo(robot_stage_3)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
		return retval;
	}//end if

	while (curPose.distance(robot_stage_3) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	cout << "Moving to ";
	robot_stage_4.print();
	cout << endl;
	if ((retval = ur_robot->MoveStraightTo(robot_stage_4)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
		return retval;
	}//end if

	while (curPose.distance(robot_stage_4) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	return retval;

}//end stage arm

 /*
 stow_arm:

 Description
 ===============================================
 Moves the robot arm into a position whereby the TCP is over the lynx payload structure and 
 the lynx can safely move when robot is placed in this position. Series of intermediate points used to establish
 stow position.
 Constants defined in ur5_control.h
 Arm must not already be in stow position.

 If the robot arm drifts from the stow position, and thus cannot be staged, manually jog the robot arm as if it is accessing the RMMA
 Then execute this function to properly send the robot arm to the stow point within the tolerances needed for this control program.

 Parameters
 ===========================================
 ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
 */
CanonReturn stow_arm(CrpiRobot<CrpiUniversal> * ur_robot)
{
	CanonReturn retval = CANON_FAILURE;
	robotPose robot_stow, robot_stage_1, robot_stage_2, robot_stage_3, robot_stage_4;
	robotPose curPose;
	ur_robot->GetRobotPose(&curPose);


	//Convert pose constants from ur5_control.h to CRPI robot pose used to command the robot.
	setPosition(robot_stow, stow, sensor_rot);
	setPosition(robot_stage_1, stage_1, sensor_rot);
	setPosition(robot_stage_2, stage_2, sensor_rot);
	setPosition(robot_stage_3, stage_3, sensor_rot);
	setPosition(robot_stage_4, stage_4, sensor_rot);

	ur_robot->GetRobotPose(&curPose);

	if (curPose.distance(robot_stow) < 1) //Check that the robot arm is not already stowed.
	{
		cout << "Cannot move robot: robot already in stow position!" << endl;
		return retval;
	}//end if

	cout << "Moving to ";
	robot_stage_3.print();
	cout << endl;
	if ((retval = ur_robot->MoveStraightTo(robot_stage_3)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
		return retval;
	}//end if

	while (curPose.distance(robot_stage_3) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	cout << "Moving to ";
	robot_stage_2.print();
	cout << endl;

	if ((retval = ur_robot->MoveStraightTo(robot_stage_2)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
		return retval;
	}//end if

	while (curPose.distance(robot_stage_2) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	cout << "Moving to ";
	robot_stage_1.print();
	cout << endl;
	if ((retval = ur_robot->MoveStraightTo(robot_stage_1)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
		return retval;
	}//end if

	while (curPose.distance(robot_stage_1) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	cout << "Moving to ";
	robot_stow.print();
	cout << endl;
	if ((retval = ur_robot->MoveStraightTo(robot_stow)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
		return retval;
	}//end if

	while (curPose.distance(robot_stow) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	return retval;

}//end stage arm

 /*
 spiral_search_large:

 Description
 ===============================================
 This function implements the coarse spiral search used to localize the large reflectors. This code is identical to the spiral_search function,
 but uses a larger step size for square spiral path.

 Parameters
 ===========================================
 ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
 rmma_point -- CRPI robot pose that holds the initial search position of the robot arm.
 pose_me -- CRPI pose that holds the result of the search after localizing the bisect reflector.
 out_file -- output file for logging the reseults of the search
 mytime_seconds -- start time used for computing the time it took to complete the search.
 */
CanonReturn spiral_search_large(CrpiRobot<CrpiUniversal> * ur_robot, robotPose rmma_point, robotPose& pose_me, ofstream& out_file, int mytime_seconds)
{
	CanonReturn retval;
	robotIO io;
	robotPose curPose, poseMe;
	crpi_timer pause;
	crpi_timer timer;
	Assembly square_spiral;
	int counter;

	// set up search parameters
	square_spiral.AddSearchSqSpiral(10, searchRadius);
	square_spiral.AddTerminatorSignal(CANON_SUCCESS, 8);
	square_spiral.AddTerminatorTimer(CANON_FAILURE, searchTimeout);

	//! Move to position j
	cout << "Moving to initial search position "; rmma_point.print(); cout << endl;

	double ts = time_since(mytime_seconds);

	ur_robot->GetRobotIO(&io);

	//Move to the initial search position where reflector is expected to be.
	if (ur_robot->MoveStraightTo(rmma_point) == CANON_SUCCESS)
	{
		pause.waitUntil(large_robot_settle_time);
		ur_robot->GetRobotPose(&curPose);
		ur_robot->GetRobotIO(&io);

		cout << "at position ";  curPose.print(); cout << endl;
	}//end if
	else
	{
		cout << "*** Move command failed! ***" << endl;
	}//end else

	 //! Run spiral search at rmma_point

	cout << "Running spiral search..." << endl;

	timer.start();//.startTimer();
	counter = 0;

	//Main loop for incrementing along square spiral search path.
	do
	{
		ur_robot->GetRobotIO(&io);
		io.dio[8] = !io.dio[8];
		retval = square_spiral.RunAssemblyStep(counter++, curPose, poseMe, io);
		
		if (retval != CANON_RUNNING)
		{
			ur_robot->GetRobotPose(&poseMe);
			break;
		}//end if

		cout << "X: " << poseMe.x << " Y: " << poseMe.y << " Z: " << poseMe.z << endl;

		ur_robot->MoveStraightTo(poseMe);
		pause.waitUntil(large_robot_settle_time);

		++counter;
	} while (retval == CANON_RUNNING);

	//Compute search time
	double tim = timer.elapsedTime() / 1000.0;  // convert from milliseconds to seconds
	timer.stop();

	//Log results of search
	ts = time_since(mytime_seconds);
	out_file << (retval == CANON_SUCCESS ? "1" : "0") << ", ";
	out_file << tim << ", ";
	out_file << counter << ", ";
	out_file << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", "
		<< poseMe.yrot << ", " << poseMe.zrot << endl;

	cout << "CanonReturn: " << retval << endl;

	pose_me = poseMe;

	return retval;

}//end spiral search

 /*
 spiral_search:

 Description
 ===============================================
 This function implements the fine spiral search used to localize the small reflectors. 
 This function is called multiple times in other functions to perform a full test around the square or circle reflector patterns.

 Parameters
 ===========================================
 ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
 rmma_point -- CRPI robot pose that holds the initial search position of the robot arm.
 pose_me -- CRPI pose that holds the result of the search after localizing the bisect reflector.
 out_file -- output file for logging the reseults of the search
 mytime_seconds -- time seed used for computing the time it took to complete the search.
 */
CanonReturn spiral_search(CrpiRobot<CrpiUniversal> * ur_robot, robotPose rmma_point, robotPose& pose_me, ofstream& out_file, int mytime_seconds)
{
	CanonReturn retval;
	robotIO io;
	robotPose curPose, poseMe;
	crpi_timer pause;
	crpi_timer timer;
	Assembly square_spiral;
	int counter;

	// set up search parameters
	square_spiral.AddSearchSqSpiral(searchStep, searchRadius);
	square_spiral.AddTerminatorSignal(CANON_SUCCESS, 8);
	square_spiral.AddTerminatorTimer(CANON_FAILURE, searchTimeout);

	//! Move to position j
	cout << "Moving to initial search position "; rmma_point.print(); cout << endl;

	double ts = time_since(mytime_seconds);

	ur_robot->GetRobotIO(&io);

	if (ur_robot->MoveStraightTo(rmma_point) == CANON_SUCCESS)
	{
		pause.waitUntil(robot_settle_time);
		ur_robot->GetRobotPose(&curPose);
		ur_robot->GetRobotIO(&io);

		cout << "at position ";  curPose.print(); cout<<endl;
	}//end if
	else
	{
		cout << "*** Move command failed! ***" << endl;
	}//end else

	//! Run spiral search at rmma_point

	cout << "Running spiral search..." << endl;

	timer.start();//.startTimer();
	counter = 0;

	//Main loop for search
	do
	{
		ur_robot->GetRobotIO(&io);
		io.dio[8] = !io.dio[8];
		retval = square_spiral.RunAssemblyStep(counter++, curPose, poseMe, io);
		if (retval != CANON_RUNNING)
		{
			ur_robot->GetRobotPose(&poseMe);
			break;
		}//end if

		cout << "X: " << poseMe.x << " Y: " << poseMe.y << " Z: " << poseMe.z << endl;

		ur_robot->MoveStraightTo(poseMe);
		pause.waitUntil(robot_settle_time);

		++counter;
	} while (retval == CANON_RUNNING);
	
	//Compute search time
	double tim = timer.elapsedTime() / 1000.0;  // convert from milliseconds to seconds
	timer.stop();

	//Log results of search
	ts = time_since(mytime_seconds);
	out_file << (retval == CANON_SUCCESS ? "1" : "0") << ", ";
	out_file << tim << ", ";
	out_file << counter << ", ";
	out_file << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", "
		<< poseMe.yrot << ", " << poseMe.zrot << endl;

	cout << "CanonReturn: " << retval << endl;

	pose_me = poseMe;

	return retval;

}//end spiral search

 /*
 square:

 Description
 ===============================================
 This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a square shape.
 After the localization of each reflector, the location of the other reflectors are updated.
 
 NOTE: This function was intended for use on AGV/UR10 mobile manipulator and should not be used on the lynx. This is because the ur5 arm cannot reach 
 all of the reflectors that are specified in this search patttern. This code is useful, however to help develop new search patterns.

 Parameters
 ===========================================
 ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
 start_point1, start_point2 -- expected position of the principal reflectors used to compute the location of the rest of the square patern reflectors.
 num_iters -- number of times to go around the square pattern
 mytime_seconds -- timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
 */
CanonReturn square(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1, PM_CARTESIAN start_point2, int num_iters, int mytime_seconds)
{
	CanonReturn retval;
	ofstream square_spiral_csv;
	robotPose poseMe;
	double ts;
	robotPose position[10];
	robotPose start_pose;
	double theta = 0.0f;
	PM_CARTESIAN point2_offset = start_point2 - start_point1;

	char log_name[128];
	time_t log_time;
	tm* log_tm;

	//Set up logs, the current date and time are used to give each log a unique name.
	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Applications\\Application_MobileRobot\\Data\\square_%d-%m-%Y_%I-%M-%S.csv", log_tm);

	square_spiral_csv.open(log_name);

	//Convert initial search position to CRPI pose
	setPosition(start_pose, start_point1, sensor_rot);

	position[0] = start_pose;

	for (int i = 0; i < num_iters; ++i) //This loop controls the number of times the entire square is traversed.
	{
		cout << "Starting iteration " << i << endl;
		for (int j = 0; j < 4; ++j) //This loop controls the number of reflectors in the square that is traversed.
		{
			//Log initial positons
			ts = time_since(mytime_seconds);
			square_spiral_csv << ts << ", ";
			square_spiral_csv << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
				<< ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";
			
			//Note the invocation of the spiral_search function
			retval = spiral_search(ur_robot, position[j], poseMe, square_spiral_csv, mytime_seconds);
			
			//If statement updates the actual position of the reflector after localization
			if (retval == CANON_SUCCESS)
				position[j] = poseMe;

			if (i == 0)//If statement handles the special case of the first square traversal
			{
				if (j == 0) //Note that the the first and second reflectors are singled out because their locations are used to deteermine the rest of the square.
				{
					if (retval != CANON_SUCCESS)//If first reflector cannot be found, entire search pattern terminates.
					{
						cout << "Failed to find first point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if

					// refine the second target position (use the original offset between the start points and add to the improved position vector for start point 1)

					position[1] = position[0];
					position[1].x = position[0].x + point2_offset.x;
					position[1].y = position[0].y + point2_offset.y;
					position[1].z = position[0].z;
				}//end if
				else if (j == 1) //Second starting reflector again used to compute the position of the other reflectors
				{
					if (retval != CANON_SUCCESS)
					{
						cout << "Failed to find second point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if
					
					//Here is where the rest of the square locations are computed.
					theta = atan2((position[1].y - position[0].y), (position[1].x - position[0].x));
					//Compute offset to new points by converting 18 in. to mm and then solving for components.
					double dx = (18.0 * 25.4) * cos(theta + PM_PI / 2);
					double dy = (18.0 * 25.4) * sin(theta + PM_PI / 2);
					position[2] = position[1];
					position[3] = position[0];
					position[2].x += dx;
					position[2].y += dy;
					position[3].x += dx;
					position[3].y += dy;
				} //end else if
			}//end if
			cout << "Completed iteration " << i << endl;
		} // for (int j = 0; j < numPoints; ++j)

	} // for (i = 0; i < numIter; ++i)
	
	square_spiral_csv.close();

	cout << "CanonReturn: " << retval << endl;
	return retval;
}//end square

 /*
 square_bisect:

 Description
 ===============================================
 This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a square shape.
 After the localization of each reflector, the location of the other reflectors are updated. Bisection registration is also used to 
 further localize the start points of the square pattern.

 NOTE: This function was intended for use on AGV/UR10 mobile manipulator and should not be used on the lynx. This is because the ur5 arm cannot reach
 all of the reflectors that are specified in this search patttern. This code is useful, however to help develop new search patterns.

 Parameters
 ===========================================
 ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
 start_point1_large, start_point2_l;arge -- expected position of the bisect reflectors used to compute the location of the rest of the square patern reflectors.
 num_iters -- number of times to go around the square pattern
 mytime_seconds -- timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
 */
CanonReturn square_bisect(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds)
{
	CanonReturn retval;
	ofstream square_spiral_bisect_csv;
	robotPose poseMe;
	robotPose position[10];
	robotPose start_pose;
	double theta = 0.0f;
	PM_CARTESIAN point2_offset;
	
	char log_name [128];
	time_t log_time;
	tm* log_tm;
	
	//Prepare log file, the current date and time are used to give each log a unique name.
	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Applications\\Application_MobileRobot\\Data\\square_bisect_%d-%m-%Y_%I-%M-%S.csv", log_tm);

	square_spiral_bisect_csv.open(log_name);

////////////////////////////////////////////////////////////////////////////////////
	robotPose ref_point1, ref_point2;
	robotPose start_point1, start_point2;
	double searchTime1, searchTime2;
	double ts;
	int stepCount1, stepCount2;
	int success1 = 1;
	int success2 = 1;

	//Initialize pose constants ref_points are updated after bisection to hold the actual reflector center.
	setPosition(start_point1, start_point1_large, sensor_rot);
	setPosition(start_point2, start_point2_large, sensor_rot);
	setPosition(ref_point1, start_point1_large, sensor_rot);
	setPosition(ref_point2, start_point2_large, sensor_rot);

	ur_robot->SetRelativeSpeed(.1);

	// move to markers and refine position
	ur_robot->MoveStraightTo(ref_point1);
	double start_point1_time = time_since(mytime_seconds);
	if (!bisect(ur_robot, ref_point1, largeTargetStepSize, &stepCount1, &searchTime1))
	{
		cout << "Error finding center of reflector 1" << endl;
		success1 = 0;
	}//end if

	ur_robot->MoveStraightTo(ref_point2);
	double start_point2_time = time_since(mytime_seconds);
	if (!bisect(ur_robot, ref_point2, largeTargetStepSize, &stepCount2, &searchTime2))
	{
		cout << "Error finding center of reflector 2" << endl;
		success2 = 0;
	}//end if

	// write large reflector stats to CSV file with target id -1 and -2
	square_spiral_bisect_csv << start_point1_time << ", ";
	square_spiral_bisect_csv << "0, -1, " << start_point1.x << ", " << start_point1.y << ", " << start_point1.z << ", "
		<< start_point1.xrot << ", " << start_point1.yrot << ", " << start_point1.zrot;
	square_spiral_bisect_csv << ", " << success1 << ", " << searchTime1 << ", " << stepCount1;
	square_spiral_bisect_csv << ", " << ref_point1.x << ", " << ref_point1.y << ", " << ref_point1.z << ", "
		<< ref_point1.xrot << ", " << ref_point1.yrot << ", " << ref_point1.zrot << endl;

	square_spiral_bisect_csv << start_point2_time << ", ";
	square_spiral_bisect_csv << "0, -2, " << start_point2.x << ", " << start_point2.y << ", " << start_point2.z << ", "
		<< start_point2.xrot << ", " << start_point2.yrot << ", " << start_point2.zrot;
	square_spiral_bisect_csv << ", " << success2 << ", " << searchTime2 << ", " << stepCount2;
	square_spiral_bisect_csv << ", " << ref_point2.x << ", " << ref_point2.y << ", " << ref_point2.z << ", "
		<< ref_point2.xrot << ", " << ref_point2.yrot << ", " << ref_point2.zrot << endl;

	if (success1 == 0 || success2 == 0)//If the first or second bisection fails, the rest of the search routine is terminated.
	{
		return CANON_FAILURE;
	}//end if

	// compute index locations
	point2_offset = rob2cart(ref_point2 - ref_point1);
	point2_offset.z = 0;
	PM_CARTESIAN offset90 = unit(PM_QUATERNION(PM_Z, PM_PI / 2) * point2_offset);
	PM_CARTESIAN cornerOffset = (-8 * 25.4) * offset90; //The 8 inch offset here can be modified if the bisect reflector positions are changed.
	setPosition(position[0], rob2cart(ref_point1) + cornerOffset, sensor_rot);

	ur_robot->SetRelativeSpeed(.1);

	//This code follows the same logic as square function
	for (int i = 0; i < num_iters; ++i)
	{
		cout << "Starting iteration " << i << endl;
		for (int j = 0; j < 4; ++j)
		{
			ts = time_since(mytime_seconds);
			square_spiral_bisect_csv << ts << ", ";
			square_spiral_bisect_csv << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
				<< ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";

			retval = spiral_search(ur_robot, position[j], poseMe, square_spiral_bisect_csv, mytime_seconds);

			if (retval == CANON_SUCCESS)
				position[j] = poseMe;

			if (i == 0)
			{
				if (j == 0)
				{
					if (retval != CANON_SUCCESS)
					{
						cout << "Failed to find first point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if

					 // refine the target position (use the original offset between the start points and add to the improved position vector for start point 1)

					position[1] = position[0];
					position[1].x = position[0].x + point2_offset.x;
					position[1].y = position[0].y + point2_offset.y;
					position[1].z = position[0].z;
				}//end if
				else if (j == 1)
				{
					if (retval != CANON_SUCCESS)
					{
						cout << "Failed to find second point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if

					theta = atan2((position[1].y - position[0].y), (position[1].x - position[0].x));
					//Compute offset to new points by converting 18 in. to mm and then solving for components.
					double dx = (18.0 * 25.4) * cos(theta + PM_PI / 2);
					double dy = (18.0 * 25.4) * sin(theta + PM_PI / 2);
					position[2] = position[1];
					position[3] = position[0];
					position[2].x += dx;
					position[2].y += dy;
					position[3].x += dx;
					position[3].y += dy;
				} //end else if
			}//end if
			cout << "Completed iteration " << i << endl;
		} // for (int j = 0; j < numPoints; ++j)

	} // for (i = 0; i < numIter; ++i)

	square_spiral_bisect_csv.close();

	cout << "CanonReturn: " << retval << endl;
	return retval;
}//end square

 /*
 square_bisect_short:

 Description
 ===============================================
 This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a square shape.
 After the localization of each reflector, the location of the other reflectors are updated. Bisection registration is also used to
 further localize the start points of the square pattern.

 This function is modified for the lynx/ur5 mobile manipulator, adjusting the location of the bisect reflectors to account for the limited reach of the ur5.
 In addition, the function implements spiral_search_large to account for the larger uncertainty in the lynx vehicle docking.

 This is the recommended function to use when testing on the lynx.

 Parameters
 ===========================================
 ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
 start_point1_large, start_point2_l;arge -- expected position of the bisect reflectors used to compute the location of the rest of the square patern reflectors.
 num_iters -- number of times to go around the square pattern
 mytime_seconds -- timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
 goal -- goal number associated with the position that the lynx is parked at. Primarily used for logging purposes.
 */
CanonReturn square_bisect_short(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds, int goal)
{
	CanonReturn retval;
	ofstream square_spiral_bisect_csv;
	robotPose poseMe;
	robotPose position[10];
	robotPose start_pose;
	double theta = 0.0f;
	PM_CARTESIAN point2_offset;
	robotIO io;
	crpi_timer pause;

	char log_name[128];
	time_t log_time;
	tm* log_tm;

	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Applications\\Application_MobileRobot\\Data\\square_bisect_short_%d-%m-%Y_%I-%M-%S.csv", log_tm);
	cout << "Log file created at: " << log_name << endl;
	square_spiral_bisect_csv.open(log_name);

	//setPosition(start_pose, start_point1, sensor_rot);

	////////////////////////////////////////////////////////////////////////////////////
	robotPose ref_point1, ref_point2;
	robotPose start_point1, start_point2;
	double searchTime1, searchTime2;
	int stepCount1, stepCount2;
	double ts;
	int success1 = 1;
	int success2 = 1;
	setPosition(start_point1, start_point1_large, sensor_rot);
	setPosition(start_point2, start_point2_large, sensor_rot);
	setPosition(ref_point1, start_point1_large, sensor_rot);
	setPosition(ref_point2, start_point2_large, sensor_rot);

	// move to markers and refine position
	ur_robot->MoveStraightTo(ref_point1);

	ur_robot->GetRobotIO(&io);

	square_spiral_bisect_csv << "Goal, " << goal << endl;
	square_spiral_bisect_csv << "time, iter, marker, ";
	square_spiral_bisect_csv << "init_x, init_y, init_z, init_xrot, init_yrot, init_zrot,";
	square_spiral_bisect_csv<<"ok, time_elapsed, steps,";
	square_spiral_bisect_csv << "x, y, z, xrot, yrot, zrot"<<endl;

	square_spiral_bisect_csv << "Coarse Spiral Search" << endl;

	if (io.dio[8] != 0)//Check for reflector signal first, then attempt coarse search if vehicle docks outside reflector tolerance.
	{
		cout << "Failed to find reflector 1 -- Now Attempting to Localize" << endl;
		ts = time_since(mytime_seconds);

		square_spiral_bisect_csv << ts << ", ";
		square_spiral_bisect_csv << -2 << ", " << -1 << ", " << ref_point1.x << ", " << ref_point1.y << ", " << ref_point1.z
			<< ", " << ref_point1.xrot << ", " << ref_point1.yrot << ", " << ref_point1.zrot << ", ";

		spiral_search_large(ur_robot, ref_point1, ref_point1, square_spiral_bisect_csv, mytime_seconds);
		ur_robot->MoveStraightTo(ref_point1);
		pause.waitUntil(robot_settle_time);
	}//end if

	setPosition(start_point1, rob2cart(ref_point1), sensor_rot);

	double start_point1_time = time_since(mytime_seconds);
	if (!bisect(ur_robot, ref_point1, largeTargetStepSize, &stepCount1, &searchTime1))
	{
		cout << "Error finding center of reflector 1" << endl;
		success1 = 0;
	}//end if

	ur_robot->MoveStraightTo(ref_point2);//Here is a good point of the code to adjust the location of the second bisect reflector.

	ur_robot->GetRobotIO(&io);

	if (io.dio[8] != 0)
	{
		cout << "Failed to find reflector 2 -- Now Attempting to Localize" << endl;
		ts = time_since(mytime_seconds);

		square_spiral_bisect_csv << ts << ", ";
		square_spiral_bisect_csv << -2 << ", " << -2 << ", " << ref_point2.x << ", " << ref_point2.y << ", " << ref_point2.z
			<< ", " << ref_point2.xrot << ", " << ref_point2.yrot << ", " << ref_point2.zrot << ", ";
		spiral_search_large(ur_robot, ref_point2, ref_point2, square_spiral_bisect_csv, mytime_seconds);
		ur_robot->MoveStraightTo(ref_point2);
		pause.waitUntil(robot_settle_time);
	}//end if

	setPosition(start_point2, rob2cart(ref_point2), sensor_rot);

	double start_point2_time = time_since(mytime_seconds);
	if (!bisect(ur_robot, ref_point2, largeTargetStepSize, &stepCount2, &searchTime2))
	{
		cout << "Error finding center of reflector 2" << endl;
		success2 = 0;
	}//end if

	 // write large reflector stats to CSV file with target id -1 and -2
	square_spiral_bisect_csv << "Bisect" << endl;
	square_spiral_bisect_csv << start_point1_time << ", ";
	square_spiral_bisect_csv << "-1, -1, " << start_point1.x << ", " << start_point1.y << ", " << start_point1.z << ", "
		<< start_point1.xrot << ", " << start_point1.yrot << ", " << start_point1.zrot;
	square_spiral_bisect_csv << ", " << success1 << ", " << searchTime1 << ", " << stepCount1;
	square_spiral_bisect_csv << ", " << ref_point1.x << ", " << ref_point1.y << ", " << ref_point1.z << ", "
		<< ref_point1.xrot << ", " << ref_point1.yrot << ", " << ref_point1.zrot << endl;

	square_spiral_bisect_csv << start_point2_time << ", ";
	square_spiral_bisect_csv << "-1, -2, " << start_point2.x << ", " << start_point2.y << ", " << start_point2.z << ", "
		<< start_point2.xrot << ", " << start_point2.yrot << ", " << start_point2.zrot;
	square_spiral_bisect_csv << ", " << success2 << ", " << searchTime2 << ", " << stepCount2;
	square_spiral_bisect_csv << ", " << ref_point2.x << ", " << ref_point2.y << ", " << ref_point2.z << ", "
		<< ref_point2.xrot << ", " << ref_point2.yrot << ", " << ref_point2.zrot << endl;

	if (success1 == 0 || success2 == 0)//Note that if either bisection fails, the routine temrinates.
	{
		return CANON_FAILURE;
	}//end if

	 // compute index locations
	point2_offset = rob2cart(ref_point2 - ref_point1);
	point2_offset.z = 0;
	PM_CARTESIAN offset90 = unit(PM_QUATERNION(PM_Z, PM_PI / 2) * point2_offset);
	PM_CARTESIAN cornerOffset = (-2 * 25.4) * offset90; //Was -8, changed to -2 to account for changed position of large reflectors on the RMMA.
	setPosition(position[0], rob2cart(ref_point1) + cornerOffset, sensor_rot);

	square_spiral_bisect_csv << "Fine Spiral Search" << endl;

	for (int i = 0; i < num_iters; ++i)
	{
		cout << "Starting iteration " << i << endl;
		for (int j = 0; j < 2; ++j) // Short version only searches two points to prevent ur5 from exceeding joint limits.
		{
			
			ts = time_since(mytime_seconds);
			square_spiral_bisect_csv << ts << ", ";
			square_spiral_bisect_csv << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
				<< ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";

			retval = spiral_search(ur_robot, position[j], poseMe, square_spiral_bisect_csv, mytime_seconds);

			if (retval == CANON_SUCCESS)
				position[j] = poseMe;

			if (i == 0)
			{
				if (j == 0)
				{
					if (retval != CANON_SUCCESS)
					{
						cout << "Failed to find first point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if

					 // refine the target position (use the original offset between the start points and add to the improved position vector for start point 1)

					position[1] = position[0];
					position[1].x = position[0].x + point2_offset.x;
					position[1].y = position[0].y + point2_offset.y;
					position[1].z = position[0].z;
				}//end if
				else if (j == 1)
				{
					if (retval != CANON_SUCCESS)
					{
						cout << "Failed to find second point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if

					theta = atan2((position[1].y - position[0].y), (position[1].x - position[0].x));
					//Compute offset to new points by converting 18 in. to mm and then solving for components.
					double dx = (18.0 * 25.4) * cos(theta + PM_PI / 2);
					double dy = (18.0 * 25.4) * sin(theta + PM_PI / 2);
					position[2] = position[1];
					position[3] = position[0];
					position[2].x += dx;
					position[2].y += dy;
					position[3].x += dx;
					position[3].y += dy;
				} //end else if
			}//end if
			cout << "Completed iteration " << i << endl;
		} // for (int j = 0; j < numPoints; ++j)

	} // for (i = 0; i < numIter; ++i)

	square_spiral_bisect_csv.close();

	cout << "CanonReturn: " << retval << endl;
	return retval;
}//end circle

 /*
 square_bisect:

 Description
 ===============================================
 This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a circular shape.
 After the localization of each reflector, the location of the other reflectors are updated. Bisection registration is also used to
 further localize the start points of the circle pattern.

 NOTE: This function was intended for use on AGV/UR10 mobile manipulator and should not be used on the lynx. This is because the ur5 arm cannot reach
 all of the reflectors that are specified in this search patttern. This code is useful, however to help develop new search patterns.

 Parameters
 ===========================================
 ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
 start_point1_large, start_point2_l;arge -- expected position of the bisect reflectors used to compute the location of the rest of the square patern reflectors.
 num_iters -- number of times to go around the square pattern
 mytime_seconds -- timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
 */
CanonReturn circle_bisect(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds)
{
	CanonReturn retval;
	ofstream circle_spiral_bisect_csv;
	robotPose poseMe;
	robotPose position[10];
	robotPose start_pose;
	double theta = 0.0f;
	double ts;
	PM_CARTESIAN point2_offset;

	char log_name[128];
	time_t log_time;
	tm* log_tm;

	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Applications\\Application_MobileRobot\\Data\\circle_bisect_%d-%m-%Y_%I-%M-%S.csv", log_tm);

	circle_spiral_bisect_csv.open(log_name);

	//setPosition(start_pose, start_point1, sensor_rot);

	////////////////////////////////////////////////////////////////////////////////////
	robotPose ref_point1, ref_point2;
	robotPose start_point1, start_point2;
	double searchTime1, searchTime2;
	int stepCount1, stepCount2;
	int success1 = 1;
	int success2 = 1;
	setPosition(start_point1, start_point1_large, sensor_rot);
	setPosition(start_point2, start_point2_large, sensor_rot);
	setPosition(ref_point1, start_point1_large, sensor_rot);
	setPosition(ref_point2, start_point2_large, sensor_rot);

	// move to markers and refine position
	ur_robot->MoveStraightTo(ref_point1);
	double start_point1_time = time_since(mytime_seconds);
	if (!bisect(ur_robot, ref_point1, largeTargetStepSize, &stepCount1, &searchTime1))
	{
		cout << "Error finding center of reflector 1" << endl;
		success1 = 0;
	}//end if

	ur_robot->MoveStraightTo(ref_point2);
	double start_point2_time = time_since(mytime_seconds);
	if (!bisect(ur_robot, ref_point2, largeTargetStepSize, &stepCount2, &searchTime2))
	{
		cout << "Error finding center of reflector 2" << endl;
		success2 = 0;
	}//end if

	 // write large reflector stats to CSV file with target id -1 and -2
	circle_spiral_bisect_csv << start_point1_time << ", ";
	circle_spiral_bisect_csv << "0, -1, " << start_point1.x << ", " << start_point1.y << ", " << start_point1.z << ", "
		<< start_point1.xrot << ", " << start_point1.yrot << ", " << start_point1.zrot;
	circle_spiral_bisect_csv << ", " << success1 << ", " << searchTime1 << ", " << stepCount1;
	circle_spiral_bisect_csv << ", " << ref_point1.x << ", " << ref_point1.y << ", " << ref_point1.z << ", "
		<< ref_point1.xrot << ", " << ref_point1.yrot << ", " << ref_point1.zrot << endl;

	circle_spiral_bisect_csv << start_point2_time << ", ";
	circle_spiral_bisect_csv << "0, -2, " << start_point2.x << ", " << start_point2.y << ", " << start_point2.z << ", "
		<< start_point2.xrot << ", " << start_point2.yrot << ", " << start_point2.zrot;
	circle_spiral_bisect_csv << ", " << success2 << ", " << searchTime2 << ", " << stepCount2;
	circle_spiral_bisect_csv << ", " << ref_point2.x << ", " << ref_point2.y << ", " << ref_point2.z << ", "
		<< ref_point2.xrot << ", " << ref_point2.yrot << ", " << ref_point2.zrot << endl;

	if (success1 == 0 || success2 == 0)//If either bisect registration fails, the entire routine terminates.
	{
		return CANON_FAILURE;
	}//end if

	 // compute index locations
	point2_offset = rob2cart(ref_point2 - ref_point1);
	point2_offset.z = 0;
	PM_CARTESIAN offset90 = unit(PM_QUATERNION(PM_Z, PM_PI / 2) * point2_offset);
	PM_CARTESIAN first_point = (rob2cart(ref_point1) + rob2cart(ref_point2)) / 2; // center, calculated as midpoint of bsiect reflector locations
	first_point = first_point + offset90 * (6 * 25.4);
	setPosition(position[0], first_point, sensor_rot);

	// recompute offset
	point2_offset = offset90 * (-12 * 25.4); //Modify this to change initial spiral point, should be -1 * (12 - 12*cos(30))

	for (int i = 0; i < num_iters; ++i)
	{
		cout << "Starting iteration " << i << endl;
		for (int j = 0; j < 4; ++j)
		{
			ts = time_since(mytime_seconds);
			circle_spiral_bisect_csv << ts << ", ";
			circle_spiral_bisect_csv << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
				<< ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";

			retval = spiral_search(ur_robot, position[j], poseMe, circle_spiral_bisect_csv, mytime_seconds);

			if (retval == CANON_SUCCESS)
				position[j] = poseMe;

			if (i == 0)
			{
				if (j == 0)
				{
					if (retval != CANON_SUCCESS)
					{
						cout << "Failed to find first point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if

					 // refine the target position (use the original offset between the start points and add to the improved position vector for start point 1)

					position[1] = position[0];
					position[1].x = position[0].x + point2_offset.x;
					position[1].y = position[0].y + point2_offset.y;
					position[1].z = position[0].z;
				}//end if
				else if (j == 1)
				{
					if (retval != CANON_SUCCESS)
					{
						cout << "Failed to find second point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if

					theta = atan2((position[1].y - position[0].y), (position[1].x - position[0].x));
					//Compute offset to new points by converting 18 in. to mm and then solving for components.
					double dx = (18.0 * 25.4) * cos(theta + PM_PI / 2);
					double dy = (18.0 * 25.4) * sin(theta + PM_PI / 2);
					position[2] = position[1];
					position[3] = position[0];
					position[2].x += dx;
					position[2].y += dy;
					position[3].x += dx;
					position[3].y += dy;
				} //end else if
			}//end if
			cout << "Completed iteration " << i << endl;
		} // for (int j = 0; j < numPoints; ++j)

	} // for (i = 0; i < numIter; ++i)

	circle_spiral_bisect_csv.close();

	cout << "CanonReturn: " << retval << endl;
	return retval;
}//end square

 /*
 circle:

 Description
 ===============================================
 This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a circular shape.
 After the localization of each reflector, the location of the other reflectors are updated.

 NOTE: This function was intended for use on AGV/UR10 mobile manipulator and should not be used on the lynx. This is because the ur5 arm cannot reach
 all of the reflectors that are specified in this search patttern. This code is useful, however to help develop new search patterns.

 Parameters
 ===========================================
 ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
 start_point1_large, start_point2_l;arge -- expected position of the bisect reflectors used to compute the location of the rest of the square patern reflectors.
 num_iters -- number of times to go around the square pattern
 mytime_seconds -- timestamp used to compute the time it took to complete the searches. This parameter is passed down to the spiral_search function.
 */

CanonReturn circle(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1, PM_CARTESIAN start_point2, int num_iters, int mytime_seconds)
{
	CanonReturn retval;
	ofstream circle_spiral_csv;
	robotPose poseMe;
	robotPose position[10];
	robotPose start_pose;
	double theta = 0.0f;
	double ts;

	PM_CARTESIAN point2_offset = start_point2 - start_point1;

	char log_name[128];
	time_t log_time;
	tm* log_tm;


	//Prepare logs, current date and time used to generate unique log names.
	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Applications\\Application_MobileRobot\\Data\\circle_%d-%m-%Y_%I-%M-%S.csv", log_tm);

	circle_spiral_csv.open(log_name);

	setPosition(start_pose, start_point1, sensor_rot);

	position[0] = start_pose;

	for (int i = 0; i < num_iters; ++i)
	{
		cout << "Starting iteration " << i << endl;
		for (int j = 0; j < 6; ++j)
		{
			ts = time_since(mytime_seconds);
			circle_spiral_csv << ts << ", ";
			circle_spiral_csv << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
				<< ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";

			retval = spiral_search(ur_robot, position[j], poseMe, circle_spiral_csv, mytime_seconds);

			if (retval == CANON_SUCCESS)
				position[j] = poseMe;

			if (i == 0)
			{
				if (j == 0)
				{
					if (retval != CANON_SUCCESS)
					{
						cout << "Failed to find first point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if

					 // refine the target position (use the original offset between the start points and add to the improved position vector for start point 1)

					position[1] = position[0];
					position[1].x = position[0].x + point2_offset.x;
					position[1].y = position[0].y + point2_offset.y;
					position[1].z = position[0].z;
				}//end if
				else if (j == 1)
				{
					if (retval != CANON_SUCCESS)
					{
						cout << "Failed to find second point" << endl;
						i = num_iters;  // make sure we dont do verification loops
						break;
					}//end if

					 // find the center point of the circle and the angle for position[0]
					robotPose center = position[0];
					center.x = (position[0].x + position[1].x) / 2;
					center.y = (position[0].y + position[1].y) / 2;
					double t0 = atan2(position[0].y - position[1].y, position[0].x - position[1].x);

					// recompute positions based on center, radius and initial angle
					double radius = 6 * 25.4; // 6 inch radius
					for (int a = 0; a<6; ++a)
					{
						double angle = a * 60 * PM_PI / 180;
						position[a] = center;  // to copy height and orientation from initial point
						position[a].x = center.x + radius * cos(angle + t0);
						position[a].y = center.y + radius * sin(angle + t0);
					}//end for

					// skip directly to second loop where the robot will progress counter clockwise around the circle
					j = 6;

				} //end else if
			}//end if
			cout << "Completed iteration " << i << endl;
		} // for (int j = 0; j < numPoints; ++j)

	} // for (i = 0; i < numIter; ++i)

	circle_spiral_csv.close();

	cout << "CanonReturn: " << retval << endl;
	return retval;
}//end square

template <class T>
T sq(T val)
{
	return val * val;
}//end sq
 
 /*
 acc_search:

 Description
 ===============================================
 From scanUtils.cpp by S. Legowik:
 ///////////////////////////////////////////////////////////////
 // Fast search bisection search.  Starts moving at step size
 // doubling the distance each time till the transition is detected.
 // Then it starts searching with decreasing step size.
 // Subroutine assumes that robot has been paused long enough 
 // prior to call to allow for a different pause at beginning and
 // end of search.
 // Routine increments count by 1 per step.  Count should be reset 
 // externally.

 Parameters
 ===========================================
 
 // @param arm    Pointer to robot control data structure
 // @param pose   Returns center pose of reflector.
 // @param up     True if serching in positive direction.
 // @param xAxis  True if motion is along x-axis, false for y-axis.
 // @param count  Incremented for each step taken by the robot.
 */
bool accSearch(CrpiRobot<CrpiUniversal> *arm, robotPose &pose, bool up, bool xAxis, double stepSize, int &count)
{
	crpi_timer pause;
	robotIO io;

	// get current pose and io
	arm->GetRobotIO(&io);
	//arm->GetRobotPose( &pose );

	// fail if initial detect is false
	if (io.dio[8])
	{
		return false;
	}

	// search state variables
	bool out = true;
	bool edgeCrossed = false;
	int stepExponent = 0;
	bool detected = true;
	bool prevDetected = true;

	// search till step size is back to its minimum
	//logger << "Start search" << endl;
	while (stepExponent >= 0)
	{
		// compute move pose based on current step size and direction
		double step = stepSize * (1 << stepExponent) * (out ? 1 : -1) * (up ? 1 : -1);
		if (xAxis)
			pose.x += step;
		else
			pose.y += step;

		// make move, pause, and read sensor
		arm->MoveStraightTo(pose);
		pause.waitUntil(robot_settle_time);
		arm->GetRobotIO(&io);
		++count;

		// adjust search flags
		detected = !io.dio[8];
		if (!edgeCrossed && !detected)
			edgeCrossed = true;
		if (edgeCrossed)
			--stepExponent;
		else
			++stepExponent;
		if (detected != prevDetected)
			out = !out;

		// record previous state
		prevDetected = detected;

		/*logger << step
		<< " " << stepExponent
		<< " " << edgeCrossed
		<< " " << detected
		<< " " << out
		<< " " << up << endl; */
	}
	//logger << "end search" << endl;

	// Add a step if last move left us just inside the edge of the reflector
	//    for consistency with previous bisection algorithm which stopped
	//    after stepping out of the radius of the reflector.
	if (detected)
	{
		if (xAxis)
			pose.x += stepSize * (up ? 1 : -1);
		else
			pose.y += stepSize * (up ? 1 : -1);
		//logger << "Bumped out a step" << endl;
	}

	return true;
}//end accSearch
 
 /*
 bisect:
 From scanUtils.cpp by S. Legowik:
 Description
 ===============================================
 From 
 //////////////////////////////////////////////////////////////////
 // Bisect the current target to find the center more accurately.
 // Detects target when dio[8] is false;

 Parameters
 ========================================================
 // @param arm    Pointer to robot control data structure
 // @param pose   Returns center pose of reflector.
*/
bool bisect(CrpiRobot<CrpiUniversal> *arm, robotPose &pose, double stepSize, int *stepCount, double *searchTime)
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;
	robotPose curPose;
	CanonReturn retval;

	*stepCount = 0;
	*searchTime = 0;

	cout << "Moving to ";
	pose.print();
	cout << endl;
	if ((retval = arm->MoveStraightTo(pose)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
	}//end if

	while (curPose.distance(pose) > 0.1)
	{
		arm->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	// give robot a chance to settle down
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	// get current pose and io
	arm->GetRobotIO(&io);
	arm->GetRobotPose(&pose);

	robotPose initialPose = pose;

	cout << "Bisecting point..." << endl;

	// make sure we have detection at initial location
	if (io.dio[8])
	{
		cout << "no signal at p1" << std::endl;  // debug
		return false;
	}//end if

	timer.start();

	// search for x center
	robotPose x1, x2;
	robotPose startPose = pose;
	robotPose commandPose = pose;

	if (use_new_bisection_algorithm)
	{
		x1 = startPose;
		accSearch(arm, x1, false, true, stepSize, count);
	}//end if
	else
	{
		while (!io.dio[8])
		{
			commandPose.x -= stepSize;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while
		arm->GetRobotPose(&x1);
	}//end else

	commandPose = startPose;
	arm->MoveStraightTo(commandPose);

	// give robot a chance to settle down
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);
	pause.waitUntil(robot_settle_time);
	arm->GetRobotIO(&io);
	if (io.dio[8])
	{
		cout << "no signal at p2" << endl;   // debug
		return false;
	}//end if

	if (use_new_bisection_algorithm)
	{
		x2 = startPose;
		accSearch(arm, x2, true, true, stepSize, count);
	}//end if
	else
	{
		while (!io.dio[8])
		{
			commandPose.x += stepSize;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}
		arm->GetRobotPose(&x2);
	}//end if

	double xCenter = (x1.x + x2.x) / 2;

	cout << "x dim = " << x2.x - x1.x << endl;  // debug

												  // search for y center
	robotPose y1, y2;
	commandPose = startPose;
	commandPose.x = xCenter;
	startPose = commandPose;
	arm->MoveStraightTo(commandPose);

	// give robot a chance to settle down
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	arm->GetRobotIO(&io);
	if (io.dio[8])
	{
		cout << "no signal at p3" << endl;   // debug
		return false;
	}//end if

	if (use_new_bisection_algorithm)
	{
		y1 = startPose;
		accSearch(arm, y1, false, false, stepSize, count);
	}//end if
	else
	{
		while (!io.dio[8])
		{
			commandPose.y -= stepSize;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while
		arm->GetRobotPose(&y1);
	}//end else

	commandPose = startPose;
	arm->MoveStraightTo(commandPose);

	// give robot a chance to settle down
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	arm->GetRobotIO(&io);
	if (io.dio[8])
	{
		cout << "no signal at p4" << endl;   // debug
		return false;
	}//end if

	if (use_new_bisection_algorithm)
	{
		y2 = startPose;
		accSearch(arm, y2, true, false, stepSize, count);
	}//end if
	else
	{
		while (!io.dio[8])
		{
			commandPose.y += stepSize;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while
	}//end else

	arm->GetRobotPose(&y2);
	double yCenter = (y1.y + y2.y) / 2;

	cout << "y dim = " << y2.y - y1.y << endl;  // debug

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();
	*stepCount = count;

	// set pose to center
	pose.x = xCenter;
	pose.y = yCenter;

	// move robot to center point and pause there
	arm->MoveStraightTo(pose);
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	cout << "point moved from " << initialPose.x << " " << initialPose.y <<
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt(sq(initialPose.x - pose.x) + sq(initialPose.y - pose.y)) << endl;

	return true;
}//end bisect

 /*
 runTargetScan:
 From scanUtils.cpp by S. Legowik:
 
 Description
 ===============================================
 Raster scan function, not currently used, but ported from Steve Legowik's code for reference.

 Parameters
 ========================================================
 
 */
void runTargetScan(CrpiRobot<CrpiUniversal> *arm, string filename, double stepSize, robotPose center)
{
	ofstream log(filename.c_str());
	int count = 0; // not used
	bool ok;
	robotPose border;
	robotPose startPose;

	for (int xAxis = 0; xAxis<2; ++xAxis)
	{ // scan x or y direction
		for (int up = 0; up<2; ++up)
		{ // move up or down from center
			startPose = center;
			ok = true;
			while (ok)
			{
				// search to lower border
				border = startPose;
				arm->MoveStraightTo(startPose);
				ulapi_sleep(robot_settle_time / 1000.);
				ok = ok && accSearch(arm, border, false, (xAxis == 1), stepSize, count);
				if (ok)
					log << border.x << " " << border.y << endl;

				// search to higher border
				border = startPose;
				arm->MoveStraightTo(startPose);
				ulapi_sleep(robot_settle_time / 1000.);
				ok = ok && accSearch(arm, border, true, (xAxis == 1), stepSize, count);
				if (ok)
					log << border.x << " " << border.y << endl;

				// move up or down from center
				if (xAxis == 1)
				{
					// advance along y axis
					startPose.y += stepSize * (up == 1 ? 1 : -1);
				}//end if
				else
				{
					// advance along x axis
					startPose.x += stepSize * (up == 1 ? 1 : -1);
				}//end else
			}//end while
		}//end for
	}//end for
}//end runtarget scan