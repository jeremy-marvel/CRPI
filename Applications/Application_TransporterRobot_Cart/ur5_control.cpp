/**
*\file ur5_control.cpp
*\brief Function implementation for controlling the ur5 arm. Includes pose conversion helper functions, spiral searches, and laser registration methods.
*Note: some comments that appear in this file were made by the original author of the copied or adapted functions, S. Legowik.\n

*setPosition(), cart2rob(), pm2robotConvert(), robot2pmConvert(), rob2cart(), time_since(int seconds), get_seconds(), \n
*copied from mobmanmain.cpp by S. Legowik\n

*bisect(), accSearch(), runTargetScan() copied from scanUtils.cpp by S. Legowik\n

*compute_start(), spiral_search(), square(), circle(), square_bisect(), circle_bisect(), square_bisect_short() are adapted\n
*from scanUtils.cpp by S. Legowik\n

*Adapted from:\n
*mobmanmain.cpp by S. Legowik\n
*scanUtils.cpp by S. Legowik

*\author Omar Aboul-Enein
*\date 2017-06-05
*/

#include <iostream>
#include "ur5_control.h"
#include "AssemblyPrims.h"
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ld_msg.h"

//using namespace std;
using namespace crpi_robot;
using namespace MotionPrims;

//CONVERSION FUNCTIONS

void setPosition(robotPose &pose, const PM_CARTESIAN point, const double rot[])
{
	pose.x = point.x;
	pose.y = point.y;
	pose.z = point.z;
	pose.xrot = rot[0];
	pose.yrot = rot[1];
	pose.zrot = rot[2];
}//end setPosition

robotPose cart2rob(PM_CARTESIAN position, robotPose orientation)
{
	robotPose pose = orientation;
	pose.x = position.x;
	pose.y = position.y;

	return pose;
}//end cart2rob

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

PM_CARTESIAN rob2cart(robotPose pose)
{
	return PM_CARTESIAN(pose.x, pose.y, sensor_height);
}//end rob2cart

void compute_start(PM_CARTESIAN& point1_world, PM_CARTESIAN& point2_world, PM_CARTESIAN point1_ur5, PM_CARTESIAN point2_ur5, PM_POSE rmma_to_ld, PM_POSE ld_pose)
{

	cout << "Point 1: (" << point1_ur5.x << ", " << point1_ur5.y << ", " << point1_ur5.z << ")" << endl;
	cout << "Point 2: (" << point2_ur5.x << ", " << point2_ur5.y << ", " << point1_ur5.z << ")" << endl;

//Compute world coordinates of expected reflector points based on commanded ld pose.
	point1_world = rmma_to_ld * ld_to_ur5 * point1_ur5;
	point2_world = rmma_to_ld * ld_to_ur5 * point2_ur5;

	
	cout << "Point 1: (" << point1_world.x << ", " << point1_world.y <<", "<<point1_world.z<<")" << endl;
	cout << "Point 2: (" << point2_world.x << ", " << point2_world.y <<", "<<point1_world.z<<")"<< endl;

//Compute inverse using actual position
	point1_world = inv(ld_to_ur5) * inv(ld_pose) * point1_world;
	point2_world = inv(ld_to_ur5) * inv(ld_pose) * point2_world;

	
	cout << "Point 1: (" << point1_world.x << ", " << point1_world.y <<", "<<point1_world.z<<")"<<endl;
	cout << "Point 2: (" << point2_world.x << ", " << point2_world.y <<", "<<point1_world.z<<")"<< endl;
	

	return;

}//end compute_start

void compute_start(PM_CARTESIAN& point_world, PM_CARTESIAN point_ur5, PM_POSE rmma_to_ld, PM_POSE ld_pose)
{

	cout << "Point 1: (" << point_ur5.x << ", " << point_ur5.y << ", " << point_ur5.z << ")" << endl;

	//Compute world coordinates of expected reflector points based on commanded ld pose.
	point_world = rmma_to_ld * ld_to_ur5 * point_ur5;


	cout << "Point 1: (" << point_world.x << ", " << point_world.y << ", " << point_world.z << ")" << endl;

	//Compute inverse using actual position
	point_world = inv(ld_to_ur5) * inv(ld_pose) * point_world;

	cout << "Point 1: (" << point_world.x << ", " << point_world.y << ", " << point_world.z << ")" << endl;

	return;

}//end compute_start

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

int get_seconds()
{
	SYSTEMTIME sysTime;
	GetSystemTime(&sysTime);

	return
		sysTime.wHour * 60 * 60 +
		sysTime.wMinute * 60 +
		sysTime.wSecond;
}//end get_seconds


CanonReturn lower_feet(CrpiRobot<CrpiUniversal> * ur_robot)
{
	CanonReturn retval;

	if ((retval = ur_robot->SetRobotDO(4, true)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 5" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);

	if ((retval = ur_robot->SetRobotDO(6, true)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 5" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);

	//Left = 4.2 Sec
	//Right = 3.9 Sec

	if ((retval = ur_robot->SetRobotDO(5, true)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 5" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);

	if ((retval = ur_robot->SetRobotDO(7, true)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 7" << endl;
		return retval;
	}//end if

	ulapi_sleep(3.4);

	if ((retval = ur_robot->SetRobotDO(5, false)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 5" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);

	if ((retval = ur_robot->SetRobotDO(7, false)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 7" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);

	return CANON_SUCCESS;
}//end lower_feet

CanonReturn raise_feet(CrpiRobot<CrpiUniversal> * ur_robot)
{
	CanonReturn retval;

	if ((retval = ur_robot->SetRobotDO(4, false)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 4" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);
	
	if ((retval = ur_robot->SetRobotDO(6, false)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 6" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);
	
	if ((retval = ur_robot->SetRobotDO(5, true)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 5" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);

	if ((retval = ur_robot->SetRobotDO(7, true)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 7" << endl;
		return retval;
	}//end if

	ulapi_sleep(5);

	if ((retval = ur_robot->SetRobotDO(5, false)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 5" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);

	if ((retval = ur_robot->SetRobotDO(7, false)) != CANON_SUCCESS)
	{
		cout << "Error: Could not set DO pin 7" << endl;
		return retval;
	}//end if

	ulapi_sleep(.5);

	return retval;
}//end raise_feet

//CONTROL FUNCTIONS
CanonReturn stage_arm(CrpiRobot<CrpiUniversal> * ur_robot)
{
	CanonReturn retval = CANON_FAILURE;
	robotPose robot_stow, robot_stage_1, robot_stage_2, robot_stage_3;// , robot_stage_4;
	robotPose curPose;
	crpi_timer pause;

	//Copy pose constants from ur5_control.h into a CRPI pose used to command robot arm.
	setPosition(robot_stow, stow, sensor_rot);
	setPosition(robot_stage_1, stage_1, sensor_rot);
	setPosition(robot_stage_2, stage_2, sensor_rot);
	setPosition(robot_stage_3, stage_3, sensor_rot);
	//setPosition(robot_stage_4, stage_4, sensor_rot);

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
	//Please note that this was included for an older version of CRPI. Current versions now offer blocking moves.
	while (curPose.distance(robot_stage_1) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	pause.waitUntil(robot_settle_time*2.5);

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

	pause.waitUntil(robot_settle_time*2.5);

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

	pause.waitUntil(robot_settle_time*2.5);//Added 08/08/2019

	/* Old stage pose for lynx
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
	*/

	return retval;

}//end stage arm

CanonReturn stage_arm_edge(CrpiRobot<CrpiUniversal> * ur_robot, robotPose& edgePose)
{
	CanonReturn retval = CANON_FAILURE;
	robotPose robot_stow, robot_stage_1, robot_stage_2, robot_stage_3;// , robot_stage_4;
	robotPose curPose;
	crpi_timer pause;
	robotIO io;

	bool found = false;

	//Copy pose constants from ur5_control.h into a CRPI pose used to command robot arm.
	setPosition(robot_stow, stow, sensor_rot);
	setPosition(robot_stage_1, stage_1, sensor_rot);
	setPosition(robot_stage_2, stage_2, sensor_rot);
	setPosition(robot_stage_3, stage_3, sensor_rot);
	//setPosition(robot_stage_4, stage_4, sensor_rot);

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
	//Please note that this was included for an older version of CRPI. Current versions now offer blocking moves.
	while (curPose.distance(robot_stage_1) > 0.1)
	{

		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);

	}//end while

	pause.waitUntil(robot_settle_time*2.5);

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

	pause.waitUntil(robot_settle_time*2.5);

	cout << "Moving to ";
	robot_stage_3.print();
	cout << endl;

	if ((retval = ur_robot->MoveStraightTo(robot_stage_3, false)) != CANON_SUCCESS)
	{
			cout << retval << " could not move robot" << endl;
			return retval;

	}//end if

	while (curPose.distance(robot_stage_3) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);

		if (!found)
		{
			ur_robot->GetRobotIO(&io);

			if (!io.dio[8])
			{
				found = true;
				edgePose = curPose;
				edgePose.z = sensor_height;
			}//end if
		}//end if
		ulapi_sleep(.1);
	}//end while

	if (!found)
	{
		ur_robot->GetRobotPose(&curPose);
		edgePose = curPose;
		edgePose.z = sensor_height;
	}//end if

	/* Old stage pose for lynx
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
	*/

	return retval;

}//end stage arm

CanonReturn stow_arm(CrpiRobot<CrpiUniversal> * ur_robot)
{
	CanonReturn retval = CANON_FAILURE;
	robotPose robot_stow, robot_stage_1, robot_stage_2, robot_stage_3;// , robot_stage_4;
	robotPose curPose;
	crpi_timer pause;
	ur_robot->GetRobotPose(&curPose);


	//Convert pose constants from ur5_control.h to CRPI robot pose used to command the robot.
	setPosition(robot_stow, stow, sensor_rot);
	setPosition(robot_stage_1, stage_1, sensor_rot);
	setPosition(robot_stage_2, stage_2, sensor_rot);
	setPosition(robot_stage_3, stage_3, sensor_rot);
	//setPosition(robot_stage_4, stage_4, sensor_rot); Old stage pose for Lynx

	ur_robot->GetRobotPose(&curPose);

	if (curPose.distance(robot_stow) < 1) //Check that the robot arm is not already stowed.
	{
		cout << "Cannot move robot: robot already in stow position!" << endl;
		return retval;
	}//end if

	/* Old stage pose for Lynx
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
	*/

	cout << "Moving to ";
	robot_stage_2.print();
	cout << endl;

	if ((retval = ur_robot->MoveStraightTo(robot_stage_2)) != CANON_SUCCESS)
	{
		pause.waitUntil(robot_settle_time*2.5);

		if ((retval = ur_robot->MoveStraightTo(robot_stage_2)) != CANON_SUCCESS)
		{
			cout << retval << " could not move robot" << endl;
			return retval;
		}//end if

	}//end if

	while (curPose.distance(robot_stage_2) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	pause.waitUntil(robot_settle_time*2.5);

	cout << "Moving to ";
	robot_stage_1.print();
	cout << endl;
	if ((retval = ur_robot->MoveStraightTo(robot_stage_1)) != CANON_SUCCESS)
	{
		pause.waitUntil(robot_settle_time*2.5);
		
		if ((retval = ur_robot->MoveStraightTo(robot_stage_1)) != CANON_SUCCESS)
		{
			cout << retval << " could not move robot" << endl;
			return retval;
		}//end if

	}//end if

	while (curPose.distance(robot_stage_1) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	pause.waitUntil(robot_settle_time*2.5);

	cout << "Moving to ";
	robot_stow.print();
	cout << endl;
	if ((retval = ur_robot->MoveStraightTo(robot_stow)) != CANON_SUCCESS)
	{
		pause.waitUntil(robot_settle_time*2.5);
		
		if ((retval = ur_robot->MoveStraightTo(robot_stow)) != CANON_SUCCESS)
		{
			cout << retval << " could not move robot" << endl;
			return retval;
		}//end if

	}//end if

	while (curPose.distance(robot_stow) > 0.1)
	{
		ur_robot->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	return retval;

}//end stage arm

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
		pause.waitUntil(robot_settle_time*2.5); //(Changed to 2.5*robot_settle_time on 08/16/2019) You may need to increase this pause, adding pause after does not prevent long spiral search from happening.
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
	out_file << "NA"<<", "<<counter << ", ";
	out_file << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", "
		<< poseMe.yrot << ", " << poseMe.zrot << endl;

	cout << "CanonReturn: " << retval << endl;

	pose_me = poseMe;

	return retval;

}//end spiral search

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

CanonReturn square_bisect(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds)
{
	CanonReturn retval;
	ofstream square_spiral_bisect_csv;
	robotPose poseMe;
	robotPose position[10];
	robotPose start_pose;
	double theta = 0.0f;
	PM_CARTESIAN point2_offset;
	crpi_timer pause;
	
	char log_name [128];
	time_t log_time;
	tm* log_tm;
	
	//Prepare log file, the current date and time are used to give each log a unique name.
	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Data\\square_bisect_%d-%m-%Y_%I-%M-%S.csv", log_tm);

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

CanonReturn square_bisect_short(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds, char* goal, bool ots_pause, PM_CARTESIAN* updated_point1, PM_CARTESIAN* updated_point2)
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
	strftime(log_name, sizeof(log_name), "..\\Data\\square_bisect_short_%d-%m-%Y_%I-%M-%S.csv", log_tm);
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

	//Delay to allow for optitrack captures
	//ulapi_sleep(10);

	ur_robot->GetRobotIO(&io);

	square_spiral_bisect_csv << "Goal, " << goal << endl;
	square_spiral_bisect_csv << "time, iter, marker, ";
	square_spiral_bisect_csv << "init_x, init_y, init_z, init_xrot, init_yrot, init_zrot,";
	square_spiral_bisect_csv << "ok, time_elapsed, steps,";
	square_spiral_bisect_csv << "x, y, z, xrot, yrot, zrot" << endl;

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

	if (success1 != 0 && ots_pause)
		pause.waitUntil(5000);

	//Delay to allow for optitrack captures
	//ulapi_sleep(10);

	ur_robot->MoveStraightTo(ref_point2);//Here is a good point of the code to adjust the location of the second bisect reflector.

	//Delay to allow for optitrack captures
	//ulapi_sleep(10);

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

	if (success2 != 0 && ots_pause)
		pause.waitUntil(5000);

	//Delay to allow for optitrack captures
	//ulapi_sleep(10);

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

	if (success1 == 0 || success2 == 0)//Note that if either bisection fails, the routine terminates.
	{
		return CANON_FAILURE;
	}//end if

	//Stored updated bisect locations to return

	updated_point1->x = ref_point1.x;
	updated_point1->y = ref_point1.y;
	updated_point1->z = sensor_height;

	updated_point2->x = ref_point2.x;
	updated_point2->y = ref_point2.y;
	updated_point2->z = sensor_height;

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
			{
				position[j] = poseMe;

				//Delay to allow for optitrack captures
				ulapi_sleep(10);
			}//end if

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

CanonReturn square_bisect_short(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN start_point1_large, PM_CARTESIAN start_point2_large, int num_iters, int mytime_seconds, char* goal, bool ots_pause)
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
	strftime(log_name, sizeof(log_name), "..\\Data\\square_bisect_short_%d-%m-%Y_%I-%M-%S.csv", log_tm);
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

	if (success1 != 0 && ots_pause)
		pause.waitUntil(15000);

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

	if (success2 != 0 && ots_pause)
		pause.waitUntil(15000);

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

			if (retval == CANON_SUCCESS && ots_pause)
				pause.waitUntil(15000);

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
}//end square_bisect_short

CanonReturn square_edge_short(CrpiRobot<CrpiUniversal>* ur_robot, PM_CARTESIAN start_point, double r1_start, double r3_start, double max_dist, double large_step, double r2_offset, int num_iters, int mytime_seconds, char* goal, bool ots_pause)
{
	CanonReturn retval = CANON_SUCCESS;
	ofstream square_spiral_edge_csv;
	robotPose poseMe;
	robotPose position[10];

	double theta = 0.0f;
	PM_CARTESIAN point2_offset;
	robotIO io;
	crpi_timer pause;

	char log_name[128];
	time_t log_time;
	tm* log_tm;

	int success1 = 1;
	int success2 = 1;
	int success3 = 1;

	double m;
	double b1;
	double b2;

	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Data\\square_edge_short_%d-%m-%Y_%I-%M-%S.csv", log_tm);
	cout << "Log file created at: " << log_name << endl;
	square_spiral_edge_csv.open(log_name);

	//setPosition(start_pose, start_point1, sensor_rot);

	////////////////////////////////////////////////////////////////////////////////////
	robotPose ref_point1, ref_point2, ref_point3, start_pose1, start_pose2, start_pose3, table_corner;
	PM_CARTESIAN corner_point;
	double searchTime1, searchTime2, searchTime3;
	int stepCount1, stepCount2, stepCount3;
	double ts;

	double start_point1_time, start_point2_time, start_point3_time;

	square_spiral_edge_csv << "Goal, " << goal << endl;
	square_spiral_edge_csv << "time, iter, marker, ";
	square_spiral_edge_csv << "init_x, init_y, init_z, init_xrot, init_yrot, init_zrot,";
	square_spiral_edge_csv << "ok, time_elapsed, steps,";
	square_spiral_edge_csv << "x, y, z, xrot, yrot, zrot" << endl;


	setPosition(ref_point1, start_point, sensor_rot);
	ref_point1.x = r1_start;
	setPosition(start_pose1, start_point, sensor_rot);

	ur_robot->GetRobotIO(&io);

	start_point1_time = time_since(mytime_seconds);

	if (!vertical_pan(ur_robot, ref_point1, large_step, 0.25, max_dist, &stepCount1, &searchTime1))
	{
		cout << "Error in vertical pan" << endl;
		success1 = 0;
	}//end if

	if (success1 != 0)
	{
		if (ots_pause)
			pause.waitUntil(15000);

		start_point2_time = time_since(mytime_seconds);

		setPosition(start_pose2, rob2cart(ref_point1), sensor_rot);
		
		if(r3_start > 0)
			start_pose2.y -= r2_offset;
		else
			start_pose2.y += r2_offset;

		setPosition(ref_point2, rob2cart(start_pose2), sensor_rot);

		if (!vertical_pan(ur_robot, ref_point2, large_step, 0.25, 25.4, &stepCount2, &searchTime2))
		{
			cout << "Error in vertical pan 2" << endl;
			success2 = 0;
		}//end if

		if (success2 != 0 && ots_pause)
			pause.waitUntil(15000);

		start_point3_time = time_since(mytime_seconds);
		setPosition(start_pose3, rob2cart(ref_point1), sensor_rot);
		start_pose3.y = r3_start;
		start_pose3.x += 2 * 25.4;
		start_pose3.z = sensor_height;
		setPosition(ref_point3, rob2cart(start_pose3), sensor_rot);

		if (r3_start > 0)
		{
			if (!horizontal_pan(ur_robot, ref_point3, large_step, 0.25, max_dist, false, &stepCount3, &searchTime3))
			{
				cout << "Error in horizontal pan" << endl;
				success3 = 0;
			}//end if
		}//end if
		if (r3_start < 0)
		{
			if (!horizontal_pan(ur_robot, ref_point3, large_step, 0.25, max_dist, true, &stepCount3, &searchTime3))
			{
				cout << "Error in horizontal pan" << endl;
				success3 = 0;
			}//end if
		}//end if

		if (success3 != 0 && ots_pause)
			pause.waitUntil(15000);

	}//end if

	if (success1 == 0 || success2 == 0 || success3 == 0)//Don't attempt verification loops because one of the reference points could not be found.
		return CANON_FAILURE;

	// write edge detection stats to CSV. ID's -1, -2, -3 refer to references points 1, 2, and 3.
	square_spiral_edge_csv << "Edge" << endl;
	square_spiral_edge_csv << start_point1_time << ", ";
	square_spiral_edge_csv << "-1, -1, " << start_pose1.x << ", " << start_pose1.y << ", " << start_pose1.z << ", "
		<< start_pose1.xrot << ", " << start_pose1.yrot << ", " << start_pose1.zrot;
	square_spiral_edge_csv << ", " << success1 << ", " << searchTime1 << ", " << stepCount1;
	square_spiral_edge_csv << ", " << ref_point1.x << ", " << ref_point1.y << ", " << ref_point1.z << ", "
		<< ref_point1.xrot << ", " << ref_point1.yrot << ", " << ref_point1.zrot << endl;

	square_spiral_edge_csv << start_point2_time << ", ";
	square_spiral_edge_csv << "-1, -2, " << start_pose2.x << ", " << start_pose2.y << ", " << start_pose2.z << ", "
		<< start_pose2.xrot << ", " << start_pose2.yrot << ", " << start_pose2.zrot;
	square_spiral_edge_csv << ", " << success2 << ", " << searchTime2 << ", " << stepCount2;
	square_spiral_edge_csv << ", " << ref_point2.x << ", " << ref_point2.y << ", " << ref_point2.z << ", "
		<< ref_point2.xrot << ", " << ref_point2.yrot << ", " << ref_point2.zrot << endl;

	square_spiral_edge_csv << start_point3_time << ", ";
	square_spiral_edge_csv << "-1, -3, " << start_pose3.x << ", " << start_pose3.y << ", " << start_pose3.z << ", "
		<< start_pose3.xrot << ", " << start_pose3.yrot << ", " << start_pose3.zrot;
	square_spiral_edge_csv << ", " << success3 << ", " << searchTime3 << ", " << stepCount3;
	square_spiral_edge_csv << ", " << ref_point3.x << ", " << ref_point3.y << ", " << ref_point3.z << ", "
		<< ref_point3.xrot << ", " << ref_point3.yrot << ", " << ref_point3.zrot << endl;

	m = (ref_point2.y - ref_point1.y) / (ref_point2.x - ref_point1.x);
	b1 = ref_point1.y - m * ref_point1.x;
	b2 = ref_point3.y + (1 / m)*ref_point3.x;
	corner_point.x = (b2 - b1) / (m + (1 / m));
	corner_point.y = m * corner_point.x + b1;
	corner_point.z = sensor_height;

	point2_offset = rob2cart(ref_point2 - ref_point1);
	point2_offset.z = 0;

	setPosition(table_corner, corner_point, sensor_rot);

	//Write stats for table corner to csv. Table corner has a target id of -4.
	square_spiral_edge_csv << start_point3_time << ", ";
	square_spiral_edge_csv << "-1, -4, " << "N/A" << ", " << "N/A" << ", " << "N/A" << ", "
		<< "NA" << ", " << "N/A" << ", " << "N/A";
	square_spiral_edge_csv << ", " << success3 * success2 * success1 << ", " << searchTime3 + searchTime2 + searchTime1 << ", " << stepCount3 + stepCount2 + stepCount1;
	square_spiral_edge_csv << ", " << table_corner.x << ", " << table_corner.y << ", " << table_corner.z << ", "
		<< table_corner.xrot << ", " << table_corner.yrot << ", " << table_corner.zrot << endl;


	//This code was only used to test the ability to find the corner.
	//ur_robot->MoveStraightTo(table_corner);
	//pause.waitUntil(robot_settle_time);
	//pause.waitUntil(10000);

	PM_CARTESIAN offsetAngle;
		
		if(r3_start > 0)
			offsetAngle = unit(PM_QUATERNION(PM_Z, PM_PI / 4) * point2_offset);//FRIDAY LEFT OFF WITH: Change this to PM_PI/4 and use (2/sqrt(2))*(4*25.4)*offsetAngle
		else
			offsetAngle = unit(PM_QUATERNION(PM_Z, -1*atan2(0.5, 1)) * point2_offset);

	//Corner refers to corner of the SQUARE not the table. This code is used to locate the first spiral position.
	//Some thoughts, remember that the spiral method uses two registration points before verification.
	//You may want to adjust your bisect methods so that the offsets from the bissect reflectors are used to imporove the search.
	//Here it seems, the PM_PI/2 is only used to find that first spiral reflector. After that point2offset is used.
	PM_CARTESIAN cornerOffset;
		
		if(r3_start > 0)
			cornerOffset = sqrt(2)*(4 * 25.4) * offsetAngle; //Was -2, changed to sqrt(2) * 4 since we are now looking at the table corner. Additional sqrt(2) scaling is needed so distance is not scaled on unit circle.
		else
			cornerOffset = sqrt(5)*(2 * 25.4) * offsetAngle;

	cout << "REFPOINT1=(" << ref_point1.x << ", " << ref_point1.y << ")" << endl;
	cout << "REFPOINT2=(" << ref_point2.x << ", " << ref_point2.y << ")" << endl;
	cout << "REFPOINT3=(" << ref_point3.x << ", " << ref_point3.y << ")" << endl;
	cout << "m=" << m << endl;
	cout << "b1=" << b1 << endl;
	cout << "b2=" << b2 << endl;
	cout << "corner_point.y=" << corner_point.y << endl;
	cout << "corner_point.x=" << corner_point.x << endl;
	cout << "offsetAngle_notnormed=(" << (PM_QUATERNION(PM_Z, PM_PI / 2) * point2_offset).x << ", " << (PM_QUATERNION(PM_Z, PM_PI / 2) * point2_offset).y << ")" << endl;
	cout << "offsetAngle=(" << offsetAngle.x << ", " << offsetAngle.y << ")" << endl;
	cout << "CORNEROFFSET=(" << cornerOffset.x << ", " << cornerOffset.y << ")" << endl;
	cout << "First position=(" << (rob2cart(table_corner) + cornerOffset).x << ", " << (rob2cart(table_corner) + cornerOffset).y << ")" << endl;


	setPosition(position[0], rob2cart(table_corner) + cornerOffset, sensor_rot);

	ur_robot->MoveStraightTo(position[0]);
	pause.waitUntil(robot_settle_time);

	square_spiral_edge_csv << "Fine Spiral Search" << endl;

	for (int i = 0; i < num_iters; ++i)
	{
		cout << "Starting iteration " << i << endl;
		for (int j = 0; j < 2; ++j) // Short version only searches two points to prevent ur5 from exceeding joint limits.
		{

			ts = time_since(mytime_seconds);
			square_spiral_edge_csv << ts << ", ";
			square_spiral_edge_csv << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
				<< ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";

			retval = spiral_search(ur_robot, position[j], poseMe, square_spiral_edge_csv, mytime_seconds);

			if (retval==CANON_SUCCESS && ots_pause)
				pause.waitUntil(15000);

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
					position[1].x = position[0].x + 18 * 25.4*(unit(point2_offset).x);
					position[1].y = position[0].y + 18 * 25.4*(unit(point2_offset).y);
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

	square_spiral_edge_csv.close();
	cout << "CanonReturn: " << retval << endl;

	return retval;

}//end square_edge_short

CanonReturn square_edge_short_cont(CrpiRobot<CrpiUniversal>* ur_robot, PM_CARTESIAN start_point, double r1_start, double r3_start, double max_dist, double large_step, double r2_offset, int num_iters, int mytime_seconds, char* goal, bool ots_pause)
{
	CanonReturn retval = CANON_SUCCESS;
	ofstream square_spiral_edge_csv;
	robotPose poseMe;
	robotPose position[10];

	double theta = 0.0f;
	PM_CARTESIAN point2_offset;
	robotIO io;
	crpi_timer pause;

	char log_name[128];
	time_t log_time;
	tm* log_tm;

	int success1 = 1;
	int success2 = 1;
	int success3 = 1;

	double m;
	double b1;
	double b2;

	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Data\\square_edge_short_cont_%d-%m-%Y_%I-%M-%S.csv", log_tm);
	cout << "Log file created at: " << log_name << endl;
	square_spiral_edge_csv.open(log_name);

	//setPosition(start_pose, start_point1, sensor_rot);

	////////////////////////////////////////////////////////////////////////////////////
	robotPose ref_point1, ref_point2, ref_point3, start_pose1, start_pose2, start_pose3, table_corner;
	PM_CARTESIAN corner_point;
	double searchTime1, searchTime2, searchTime3;
	int stepCount1, stepCount2, stepCount3;
	double ts;

	double start_point1_time, start_point2_time, start_point3_time;

	square_spiral_edge_csv << "Goal, " << goal << endl;
	square_spiral_edge_csv << "time, iter, marker, ";
	square_spiral_edge_csv << "init_x, init_y, init_z, init_xrot, init_yrot, init_zrot,";
	square_spiral_edge_csv << "ok, time_elapsed, steps,";
	square_spiral_edge_csv << "x, y, z, xrot, yrot, zrot" << endl;


	setPosition(ref_point1, start_point, sensor_rot);
	ref_point1.x = r1_start;
	setPosition(start_pose1, start_point, sensor_rot);

	ur_robot->GetRobotIO(&io);

	start_point1_time = time_since(mytime_seconds);

	if (!vertical_pan_cont(ur_robot, ref_point1, large_step, 0.25, max_dist, &stepCount1, &searchTime1))
	{
		cout << "Error in vertical pan" << endl;
		success1 = 0;
	}//end if

	if (success1 != 0)
	{
		if (ots_pause)
			pause.waitUntil(15000);

		start_point2_time = time_since(mytime_seconds);

		setPosition(start_pose2, rob2cart(ref_point1), sensor_rot);

		if (r3_start > 0)
			start_pose2.y -= r2_offset;
		else
			start_pose2.y += r2_offset;

		setPosition(ref_point2, rob2cart(start_pose2), sensor_rot);

		if (!vertical_pan_cont(ur_robot, ref_point2, large_step, 0.25, 25.4, &stepCount2, &searchTime2))
		{
			cout << "Error in vertical pan 2" << endl;
			success2 = 0;
		}//end if

		if (success2 != 0 && ots_pause)
			pause.waitUntil(15000);

		start_point3_time = time_since(mytime_seconds);
		setPosition(start_pose3, rob2cart(ref_point1), sensor_rot);
		start_pose3.y = r3_start;
		start_pose3.x += 2 * 25.4;
		start_pose3.z = sensor_height;
		setPosition(ref_point3, rob2cart(start_pose3), sensor_rot);

		if (r3_start > 0)
		{
			if (!horizontal_pan_cont(ur_robot, ref_point3, large_step, 0.25, max_dist, false, &stepCount3, &searchTime3))
			{
				cout << "Error in horizontal pan" << endl;
				success3 = 0;
			}//end if
		}//end if
		if (r3_start < 0)
		{
			if (!horizontal_pan_cont(ur_robot, ref_point3, large_step, 0.25, max_dist, true, &stepCount3, &searchTime3))
			{
				cout << "Error in horizontal pan" << endl;
				success3 = 0;
			}//end if
		}//end if

		if (success3 != 0 && ots_pause)
			pause.waitUntil(15000);

	}//end if

	if (success1 == 0 || success2 == 0 || success3 == 0)//Don't attempt verification loops because one of the reference points could not be found.
		return CANON_FAILURE;

	// write edge detection stats to CSV. ID's -1, -2, -3 refer to references points 1, 2, and 3.
	square_spiral_edge_csv << "Edge" << endl;
	square_spiral_edge_csv << start_point1_time << ", ";
	square_spiral_edge_csv << "-1, -1, " << start_pose1.x << ", " << start_pose1.y << ", " << start_pose1.z << ", "
		<< start_pose1.xrot << ", " << start_pose1.yrot << ", " << start_pose1.zrot;
	square_spiral_edge_csv << ", " << success1 << ", " << searchTime1 << ", " << stepCount1;
	square_spiral_edge_csv << ", " << ref_point1.x << ", " << ref_point1.y << ", " << ref_point1.z << ", "
		<< ref_point1.xrot << ", " << ref_point1.yrot << ", " << ref_point1.zrot << endl;

	square_spiral_edge_csv << start_point2_time << ", ";
	square_spiral_edge_csv << "-1, -2, " << start_pose2.x << ", " << start_pose2.y << ", " << start_pose2.z << ", "
		<< start_pose2.xrot << ", " << start_pose2.yrot << ", " << start_pose2.zrot;
	square_spiral_edge_csv << ", " << success2 << ", " << searchTime2 << ", " << stepCount2;
	square_spiral_edge_csv << ", " << ref_point2.x << ", " << ref_point2.y << ", " << ref_point2.z << ", "
		<< ref_point2.xrot << ", " << ref_point2.yrot << ", " << ref_point2.zrot << endl;

	square_spiral_edge_csv << start_point3_time << ", ";
	square_spiral_edge_csv << "-1, -3, " << start_pose3.x << ", " << start_pose3.y << ", " << start_pose3.z << ", "
		<< start_pose3.xrot << ", " << start_pose3.yrot << ", " << start_pose3.zrot;
	square_spiral_edge_csv << ", " << success3 << ", " << searchTime3 << ", " << stepCount3;
	square_spiral_edge_csv << ", " << ref_point3.x << ", " << ref_point3.y << ", " << ref_point3.z << ", "
		<< ref_point3.xrot << ", " << ref_point3.yrot << ", " << ref_point3.zrot << endl;

	m = (ref_point2.y - ref_point1.y) / (ref_point2.x - ref_point1.x);
	b1 = ref_point1.y - m * ref_point1.x;
	b2 = ref_point3.y + (1 / m)*ref_point3.x;
	corner_point.x = (b2 - b1) / (m + (1 / m));
	corner_point.y = m * corner_point.x + b1;
	corner_point.z = sensor_height;

	point2_offset = rob2cart(ref_point2 - ref_point1);
	point2_offset.z = 0;

	setPosition(table_corner, corner_point, sensor_rot);

	//Write stats for table corner to csv. Table corner has a target id of -4.
	square_spiral_edge_csv << start_point3_time << ", ";
	square_spiral_edge_csv << "-1, -4, " << "N/A" << ", " << "N/A" << ", " << "N/A" << ", "
		<< "NA" << ", " << "N/A" << ", " << "N/A";
	square_spiral_edge_csv << ", " << success3 * success2 * success1 << ", " << searchTime3 + searchTime2 + searchTime1 << ", " << stepCount3 + stepCount2 + stepCount1;
	square_spiral_edge_csv << ", " << table_corner.x << ", " << table_corner.y << ", " << table_corner.z << ", "
		<< table_corner.xrot << ", " << table_corner.yrot << ", " << table_corner.zrot << endl;


	//This code was only used to test the ability to find the corner.
	//ur_robot->MoveStraightTo(table_corner);
	//pause.waitUntil(robot_settle_time);
	//pause.waitUntil(10000);

	PM_CARTESIAN offsetAngle;

	if (r3_start > 0)
		offsetAngle = unit(PM_QUATERNION(PM_Z, PM_PI / 4) * point2_offset);//FRIDAY LEFT OFF WITH: Change this to PM_PI/4 and use (2/sqrt(2))*(4*25.4)*offsetAngle
	else
		offsetAngle = unit(PM_QUATERNION(PM_Z, -1 * atan2(0.5, 1)) * point2_offset);

	//Corner refers to corner of the SQUARE not the table. This code is used to locate the first spiral position.
	//Some thoughts, remember that the spiral method uses two registration points before verification.
	//You may want to adjust your bisect methods so that the offsets from the bissect reflectors are used to imporove the search.
	//Here it seems, the PM_PI/2 is only used to find that first spiral reflector. After that point2offset is used.
	PM_CARTESIAN cornerOffset;

	if (r3_start > 0)
		cornerOffset = sqrt(2)*(4 * 25.4) * offsetAngle; //Was -2, changed to sqrt(2) * 4 since we are now looking at the table corner. Additional sqrt(2) scaling is needed so distance is not scaled on unit circle.
	else
		cornerOffset = sqrt(5)*(2 * 25.4) * offsetAngle;

	cout << "REFPOINT1=(" << ref_point1.x << ", " << ref_point1.y << ")" << endl;
	cout << "REFPOINT2=(" << ref_point2.x << ", " << ref_point2.y << ")" << endl;
	cout << "REFPOINT3=(" << ref_point3.x << ", " << ref_point3.y << ")" << endl;
	cout << "m=" << m << endl;
	cout << "b1=" << b1 << endl;
	cout << "b2=" << b2 << endl;
	cout << "corner_point.y=" << corner_point.y << endl;
	cout << "corner_point.x=" << corner_point.x << endl;
	cout << "offsetAngle_notnormed=(" << (PM_QUATERNION(PM_Z, PM_PI / 2) * point2_offset).x << ", " << (PM_QUATERNION(PM_Z, PM_PI / 2) * point2_offset).y << ")" << endl;
	cout << "offsetAngle=(" << offsetAngle.x << ", " << offsetAngle.y << ")" << endl;
	cout << "CORNEROFFSET=(" << cornerOffset.x << ", " << cornerOffset.y << ")" << endl;
	cout << "First position=(" << (rob2cart(table_corner) + cornerOffset).x << ", " << (rob2cart(table_corner) + cornerOffset).y << ")" << endl;


	setPosition(position[0], rob2cart(table_corner) + cornerOffset, sensor_rot);

	ur_robot->MoveStraightTo(position[0]);
	pause.waitUntil(robot_settle_time);

	square_spiral_edge_csv << "Fine Spiral Search" << endl;

	for (int i = 0; i < num_iters; ++i)
	{
		cout << "Starting iteration " << i << endl;
		for (int j = 0; j < 2; ++j) // Short version only searches two points to prevent ur5 from exceeding joint limits.
		{

			ts = time_since(mytime_seconds);
			square_spiral_edge_csv << ts << ", ";
			square_spiral_edge_csv << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
				<< ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";

			retval = spiral_search(ur_robot, position[j], poseMe, square_spiral_edge_csv, mytime_seconds);

			if (retval == CANON_SUCCESS && ots_pause)
				pause.waitUntil(15000);

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
					position[1].x = position[0].x + 18 * 25.4*(unit(point2_offset).x);
					position[1].y = position[0].y + 18 * 25.4*(unit(point2_offset).y);
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

	square_spiral_edge_csv.close();
	cout << "CanonReturn: " << retval << endl;

	return retval;

}//end square_edge_short_cont

CanonReturn square_edge_short_cont2(CrpiRobot<CrpiUniversal>* ur_robot, PM_CARTESIAN start_point, double r3_start, double max_dist, double large_step, double r2_offset, int num_iters, int mytime_seconds, char* goal, bool ots_pause)
{
	CanonReturn retval = CANON_SUCCESS;
	ofstream square_spiral_edge_csv;
	robotPose poseMe;
	robotPose position[10];

	double theta = 0.0f;
	PM_CARTESIAN point2_offset;
	robotIO io;
	crpi_timer pause;

	char log_name[128];
	time_t log_time;
	tm* log_tm;

	int success1 = 1;
	int success2 = 1;
	int success3 = 1;

	double m;
	double b1;
	double b2;

	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Data\\square_edge_short_cont2_%d-%m-%Y_%I-%M-%S.csv", log_tm);
	cout << "Log file created at: " << log_name << endl;
	square_spiral_edge_csv.open(log_name);

	//setPosition(start_pose, start_point1, sensor_rot);

	////////////////////////////////////////////////////////////////////////////////////
	robotPose ref_point1, ref_point2, ref_point3, start_pose1, start_pose2, start_pose3, table_corner;
	PM_CARTESIAN corner_point;
	double searchTime1, searchTime2, searchTime3;
	int stepCount1L, stepCount1s, stepCount2L, stepCount2s, stepCount3L, stepCount3s;
	double ts;

	double start_point1_time, start_point2_time, start_point3_time;

	square_spiral_edge_csv << "Goal, " << goal << endl;
	square_spiral_edge_csv << "time, iter, marker, ";
	square_spiral_edge_csv << "init_x, init_y, init_z, init_xrot, init_yrot, init_zrot,";
	square_spiral_edge_csv << "ok, time_elapsed, steps_large, steps_small,";
	square_spiral_edge_csv << "x, y, z, xrot, yrot, zrot" << endl;


	setPosition(ref_point1, start_point, sensor_rot);
	setPosition(start_pose1, start_point, sensor_rot);

	start_point1_time = time_since(mytime_seconds);

	if (!vertical_pan_cont2(ur_robot, ref_point1, large_step, 0.25, max_dist, &stepCount1L, &stepCount1s, &searchTime1))
	{
		cout << "Error in vertical pan" << endl;
		success1 = 0;
	}//end if

	if (success1 != 0)
	{
		if (ots_pause)
			pause.waitUntil(15000);

		start_point2_time = time_since(mytime_seconds);

		setPosition(start_pose2, rob2cart(ref_point1), sensor_rot);

		if (r3_start > 0)
			start_pose2.y -= r2_offset;
		else
			start_pose2.y += r2_offset;

		setPosition(ref_point2, rob2cart(start_pose2), sensor_rot);

		if (!vertical_pan_cont(ur_robot, ref_point2, large_step, 0.25, max_dist, &stepCount2L, &stepCount2s, &searchTime2))
		{
			cout << "Error in vertical pan 2" << endl;
			success2 = 0;
		}//end if

		if (success2 != 0)
		{

			if(ots_pause)
				pause.waitUntil(15000);

			start_point3_time = time_since(mytime_seconds);

			ur_robot->GetRobotPose(&start_pose3);
			setPosition(ref_point3, rob2cart(start_pose3), sensor_rot);

			if (r3_start > 0)
			{
				if (!horizontal_pan_cont2(ur_robot, ref_point3, ref_point1, ref_point2, ref_point2.x, large_step, 0.25, max_dist, false, &stepCount3L, &stepCount3s, &searchTime3))
				{
					cout << "Error in horizontal pan" << endl;
					success3 = 0;
				}//end if
			}//end if
			if (r3_start < 0)
			{
				if (!horizontal_pan_cont2(ur_robot, ref_point3, ref_point1, ref_point2, ref_point1.x, large_step, 0.25, max_dist, true, &stepCount3L, &stepCount3s, &searchTime3))
				{
					cout << "Error in horizontal pan" << endl;
					success3 = 0;
				}//end if
			}//end if

			if (success3 != 0 && ots_pause)
				pause.waitUntil(15000);
		}//end if
	}//end if

	if (success1 == 0 || success2 == 0 || success3 == 0)//Don't attempt verification loops because one of the reference points could not be found.
		return CANON_FAILURE;

	// write edge detection stats to CSV. ID's -1, -2, -3 refer to references points 1, 2, and 3.
	square_spiral_edge_csv << "Edge" << endl;
	square_spiral_edge_csv << start_point1_time << ", ";
	square_spiral_edge_csv << "-1, -1, " << start_pose1.x << ", " << start_pose1.y << ", " << start_pose1.z << ", "
		<< start_pose1.xrot << ", " << start_pose1.yrot << ", " << start_pose1.zrot;
	square_spiral_edge_csv << ", " << success1 << ", " << searchTime1 << ", " << stepCount1L << ", "<<stepCount1s;
	square_spiral_edge_csv << ", " << ref_point1.x << ", " << ref_point1.y << ", " << ref_point1.z << ", "
		<< ref_point1.xrot << ", " << ref_point1.yrot << ", " << ref_point1.zrot << endl;

	square_spiral_edge_csv << start_point2_time << ", ";
	square_spiral_edge_csv << "-1, -2, " << start_pose2.x << ", " << start_pose2.y << ", " << start_pose2.z << ", "
		<< start_pose2.xrot << ", " << start_pose2.yrot << ", " << start_pose2.zrot;
	square_spiral_edge_csv << ", " << success2 << ", " << searchTime2 << ", " << stepCount2L << ", " << stepCount2s;
	square_spiral_edge_csv << ", " << ref_point2.x << ", " << ref_point2.y << ", " << ref_point2.z << ", "
		<< ref_point2.xrot << ", " << ref_point2.yrot << ", " << ref_point2.zrot << endl;

	square_spiral_edge_csv << start_point3_time << ", ";
	square_spiral_edge_csv << "-1, -3, " << start_pose3.x << ", " << start_pose3.y << ", " << start_pose3.z << ", "
		<< start_pose3.xrot << ", " << start_pose3.yrot << ", " << start_pose3.zrot;
	square_spiral_edge_csv << ", " << success3 << ", " << searchTime3 << ", " << stepCount3L << ", " << stepCount3s;
	square_spiral_edge_csv << ", " << ref_point3.x << ", " << ref_point3.y << ", " << ref_point3.z << ", "
		<< ref_point3.xrot << ", " << ref_point3.yrot << ", " << ref_point3.zrot << endl;

	if (ref_point2.x - ref_point1.x != 0 && ref_point2.y - ref_point1.y != 0)
	{
		m = (ref_point2.y - ref_point1.y) / (ref_point2.x - ref_point1.x);

		b1 = ref_point1.y - m * ref_point1.x;
		b2 = ref_point3.y + (1 / m)*ref_point3.x;
		corner_point.x = (b2 - b1) / (m + (1 / m));
		corner_point.y = m * corner_point.x + b1;
		corner_point.z = sensor_height;
	}//end if
	else
		return CANON_FAILURE;

	point2_offset = rob2cart(ref_point2 - ref_point1);
	point2_offset.z = 0;

	setPosition(table_corner, corner_point, sensor_rot);

	//Write stats for table corner to csv. Table corner has a target id of -4.
	square_spiral_edge_csv << start_point3_time << ", ";
	square_spiral_edge_csv << "-1, -4, " << "N/A" << ", " << "N/A" << ", " << "N/A" << ", "
		<< "NA" << ", " << "N/A" << ", " << "N/A";
	square_spiral_edge_csv << ", " << success3 * success2 * success1 << ", " << searchTime3 + searchTime2 + searchTime1 << ", " << stepCount3L + stepCount2L + stepCount1L<<", "<< stepCount3s + stepCount2s + stepCount1s;
	square_spiral_edge_csv << ", " << table_corner.x << ", " << table_corner.y << ", " << table_corner.z << ", "
		<< table_corner.xrot << ", " << table_corner.yrot << ", " << table_corner.zrot << endl;


	//This code was only used to test the ability to find the corner.
	//ur_robot->MoveStraightTo(table_corner);
	//pause.waitUntil(robot_settle_time);
	//pause.waitUntil(10000);

	PM_CARTESIAN offsetAngle;

	if (r3_start > 0)
		offsetAngle = unit(PM_QUATERNION(PM_Z, PM_PI / 4) * point2_offset);//FRIDAY LEFT OFF WITH: Change this to PM_PI/4 and use (2/sqrt(2))*(4*25.4)*offsetAngle
	else
		offsetAngle = unit(PM_QUATERNION(PM_Z, -1 * atan2(0.5, 1)) * point2_offset);

	//Corner refers to corner of the SQUARE not the table. This code is used to locate the first spiral position.
	//Some thoughts, remember that the spiral method uses two registration points before verification.
	//You may want to adjust your bisect methods so that the offsets from the bissect reflectors are used to imporove the search.
	//Here it seems, the PM_PI/2 is only used to find that first spiral reflector. After that point2offset is used.
	PM_CARTESIAN cornerOffset;

	if (r3_start > 0)
		cornerOffset = sqrt(2)*(4 * 25.4) * offsetAngle; //Was -2, changed to sqrt(2) * 4 since we are now looking at the table corner. Additional sqrt(2) scaling is needed so distance is not scaled on unit circle.
	else
		cornerOffset = sqrt(5)*(2 * 25.4) * offsetAngle;

	cout << "REFPOINT1=(" << ref_point1.x << ", " << ref_point1.y << ")" << endl;
	cout << "REFPOINT2=(" << ref_point2.x << ", " << ref_point2.y << ")" << endl;
	cout << "REFPOINT3=(" << ref_point3.x << ", " << ref_point3.y << ")" << endl;
	cout << "m=" << m << endl;
	cout << "b1=" << b1 << endl;
	cout << "b2=" << b2 << endl;
	cout << "corner_point.y=" << corner_point.y << endl;
	cout << "corner_point.x=" << corner_point.x << endl;
	cout << "offsetAngle_notnormed=(" << (PM_QUATERNION(PM_Z, PM_PI / 2) * point2_offset).x << ", " << (PM_QUATERNION(PM_Z, PM_PI / 2) * point2_offset).y << ")" << endl;
	cout << "offsetAngle=(" << offsetAngle.x << ", " << offsetAngle.y << ")" << endl;
	cout << "CORNEROFFSET=(" << cornerOffset.x << ", " << cornerOffset.y << ")" << endl;
	cout << "First position=(" << (rob2cart(table_corner) + cornerOffset).x << ", " << (rob2cart(table_corner) + cornerOffset).y << ")" << endl;


	setPosition(position[0], rob2cart(table_corner) + cornerOffset, sensor_rot);

	ur_robot->MoveStraightTo(position[0]);
	pause.waitUntil(robot_settle_time);

	square_spiral_edge_csv << "Fine Spiral Search" << endl;

	for (int i = 0; i < num_iters; ++i)
	{
		cout << "Starting iteration " << i << endl;
		for (int j = 0; j < 2; ++j) // Short version only searches two points to prevent ur5 from exceeding joint limits.
		{

			ts = time_since(mytime_seconds);
			square_spiral_edge_csv << ts << ", ";
			square_spiral_edge_csv << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
				<< ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";

			retval = spiral_search(ur_robot, position[j], poseMe, square_spiral_edge_csv, mytime_seconds);

			if (retval == CANON_SUCCESS && ots_pause)
				pause.waitUntil(15000);

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
					position[1].x = position[0].x + 18 * 25.4*(unit(point2_offset).x);
					position[1].y = position[0].y + 18 * 25.4*(unit(point2_offset).y);
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

	square_spiral_edge_csv.close();
	cout << "CanonReturn: " << retval << endl;

	return retval;

}//end square_edge_short_cont

 /*
 circle_bisect:

 Description
 ===============================================
 This function implements the spiral search multiple times to locate the set of reflectors on the RMMA organized along a circular shape.
 After the localization of each reflector, the location of the other reflectors are updated. Bisection registration is also used to
 further localize the start points of the circle pattern.

 NOTE: This function was intended for use on AGV/UR10 mobile manipulator and should not be used on the ld. This is because the ur5 arm cannot reach
 all of the reflectors that are specified in this search patttern. This code is useful, however to help develop new search patterns.

 Parameters
 ===========================================
 ur_robot -- CRPI object that stores configuration and connection parameters for robot arm.
 start_point1_large, start_point2_large -- expected position of the bisect reflectors used to compute the location of the rest of the square patern reflectors.
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

bool vertical_pan(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_x, int* stepCount, double* searchTime)
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;
	robotPose curPose;
	CanonReturn retval;

	bool found = false;

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

	// get current pose and io
	cout << "GETTING POSE" << endl;
	arm->GetRobotPose(&pose);
	cout << "GETTING IO" << endl;
	arm->GetRobotIO(&io);

	robotPose initialPose = pose;

	cout << "Panning to tape..." << endl;

	timer.start();

	// search for edge
	robotPose startPose = pose;
	robotPose commandPose = pose;

	cout << "Panning towards cart" << endl;

	while (io.dio[8] && abs(commandPose.x - startPose.x) <= abs(max_x))
	{
		commandPose.x += stepSize1;

		arm->MoveStraightTo(commandPose);
		pause.waitUntil(robot_settle_time);

		arm->GetRobotIO(&io);

		cout << "max_x = " << max_x << endl;
		cout << "io.dio[8]=" << io.dio[8] << endl;
		cout << "abs(commandPose.x - startPose.x) ="<< abs(commandPose.x - startPose.x) << endl;

		++count;
	}//end while

	if (!io.dio[8])
	{
		cout << "On tape refining edge position." << endl;
		while (!io.dio[8])
		{
			commandPose.x -= stepSize1;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			cout << "io.dio[8]=" << io.dio[8] << endl;

			++count;
		}//end while

		while (io.dio[8])
		{
			commandPose.x += stepSize2;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			cout << "io.dio[8]=" << io.dio[8] << endl;

			++count;
		}//end while

		found = true;

	}//end if
	else
	{
		cout << "Could not find tape, now attempting to search in other direction" << endl;

		commandPose.x -= stepSize1 * count;

		arm->MoveStraightTo(commandPose);

		while (curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotPose(&curPose);
			ulapi_sleep(.1);
		}//end while

		while (io.dio[8] && abs(commandPose.x - startPose.x) <= abs(max_x))
		{
			commandPose.x -= stepSize1;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			cout << "max_x = " << max_x << endl;
			cout << "io.dio[8]=" << io.dio[8] << endl;
			cout << "abs(commandPose.x - startPose.x) =" << abs(commandPose.x - startPose.x) << endl;

			++count;
		}//end while

		if (!io.dio[8])
		{
			cout<<"Found tape, now refining edge position" << endl;

			arm->GetRobotIO(&io);

			while (!io.dio[8])
			{
				commandPose.x -= stepSize1;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				cout << "io.dio[8]=" << io.dio[8] << endl;

				++count;
			}//end while

			while (io.dio[8])
			{
				commandPose.x += stepSize2;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				cout << "io.dio[8]=" << io.dio[8] << endl;

				++count;
			}//end while

			found = true;
		}//end if
	}//end else

	arm->GetRobotPose(&pose);

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();
	*stepCount = count;

	// move robot to center point and pause there
	arm->MoveStraightTo(pose);
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	cout << "point moved from " << initialPose.x << " " << initialPose.y <<
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt(sq(initialPose.x - pose.x) + sq(initialPose.y - pose.y)) << endl;

	return found;
}//end if

/*Continuous version*/
bool vertical_pan_cont(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_x, int* stepCount, double* searchTime)
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;
	robotPose curPose;
	CanonReturn retval;

	bool found = false;

	*stepCount = 0;
	*searchTime = 0;

	cout << "Moving to ";
	pose.print();
	cout << endl;

	if ((retval = arm->MoveStraightTo(pose, true)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
	}//end if

	while (curPose.distance(pose) > 0.1)
	{
		arm->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	// get current pose and io
	cout << "GETTING POSE" << endl;
	arm->GetRobotPose(&pose);
	cout << "GETTING IO" << endl;
	arm->GetRobotIO(&io);

	robotPose initialPose = pose;

	timer.start();

	// search for edge
	robotPose startPose = pose;
	robotPose commandPose = pose;

	//cout << "Panning towards cart" << endl;

	if (io.dio[8])
	{
		commandPose.x += max_x;

		arm->MoveStraightTo(commandPose, false);
		pause.waitUntil(robot_settle_time);

		while (io.dio[8] && curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotIO(&io);
			arm->GetRobotPose(&curPose);

			cout << "Panning to tape..." << endl;
			cout << "curPose.x =" << curPose.x << endl;
			cout << "io.dio[8]=" << io.dio[8] << endl;

			if (!io.dio[8])
			{
				commandPose = curPose;
				arm->MoveStraightTo(commandPose, true);
				pause.waitUntil(robot_settle_time);
			}//end if

		}//end while

	}//end if
	if (!io.dio[8])
	{
		
		cout << "On tape refining edge position." << endl;
		while (!io.dio[8])
		{
			commandPose.x -= stepSize1;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			cout << "io.dio[8]=" << io.dio[8] << endl;

			++count;
		}//end while

		while (io.dio[8])
		{
			commandPose.x += stepSize2;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			cout << "io.dio[8]=" << io.dio[8] << endl;

			++count;
		}//end while

		found = true;

	}//end if
	else
	{

		commandPose.x -= max_x;

		arm->MoveStraightTo(commandPose);

		while (curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotPose(&curPose);
			ulapi_sleep(.1);
		}//end while

		commandPose.x -= max_x;

		arm->MoveStraightTo(commandPose, false);
		pause.waitUntil(robot_settle_time);

		while (io.dio[8] && curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotIO(&io);
			arm->GetRobotPose(&curPose);

			cout << "Could not find tape, now attempting to search in other direction" << endl;
			cout << "curPose.x =" << curPose.x << endl;
			cout << "io.dio[8]=" << io.dio[8] << endl;

			if (!io.dio[8])
			{
				commandPose = curPose;
				arm->MoveStraightTo(commandPose, true);
				pause.waitUntil(robot_settle_time);
			}//end if

		}//end while

		if (!io.dio[8])
		{

			cout << "On tape refining edge position." << endl;
			while (!io.dio[8])
			{
				commandPose.x -= stepSize1;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				cout << "io.dio[8]=" << io.dio[8] << endl;

				++count;
			}//end while

			while (io.dio[8])
			{
				commandPose.x += stepSize2;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				cout << "io.dio[8]=" << io.dio[8] << endl;

				++count;
			}//end while

			found = true;
		}//end if
	}//end else

	arm->GetRobotPose(&pose);

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();
	*stepCount = count;

	// move robot to center point and pause there
	arm->MoveStraightTo(pose);
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	cout << "point moved from " << initialPose.x << " " << initialPose.y <<
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt(sq(initialPose.x - pose.x) + sq(initialPose.y - pose.y)) << endl;

	return found;
}//end if

bool vertical_pan_cont(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_x, int* stepCount1, int* stepCount2, double* searchTime)
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;
	robotPose curPose;
	CanonReturn retval;

	bool found = false;

	*stepCount1 = 0;
	*stepCount2 = 0;
	*searchTime = 0;

	cout << "Moving to ";
	pose.print();
	cout << endl;

	if ((retval = arm->MoveStraightTo(pose, true)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
	}//end if

	while (curPose.distance(pose) > 0.1)
	{
		arm->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while

	pause.waitUntil(robot_settle_time);//Added 08/08/2019

	// get current pose and io
	cout << "GETTING POSE" << endl;
	arm->GetRobotPose(&pose);
	cout << "GETTING IO" << endl;
	arm->GetRobotIO(&io);

	robotPose initialPose = pose;

	timer.start();

	// search for edge
	robotPose startPose = pose;
	robotPose commandPose = pose;

	//cout << "Panning towards cart" << endl;

	if (io.dio[8])
	{
		commandPose.x += max_x;

		arm->MoveStraightTo(commandPose, false);
		pause.waitUntil(robot_settle_time);

		while (io.dio[8] && curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotIO(&io);
			arm->GetRobotPose(&curPose);

			cout << "Panning to tape..." << endl;
			cout << "curPose.x =" << curPose.x << endl;
			cout << "io.dio[8]=" << io.dio[8] << endl;

			if (!io.dio[8])
			{
				commandPose = curPose;
				arm->MoveStraightTo(commandPose, true);
				pause.waitUntil(robot_settle_time*2.5);//Pause increased on 08/16/2019
			}//end if

		}//end while

	}//end if
	if (!io.dio[8])
	{

		cout << "On tape refining edge position." << endl;
		while (!io.dio[8])
		{
			commandPose.x -= stepSize1;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			cout << "io.dio[8]=" << io.dio[8] << endl;

			++count;
		}//end while

		*stepCount1 = count;

		while (io.dio[8])
		{
			commandPose.x += stepSize2;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			cout << "io.dio[8]=" << io.dio[8] << endl;

			++count;
		}//end while
		*stepCount2 = count;
		found = true;

	}//end if
	else
	{

		commandPose.x -= max_x;

		arm->MoveStraightTo(commandPose);

		while (curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotPose(&curPose);
			ulapi_sleep(.1);
		}//end while

		commandPose.x -= max_x;

		arm->MoveStraightTo(commandPose, false);
		pause.waitUntil(robot_settle_time);

		while (io.dio[8] && curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotIO(&io);
			arm->GetRobotPose(&curPose);

			cout << "Could not find tape, now attempting to search in other direction" << endl;
			cout << "curPose.x =" << curPose.x << endl;
			cout << "io.dio[8]=" << io.dio[8] << endl;

			if (!io.dio[8])
			{
				commandPose = curPose;
				arm->MoveStraightTo(commandPose, true);
				pause.waitUntil(robot_settle_time);
			}//end if

		}//end while

		if (!io.dio[8])
		{

			cout << "On tape refining edge position." << endl;
			while (!io.dio[8])
			{
				commandPose.x -= stepSize1;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				cout << "io.dio[8]=" << io.dio[8] << endl;

				++count;
			}//end while

			*stepCount1 = count;

			while (io.dio[8])
			{
				commandPose.x += stepSize2;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				cout << "io.dio[8]=" << io.dio[8] << endl;

				++count;
			}//end while

			*stepCount2 = count;

			found = true;
		}//end if
	}//end else

	arm->GetRobotPose(&pose);

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();

	// move robot to center point and pause there
	arm->MoveStraightTo(pose);
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	cout << "point moved from " << initialPose.x << " " << initialPose.y <<
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt(sq(initialPose.x - pose.x) + sq(initialPose.y - pose.y)) << endl;

	return found;
}//end if

//Base = (249.078234, -0.013650, 296.989460, 179.994362, 0.000818, -0.002229, -1, -1)
bool vertical_pan_cont2(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_x, int* stepCount1, int* stepCount2, double* searchTime)
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;
	robotPose curPose, stagePose, panLimit;

	bool found = false;

	setPosition(stagePose, stage_3, sensor_rot);
	setPosition(panLimit, pan_limit, sensor_rot);

	*stepCount1 = 0;
	*stepCount2 = 0;
	*searchTime = 0;

	// get current pose and io
	pause.waitUntil(robot_settle_time);//Added 11/20/2019
	cout << "GETTING POSE" << endl;
	arm->GetRobotPose(&pose);
	cout << "GETTING IO" << endl;
	arm->GetRobotIO(&io);
	pause.waitUntil(robot_settle_time);//Added 08/08/2019

	if (pose.distance(stagePose) > 0.1)
	{
		cout << "Error: Starting Point is ill-conditioned" << endl;
		return false;
	}//end if

	robotPose initialPose = pose;

	timer.start();

	// search for edge
	robotPose startPose = pose;
	robotPose commandPose = initialPose;

	cout << "Panning towards cart" << endl;

	if (io.dio[8])
	{
		commandPose = panLimit;
		cout << "NOT ON TAPE!" << endl;
		arm->MoveStraightTo(commandPose, false);
		pause.waitUntil(robot_settle_time);

		while (io.dio[8] && curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotIO(&io);
			arm->GetRobotPose(&curPose);

			cout << "Panning to tape..." << endl;
			cout << "curPose.x =" << curPose.x << endl;
			cout << "io.dio[8]=" << io.dio[8] << endl;

			if (!io.dio[8])
			{
				commandPose = curPose;
				arm->MoveStraightTo(commandPose, true);
				pause.waitUntil(robot_settle_time*2.5);//Pause increased on 08/16/2019
			}//end if

		}//end while
	}//end while
	if (!io.dio[8])
	{

		cout << "On tape refining edge position." << endl;
		while (!io.dio[8])
		{
			commandPose.x -= stepSize1;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			cout << "io.dio[8]=" << io.dio[8] << endl;

			++count;
		}//end while

		*stepCount1 = count;

		while (io.dio[8])
		{
			commandPose.x += stepSize2;
			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			cout << "io.dio[8]=" << io.dio[8] << endl;

			++count;
		}//end while

		*stepCount2 = count;

		found = true;

	}//end if
	else
	{

		commandPose.x = stagePose.x + max_x;

		arm->MoveStraightTo(commandPose, false);
		pause.waitUntil(robot_settle_time);

		while (io.dio[8] && curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotIO(&io);
			arm->GetRobotPose(&curPose);

			cout << "Could not find tape, now attempting to search in other direction" << endl;
			cout << "curPose.x =" << curPose.x << endl;
			cout << "io.dio[8]=" << io.dio[8] << endl;

			if (!io.dio[8])
			{
				commandPose = curPose;
				arm->MoveStraightTo(commandPose, true);
				pause.waitUntil(robot_settle_time);
			}//end if

		}//end while

		if (!io.dio[8])
		{

			cout << "On tape refining edge position." << endl;
			while (!io.dio[8])
			{
				commandPose.x -= stepSize1;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				cout << "io.dio[8]=" << io.dio[8] << endl;

				++count;
			}//end while

			*stepCount1 = count;

			while (io.dio[8])
			{
				commandPose.x += stepSize2;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				cout << "io.dio[8]=" << io.dio[8] << endl;

				++count;
			}//end while

			*stepCount2 = count;
			found = true;
		}//end if
	}//end else

	arm->GetRobotPose(&pose);

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();

	// move robot to center point and pause there
	arm->MoveStraightTo(pose);
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	cout << "point moved from " << initialPose.x << " " << initialPose.y <<
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt(sq(initialPose.x - pose.x) + sq(initialPose.y - pose.y)) << endl;

	return found;
}//end if

bool horizontal_pan(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_y, bool reverse, int* stepCount, double* searchTime)
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;
	robotPose curPose;
	CanonReturn retval;

	bool found = false;

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

	cout << "Panning to tape..." << endl;

	timer.start();

	// search for edge
	robotPose startPose = pose;
	robotPose commandPose = pose;

	while (io.dio[8] && abs(commandPose.y - startPose.y) <= abs(max_y))
	{
		if(!reverse)
			commandPose.y -= stepSize1;
		else
			commandPose.y += stepSize1;
		arm->MoveStraightTo(commandPose);
		pause.waitUntil(robot_settle_time);

		arm->GetRobotIO(&io);

		++count;
	}//end while
	if (!io.dio[8])
	{
		while (!io.dio[8])
		{
			if(!reverse)
				commandPose.y += stepSize1;
			else
				commandPose.y -= stepSize1;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while

		while (io.dio[8])
		{
			if (!reverse)
				commandPose.y -= stepSize2;
			else
				commandPose.y += stepSize2;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while

		found = true;
	}//end if
	else
	{
		if(!reverse)
			commandPose.y += stepSize1 * count;
		else
			commandPose.y -= stepSize1 * count;

		arm->MoveStraightTo(commandPose);

		while (curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotPose(&curPose);
			ulapi_sleep(.1);
		}//end while

		while (io.dio[8] && abs(commandPose.y - startPose.y) <= abs(max_y))
		{
			if(!reverse)
				commandPose.y += stepSize1;
			else
				commandPose.y -= stepSize1;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while

		if (!io.dio[8])
		{
			while (!io.dio[8])
			{
				if(!reverse)
					commandPose.y += stepSize1;
				else
					commandPose.y -= stepSize1;

				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				++count;
			}//end while

			while (io.dio[8])
			{
				if(!reverse)
					commandPose.y -= stepSize2;
				else
					commandPose.y += stepSize2;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				++count;
			}//end while

			found = true;
		}//end if
	}//end else

	arm->GetRobotPose(&pose);

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();
	*stepCount = count;

	// move robot to center point and pause there
	arm->MoveStraightTo(pose);
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	cout << "point moved from " << initialPose.x << " " << initialPose.y <<
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt(sq(initialPose.x - pose.x) + sq(initialPose.y - pose.y)) << endl;

	return found;
}//end if

bool horizontal_pan_cont(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double stepSize1, double stepSize2, double max_y, bool reverse, int* stepCount, double* searchTime)
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;
	robotPose curPose;
	CanonReturn retval;

	bool found = false;

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

	cout << "Panning to tape..." << endl;

	timer.start();

	// search for edge
	robotPose startPose = pose;
	robotPose commandPose = pose;

	if (io.dio[8])
	{

		if(!reverse)
			commandPose.y -= max_y;
		else
			commandPose.y += max_y;

		arm->MoveStraightTo(commandPose, false);
		pause.waitUntil(robot_settle_time);

		while (io.dio[8] && curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotIO(&io);
			arm->GetRobotPose(&curPose);

			cout << "Panning to tape..." << endl;
			cout << "curPose.y =" << curPose.y << endl;
			cout << "io.dio[8]=" << io.dio[8] << endl;

			if (!io.dio[8])
			{
				commandPose = curPose;
				arm->MoveStraightTo(commandPose, true);
				pause.waitUntil(robot_settle_time);
			}//end if

		}//end while

	}//end if
	if (!io.dio[8])
	{
		while (!io.dio[8])
		{
			if (!reverse)
				commandPose.y += stepSize1;
			else
				commandPose.y -= stepSize1;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while

		while (io.dio[8])
		{
			if (!reverse)
				commandPose.y -= stepSize2;
			else
				commandPose.y += stepSize2;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while

		found = true;
	}//end if
	else
	{
		if(!reverse)
			commandPose.y += max_y;
		else
			commandPose.y -= max_y;

		arm->MoveStraightTo(commandPose);

		while (curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotPose(&curPose);
			ulapi_sleep(.1);
		}//end while

		if (!reverse)
			commandPose.y += max_y;
		else
			commandPose.y -= max_y;

		arm->MoveStraightTo(commandPose, false);
		pause.waitUntil(robot_settle_time);

		while (io.dio[8] && curPose.distance(commandPose) > 0.1)
		{
			arm->GetRobotIO(&io);
			arm->GetRobotPose(&curPose);

			cout << "Could not find tape, now attempting to search in other direction" << endl;
			cout << "curPose.y =" << curPose.y << endl;
			cout << "io.dio[8]=" << io.dio[8] << endl;

			if (!io.dio[8])
			{
				commandPose = curPose;
				arm->MoveStraightTo(commandPose, true);
				pause.waitUntil(robot_settle_time);
			}//end if

		}//end while

		if (!io.dio[8])
		{
			while (!io.dio[8])
			{
				if (!reverse)
					commandPose.y += stepSize1;
				else
					commandPose.y -= stepSize1;

				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				++count;
			}//end while

			while (io.dio[8])
			{
				if (!reverse)
					commandPose.y -= stepSize2;
				else
					commandPose.y += stepSize2;
				arm->MoveStraightTo(commandPose);
				pause.waitUntil(robot_settle_time);

				arm->GetRobotIO(&io);

				++count;
			}//end while

			found = true;
		}//end if
	}//end else

	arm->GetRobotPose(&pose);

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();
	*stepCount = count;

	// move robot to center point and pause there
	arm->MoveStraightTo(pose);
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	cout << "point moved from " << initialPose.x << " " << initialPose.y <<
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt(sq(initialPose.x - pose.x) + sq(initialPose.y - pose.y)) << endl;

	return found;
}//end if

bool horizontal_pan_cont2(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, double offset, double stepSize1, double stepSize2, double max_y, bool reverse, int* stepCount1, int* stepCount2, double* searchTime)
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;
	robotPose curPose, stagePose, offsetPose;
	CanonReturn retval;

	bool found = false;

	*stepCount1 = 0;
	*stepCount2 = 0;
	*searchTime = 0;

	setPosition(stagePose, stage_3, sensor_rot);

	arm->GetRobotPose(&curPose);
	offsetPose = curPose;
	//offsetPose.x = offset + 1.5*25.4;
	offsetPose.x = offset + 1.75*25.4; //Changed 08/16/2019

	cout << "Moving to ";
	offsetPose.print();
	cout << endl;

	if ((retval = arm->MoveStraightTo(offsetPose)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
	}//end if

	while (curPose.distance(offsetPose) > 0.1)
	{
		arm->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while
	pause.waitUntil(robot_settle_time);//Added 08/08/2019

	// get current pose and io
	arm->GetRobotIO(&io);
	arm->GetRobotPose(&pose);

	robotPose initialPose = offsetPose;

	cout << "Panning to tape..." << endl;

	timer.start();

	// search for edge
	robotPose startPose = offsetPose;
	robotPose commandPose = offsetPose;

	if(!reverse)
		commandPose.y = stagePose.y + (13 * 25.4 + max_y);
	else
		commandPose.y = stagePose.y - (13 * 25.4 + max_y);

	arm->MoveStraightTo(commandPose, false);
	pause.waitUntil(robot_settle_time);

	while (io.dio[8] && curPose.distance(commandPose) > 0.1)
	{
		arm->GetRobotIO(&io);
		arm->GetRobotPose(&curPose);

		cout << "Panning to tape..." << endl;
		cout << "curPose.y =" << curPose.y << endl;
		cout << "io.dio[8]=" << io.dio[8] << endl;

		if (!io.dio[8])
		{
			commandPose = curPose;
			arm->MoveStraightTo(commandPose, true);
			pause.waitUntil(robot_settle_time*2.5);//Pause increased on 08/16/2019
		}//end if

	}//end while
	if (!io.dio[8])
	{
		while (!io.dio[8])
		{
			if (!reverse)
				commandPose.y += stepSize1;
			else
				commandPose.y -= stepSize1;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while

		*stepCount1 = count;

		while (io.dio[8])
		{
			if (!reverse)
				commandPose.y -= stepSize2;
			else
				commandPose.y += stepSize2;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while
		*stepCount2 = count;
		found = true;
	}//end if

	arm->GetRobotPose(&pose);

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();

	// move robot to center point and pause there
	arm->MoveStraightTo(pose);
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	cout << "point moved from " << initialPose.x << " " << initialPose.y <<
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt(sq(initialPose.x - pose.x) + sq(initialPose.y - pose.y)) << endl;

	return found;
}//end if

bool horizontal_pan_cont2(CrpiRobot<CrpiUniversal>* arm, robotPose &pose, robotPose r1, robotPose r2, double offset, double stepSize1, double stepSize2, double max_y, bool reverse, int* stepCount1, int* stepCount2, double* searchTime)
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;
	robotPose curPose, stagePose, offsetPose;
	CanonReturn retval;

	bool found = false;

	double m = 0;
	double b = 0;
	double b0 = 0;

	*stepCount1 = 0;
	*stepCount2 = 0;
	*searchTime = 0;

	setPosition(stagePose, stage_3, sensor_rot);

	arm->GetRobotPose(&curPose);
	offsetPose = curPose;

	// search for edge
	robotPose startPose = offsetPose;
	robotPose commandPose = offsetPose;

	cout << "r1=(" << r1.y << ", " << r1.x << ")" << endl;;
	cout << "r1=(" << r2.y << ", " << r2.x << ")" << endl;;
	if (r2.y - r1.y != 0)
	{
		m = (r2.x - r1.x) / (r2.y - r1.y);
		cout << "SLOPE=" << m << endl;
		cout << "r2.x-r1.x=" << (r2.x - r1.x) << endl;
		cout << "r2.y-r1.y=" << (r2.y - r1.y) << endl;
		cout << "DIVIDED=" << (r2.x - r1.x) / (r2.y - r1.y) << endl;
	}//end if
	else
	{
		cout << "ZERO!!!!" << endl;
		m = 0;
	}//end else

	b0 = r2.x + (1 / m)*r2.y+1.75*25.4;
	offsetPose.y = r2.y;
	offsetPose.x = (-1 / m)*offsetPose.y + b0;

	b = offsetPose.x - m * offsetPose.y;

	if (!reverse)
	{
		commandPose.y = stagePose.y + (13 * 25.4 + max_y);
		commandPose.x = (m * commandPose.y) + b;
	}//end if
	else
	{
		commandPose.y = stagePose.y - (13 * 25.4 + max_y);
		commandPose.x = (m * commandPose.y) + b;
	}//end else

	cout << "Moving to ";
	offsetPose.print();
	cout << endl;

	if ((retval = arm->MoveStraightTo(offsetPose)) != CANON_SUCCESS)
	{
		cout << retval << " could not move robot" << endl;
	}//end if

	while (curPose.distance(offsetPose) > 0.1)
	{
		arm->GetRobotPose(&curPose);
		ulapi_sleep(.1);
	}//end while
	pause.waitUntil(robot_settle_time);//Added 08/08/2019

	// get current pose and io
	arm->GetRobotIO(&io);
	arm->GetRobotPose(&pose);

	robotPose initialPose = offsetPose;

	cout << "Panning to tape..." << endl;

	timer.start();

	arm->MoveStraightTo(commandPose, false);
	pause.waitUntil(robot_settle_time);
	cout << "commandPose=(" << commandPose.x << ", " << commandPose.y << ")" << endl;
	cout << "m=" << m << endl;
	cout << "b=" << b << endl;
	cout<<"offsetPose=(" << offsetPose.x << ", " << offsetPose.y << ")" << endl;
	while (io.dio[8] && curPose.distance(commandPose) > 0.1)
	{
		arm->GetRobotIO(&io);
		arm->GetRobotPose(&curPose);

		cout << "Panning to tape..." << endl;
		cout << "curPose.y =" << curPose.y << endl;
		cout << "io.dio[8]=" << io.dio[8] << endl;

		if (!io.dio[8])
		{
			commandPose = curPose;
			arm->MoveStraightTo(commandPose, true);
			pause.waitUntil(robot_settle_time*2.5);//Pause increased on 08/16/2019
		}//end if

	}//end while
	if (!io.dio[8])
	{
		while (!io.dio[8])
		{
			if (!reverse)
				commandPose.y += stepSize1;
			else
				commandPose.y -= stepSize1;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while

		*stepCount1 = count;

		while (io.dio[8])
		{
			if (!reverse)
				commandPose.y -= stepSize2;
			else
				commandPose.y += stepSize2;

			arm->MoveStraightTo(commandPose);
			pause.waitUntil(robot_settle_time);

			arm->GetRobotIO(&io);

			++count;
		}//end while
		*stepCount2 = count;
		found = true;
	}//end if

	arm->GetRobotPose(&pose);

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();

	// move robot to center point and pause there
	arm->MoveStraightTo(pose);
	cout << "Pausing " << bisectPauseMs / 1000.0 << " seconds." << endl;
	pause.waitUntil(bisectPauseMs);

	cout << "point moved from " << initialPose.x << " " << initialPose.y <<
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt(sq(initialPose.x - pose.x) + sq(initialPose.y - pose.y)) << endl;

	return found;
}//end if

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