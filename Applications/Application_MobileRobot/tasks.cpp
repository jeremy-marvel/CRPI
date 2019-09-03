/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

tasks.h

Description
===========

Main task code. (e.g. thread function definitions)
Also corresponds to the actual tasks each robot performs during registration.

Code Citations
==============

Based on:
	mobmanmain.cpp by S. Legowik
	unittest.cpp by J. Marvel

References
==========

	"_ftime, _ftime32, _ftime64 from MSDN"
		https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64


*/

#include <iostream>
#include <fstream>
#include <string>
#include <sys\timeb.h>
#include "ulapi.h"
#include "lynx_comm.h"
#include "lynx_comm_client.h"
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ur5_control.h"

using namespace std;
using namespace crpi_robot;

/*
lynx_task_code

Description
===============================================================
Used to test various functions related to the lynx ARCL connections.
Currently used to test the Outgoing ARCL Connection, specifically
*/
void lynx_task_code(void* args)
{
	int err;
	lynx_msg_pose cur_pose;
	int user_in;
	const int iter = 50;
	_timeb recv_time;
	int i = 0;
	char* msg_buf;
	char* goal_1 = (char*) square_goal1;
	
	lynx_comm* mobile_robot;
	mobile_robot = new lynx_comm(5353, TRUE);

	if ((err = mobile_robot->lynx_connect()) < 0)
	{
		mobile_robot->~lynx_comm();
		exit(err);
	}//end if

	mobile_robot->log_comment("seconds since epoch (s), robot_x (mm), robot_y (mm), robot_th (degrees), recv time (s), x recv time (s), y recv time (s), th recv time (s), time recv time (s)");

	while(1)
	{
		cout << endl;
		cout << "1. Lynx Pose Stream" << endl;
		cout << "2. ARCL Message Stream" << endl;
		cout << "3. Arrival Message Stream" << endl;
		cout << "Enter -1 to Quit" << endl;
		cout << "Please select a pose to transmit:" << endl;
		cin >> user_in;

		switch (user_in)
		{
			case 1:
				mobile_robot->log_comment("RobotPose");
				break;
		}//end switch

		switch(user_in)
		{
			case 1:
				while(1)
				{
					if ((err = mobile_robot->arcl_dsfv_pose(&cur_pose)) < 0)
					{
						mobile_robot->~lynx_comm();
						exit(err);
					}//end if


					cout << i << ", ";
					i++;
					_ftime(&recv_time);
					cur_pose.server_time = recv_time;
					mobile_robot->print_pose(&cur_pose);
					mobile_robot->log_pose(&cur_pose);

				}//end while

				break;

			case 2:

				while (1)
				{

					if ((err = mobile_robot->arcl_read(&msg_buf)) < 0)
					{
						mobile_robot->~lynx_comm();
						exit(err);
					}//end if

					cout << msg_buf << endl;

				}//end while

				break;

			case 3:

				while (1)
				{

					if ((err = mobile_robot->arcl_read_status(goal_1)) < 0)
					{
						mobile_robot->~lynx_comm();
						exit(err);
					}//end if

				}//end while

				break;

			case -1:
				mobile_robot->~lynx_comm();
				exit(err);

			default:
				cout << "Invalid option: Please try again" << endl;
				break;
		}//end switch
	}//end while
}//end lynx_control

 /*
 ur5_task_code_control

 Description
 ===============================================================
 Used to test various functions related to manipualtor arm pose transformations and positioning.
 Can be used to manually initiate the staging and stowing of the arm, as well as test registration methods.
 */
void ur5_task_code_control(void* args)
{
	CanonReturn retval;
	robotPose cur_pose, world_pose, large_target;
	robotPose sq_search_pose1;
	ofstream point_spiral;

	robotPose start_r;
	PM_CARTESIAN square_point1_world;
	PM_CARTESIAN square_point2_world;
	int mytime_seconds = get_seconds();

	int step_count;
	double search_time;

	point_spiral.open("..\\Applications\\Application_MobileRobot\\point_spiral.csv");

	int user_in;

	cout << "Connecting to UR5 controller..." << endl;
	CrpiRobot<CrpiUniversal> ur5("universal_ur5_agv.xml");//Please note the filename of this xml document
	//It contains parameters CRPI uses to initialize and control the robot with.
	//The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
	//of the UR5 so that the coordinate system aligns with the coordinate system of the lynx.
	
	do
	{
		cout << "Waiting to connect to the UR5 controller..." << endl;
		ulapi_sleep(.1);
		ur5.GetRobotPose(&cur_pose);
	} while (cur_pose.x==0 && cur_pose.y == 0 && cur_pose.z == 0);
	
	cout << "Connection established." << endl;

	ur5.SetAngleUnits("degree");
	ur5.SetLengthUnits("mm");
	ur5.SetAbsoluteSpeed(.1);
	ur5.SetAbsoluteAcceleration(0.1f);
	ur5.Couple("laser");//Sets a tool that is defined with the xml document.

	ulapi_sleep(.1);

	do
	{
		cout << "1. Stage Arm" << endl;
		cout << "2. Stow Arm" << endl;
		cout << "3. Print current pose" << endl;
		cout << "4. Perform spiral search at first square point" << endl;
		cout << "5. Perform spiral search at first square point offset" << endl;
		cout << "6. Search Square RMMA Pattern" << endl;
		cout << "7. Search Circle RMMA Pattern" << endl;
		cout << "8. Compute start points from test constants and spiral search" << endl;
		cout << "9. Test bisect registration" << endl;
		cout << "10. Search square RMMA pattern with bisection" << endl;
		cout << "11. Search square RMMA pattern with bisection (Two-Point version)" << endl;
		cout << "Enter -1 to quit" << endl;
		cin >> user_in;

		switch (user_in)
		{
		case 1:
			stage_arm(&ur5);
			break;
		case 2:
			stow_arm(&ur5);
			break;
		case 3:
			ur5.GetRobotPose(&cur_pose);
			cout << "TCP Currently at: ";
			cur_pose.print();
			cout << endl;
			break;
		case 4:
			setPosition(sq_search_pose1, square_point1_ur5, sensor_rot);
			spiral_search(&ur5, sq_search_pose1, cur_pose, point_spiral, mytime_seconds);
			break;
		case 5:
			setPosition(sq_search_pose1, square_point1_ur5_offset, sensor_rot);
			spiral_search(&ur5, sq_search_pose1, cur_pose, point_spiral, mytime_seconds);
			break;
		case 6:
			square(&ur5, square_point1_ur5_offset, square_point2_ur5_offset, numIters, mytime_seconds);
			break;
		case 7:
			circle(&ur5, circle_point1_ur5_offset, circle_point2_ur5_offset, numIters, mytime_seconds);
			break;
		case 8:

			compute_start(square_point1_world, square_point2_world, large_square_point1, large_square_point2, square_lynx_goal5, lynx_pose_test);//lynx_pose_test);

			cout << "Point 1: (" << square_point1_world.x << ", " << square_point1_world.y <<", "<< square_point1_world.z<<")"<<endl;
			cout << "Point 2: (" << square_point2_world.x << ", " << square_point2_world.y <<", "<< square_point2_world.z<<")"<<endl;
			//setPosition(sq_search_pose1, square_point1_world, sensor_rot);
			square_bisect_short(&ur5, square_point1_world, square_point2_world, 2, mytime_seconds, -1);
			break;
		case 9:
			setPosition(large_target, large_square_point1, sensor_rot);
			
			if (!bisect(&ur5, large_target, largeTargetStepSize, &step_count, &search_time))
			{
				cout << "bisect failure" << endl;
			}//end if

			break;

		case 10:
			square_bisect(&ur5, large_square_point1, large_square_point2, numIters, mytime_seconds);
			break;
		
		case 11:
			square_bisect_short(&ur5, large_square_point1, large_square_point2, numIters, mytime_seconds, -1);
			break;

		default:
			cout << "Invalid Choice Entered" << endl;
			break;

		}//end switch
	} while (user_in != -1);

	point_spiral.close();

	cout << "All Done." << endl;

}//end ur5 control

 /*
 test_square

 Description
 ===============================================================
 Helper function for main performance test. Program waits to recieve a specified ARCL message from the lynx core, then
 initiates the manipulator arm registration.

 Parameters
 ===============================================================
 lynx_robot -- Object containing the lynx ARCL communication parameters
 ur_robot -- CRPI object handling communications and control of manipulator arm
 large1, large2 -- expected positions of bisect reflectors
 square_lynx -- commanded position of the lynx when docking next to RMMA
 goal -- String containing the ARCL broadcast message that is used to initiate the performance test.
 
 */
void test_square(lynx_comm *lynx_robot, CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN large1, PM_CARTESIAN large2, PM_POSE square_lynx, char* goal)
{

	CanonReturn retval;
	robotPose cur_pose, world_pose;
	robotPose sq_search_pose1;
	robotPose start_r;
	PM_CARTESIAN square_point1_world;
	PM_CARTESIAN square_point2_world;
	PM_POSE* pm_lynx;
	crpi_timer pause;
	int err;
	lynx_msg_pose cur_lynx_pose;
	int mytime_seconds;

	cout << "Waiting for Lynx to dock at " << goal << endl;

	if ((err = lynx_robot->arcl_read_status(goal)) < 0)
		exit(err);

	mytime_seconds = get_seconds();

	cout << "Lynx arrived at " << goal << endl;
	cout << "Now staging manipulator..." << endl;

	if ((retval = stage_arm(ur_robot)) != CANON_SUCCESS)
		exit(retval);

	cout << "Manipulator staged" << endl;

	pause.waitUntil(robot_settle_time);

	lynx_robot->arcl_dsfv_pose(&cur_lynx_pose);

	lynx_robot->log_pose(&cur_lynx_pose);

	pm_lynx = new PM_POSE(PM_CARTESIAN(cur_lynx_pose.robot_x, cur_lynx_pose.robot_y, 0), PM_RPY(0, 0, cur_lynx_pose.robot_th * TO_RAD));

	cout << "Lynx Point: (" << cur_lynx_pose.robot_x << ", " << cur_lynx_pose.robot_y <<","<<cur_lynx_pose.robot_th<<")"<< endl;

	compute_start(square_point1_world, square_point2_world, large1, large2, square_lynx, *pm_lynx);//lynx_pose_test);

	cout << "Start Point 1: (" << square_point1_world.x << ", " << square_point1_world.y << endl;
	cout << "Start Point 2: (" << square_point2_world.x << ", " << square_point2_world.y << endl;
	square_bisect_short(ur_robot, square_point1_world, square_point2_world, numIters, mytime_seconds, 3);

	cout << "Now stowing manipulator..." << endl;
	if ((retval = stow_arm(ur_robot)) != CANON_SUCCESS)
		exit(retval);
	cout << "Manipulator stowed" << endl;
}//end void

 /*
 performance_test

 Description
 ===============================================================
 Main rotuine for executing the mobile manipulator performance test. This version is
 "manual" in the sense that the operator must send the adept lynx to the goal points at the RMMA.
 */
void performance_test(void* args)
{
	CanonReturn retval;
	robotPose cur_pose;
	ofstream point_spiral;
	int err;
	char* goal_1 = (char*)square_route1;
	char* goal_2 = (char*)square_route2;

//Initialize and Connect to Lynx ARCL Server
	cout << "Connecting to ARCL Server..." << endl;
	lynx_comm mobile_robot(5353, TRUE);
	
	if ((err = mobile_robot.lynx_connect()) < 0)
		exit(err);

//Initialize and Connect to UR5 Controller
	cout << "Connecting to UR5 controller..." << endl;
	CrpiRobot<CrpiUniversal> ur5("universal_ur5_agv.xml");//Please note the filename of this xml document
	//It contains parameters CRPI uses to initialize and control the robot with.
	//The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
	//of the UR5 so that the coordinate system aligns with the coordinate system of the lynx.

	do
	{
		cout << "Waiting to connect to the UR5 controller..." << endl;
		ulapi_sleep(.1);
		ur5.GetRobotPose(&cur_pose);
	} while (cur_pose.x == 0 && cur_pose.y == 0 && cur_pose.z == 0);

	cout << "Connection established to UR5 Controller." << endl;

	ur5.SetAngleUnits("degree");
	ur5.SetLengthUnits("mm");
	ur5.SetAbsoluteSpeed(.1);
	ur5.SetAbsoluteAcceleration(0.001f);
	ur5.Couple("laser");//Sets a tool that is stored within the xml document.

	cout << "Now entering main loop. Press Ctrl-C at any time to terminate the program" << endl;

	while (1)
	{
		test_square(&mobile_robot, &ur5, large_square_point1_goal3, large_square_point2_goal3, square_lynx_goal3, goal_1);
		test_square(&mobile_robot, &ur5, large_square_point1_goal5, large_square_point2_goal5, square_lynx_goal5, goal_2);
	}//end while

}//end performance test

 /*
 performance_test

 Description
 ===============================================================
 Main rotuine for executing the mobile manipulator performance test.
 This automatic version controls the lynx and manipulator arm simultaenously,
 sending the lynx to goal points, executing the manipulator registration, then sending the
 lynx to the next goal.
 */
void performance_test_auto(void* args)
{
	CanonReturn retval;
	robotPose cur_pose;
	ofstream point_spiral;
	int err;
	char* goal_1 = (char*)square_route1;
	char* goal_2 = (char*)square_route2;
	char* goal_s = (char*)stage_route;
	char* goal_d = (char*)dock_route;

	//Initialize and Connect to Lynx ARCL Server
	cout << "Connecting to Outgoing ARCL Client..." << endl;
	lynx_comm mobile_robot(5353, TRUE);

	cout << "Connecting to ARCL Server" << endl;
	lynx_comm_client mobile_robot_cmd("10.0.127.35", 7171, "adept", false);

	if ((err = mobile_robot.lynx_connect()) < 0)
		exit(err);

	if ((err = mobile_robot_cmd.lynx_connect()) < 0)
		exit(err);

	//Initialize and Connect to UR5 Controller
	cout << "Connecting to UR5 controller..." << endl;
	CrpiRobot<CrpiUniversal> ur5("universal_ur5_agv.xml");//Please note the filename of this xml document
	//It contains parameters CRPI uses to initialize and control the robot with.
	//The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
	//of the UR5 so that the coordinate system aligns with the coordinate system of the lynx.

	do
	{
		cout << "Waiting to connect to the UR5 controller..." << endl;
		ulapi_sleep(.1);
		ur5.GetRobotPose(&cur_pose);
	} while (cur_pose.x == 0 && cur_pose.y == 0 && cur_pose.z == 0);

	cout << "Connection established to UR5 Controller." << endl;

	ur5.SetAngleUnits("degree");
	ur5.SetLengthUnits("mm");
	ur5.SetAbsoluteSpeed(.1);
	ur5.SetAbsoluteAcceleration(0.001f);
	ur5.Couple("laser");//Sets a tool that is defined within the xml document.

	cout << "Sending Mobile Robot to RMMA from Dock" << endl;

	if ((err = mobile_robot_cmd.arcl_write("patrolOnce Route-Stage\n")) < 0)
		exit(err);

	test_square(&mobile_robot, &ur5, large_square_point1_goal3, large_square_point2_goal3, square_lynx_goal3, goal_s);


	cout << "Now entering main loop. Press Ctrl-C at any time to terminate the program" << endl;

	for(int i=0; i<2; i++)
	{
		if ((err = mobile_robot_cmd.arcl_write("patrolOnce Route-Goal5\n")) < 0)
			exit(err);

		test_square(&mobile_robot, &ur5, large_square_point1_goal5, large_square_point2_goal5, square_lynx_goal5, goal_2);

		if ((err = mobile_robot_cmd.arcl_write("patrolOnce Route-Goal3\n")) < 0)
			exit(err);

		test_square(&mobile_robot, &ur5, large_square_point1_goal3, large_square_point2_goal3, square_lynx_goal3, goal_1);

	}//end while

	if ((err = mobile_robot_cmd.arcl_write("patrolOnce Route-DockHelp\n")) < 0)
		exit(err);

	if ((err = mobile_robot.arcl_read_status(goal_d)) < 0)
		exit(err);

	if ((err = mobile_robot_cmd.arcl_write("dock\n")) < 0)
		exit(err);

}//end performance test