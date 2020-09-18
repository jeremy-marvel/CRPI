/**
*\file tasks.cpp
*\brief Main task code implementation. (e.g. thread function definitions). Contains tests for hardware/communications as well as control code that corresponds to the tasks each manipulator-on-cart performs during registration. 
*
*Based on:\n
*mobmanmain.cpp by S. Legowik\n
*unittest.cpp by J. Marvel\n\n
*
*References:\n
*"_ftime, _ftime32, _ftime64 from MSDN":\n
*https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64 \n\n
*
*Using safe string functions:\n
*https://stackoverflow.com/questions/11752705/does-stdstring-contain-null-terminator \n
*https://stackoverflow.com/questions/19196301/buffer-is-too-small-when-doing-strcpy-s \n
*docs.microsoft.com/en-us/cpp/c-runtime-library/reference/strcpy-s-wcscpy-s-mbscpy-s \n\n
*
*Recursive mutex:\n 
*https://stackoverflow.com/questions/20830525/cannot-lock-mutex-in-c/ \n\n
*
*\author Omar Aboul-Enein
*\date 2017-06-05
*/

#include <iostream>
#include <fstream>
#include <string>
#include <sys\timeb.h>
#include "ulapi.h"
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ur5_control.h"
#include "cart_comm_client.h"
#include "cart_status.h"
#include "tasks.h"
#include "cart_config.h"

using namespace std;
using namespace crpi_robot;

void ur5_task_code_control(void* args)
{
	CanonReturn retval;
	robotPose cur_pose, world_pose, large_target;
	robotPose sq_search_pose1;
	ofstream point_spiral;

	robotPose start_r;
	PM_CARTESIAN square_point1_world;
	PM_CARTESIAN square_point2_world;

	PM_CARTESIAN train_point1;
	PM_CARTESIAN train_point2;
	int train_in;

	robotPose check_pose;
	robotPose r3_start;

	int mytime_seconds = get_seconds();

	int step_count;
	int step_count2;
	double search_time;

	robotPose ref_point1;

	point_spiral.open("..\\Applications\\Application_MobileRobot\\point_spiral.csv");

	int user_in;

	int feet_raised = true;

	cart_config* saved_config = NULL;

	cout << "Connecting to UR5 controller..." << endl;
	CrpiRobot<CrpiUniversal> ur5(".\\universal_ur5_agv.xml");//Please note the filename of this xml document
														  //It contains parameters CRPI uses to initialize and control the robot with.
														  //The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
														  //of the UR5 so that the coordinate system aligns with the coordinate system of the ld.

	do
	{
		cout << "Waiting to connect to the UR5 controller..." << endl;
		ulapi_sleep(.1);
		ur5.GetRobotPose(&cur_pose);
	} while (cur_pose.x == 0 && cur_pose.y == 0 && cur_pose.z == 0);

	cout << "Connection established." << endl;

	ur5.SetAngleUnits("degree");
	ur5.SetLengthUnits("mm");
	ur5.SetAbsoluteSpeed(.1);
	ur5.SetAbsoluteAcceleration(0.1f);
	ur5.Couple("laser");//Sets a tool that is defined with the xml document.

	ulapi_sleep(.1);

	cout << "Raising Feet..." << endl;
	
	raise_feet(&ur5);

	cout << "Feet retracted." << endl;

	do
	{
		cout << "1. Stage Arm" << endl;
		cout << "2. Stow Arm" << endl;
		cout << "3. Print current pose" << endl;
		cout << "4. Train Initial Search Position for Large Reflectors" << endl;
		cout << "5. Train Initial Search Position for Large Reflectors (Static Load)" << endl;
		cout << "6. Train Initial Search Position for Edge Registration" << endl;
		cout << "7. Train Initial Search Position for Edge Registration (Static Load)" << endl;
		cout << "8. Perform spiral search at first square point" << endl;
		cout << "9. Perform spiral search at first square point offset" << endl;
		cout << "10. Search Square RMMA Pattern" << endl;
		cout << "11. Search Circle RMMA Pattern" << endl;
		cout << "12. Compute start points from test constants and spiral search" << endl;
		cout << "13. Test bisect registration" << endl;
		cout << "14. Search square RMMA pattern with bisection" << endl;
		cout << "15. Search square RMMA pattern with bisection (Two-Point version)" << endl;
		cout << "16. Lower Feet" << endl;
		cout << "17. Raise Feet" << endl;
		cout << "18. Test Continuous Vertical Pan" << endl;
		cout << "19. Test Continuous Horizontal Pan" << endl;
		cout << "20. Test Edge Registration (Side 1)" << endl;
		cout << "21. Test Edge Registration (Side 2)" << endl;
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
				saved_config = new cart_config(0);

				if (saved_config->get_err() == 0)
				{
					saved_config->read_file();

					cout << "Please enter the stop number to be updated: " << endl;
					cin >> train_in;
					cin.ignore();

					cout << "Please position the manipulator over the first large reflector and press 'enter'" << endl;
					cin.ignore();
					ur5.GetRobotPose(&cur_pose);
					train_point1.x = cur_pose.x;
					train_point1.y = cur_pose.y;
					train_point1.z = sensor_height;

					cout << "Please position the manipulator over the second large reflector and press 'enter'" << endl;
					cin.ignore();
					ur5.GetRobotPose(&cur_pose);
					train_point2.x = cur_pose.x;
					train_point2.y = cur_pose.y;
					train_point2.z = sensor_height;

					saved_config->update_large_points(train_in, train_point1, train_point2);

					saved_config->write_file();
					saved_config->~cart_config();

					cout << "Configuration successfully Updated..." << endl;
				}//end if

				delete saved_config;
				saved_config = NULL;

			break;
		case 5:
			saved_config = new cart_config(1);

			if (saved_config->get_err() == 0)
			{
				saved_config->read_file();

				cout << "Please enter the stop number to be updated: " << endl;
				cin >> train_in;
				cin.ignore();

				cout << "Please position the manipulator over the first large reflector and press 'enter'" << endl;
				cin.ignore();
				ur5.GetRobotPose(&cur_pose);
				train_point1.x = cur_pose.x;
				train_point1.y = cur_pose.y;
				train_point1.z = sensor_height;

				cout << "Please position the manipulator over the second large reflector and press 'enter'" << endl;
				cin.ignore();
				ur5.GetRobotPose(&cur_pose);
				train_point2.x = cur_pose.x;
				train_point2.y = cur_pose.y;
				train_point2.z = sensor_height;

				saved_config->update_large_points(train_in, train_point1, train_point2);

				saved_config->write_file();
				saved_config->~cart_config();

				cout << "Configuration successfully Updated..." << endl;
			}//end if

			delete saved_config;
			saved_config = NULL;

			break;
		case 6:
			saved_config = new cart_config(0);

			if (saved_config->get_err() == 0)
			{
				saved_config->read_file();

				cout << "Please enter the stop number to be updated: " << endl;
				cin >> train_in;
				cin.ignore();

				train_point1.z = sensor_height;

				cout << "Please position the manipulator over the long table edge and press 'enter'" << endl;
				cin.ignore();
				ur5.GetRobotPose(&cur_pose);
				train_point1.x = cur_pose.x;

				cout << "Please position the manipulator over the short table edge and press 'enter'" << endl;
				cin.ignore();
				ur5.GetRobotPose(&cur_pose);
				train_point1.y = cur_pose.y;

				saved_config->update_edge_points(train_in, train_point1);

				saved_config->write_file();
				saved_config->~cart_config();

				cout << "Configuration successfully Updated..." << endl;
			}//end if

			delete saved_config;
			saved_config = NULL;

			break;
		case 7:
			saved_config = new cart_config(1);

			if (saved_config->get_err() == 0)
			{
				saved_config->read_file();

				cout << "Please enter the stop number to be updated: " << endl;
				cin >> train_in;
				cin.ignore();

				train_point1.z = sensor_height;

				cout << "Please position the manipulator over the long table edge and press 'enter'" << endl;
				cin.ignore();
				ur5.GetRobotPose(&cur_pose);
				train_point1.x = cur_pose.x;

				cout << "Please position the manipulator over the short table edge and press 'enter'" << endl;
				cin.ignore();
				ur5.GetRobotPose(&cur_pose);
				train_point1.y = cur_pose.y;

				saved_config->update_edge_points(train_in, train_point1);

				saved_config->write_file();
				saved_config->~cart_config();

				cout << "Configuration successfully Updated..." << endl;
			}//end if

			delete saved_config;
			saved_config = NULL;

			break;
		case 8:
			setPosition(sq_search_pose1, square_point1_ur5, sensor_rot);
			spiral_search(&ur5, sq_search_pose1, cur_pose, point_spiral, mytime_seconds);
			break;
		case 9:
			setPosition(sq_search_pose1, square_point1_ur5_offset, sensor_rot);
			spiral_search(&ur5, sq_search_pose1, cur_pose, point_spiral, mytime_seconds);
			break;
		case 10:
			square(&ur5, square_point1_ur5_offset, square_point2_ur5_offset, numIters, mytime_seconds);
			break;
		case 11:
			circle(&ur5, circle_point1_ur5_offset, circle_point2_ur5_offset, numIters, mytime_seconds);
			break;
		case 12:

			compute_start(square_point1_world, square_point2_world, large_square_point1, large_square_point2, square_ld_goal5, ld_pose_test);//ld_pose_test);

			cout << "Point 1: (" << square_point1_world.x << ", " << square_point1_world.y << ", " << square_point1_world.z << ")" << endl;
			cout << "Point 2: (" << square_point2_world.x << ", " << square_point2_world.y << ", " << square_point2_world.z << ")" << endl;
			//setPosition(sq_search_pose1, square_point1_world, sensor_rot);
			square_bisect_short(&ur5, square_point1_world, square_point2_world, 2, mytime_seconds, "TEST", false);
			break;
		case 13:
			setPosition(large_target, large_square_point1, sensor_rot);

			if (!bisect(&ur5, large_target, largeTargetStepSize, &step_count, &search_time))
			{
				cout << "bisect failure" << endl;
			}//end if

			break;

		case 14:
			square_bisect(&ur5, large_square_point1, large_square_point2, numIters, mytime_seconds);
			break;

		case 15:
			square_bisect_short(&ur5, large_square_point1, large_square_point2, numIters, mytime_seconds, "TEST", false);
			break;

		case 16:
			if (feet_raised)
			{
				lower_feet(&ur5);
				feet_raised = false;
			}//end if
			else
				cout << "Cannot Lower Feet: Feet are already extended." << endl;
			break;
		case 17:
			raise_feet(&ur5);
			feet_raised = true;
			break;

		case 18:
			
			ur5.GetRobotPose(&cur_pose);

			setPosition(ref_point1, rob2cart(cur_pose), sensor_rot);
			ref_point1.x = 322.014531;

			if (!vertical_pan_cont2(&ur5, ref_point1, 2, 0.25, 4*25.4, &step_count, &step_count2, &search_time))
			{
				cout << "Error in vertical pan" << endl;
			}//end if
			break;
		case 19:
			ur5.GetRobotPose(&cur_pose);

			if (!horizontal_pan_cont2(&ur5, ref_point1, 322.014531, 2, 0.25, 4 * 25.4, false, &step_count, &step_count2, &search_time))
			{
				cout << "Error in vertical pan" << endl;
			}//end if
		case 20:

			setPosition(check_pose, stage_3, sensor_rot);
			ur5.GetRobotPose(&cur_pose);

			if (cur_pose.distance(check_pose) < 0.1)
				square_edge_short_cont2(&ur5, rob2cart(cur_pose), 10, 4 * 25.4, 2, 9 * 25.4, numIters, mytime_seconds, "TEST", true);
			else
				cout << "Could not start edge registration test. Pose check failed" << endl;
		case 21:

			setPosition(check_pose, stage_3, sensor_rot);
			ur5.GetRobotPose(&cur_pose);

			if (cur_pose.distance(check_pose) < 0.1)
				square_edge_short_cont2(&ur5, rob2cart(cur_pose), -10, 4 * 25.4, 2, 9 * 25.4, numIters, mytime_seconds, "TEST", true);
			else
				cout << "Could not start edge registration test. Pose check failed" << endl;
		case -1:
			cout << "Stopping..." << endl;
			break;

		default:
			cout << "Invalid Choice Entered" << endl;
			user_in = -1;
			break;

		}//end switch
	} while (user_in != -1);

	point_spiral.close();

	cout << "All Done." << endl;

}//end ur5 control

void cart_client_connect(void* args)
{
	int err;
	char* vehicle_server_buf;
	task_args* passed_args = (task_args*)args;
	int start_flag = 0;

	if ((err = passed_args->cart_client->cart_client_connect()) < 0)
		exit(err);
	
	while (passed_args->cart_client->get_stat_count() > 0)
	{
		start_flag = 0;

		//passed_args->cart_client->print_info();

		if ((err = passed_args->cart_client->cart_comm_client_read(&vehicle_server_buf)) <= 0)
			exit(err);

		//cout << "Recieved Message: '" << vehicle_server_buf << "'" << endl;

		if (strstr(vehicle_server_buf, "rcupdate2") !=NULL && passed_args->cart_client->get_stat_value() == 1)
			start_flag = 1;

		if ((err = passed_args->cart_client->parse_msg(vehicle_server_buf)) < 0)
			exit(err);

		if (start_flag == 1)
		{
			cout << "GIVING ACTUATE MUTEX" << endl;
			ulapi_sem_give(passed_args->actuate);
		}//end if
		

	}//end while
	
}//end cart_client_connect

void actuation_test(void* args)
{
	int err;
	task_args* passed_args = (task_args*)args;

	while (passed_args->cart_client->get_stat_count() > 0)
	{
		
		ulapi_sem_take(passed_args->actuate);
		cout << "Waiting for semaphore to signal a status change has occurred..." << endl;

		if (passed_args->cart_client->get_stat_value() == 2)
		{
			cout << "STARTING TEST SQUARE" << endl;
			passed_args->cart_client->test_square(passed_args->ur5, true);
			passed_args->cart_client->local_command("lcupdatedock");
			passed_args->cart_client->local_command("lcupdate0");
		}//end if
		
	}//end while
}//end actuation test

void actuation_test_auto_update(void* args)
{
	int err;
	task_args* passed_args = (task_args*)args;

	int bisect_check = 0;

	PM_CARTESIAN updated_large1;
	PM_CARTESIAN updated_large2;

	while (passed_args->cart_client->get_stat_count() > 0)
	{

		ulapi_sem_take(passed_args->actuate);
		cout << "Waiting for semaphore to signal a status change has occurred..." << endl;

		if (passed_args->cart_client->get_stat_value() == 2)
		{
			cout << "STARTING TEST SQUARE" << endl;
			bisect_check = 0;
			passed_args->cart_client->test_square_auto_update(passed_args->ur5, &updated_large1, &updated_large2, &bisect_check);
			//Note: You could modify this to work better. You need to remember to also update the vehicle pose at the time of registration.
			//Remember that in the config file, you found out that the vehicle pose should not be the ideal goal pose, but the actual vehicle pose
			//at the time of training the initial search points.
			if (bisect_check == 0)
			{
				//CODE TO AVERAGE OR OTHERWISE ADJUST LARGE POINT LOCATION PREDICTION
				
				/////////////////////////////////////////////////////////////////////
				passed_args->saved_config->avg_large_points(abs(passed_args->saved_config->get_count() - passed_args->cart_client->get_stat_count()), updated_large1, updated_large2);
			}//end if

			passed_args->saved_config->write_file();

			passed_args->cart_client->local_command("lcupdatedock");
			passed_args->cart_client->local_command("lcupdate0");
		}//end if

	}//end while
}//end actuation test

void actuation_test_edge(void* args)
{
	int err;
	task_args* passed_args = (task_args*)args;

	while (passed_args->cart_client->get_stat_count() > 0)
	{

		ulapi_sem_take(passed_args->actuate);
		cout << "Waiting for semaphore to signal a status change has occurred..." << endl;

		if (passed_args->cart_client->get_stat_value() == 2)
		{
			cout << "STARTING TEST EDGE" << endl;
			passed_args->cart_client->test_edge(passed_args->ur5, true);
			passed_args->cart_client->local_command("lcupdatedock");
			passed_args->cart_client->local_command("lcupdate0");
		}//end if

	}//end while
}//end actuation test

void actuation_test_edge_cont(void* args)
{
	int err;
	task_args* passed_args = (task_args*)args;

	while (passed_args->cart_client->get_stat_count() > 0)
	{

		ulapi_sem_take(passed_args->actuate);
		cout << "Waiting for semaphore to signal a status change has occurred..." << endl;

		if (passed_args->cart_client->get_stat_value() == 2)
		{
			cout << "STARTING TEST EDGE" << endl;
			passed_args->cart_client->test_edge_cont(passed_args->ur5, true);
			passed_args->cart_client->local_command("lcupdatedock");
			passed_args->cart_client->local_command("lcupdate0");
		}//end if

	}//end while
}//end actuation test

void actuation_test_edge_cont2(void* args)
{
	int err;
	task_args* passed_args = (task_args*)args;

	while (passed_args->cart_client->get_stat_count() > 0)
	{

		ulapi_sem_take(passed_args->actuate);
		cout << "Waiting for semaphore to signal a status change has occurred..." << endl;

		if (passed_args->cart_client->get_stat_value() == 2)
		{
			cout << "STARTING TEST EDGE" << endl;
			passed_args->cart_client->test_edge_cont2(passed_args->ur5, true);
			passed_args->cart_client->local_command("lcupdatedock");
			passed_args->cart_client->local_command("lcupdate0");
		}//end if

	}//end while
}//end actuation test

void actuation_test_edge_bisect(void* args)
{
	int err;
	task_args* passed_args = (task_args*)args;

	while (passed_args->cart_client->get_stat_count() > 0)
	{

		ulapi_sem_take(passed_args->actuate);
		cout << "Waiting for semaphore to signal a status change has occurred..." << endl;

		if (passed_args->cart_client->get_stat_value() == 2)
		{
			cout << "STARTING TEST EDGE" << endl;
			passed_args->cart_client->test_edge_cont_bisect(passed_args->ur5, true);
			passed_args->cart_client->local_command("lcupdatedock");
			passed_args->cart_client->local_command("lcupdate0");
		}//end if

	}//end while
}//end actuation test