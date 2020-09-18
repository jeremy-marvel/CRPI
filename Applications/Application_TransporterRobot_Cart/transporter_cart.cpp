/**
*\file transporter_cart.cpp
*\brief Main function to implement performance test of mobile robot. Calls task code functions.
*\author Omar Aboul-Enein
*\date 2018-06-05
*/

/**
*\mainpage
*\section network Network Diagram
*\image html NetworkDiagramDoxy.png
*\section overview Control Algorithm Overview
*\image html Slide40.jpg
*\image html Slide27.jpg
*\image html Slide28.jpg
*\image html Slide29.jpg
*\section config Project Configuration
*(Note: Be sure to compile for 32-bit architectures)
*\subsection incl Include Directories
*../../../Libraries/ulapi/src; \n
*../../../Libraries/; \n
*../../../Libraries/CRPI; \n
*../posemath; \n
*../../../Libraries/MotionPrims/ \n\n
*\subsection lnk Linker Input
*Winmm.lib; \n
*../MotionPrims.lib; \n
*../CRPI_Library.lib; \n
*../dlfuncs.lib; \n
*ws2_32.lib; \n
*../ulapi.lib; \n
*kernel32.lib; \n
*user32.lib; \n
*gdi32.lib; \n
*winspool.lib; \n
*comdlg32.lib; \n
*advapi32.lib; \n
*shell32.lib; \n
*ole32.lib; \n
*oleaut32.lib; \n
*uuid.lib; \n
*odbc32.lib; \n
*odbccp32.lib; \n
*%(AdditionalDependencies)
*\subsection rsrc Resource Files
*The following compiled libraries should be added to the same directory that contains the source files:\n
*CRPI_Library.lib \n
*dlfuncs.lib \n
*MotionPrims.lib \n
*ulapi.lib
*/

#include <iostream>
#include "ulapi.h"
#include "tasks.h"
#include "ur5_control.h"
#include "cart_comm_client.h"
#include "cart_config.h"

using namespace std;

int main()
{
	int err;
	int user_in;
	CanonReturn retval;

	//Initialize the ulapi library
	if ((err = ulapi_init()) < 0)
	{
		cout << "Error " << err << " : could not initialize ulapi" << endl;
		exit(err);
	}//end if

	ulapi_set_debug(ULAPI_DEBUG_ALL);

	cout << "==========================================================" << endl;
	cout << "Manpiulator on Cart Test: Cart" << endl;
	cout << "==========================================================" << endl;
	cout << "0. UR5 Control Options" << endl;
	cout << "1. Manipulator-on-Cart Assembly Tasks (Cart and Arm Present)" << endl;
	cout << "2. Manipulator-on-Cart Assembly Tasks (Cart and Arm Present with Auto-Updated Search)" << endl;
	cout << "3. Manipulator-on-Cart Assembly Tasks (Cart Not Present)" << endl;
	cout << "4. Manipulator-on-Cart Assembly Tasks (Static Load with Cart)" << endl;
	cout << "5. Manipulator-on-Cart Assembly Tasks with Edge Registration (Cart and Arm Present)" << endl;
	cout << "6. Manipulator-on-Cart Assembly Tasks with Edge Registration (Continuous move, Cart and Arm Present)" << endl;
	cout << "7. Manipulator-on-Cart Assembly Tasks with Edge Registration (Improved Continuous move, Cart and Arm Present)" << endl;
	cout << "8. Manipulator-on-Cart Assembly Tasks with Edge Registration and Bisect Registration" << endl;
	cout << "9. Test Configuration File Loading" << endl;
	cout << "Please select an option: " << endl;
	cin >> user_in;

	//Submenu for testing UR5 control

	if(user_in == 0)
		ur5_task_code_control(NULL);
		
	else if(user_in == 1)
	{
		task_args* args = (task_args*)malloc(sizeof(task_args));
		robotPose cur_pose;
		config_data* copy_config;

		copy_config = new config_data();
		args->saved_config = new cart_config(0);

		if (args->saved_config->get_err()==0)
		{

			args->saved_config->read_file();

			if (args->saved_config->get_err()==0)
			{

				args->saved_config->get_config(copy_config);

				cout << "Connecting to UR5 controller..." << endl;
				args->ur5 = new CrpiRobot<CrpiUniversal>(".\\universal_ur5_agv.xml");//Please note the filename of this xml document
																	  //It contains parameters CRPI uses to initialize and control the robot with.
																	  //The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
																	  //of the UR5 so that the coordinate system aligns with the coordinate system of the ld.

				do
				{
					cout << "Waiting to connect to the UR5 controller..." << endl;
					ulapi_sleep(.1);
					args->ur5->GetRobotPose(&cur_pose);
				} while (cur_pose.x == 0 && cur_pose.y == 0 && cur_pose.z == 0);

				cout << "Connection established." << endl;

				args->ur5->SetAngleUnits("degree");
				args->ur5->SetLengthUnits("mm");
				args->ur5->SetAbsoluteSpeed(.1);
				args->ur5->SetAbsoluteAcceleration(0.1f);
				args->ur5->Couple("laser");//Sets a tool that is defined with the xml document.

				cout << "Raising feet..." << endl;
				
				if ((retval = raise_feet(args->ur5)) != CANON_SUCCESS)
				{
					cout << "Error: failed to retract feet." << endl;
					exit(retval);
				}//end if

				ulapi_sleep(.1);

				cout << "Feet retracted." << endl;

				ulapi_task_struct* cart_client_connect_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
				ulapi_task_struct* cart_actuate_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));

				/*char* start = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
				strcpy_s(start, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

				char* task_1 = (char*)malloc(sizeof(char)*(strlen("RMMASq1") + 1));
				strcpy_s(task_1, sizeof(char)*(strlen("RMMASq1") + 1), "RMMASq1");

				char* task_2 = (char*)malloc(sizeof(char)*(strlen("RMMASq2") + 1));
				strcpy_s(task_2, sizeof(char)*(strlen("RMMASq2") + 1), "RMMASq2");

				char* task_3 = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
				strcpy_s(task_3, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

				char** dock_l = (char**)(malloc(sizeof(char*) * 3));
				dock_l[0] = task_1;
				dock_l[1] = task_2;
				dock_l[2] = task_3;

				ld_msg_pose** goal_l = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*)*3);
				goal_l[0] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
				goal_l[0]->robot_th = 0;
				goal_l[0]->robot_x = 7945 + 1000 * cos(goal_l[0]->robot_th * TO_RAD);
				goal_l[0]->robot_y = -1757 + 1000 * sin(goal_l[0]->robot_th * TO_RAD);

				goal_l[1] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
				goal_l[1]->robot_th = 90;
				goal_l[1]->robot_x = 5374 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
				goal_l[1]->robot_y = -2069 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

				goal_l[2] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
				goal_l[2]->robot_th = 180;
				goal_l[2]->robot_x = 5137 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
				goal_l[2]->robot_y = -3341 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

				PM_CARTESIAN** large_point1_l = new PM_CARTESIAN*[3];
				large_point1_l[0] = new PM_CARTESIAN(487.139, 249.622, sensor_height);
				large_point1_l[1] = new PM_CARTESIAN(469.783, 235.600, sensor_height);
				large_point1_l[2] = new PM_CARTESIAN(465.375, 279.525, sensor_height);

				PM_CARTESIAN** large_point2_l = new PM_CARTESIAN*[3];
				large_point2_l[0] = new PM_CARTESIAN(478.538, -211.081, sensor_height);
				large_point2_l[1] = new PM_CARTESIAN(465.595, -222.640, sensor_height);
				large_point2_l[2] = new PM_CARTESIAN(457.988, -179.663, sensor_height);

				int* no_arm_l = new int[3];
				no_arm_l[0] = 0;
				no_arm_l[1] = 0;
				no_arm_l[2] = 1;

				args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, start, no_arm_l, dock_l, goal_l, large_point1_l, large_point2_l, 3);*/
				
				//args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->task_count);
				args->actuate = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

				if ((args->actuate = ulapi_sem_new(35)) == NULL)
				{
					cout << "Error: could not create ulapi mutex" << endl;
					exit(-1);
				}//end if

				//cart_client_connect_task(NULL);

				if ((cart_client_connect_task = ulapi_task_new()) == NULL)
				{
					cout << "Error: could not create ulapi task" << endl;
					exit(-1);
				}//end if

				if ((cart_actuate_task = ulapi_task_new()) == NULL)
				{
					cout << "Error: could not create ulapi task" << endl;
					exit(-1);
				}//end if

				if ((err = ulapi_task_init(cart_client_connect_task)) < 0)
				{
					cout << "Error: could not initialize ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_init(cart_actuate_task)) < 0)
				{
					cout << "Error: could not initialize ulapi task" << endl;
					exit(err);
				}//end if
				
				args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->edge_start_list, copy_config->task_count, copy_config->runs);

				//PROBLEM: THIS task still runs after cart_client has been deallocated, need to stop this task
				if ((err = ulapi_task_start(cart_client_connect_task, cart_client_connect, (void*)args, ulapi_prio_lowest(), 0)) < 0)
				{
					cout << "Error: could not start ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_start(cart_actuate_task, actuation_test, (void*)args, ulapi_prio_lowest(), 0)) < 0)
				{
					cout << "Error: could not start ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_join(cart_actuate_task, NULL)) < 0)
				{
					cout << "Error: could not join ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_stop(cart_client_connect_task)) < 0)
				{
					cout << "Error: could not stop ulapi task" << endl;
					exit(err);
				}//end if

				args->cart_client->~cart_comm_client();
				args->cart_client = NULL;

				/*
				if ((err = ulapi_task_join(cart_client_connect_task, NULL)) < 0)
				{
					cout << "Error: could not join ulapi task" << endl;
					exit(err);
				}//end if
				*/

				if ((err = ulapi_task_delete(cart_actuate_task)) < 0)
				{
					cout << "Error: could not delete ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_delete(cart_client_connect_task)) < 0)
				{
					cout << "Error: could not delete ulapi task" << endl;
					exit(err);
				}//end if

				if ((ulapi_sem_delete(args->actuate)) < 0)
				{
					cout << "Error: could not delete ulapi semaphore" << endl;
					exit(err);
				}//end if
				
				args->saved_config->write_file();

				args->saved_config->~cart_config();
				copy_config->~config_data();
				copy_config = NULL;

				//args->cart_client->~cart_comm_client();
				args->ur5->~CrpiRobot();
				free(args);
			}//end if
		}//end if
		
	}//end else if

	else if (user_in == 2)
	{
	task_args* args = (task_args*)malloc(sizeof(task_args));
	robotPose cur_pose;
	config_data* copy_config;

	copy_config = new config_data();
	args->saved_config = new cart_config(0);

	if (args->saved_config->get_err() == 0)
	{

		args->saved_config->read_file();

		if (args->saved_config->get_err() == 0)
		{

			args->saved_config->get_config(copy_config);

			cout << "Connecting to UR5 controller..." << endl;
			args->ur5 = new CrpiRobot<CrpiUniversal>("universal_ur5_agv.xml");//Please note the filename of this xml document
																  //It contains parameters CRPI uses to initialize and control the robot with.
																  //The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
																  //of the UR5 so that the coordinate system aligns with the coordinate system of the ld.

			do
			{
				cout << "Waiting to connect to the UR5 controller..." << endl;
				ulapi_sleep(.1);
				args->ur5->GetRobotPose(&cur_pose);
			} while (cur_pose.x == 0 && cur_pose.y == 0 && cur_pose.z == 0);

			cout << "Connection established." << endl;

			args->ur5->SetAngleUnits("degree");
			args->ur5->SetLengthUnits("mm");
			args->ur5->SetAbsoluteSpeed(.1);
			args->ur5->SetAbsoluteAcceleration(0.1f);
			args->ur5->Couple("laser");//Sets a tool that is defined with the xml document.

			cout << "Raising feet..." << endl;

			ulapi_sleep(.1);

			if ((retval = raise_feet(args->ur5)) != CANON_SUCCESS)
			{
				cout << "Error: failed to retract feet." << endl;
				exit(retval);
			}//end if

			cout << "Feet retracted." << endl;

			ulapi_task_struct* cart_client_connect_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
			ulapi_task_struct* cart_actuate_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));

			/*char* start = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(start, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char* task_1 = (char*)malloc(sizeof(char)*(strlen("RMMASq1") + 1));
			strcpy_s(task_1, sizeof(char)*(strlen("RMMASq1") + 1), "RMMASq1");

			char* task_2 = (char*)malloc(sizeof(char)*(strlen("RMMASq2") + 1));
			strcpy_s(task_2, sizeof(char)*(strlen("RMMASq2") + 1), "RMMASq2");

			char* task_3 = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(task_3, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char** dock_l = (char**)(malloc(sizeof(char*) * 3));
			dock_l[0] = task_1;
			dock_l[1] = task_2;
			dock_l[2] = task_3;

			ld_msg_pose** goal_l = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*)*3);
			goal_l[0] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[0]->robot_th = 0;
			goal_l[0]->robot_x = 7945 + 1000 * cos(goal_l[0]->robot_th * TO_RAD);
			goal_l[0]->robot_y = -1757 + 1000 * sin(goal_l[0]->robot_th * TO_RAD);

			goal_l[1] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[1]->robot_th = 90;
			goal_l[1]->robot_x = 5374 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[1]->robot_y = -2069 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			goal_l[2] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[2]->robot_th = 180;
			goal_l[2]->robot_x = 5137 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[2]->robot_y = -3341 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			PM_CARTESIAN** large_point1_l = new PM_CARTESIAN*[3];
			large_point1_l[0] = new PM_CARTESIAN(487.139, 249.622, sensor_height);
			large_point1_l[1] = new PM_CARTESIAN(469.783, 235.600, sensor_height);
			large_point1_l[2] = new PM_CARTESIAN(465.375, 279.525, sensor_height);

			PM_CARTESIAN** large_point2_l = new PM_CARTESIAN*[3];
			large_point2_l[0] = new PM_CARTESIAN(478.538, -211.081, sensor_height);
			large_point2_l[1] = new PM_CARTESIAN(465.595, -222.640, sensor_height);
			large_point2_l[2] = new PM_CARTESIAN(457.988, -179.663, sensor_height);

			int* no_arm_l = new int[3];
			no_arm_l[0] = 0;
			no_arm_l[1] = 0;
			no_arm_l[2] = 1;

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, start, no_arm_l, dock_l, goal_l, large_point1_l, large_point2_l, 3);*/

			//args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->task_count);
			args->actuate = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

			if ((args->actuate = ulapi_sem_new(35)) == NULL)
			{
				cout << "Error: could not create ulapi mutex" << endl;
				exit(-1);
			}//end if

			//cart_client_connect_task(NULL);

			if ((cart_client_connect_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((cart_actuate_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((err = ulapi_task_init(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_init(cart_actuate_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->edge_start_list, copy_config->task_count, copy_config->runs);

			if ((err = ulapi_task_start(cart_client_connect_task, cart_client_connect, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_start(cart_actuate_task, actuation_test_auto_update, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_join(cart_actuate_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_stop(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not stop ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client->~cart_comm_client();
			args->cart_client = NULL;

			/*
			if ((err = ulapi_task_join(cart_client_connect_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if
			*/

			if ((err = ulapi_task_delete(cart_actuate_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_delete(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((ulapi_sem_delete(args->actuate)) < 0)
			{
				cout << "Error: could not delete ulapi semaphore" << endl;
				exit(err);
			}//end if

			args->saved_config->~cart_config();
			copy_config->~config_data();
			copy_config = NULL;

			//args->cart_client->~cart_comm_client();
			args->ur5->~CrpiRobot();
			free(args);
		}//end if
	}//end if

	}//end else if

	else if (user_in == 3)
	{
		task_args* args = (task_args*)malloc(sizeof(task_args));

		args->ur5 = NULL;

		config_data* copy_config;
		cart_config* saved_config;

		copy_config = new config_data();
		saved_config = new cart_config(1);

		if (saved_config->get_err() == 0)
		{

			saved_config->read_file();

			if (saved_config->get_err() == 0)
			{

				saved_config->get_config(copy_config);

				ulapi_task_struct* cart_client_connect_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
				ulapi_task_struct* cart_actuate_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));

				/*char* start = (char*)malloc(sizeof(char)*(strlen("CartDock2") + 1));
				strcpy_s(start, sizeof(char)*(strlen("CartDock2") + 1), "CartDock2");

				char* task_1 = (char*)malloc(sizeof(char)*(strlen("TestPoint1") + 1));
				strcpy_s(task_1, sizeof(char)*(strlen("TestPoint1") + 1), "TestPoint1");

				char* task_2 = (char*)malloc(sizeof(char)*(strlen("CartDock2") + 1));
				strcpy_s(task_2, sizeof(char)*(strlen("CartDock2") + 1), "CartDock2");

				char** dock_l = (char**)(malloc(sizeof(char*) * 2));
				dock_l[0] = task_1;
				dock_l[1] = task_2;

				ld_msg_pose** goal_l = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*) * 2);
				goal_l[0] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
				goal_l[0]->robot_th = 0;
				goal_l[0]->robot_x = 7945 + 1000 * cos(goal_l[0]->robot_th * TO_RAD);
				goal_l[0]->robot_y = -1757 + 1000 * sin(goal_l[0]->robot_th * TO_RAD);

				goal_l[1] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
				goal_l[1]->robot_th = 90;
				goal_l[1]->robot_x = 5374 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
				goal_l[1]->robot_y = -2069 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

				PM_CARTESIAN** large_point1_l = new PM_CARTESIAN*[2];
				large_point1_l[0] = new PM_CARTESIAN(493.592, 307.192, sensor_height);
				large_point1_l[1] = new PM_CARTESIAN(448.006, 226.956, sensor_height);

				PM_CARTESIAN** large_point2_l = new PM_CARTESIAN*[2];
				large_point2_l[0] = new PM_CARTESIAN(510.415, -148.800, sensor_height);
				large_point2_l[1] = new PM_CARTESIAN(443.771, -230.798, sensor_height);

				int* no_arm_l = new int[2];
				no_arm_l[0] = 1;
				no_arm_l[1] = 1;

				args->cart_client = new cart_comm_client(1, "192.168.160.52", 5352, 34, start, no_arm_l, dock_l, goal_l, large_point1_l, large_point2_l, 2);*/

				//args->cart_client = new cart_comm_client(1, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->task_count);
				args->actuate = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

				if ((args->actuate = ulapi_sem_new(35)) == NULL)
				{
					cout << "Error: could not create ulapi mutex" << endl;
					exit(-1);
				}//end if

				 //cart_client_connect_task(NULL);

				if ((cart_client_connect_task = ulapi_task_new()) == NULL)
				{
					cout << "Error: could not create ulapi task" << endl;
					exit(-1);
				}//end if

				if ((cart_actuate_task = ulapi_task_new()) == NULL)
				{
					cout << "Error: could not create ulapi task" << endl;
					exit(-1);
				}//end if

				if ((err = ulapi_task_init(cart_client_connect_task)) < 0)
				{
					cout << "Error: could not initialize ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_init(cart_actuate_task)) < 0)
				{
					cout << "Error: could not initialize ulapi task" << endl;
					exit(err);
				}//end if

				args->cart_client = new cart_comm_client(1, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->edge_start_list, copy_config->task_count, copy_config->runs);

				if ((err = ulapi_task_start(cart_client_connect_task, cart_client_connect, (void*)args, ulapi_prio_lowest(), 0)) < 0)
				{
					cout << "Error: could not start ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_start(cart_actuate_task, actuation_test, (void*)args, ulapi_prio_lowest(), 0)) < 0)
				{
					cout << "Error: could not start ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_join(cart_actuate_task, NULL)) < 0)
				{
					cout << "Error: could not join ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_stop(cart_client_connect_task)) < 0)
				{
					cout << "Error: could not stop ulapi task" << endl;
					exit(err);
				}//end if

				args->cart_client->~cart_comm_client();
				args->cart_client = NULL;

				 /*
				 if ((err = ulapi_task_join(cart_client_connect_task, NULL)) < 0)
				 {
				 cout << "Error: could not join ulapi task" << endl;
				 exit(err);
				 }//end if
				 */

				if ((err = ulapi_task_delete(cart_actuate_task)) < 0)
				{
					cout << "Error: could not delete ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_delete(cart_client_connect_task)) < 0)
				{
					cout << "Error: could not delete ulapi task" << endl;
					exit(err);
				}//end if

				if ((ulapi_sem_delete(args->actuate)) < 0)
				{
					cout << "Error: could not delete ulapi semaphore" << endl;
					exit(err);
				}//end if


				saved_config->~cart_config();
				copy_config->~config_data();
				copy_config = NULL;

				//args->cart_client->~cart_comm_client();
				free(args);
			}//end if
		}//end if
	}//end else if

	else if (user_in == 4)
	{
		task_args* args = (task_args*)malloc(sizeof(task_args));

		args->ur5 = NULL;

		config_data* copy_config;
		cart_config* saved_config;

		copy_config = new config_data();
		saved_config = new cart_config(1);

		if (saved_config->get_err() == 0)
		{

			saved_config->read_file();

			if (saved_config->get_err() == 0)
			{

				saved_config->get_config(copy_config);

				ulapi_task_struct* cart_client_connect_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
				ulapi_task_struct* cart_actuate_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));

				/*char* start = (char*)malloc(sizeof(char)*(strlen("CartDock2") + 1));
				strcpy_s(start, sizeof(char)*(strlen("CartDock2") + 1), "CartDock2");

				char* task_1 = (char*)malloc(sizeof(char)*(strlen("TestPoint1") + 1));
				strcpy_s(task_1, sizeof(char)*(strlen("TestPoint1") + 1), "TestPoint1");

				char* task_2 = (char*)malloc(sizeof(char)*(strlen("CartDock2") + 1));
				strcpy_s(task_2, sizeof(char)*(strlen("CartDock2") + 1), "CartDock2");

				char** dock_l = (char**)(malloc(sizeof(char*) * 2));
				dock_l[0] = task_1;
				dock_l[1] = task_2;

				ld_msg_pose** goal_l = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*) * 2);
				goal_l[0] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
				goal_l[0]->robot_th = 0;
				goal_l[0]->robot_x = 7945 + 1000 * cos(goal_l[0]->robot_th * TO_RAD);
				goal_l[0]->robot_y = -1757 + 1000 * sin(goal_l[0]->robot_th * TO_RAD);

				goal_l[1] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
				goal_l[1]->robot_th = 90;
				goal_l[1]->robot_x = 5374 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
				goal_l[1]->robot_y = -2069 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

				PM_CARTESIAN** large_point1_l = new PM_CARTESIAN*[2];
				large_point1_l[0] = new PM_CARTESIAN(493.592, 307.192, sensor_height);
				large_point1_l[1] = new PM_CARTESIAN(448.006, 226.956, sensor_height);

				PM_CARTESIAN** large_point2_l = new PM_CARTESIAN*[2];
				large_point2_l[0] = new PM_CARTESIAN(510.415, -148.800, sensor_height);
				large_point2_l[1] = new PM_CARTESIAN(443.771, -230.798, sensor_height);

				int* no_arm_l = new int[2];
				no_arm_l[0] = 1;
				no_arm_l[1] = 1;

				args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, start, no_arm_l, dock_l, goal_l, large_point1_l, large_point2_l, 2);*/
				//args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->task_count);
				args->actuate = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

				if ((args->actuate = ulapi_sem_new(35)) == NULL)
				{
					cout << "Error: could not create ulapi mutex" << endl;
					exit(-1);
				}//end if

				 //cart_client_connect_task(NULL);

				if ((cart_client_connect_task = ulapi_task_new()) == NULL)
				{
					cout << "Error: could not create ulapi task" << endl;
					exit(-1);
				}//end if

				if ((cart_actuate_task = ulapi_task_new()) == NULL)
				{
					cout << "Error: could not create ulapi task" << endl;
					exit(-1);
				}//end if

				if ((err = ulapi_task_init(cart_client_connect_task)) < 0)
				{
					cout << "Error: could not initialize ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_init(cart_actuate_task)) < 0)
				{
					cout << "Error: could not initialize ulapi task" << endl;
					exit(err);
				}//end if

				args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->edge_start_list, copy_config->task_count, copy_config->runs);

				if ((err = ulapi_task_start(cart_client_connect_task, cart_client_connect, (void*)args, ulapi_prio_lowest(), 0)) < 0)
				{
					cout << "Error: could not start ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_start(cart_actuate_task, actuation_test, (void*)args, ulapi_prio_lowest(), 0)) < 0)
				{
					cout << "Error: could not start ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_join(cart_actuate_task, NULL)) < 0)
				{
					cout << "Error: could not join ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_stop(cart_client_connect_task)) < 0)
				{
					cout << "Error: could not stop ulapi task" << endl;
					exit(err);
				}//end if

				args->cart_client->~cart_comm_client();
				args->cart_client = NULL;

				 /*
				 if ((err = ulapi_task_join(cart_client_connect_task, NULL)) < 0)
				 {
				 cout << "Error: could not join ulapi task" << endl;
				 exit(err);
				 }//end if
				 */

				if ((err = ulapi_task_delete(cart_actuate_task)) < 0)
				{
					cout << "Error: could not delete ulapi task" << endl;
					exit(err);
				}//end if

				if ((err = ulapi_task_delete(cart_client_connect_task)) < 0)
				{
					cout << "Error: could not delete ulapi task" << endl;
					exit(err);
				}//end if

				if ((ulapi_sem_delete(args->actuate)) < 0)
				{
					cout << "Error: could not delete ulapi semaphore" << endl;
					exit(err);
				}//end if

				saved_config->~cart_config();
				copy_config->~config_data();
				copy_config = NULL;

				//args->cart_client->~cart_comm_client();
				free(args);
			}//end if
		}//end if

	}//end else if

	else if (user_in == 5)
	{
	task_args* args = (task_args*)malloc(sizeof(task_args));
	robotPose cur_pose;
	config_data* copy_config;

	copy_config = new config_data();
	args->saved_config = new cart_config(0);

	if (args->saved_config->get_err() == 0)
	{

		args->saved_config->read_file();

		if (args->saved_config->get_err() == 0)
		{

			args->saved_config->get_config(copy_config);

			cout << "Connecting to UR5 controller..." << endl;
			args->ur5 = new CrpiRobot<CrpiUniversal>(".\\universal_ur5_agv.xml");//Please note the filename of this xml document
																  //It contains parameters CRPI uses to initialize and control the robot with.
																  //The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
																  //of the UR5 so that the coordinate system aligns with the coordinate system of the ld.

			do
			{
				cout << "Waiting to connect to the UR5 controller..." << endl;
				ulapi_sleep(.1);
				args->ur5->GetRobotPose(&cur_pose);
			} while (cur_pose.x == 0 && cur_pose.y == 0 && cur_pose.z == 0);

			cout << "Connection established." << endl;

			args->ur5->SetAngleUnits("degree");
			args->ur5->SetLengthUnits("mm");
			args->ur5->SetAbsoluteSpeed(.1);
			args->ur5->SetAbsoluteAcceleration(0.1f);
			args->ur5->Couple("laser");//Sets a tool that is defined with the xml document.

			cout << "Raising feet..." << endl;

			if ((retval = raise_feet(args->ur5)) != CANON_SUCCESS)
			{
				cout << "Error: failed to retract feet." << endl;
				exit(retval);
			}//end if

			ulapi_sleep(.1);

			cout << "Feet retracted." << endl;

			ulapi_task_struct* cart_client_connect_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
			ulapi_task_struct* cart_actuate_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));

			/*char* start = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(start, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char* task_1 = (char*)malloc(sizeof(char)*(strlen("RMMASq1") + 1));
			strcpy_s(task_1, sizeof(char)*(strlen("RMMASq1") + 1), "RMMASq1");

			char* task_2 = (char*)malloc(sizeof(char)*(strlen("RMMASq2") + 1));
			strcpy_s(task_2, sizeof(char)*(strlen("RMMASq2") + 1), "RMMASq2");

			char* task_3 = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(task_3, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char** dock_l = (char**)(malloc(sizeof(char*) * 3));
			dock_l[0] = task_1;
			dock_l[1] = task_2;
			dock_l[2] = task_3;

			ld_msg_pose** goal_l = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*)*3);
			goal_l[0] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[0]->robot_th = 0;
			goal_l[0]->robot_x = 7945 + 1000 * cos(goal_l[0]->robot_th * TO_RAD);
			goal_l[0]->robot_y = -1757 + 1000 * sin(goal_l[0]->robot_th * TO_RAD);

			goal_l[1] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[1]->robot_th = 90;
			goal_l[1]->robot_x = 5374 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[1]->robot_y = -2069 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			goal_l[2] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[2]->robot_th = 180;
			goal_l[2]->robot_x = 5137 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[2]->robot_y = -3341 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			PM_CARTESIAN** large_point1_l = new PM_CARTESIAN*[3];
			large_point1_l[0] = new PM_CARTESIAN(487.139, 249.622, sensor_height);
			large_point1_l[1] = new PM_CARTESIAN(469.783, 235.600, sensor_height);
			large_point1_l[2] = new PM_CARTESIAN(465.375, 279.525, sensor_height);

			PM_CARTESIAN** large_point2_l = new PM_CARTESIAN*[3];
			large_point2_l[0] = new PM_CARTESIAN(478.538, -211.081, sensor_height);
			large_point2_l[1] = new PM_CARTESIAN(465.595, -222.640, sensor_height);
			large_point2_l[2] = new PM_CARTESIAN(457.988, -179.663, sensor_height);

			int* no_arm_l = new int[3];
			no_arm_l[0] = 0;
			no_arm_l[1] = 0;
			no_arm_l[2] = 1;

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, start, no_arm_l, dock_l, goal_l, large_point1_l, large_point2_l, 3);*/

			//args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->task_count);
			args->actuate = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

			if ((args->actuate = ulapi_sem_new(35)) == NULL)
			{
				cout << "Error: could not create ulapi mutex" << endl;
				exit(-1);
			}//end if

			//cart_client_connect_task(NULL);

			if ((cart_client_connect_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((cart_actuate_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((err = ulapi_task_init(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_init(cart_actuate_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->edge_start_list, copy_config->task_count, copy_config->runs);

			//PROBLEM: THIS task still runs after cart_client has been deallocated, need to stop this task
			if ((err = ulapi_task_start(cart_client_connect_task, cart_client_connect, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_start(cart_actuate_task, actuation_test_edge, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_join(cart_actuate_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_stop(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not stop ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client->~cart_comm_client();
			args->cart_client = NULL;

			/*
			if ((err = ulapi_task_join(cart_client_connect_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if
			*/

			if ((err = ulapi_task_delete(cart_actuate_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_delete(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((ulapi_sem_delete(args->actuate)) < 0)
			{
				cout << "Error: could not delete ulapi semaphore" << endl;
				exit(err);
			}//end if

			args->saved_config->write_file();

			args->saved_config->~cart_config();
			copy_config->~config_data();
			copy_config = NULL;

			//args->cart_client->~cart_comm_client();
			args->ur5->~CrpiRobot();
			free(args);
		}//end if
	}//end if

	}//end else if

	else if (user_in == 6)
	{
	task_args* args = (task_args*)malloc(sizeof(task_args));
	robotPose cur_pose;
	config_data* copy_config;

	copy_config = new config_data();
	args->saved_config = new cart_config(0);

	if (args->saved_config->get_err() == 0)
	{

		args->saved_config->read_file();

		if (args->saved_config->get_err() == 0)
		{

			args->saved_config->get_config(copy_config);

			cout << "Connecting to UR5 controller..." << endl;
			args->ur5 = new CrpiRobot<CrpiUniversal>(".\\universal_ur5_agv.xml");//Please note the filename of this xml document
																  //It contains parameters CRPI uses to initialize and control the robot with.
																  //The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
																  //of the UR5 so that the coordinate system aligns with the coordinate system of the ld.

			do
			{
				cout << "Waiting to connect to the UR5 controller..." << endl;
				ulapi_sleep(.1);
				args->ur5->GetRobotPose(&cur_pose);
			} while (cur_pose.x == 0 && cur_pose.y == 0 && cur_pose.z == 0);

			cout << "Connection established." << endl;

			args->ur5->SetAngleUnits("degree");
			args->ur5->SetLengthUnits("mm");
			args->ur5->SetAbsoluteSpeed(.1);
			args->ur5->SetAbsoluteAcceleration(0.1f);
			args->ur5->Couple("laser");//Sets a tool that is defined with the xml document.

			cout << "Raising feet..." << endl;

			if ((retval = raise_feet(args->ur5)) != CANON_SUCCESS)
			{
				cout << "Error: failed to retract feet." << endl;
				exit(retval);
			}//end if

			ulapi_sleep(.1);

			cout << "Feet retracted." << endl;

			ulapi_task_struct* cart_client_connect_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
			ulapi_task_struct* cart_actuate_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));

			/*char* start = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(start, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char* task_1 = (char*)malloc(sizeof(char)*(strlen("RMMASq1") + 1));
			strcpy_s(task_1, sizeof(char)*(strlen("RMMASq1") + 1), "RMMASq1");

			char* task_2 = (char*)malloc(sizeof(char)*(strlen("RMMASq2") + 1));
			strcpy_s(task_2, sizeof(char)*(strlen("RMMASq2") + 1), "RMMASq2");

			char* task_3 = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(task_3, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char** dock_l = (char**)(malloc(sizeof(char*) * 3));
			dock_l[0] = task_1;
			dock_l[1] = task_2;
			dock_l[2] = task_3;

			ld_msg_pose** goal_l = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*)*3);
			goal_l[0] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[0]->robot_th = 0;
			goal_l[0]->robot_x = 7945 + 1000 * cos(goal_l[0]->robot_th * TO_RAD);
			goal_l[0]->robot_y = -1757 + 1000 * sin(goal_l[0]->robot_th * TO_RAD);

			goal_l[1] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[1]->robot_th = 90;
			goal_l[1]->robot_x = 5374 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[1]->robot_y = -2069 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			goal_l[2] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[2]->robot_th = 180;
			goal_l[2]->robot_x = 5137 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[2]->robot_y = -3341 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			PM_CARTESIAN** large_point1_l = new PM_CARTESIAN*[3];
			large_point1_l[0] = new PM_CARTESIAN(487.139, 249.622, sensor_height);
			large_point1_l[1] = new PM_CARTESIAN(469.783, 235.600, sensor_height);
			large_point1_l[2] = new PM_CARTESIAN(465.375, 279.525, sensor_height);

			PM_CARTESIAN** large_point2_l = new PM_CARTESIAN*[3];
			large_point2_l[0] = new PM_CARTESIAN(478.538, -211.081, sensor_height);
			large_point2_l[1] = new PM_CARTESIAN(465.595, -222.640, sensor_height);
			large_point2_l[2] = new PM_CARTESIAN(457.988, -179.663, sensor_height);

			int* no_arm_l = new int[3];
			no_arm_l[0] = 0;
			no_arm_l[1] = 0;
			no_arm_l[2] = 1;

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, start, no_arm_l, dock_l, goal_l, large_point1_l, large_point2_l, 3);*/

			//args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->task_count);
			args->actuate = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

			if ((args->actuate = ulapi_sem_new(35)) == NULL)
			{
				cout << "Error: could not create ulapi mutex" << endl;
				exit(-1);
			}//end if

			//cart_client_connect_task(NULL);

			if ((cart_client_connect_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((cart_actuate_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((err = ulapi_task_init(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_init(cart_actuate_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->edge_start_list, copy_config->task_count, copy_config->runs);

			//PROBLEM: THIS task still runs after cart_client has been deallocated, need to stop this task
			if ((err = ulapi_task_start(cart_client_connect_task, cart_client_connect, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_start(cart_actuate_task, actuation_test_edge_cont, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_join(cart_actuate_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_stop(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not stop ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client->~cart_comm_client();
			args->cart_client = NULL;

			/*
			if ((err = ulapi_task_join(cart_client_connect_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if
			*/

			if ((err = ulapi_task_delete(cart_actuate_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_delete(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((ulapi_sem_delete(args->actuate)) < 0)
			{
				cout << "Error: could not delete ulapi semaphore" << endl;
				exit(err);
			}//end if

			args->saved_config->write_file();

			args->saved_config->~cart_config();
			copy_config->~config_data();
			copy_config = NULL;

			//args->cart_client->~cart_comm_client();
			args->ur5->~CrpiRobot();
			free(args);
		}//end if
	}//end if

	}//end else if

	else if (user_in == 7)
	{
	task_args* args = (task_args*)malloc(sizeof(task_args));
	robotPose cur_pose;
	config_data* copy_config;

	copy_config = new config_data();
	args->saved_config = new cart_config(0);

	if (args->saved_config->get_err() == 0)
	{

		args->saved_config->read_file();

		if (args->saved_config->get_err() == 0)
		{

			args->saved_config->get_config(copy_config);

			cout << "Connecting to UR5 controller..." << endl;
			args->ur5 = new CrpiRobot<CrpiUniversal>(".\\universal_ur5_agv.xml");//Please note the filename of this xml document
																  //It contains parameters CRPI uses to initialize and control the robot with.
																  //The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
																  //of the UR5 so that the coordinate system aligns with the coordinate system of the ld.

			do
			{
				cout << "Waiting to connect to the UR5 controller..." << endl;
				ulapi_sleep(.1);
				args->ur5->GetRobotPose(&cur_pose);
			} while (cur_pose.x == 0 && cur_pose.y == 0 && cur_pose.z == 0);

			cout << "Connection established." << endl;

			args->ur5->SetAngleUnits("degree");
			args->ur5->SetLengthUnits("mm");
			args->ur5->SetAbsoluteSpeed(.1);
			args->ur5->SetAbsoluteAcceleration(0.1f);
			args->ur5->Couple("laser");//Sets a tool that is defined with the xml document.

			cout << "Raising feet..." << endl;

			if ((retval = raise_feet(args->ur5)) != CANON_SUCCESS)
			{
				cout << "Error: failed to retract feet." << endl;
				exit(retval);
			}//end if

			ulapi_sleep(.1);

			cout << "Feet retracted." << endl;

			ulapi_task_struct* cart_client_connect_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
			ulapi_task_struct* cart_actuate_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));

			/*char* start = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(start, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char* task_1 = (char*)malloc(sizeof(char)*(strlen("RMMASq1") + 1));
			strcpy_s(task_1, sizeof(char)*(strlen("RMMASq1") + 1), "RMMASq1");

			char* task_2 = (char*)malloc(sizeof(char)*(strlen("RMMASq2") + 1));
			strcpy_s(task_2, sizeof(char)*(strlen("RMMASq2") + 1), "RMMASq2");

			char* task_3 = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(task_3, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char** dock_l = (char**)(malloc(sizeof(char*) * 3));
			dock_l[0] = task_1;
			dock_l[1] = task_2;
			dock_l[2] = task_3;

			ld_msg_pose** goal_l = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*)*3);
			goal_l[0] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[0]->robot_th = 0;
			goal_l[0]->robot_x = 7945 + 1000 * cos(goal_l[0]->robot_th * TO_RAD);
			goal_l[0]->robot_y = -1757 + 1000 * sin(goal_l[0]->robot_th * TO_RAD);

			goal_l[1] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[1]->robot_th = 90;
			goal_l[1]->robot_x = 5374 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[1]->robot_y = -2069 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			goal_l[2] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[2]->robot_th = 180;
			goal_l[2]->robot_x = 5137 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[2]->robot_y = -3341 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			PM_CARTESIAN** large_point1_l = new PM_CARTESIAN*[3];
			large_point1_l[0] = new PM_CARTESIAN(487.139, 249.622, sensor_height);
			large_point1_l[1] = new PM_CARTESIAN(469.783, 235.600, sensor_height);
			large_point1_l[2] = new PM_CARTESIAN(465.375, 279.525, sensor_height);

			PM_CARTESIAN** large_point2_l = new PM_CARTESIAN*[3];
			large_point2_l[0] = new PM_CARTESIAN(478.538, -211.081, sensor_height);
			large_point2_l[1] = new PM_CARTESIAN(465.595, -222.640, sensor_height);
			large_point2_l[2] = new PM_CARTESIAN(457.988, -179.663, sensor_height);

			int* no_arm_l = new int[3];
			no_arm_l[0] = 0;
			no_arm_l[1] = 0;
			no_arm_l[2] = 1;

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, start, no_arm_l, dock_l, goal_l, large_point1_l, large_point2_l, 3);*/

			//args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->task_count);
			args->actuate = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

			if ((args->actuate = ulapi_sem_new(35)) == NULL)
			{
				cout << "Error: could not create ulapi mutex" << endl;
				exit(-1);
			}//end if

			//cart_client_connect_task(NULL);

			if ((cart_client_connect_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((cart_actuate_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((err = ulapi_task_init(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_init(cart_actuate_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->edge_start_list, copy_config->task_count, copy_config->runs);

			//PROBLEM: THIS task still runs after cart_client has been deallocated, need to stop this task
			if ((err = ulapi_task_start(cart_client_connect_task, cart_client_connect, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_start(cart_actuate_task, actuation_test_edge_cont2, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_join(cart_actuate_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_stop(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not stop ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client->~cart_comm_client();
			args->cart_client = NULL;

			/*
			if ((err = ulapi_task_join(cart_client_connect_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if
			*/

			if ((err = ulapi_task_delete(cart_actuate_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_delete(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((ulapi_sem_delete(args->actuate)) < 0)
			{
				cout << "Error: could not delete ulapi semaphore" << endl;
				exit(err);
			}//end if

			args->saved_config->write_file();

			args->saved_config->~cart_config();
			copy_config->~config_data();
			copy_config = NULL;

			//args->cart_client->~cart_comm_client();
			args->ur5->~CrpiRobot();
			free(args);
		}//end if
	}//end if

	}//end else if

	else if (user_in == 8)
	{
	task_args* args = (task_args*)malloc(sizeof(task_args));
	robotPose cur_pose;
	config_data* copy_config;

	copy_config = new config_data();
	args->saved_config = new cart_config(0);

	if (args->saved_config->get_err() == 0)
	{

		args->saved_config->read_file();

		if (args->saved_config->get_err() == 0)
		{

			args->saved_config->get_config(copy_config);

			cout << "Connecting to UR5 controller..." << endl;
			args->ur5 = new CrpiRobot<CrpiUniversal>(".\\universal_ur5_agv.xml");//Please note the filename of this xml document
																  //It contains parameters CRPI uses to initialize and control the robot with.
																  //The most important of these is the <mount> tag as it has been modified to adjust the mounting angle offset
																  //of the UR5 so that the coordinate system aligns with the coordinate system of the ld.

			do
			{
				cout << "Waiting to connect to the UR5 controller..." << endl;
				ulapi_sleep(.1);
				args->ur5->GetRobotPose(&cur_pose);
			} while (cur_pose.x == 0 && cur_pose.y == 0 && cur_pose.z == 0);

			cout << "Connection established." << endl;

			args->ur5->SetAngleUnits("degree");
			args->ur5->SetLengthUnits("mm");
			args->ur5->SetAbsoluteSpeed(.1);
			args->ur5->SetAbsoluteAcceleration(0.1f);
			args->ur5->Couple("laser");//Sets a tool that is defined with the xml document.

			cout << "Raising feet..." << endl;

			if ((retval = raise_feet(args->ur5)) != CANON_SUCCESS)
			{
				cout << "Error: failed to retract feet." << endl;
				exit(retval);
			}//end if

			ulapi_sleep(.1);

			cout << "Feet retracted." << endl;

			ulapi_task_struct* cart_client_connect_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));
			ulapi_task_struct* cart_actuate_task = (ulapi_task_struct*)malloc(sizeof(ulapi_task_struct));

			/*char* start = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(start, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char* task_1 = (char*)malloc(sizeof(char)*(strlen("RMMASq1") + 1));
			strcpy_s(task_1, sizeof(char)*(strlen("RMMASq1") + 1), "RMMASq1");

			char* task_2 = (char*)malloc(sizeof(char)*(strlen("RMMASq2") + 1));
			strcpy_s(task_2, sizeof(char)*(strlen("RMMASq2") + 1), "RMMASq2");

			char* task_3 = (char*)malloc(sizeof(char)*(strlen("CartDock1") + 1));
			strcpy_s(task_3, sizeof(char)*(strlen("CartDock1") + 1), "CartDock1");

			char** dock_l = (char**)(malloc(sizeof(char*) * 3));
			dock_l[0] = task_1;
			dock_l[1] = task_2;
			dock_l[2] = task_3;

			ld_msg_pose** goal_l = (ld_msg_pose**)malloc(sizeof(ld_msg_pose*)*3);
			goal_l[0] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[0]->robot_th = 0;
			goal_l[0]->robot_x = 7945 + 1000 * cos(goal_l[0]->robot_th * TO_RAD);
			goal_l[0]->robot_y = -1757 + 1000 * sin(goal_l[0]->robot_th * TO_RAD);

			goal_l[1] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[1]->robot_th = 90;
			goal_l[1]->robot_x = 5374 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[1]->robot_y = -2069 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			goal_l[2] = (ld_msg_pose*)malloc(sizeof(ld_msg_pose));
			goal_l[2]->robot_th = 180;
			goal_l[2]->robot_x = 5137 + 1000 * cos(goal_l[1]->robot_th * TO_RAD);
			goal_l[2]->robot_y = -3341 + 1000 * sin(goal_l[1]->robot_th * TO_RAD);

			PM_CARTESIAN** large_point1_l = new PM_CARTESIAN*[3];
			large_point1_l[0] = new PM_CARTESIAN(487.139, 249.622, sensor_height);
			large_point1_l[1] = new PM_CARTESIAN(469.783, 235.600, sensor_height);
			large_point1_l[2] = new PM_CARTESIAN(465.375, 279.525, sensor_height);

			PM_CARTESIAN** large_point2_l = new PM_CARTESIAN*[3];
			large_point2_l[0] = new PM_CARTESIAN(478.538, -211.081, sensor_height);
			large_point2_l[1] = new PM_CARTESIAN(465.595, -222.640, sensor_height);
			large_point2_l[2] = new PM_CARTESIAN(457.988, -179.663, sensor_height);

			int* no_arm_l = new int[3];
			no_arm_l[0] = 0;
			no_arm_l[1] = 0;
			no_arm_l[2] = 1;

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, start, no_arm_l, dock_l, goal_l, large_point1_l, large_point2_l, 3);*/

			//args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->task_count);
			args->actuate = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

			if ((args->actuate = ulapi_sem_new(35)) == NULL)
			{
				cout << "Error: could not create ulapi mutex" << endl;
				exit(-1);
			}//end if

			//cart_client_connect_task(NULL);

			if ((cart_client_connect_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((cart_actuate_task = ulapi_task_new()) == NULL)
			{
				cout << "Error: could not create ulapi task" << endl;
				exit(-1);
			}//end if

			if ((err = ulapi_task_init(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_init(cart_actuate_task)) < 0)
			{
				cout << "Error: could not initialize ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client = new cart_comm_client(0, "192.168.160.52", 5352, 34, copy_config->start, copy_config->no_arm_list, copy_config->dock_list, copy_config->goal_list, copy_config->large_point1_list, copy_config->large_point2_list, copy_config->edge_start_list, copy_config->task_count, copy_config->runs);

			//PROBLEM: THIS task still runs after cart_client has been deallocated, need to stop this task
			if ((err = ulapi_task_start(cart_client_connect_task, cart_client_connect, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_start(cart_actuate_task, actuation_test_edge_bisect, (void*)args, ulapi_prio_lowest(), 0)) < 0)
			{
				cout << "Error: could not start ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_join(cart_actuate_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_stop(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not stop ulapi task" << endl;
				exit(err);
			}//end if

			args->cart_client->~cart_comm_client();
			args->cart_client = NULL;

			/*
			if ((err = ulapi_task_join(cart_client_connect_task, NULL)) < 0)
			{
				cout << "Error: could not join ulapi task" << endl;
				exit(err);
			}//end if
			*/

			if ((err = ulapi_task_delete(cart_actuate_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((err = ulapi_task_delete(cart_client_connect_task)) < 0)
			{
				cout << "Error: could not delete ulapi task" << endl;
				exit(err);
			}//end if

			if ((ulapi_sem_delete(args->actuate)) < 0)
			{
				cout << "Error: could not delete ulapi semaphore" << endl;
				exit(err);
			}//end if

			args->saved_config->write_file();

			args->saved_config->~cart_config();
			copy_config->~config_data();
			copy_config = NULL;

			//args->cart_client->~cart_comm_client();
			args->ur5->~CrpiRobot();
			free(args);
		}//end if
	}//end if

	}//end else if

	else if (user_in == 9)
	{

		PM_CARTESIAN* test_update1 = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));
		PM_CARTESIAN* test_update2 = (PM_CARTESIAN*)malloc(sizeof(PM_CARTESIAN));

		/*
		test_update1->x = 0;
		test_update1->y = 0;
		test_update1->z = 0;

		test_update2->x = 0;
		test_update2->y = 0;
		test_update2->z = 0;
		*/

		config_data* copy_config = new config_data();
		cart_config* saved_config_1 = new cart_config(0);
		saved_config_1->read_file();
		saved_config_1->get_config(copy_config);
		copy_config->print_data();
		saved_config_1->get_config(copy_config);
		copy_config->print_data();

		//saved_config_1->update_large_points(0, *test_update1, *test_update2);

		saved_config_1->write_file();
		saved_config_1->print_file();
		saved_config_1->~cart_config();

		cart_config* saved_config_2 = new cart_config(1);
		saved_config_2->read_file();
		saved_config_2->write_file();
		saved_config_2->print_file();
		saved_config_2->~cart_config();


	}//end else if

	exit(0);
}//end main