/**
*\file cart_comm_client.cpp
*\brief Function implementation for cart_comm_client object. Implements constructor and connection function to initialize socket connection
to correponding vehicle communication server. \n\n
*
*Based on:\n
*ld_comm_client by O. Aboul-Enein\n\n
*
*References:\n
*
*"_ftime, _ftime32, _ftime64 from MSDN"\n
*https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64 \n\n
*
*Using safe string functions:\n
*https://stackoverflow.com/questions/11752705/does-stdstring-contain-null-terminator \n
*https://stackoverflow.com/questions/19196301/buffer-is-too-small-when-doing-strcpy-s \n
*docs.microsoft.com/en-us/cpp/c-runtime-library/reference/strcpy-s-wcscpy-s-mbscpy-s \n
*\author Omar Aboul-Enein
*\date 2018-06-05
*/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include "cart_comm_client.h"
#include "ulapi.h"
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ur5_control.h"


using namespace std;

void cart_comm_client:: comm_err_client(int err, char* msg)
{
	cout << "Comm Error " << err << " " << msg << endl;
}//end comm_err_client

cart_comm_client::cart_comm_client(int no_cart_mode, char* server_addr, ulapi_integer server_port, ulapi_id key, char* start_str, int* no_arm_list, char** task_list, ld_msg_pose** goal_list, PM_CARTESIAN** large_point1_list, PM_CARTESIAN** large_point2_list, PM_CARTESIAN** edge_start_list, int len, int num_runs)
{
	ip_addr = (char*)malloc(sizeof(char)*(strlen(server_addr)+1));
	strcpy_s(ip_addr, sizeof(char)*(strlen(server_addr)+1), server_addr);

	port = server_port;
	client_stat = new cart_status(0, no_cart_mode, start_str, no_arm_list, task_list, goal_list, large_point1_list, large_point2_list, edge_start_list, len, num_runs);

	mutex = (ulapi_mutex_struct*)malloc(sizeof(ulapi_mutex_struct));

	if ((mutex = ulapi_mutex_new(key)) == NULL)
	{
		cout << "Error: could not create ulapi mutex" << endl;
		exit(-1);
	}//end if

}//end constructor

cart_comm_client::~cart_comm_client()
{
	int err = 0;

	ulapi_mutex_take(mutex);
		err = cart_client_disconnect();

		if (ip_addr != NULL)
		{
			free(ip_addr);
			ip_addr = NULL;
		}//end if

		if (client_stat != NULL)
		{
			delete client_stat;
			client_stat = NULL;
		}//end if
	ulapi_mutex_give(mutex);

	if ((err = ulapi_mutex_delete(mutex)) < 0)
		comm_err_client(err, "ulapi mutex delete failure");

	mutex = NULL;
	
}//end destructor

int cart_comm_client::cart_client_connect()
{
	int err = 0;

	if ((err = ulapi_socket_get_client_id(port, ip_addr)) < 0)
	{
		comm_err_client(err, "ulapi_socket_creation_failure");
		return err;
	}//end if

	ulapi_mutex_take(mutex);
		client_id = err;
		cout << "Connected to vehicle process at " << ip_addr << " on port " << port << ", using id " << client_id << endl;
	ulapi_mutex_give(mutex);

	return 0;
}//end cart_client_connect

int cart_comm_client::cart_client_disconnect()
{
	int err = 0;

	ulapi_mutex_take(mutex);
		if ((err = ulapi_socket_close(client_id)) < 0)
		{
			comm_err_client(err, "ulapi socket close failure");
			return err;
		}//end if
	ulapi_mutex_give(mutex);

	return err;
}//end cart_client_disconnect

int cart_comm_client::recv(char* buf, int len)
{
	int err;

	if ((err = ulapi_socket_read(get_client_id(), buf, len)) <= 0)
	{
		comm_err_client(err, "ulapi socket read failure");
		return err;
	}//end if

	return err;

}//end recv

int cart_comm_client::send(char* buf)
{
	int err;

	if ((err = ulapi_socket_write(get_client_id(), buf, sizeof(char)*strlen(buf))) <= 0)
	{
		comm_err_client(err, "ulapi socket read failure closing socket for client");
		return err;
	}//end if

	return err;
}//end send test


int cart_comm_client::cart_comm_client_read(char** cart_comm_client_recv_buf)
{
	string cart_comm_client_recv_str = "";
	char* buf = (char*)malloc(1);
	int err = 0;

	//cout << "READ FUNCTION:";

	do
	{
		if ((err = ulapi_socket_read(get_client_id(), buf, 1)) <= 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if

		 //cout << *buf;

		cart_comm_client_recv_str.push_back(*buf);

	} while (*buf != '\n');

	//cout<<"END READ FUNCTION" << endl;
	//arcl_recv_buf.append('\0');


	*cart_comm_client_recv_buf = (char*)malloc(sizeof(char)*(cart_comm_client_recv_str.length() + 1));
	strcpy_s(*cart_comm_client_recv_buf, sizeof(char)*(cart_comm_client_recv_str.length() + 1), cart_comm_client_recv_str.c_str());

	free(buf);

	return err;

}//end cart_comm_client_read

int cart_comm_client::remote_command(char* cmd)
{
	int err;
	char* client_str = NULL;
	//Note that the vehicle cannot send commands to update the manipulator's current task or change the status from the busy state to other states.
	//Only the manipulator can do this.

	if (strcmp(cmd, "rcstat") == 0)
	{
		ulapi_mutex_take(mutex);
		
			if (client_stat->to_string() != NULL)
			{
				client_str = (char*)malloc(sizeof(char)*(strlen(client_stat->to_string()) + 1));
				strcpy_s(client_str, sizeof(char)*(strlen(client_stat->to_string()) + 1), client_stat->to_string());//Exception triggered here, need to check if client_stat is NULL and maybe adjust mutex
			}//end if

		ulapi_mutex_give(mutex);

		if (client_str != NULL)
		{
			cout << "Sending status: '" << client_str << "'" << endl;
			if ((err = send(client_str) <= 0))
				return err;
		}//end if
		else
		{
			cart_client_disconnect();
		}//end else
	}//end if
	else if (strcmp(cmd, "rcstatinit") == 0)
	{
		ulapi_mutex_take(mutex);
			client_str = (char*)malloc(sizeof(char)*(strlen(client_stat->to_string_init()) + 1));
			strcpy_s(client_str, sizeof(char)*(strlen(client_stat->to_string_init()) + 1), client_stat->to_string_init());
		ulapi_mutex_give(mutex);

		if (client_str != NULL)
		{
			cout << "Sending status: '" << client_str << "'" << endl;
			if ((err = send(client_str) <= 0))
				return err;
		}//end if
		else
		{
			cart_client_disconnect();
		}//end else
	}//end else if
	else
	{
		ulapi_mutex_take(mutex);
			if (strcmp(cmd, "rcupdate0") == 0 && client_stat->get_status() != 2)
			{
				cout << "Changing status to 0 (Ready)" << endl;
				client_stat->set_status(0);
			}//end if

			else if (strcmp(cmd, "rcupdate1") == 0 && client_stat->get_status() != 2)
			{
				cout << "Changing status to 1 (Waiting)" << endl;
				client_stat->set_status(1);
			}//end if

			else if (strcmp(cmd, "rcupdate2") == 0 && client_stat->get_status() != 2 && client_stat->get_status() == 1)
			{
				cout << "Changing status to 2 (Busy)" << endl;
				client_stat->set_status(2);
			}//end if
		ulapi_mutex_give(mutex);
	}//end else

	free(client_str);
	client_str = NULL;

	return 0;

}//end remote_command

void cart_comm_client::local_command(char* cmd)
{
	ulapi_mutex_take(mutex);
		if (strcmp(cmd, "lcupdate0") == 0)
		{
			cout << "Changing status to 0 (Ready)" << endl;
			client_stat->set_status(0);
		}//end if

		else if (strcmp(cmd, "lcupdate1") == 0)
		{
			cout << "Changing status to 1 (Waiting)" << endl;
			client_stat->set_status(1);
		}//end if

		else if (strcmp(cmd, "lcupdate2") == 0)
		{
			cout<< "Changing status to 2 (Busy)" << endl;
			client_stat->set_status(2);
		}//end if

		else if (strcmp(cmd, "lcupdatedock") == 0)
		{
			cout << "Updating current task" << endl;
			client_stat->update_current_dock();
		}//end else if
	ulapi_mutex_give(mutex);
}//end local_command

int cart_comm_client::get_stat_value()
{
	int retval = -1;
	
	ulapi_mutex_take(mutex);
		retval = client_stat->get_status();
	ulapi_mutex_give(mutex);

	return retval;

}//end get_stat_value

ulapi_integer cart_comm_client:: get_client_id()
{
	ulapi_integer temp;

	ulapi_mutex_take(mutex);
		temp = client_id;
	ulapi_mutex_give(mutex);

	return temp;
}//end get_client_id

int cart_comm_client::parse_msg(char* msg)
{
	char* cmd;
	char* pose;
	char* next;

	int err = 0;

	cmd = strtok_s(msg, ":", &next);
	pose = strtok_s(NULL, ":", &next);

	cout << "cmd=" << cmd << endl;
	cout << "pose=" << pose << endl;

	if (client_stat->get_status() != 2)
	{
		ulapi_mutex_take(mutex);
		client_stat->string_to_pose(pose);
		ulapi_mutex_give(mutex);
	}//end if

	err = remote_command(cmd);

	return err;
}//end parse_msg

int cart_comm_client::get_stat_count()
{
	int count;

	ulapi_mutex_take(mutex);
		count = client_stat->get_dock_count();
	ulapi_mutex_give(mutex);

	return count;

}//end get_stat_count

void cart_comm_client::test_square(CrpiRobot<CrpiUniversal> * ur_robot, bool level_pause)
{
	CanonReturn retval;
	robotPose cur_pose, world_pose;
	robotPose sq_search_pose1;
	robotPose start_r;
	PM_CARTESIAN square_point1_world;
	PM_CARTESIAN square_point2_world;
	PM_POSE square_ld;
	PM_CARTESIAN large1;
	PM_CARTESIAN large2;
	PM_POSE* pm_ld = NULL;
	crpi_timer pause;
	int err;
	int err1, err2, err3;
	int no_arm, no_cart;
	ld_msg_pose cur_ld_pose;
	int mytime_seconds;

	mytime_seconds = get_seconds();

	ulapi_mutex_take(mutex);
		no_arm = client_stat->get_current_no_arm();
		no_cart = client_stat->get_no_cart();
	ulapi_mutex_give(mutex);

	if (no_arm == 0 && no_cart == 0 && ur_robot != NULL)
	{
		cout << "Lowering Feet..." << endl;

		if ((retval = lower_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet extended." << endl;

		if (level_pause)
			pause.waitUntil(15000);

		cout << "Now staging manipulator..." << endl;

		if ((retval = stage_arm(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Manipulator staged" << endl;

		pause.waitUntil(robot_settle_time);

		ulapi_mutex_take(mutex);
			client_stat->get_current_pose(cur_ld_pose);

			pm_ld = new PM_POSE(PM_CARTESIAN(cur_ld_pose.robot_x, cur_ld_pose.robot_y, 0), PM_RPY(0, 0, cur_ld_pose.robot_th * TO_RAD));
			err1 = client_stat->get_current_goal(square_ld);
			err2 = client_stat->get_current_large_point1(large1);
			err3 = client_stat->get_current_large_point2(large2);
		ulapi_mutex_give(mutex);


		if (err1 >= 0 && err2 >= 0 && err3 >= 0 && pm_ld != NULL)
		{
			cout << "Lynx Point: (" << cur_ld_pose.robot_x << ", " << cur_ld_pose.robot_y << "," << cur_ld_pose.robot_th << ")" << endl;
			compute_start(square_point1_world, square_point2_world, large1, large2, square_ld, *pm_ld);

			cout << "Start Point 1: (" << square_point1_world.x << ", " << square_point1_world.y << ", " << square_point1_world.z << ")" << endl;
			cout << "Start Point 2: (" << square_point2_world.x << ", " << square_point2_world.y << ", " << square_point2_world.z << ")" << endl;
			square_bisect_short(ur_robot, square_point1_world, square_point2_world, numIters, mytime_seconds, client_stat->get_current_dock(), false);
		}//end if

		cout << "Now stowing manipulator..." << endl;

		if ((retval = stow_arm(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Manipulator stowed" << endl;

		cout << "Raising Feet..." << endl;

		if ((retval = raise_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet rectracted." << endl;
	}//end if

	else
	{
		cout << "No cart or arm present. Simulating static load. Now Sleeping for 10 seconds..." << endl;
		Sleep(10000);
	}//end else

}//end test_square

void cart_comm_client::test_square_auto_update(CrpiRobot<CrpiUniversal> * ur_robot, PM_CARTESIAN* update1, PM_CARTESIAN* update2, int* bisect_stat)
{
	CanonReturn retval;
	robotPose cur_pose, world_pose;
	robotPose sq_search_pose1;
	robotPose start_r;
	PM_CARTESIAN square_point1_world;
	PM_CARTESIAN square_point2_world;
	PM_POSE square_ld;
	PM_CARTESIAN large1;
	PM_CARTESIAN large2;
	PM_POSE* pm_ld = NULL;
	crpi_timer pause;
	int err;
	int err1, err2, err3;
	int no_arm, no_cart;
	ld_msg_pose cur_ld_pose;
	int mytime_seconds;

	mytime_seconds = get_seconds();

	ulapi_mutex_take(mutex);
	no_arm = client_stat->get_current_no_arm();
	no_cart = client_stat->get_no_cart();
	ulapi_mutex_give(mutex);

	if (no_arm == 0 && no_cart == 0 && ur_robot != NULL)
	{
		cout << "Lowering Feet..." << endl;

		if ((retval = lower_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet extended." << endl;

		cout << "Now staging manipulator..." << endl;

		if ((retval = stage_arm(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Manipulator staged" << endl;

		pause.waitUntil(robot_settle_time);

		ulapi_mutex_take(mutex);
		client_stat->get_current_pose(cur_ld_pose);

		pm_ld = new PM_POSE(PM_CARTESIAN(cur_ld_pose.robot_x, cur_ld_pose.robot_y, 0), PM_RPY(0, 0, cur_ld_pose.robot_th * TO_RAD));
		err1 = client_stat->get_current_goal(square_ld);
		err2 = client_stat->get_current_large_point1(large1);
		err3 = client_stat->get_current_large_point2(large2);
		ulapi_mutex_give(mutex);


		if (err1 >= 0 && err2 >= 0 && err3 >= 0 && pm_ld != NULL)
		{
			cout << "Lynx Point: (" << cur_ld_pose.robot_x << ", " << cur_ld_pose.robot_y << "," << cur_ld_pose.robot_th << ")" << endl;
			compute_start(square_point1_world, square_point2_world, large1, large2, square_ld, *pm_ld);

			cout << "Start Point 1: (" << square_point1_world.x << ", " << square_point1_world.y << ", " << square_point1_world.z << ")" << endl;
			cout << "Start Point 2: (" << square_point2_world.x << ", " << square_point2_world.y << ", " << square_point2_world.z << ")" << endl;
			retval = square_bisect_short(ur_robot, square_point1_world, square_point2_world, numIters, mytime_seconds, client_stat->get_current_dock(), update1, update2, false);

			if (retval != CANON_SUCCESS)
				*bisect_stat = 1;
			else
				*bisect_stat = 0;

		}//end if

		cout << "Now stowing manipulator..." << endl;
		
		if ((retval = stow_arm(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Manipulator stowed" << endl;

		cout << "Raising Feet..." << endl;

		if ((retval = raise_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet rectracted." << endl;
	}//end if

	else
	{
		cout << "No cart or arm present. Simulating static load. Now Sleeping for 10 seconds..." << endl;
		Sleep(10000);
	}//end else

}//end test_square

void cart_comm_client::test_edge(CrpiRobot<CrpiUniversal> * ur_robot, bool level_pause)
{
	CanonReturn retval;
	robotPose cur_pose, world_pose, check_pose, r3_start;
	robotPose sq_search_pose1;
	robotPose start_r;
	PM_CARTESIAN r_world;
	PM_POSE square_ld;
	PM_CARTESIAN r;
	PM_POSE* pm_ld = NULL;
	crpi_timer pause;
	int err;
	int err1, err2, err3;
	int no_arm, no_cart;
	ld_msg_pose cur_ld_pose;
	int mytime_seconds;

	//PM_CARTESIAN start_point1;
	//start_point1.x = 180.837;
	//start_point1.y = 90.043;
	//start_point1.z = sensor_height;

	//PM_CARTESIAN start_point2;
	//start_point2.x = 180.837;
	//start_point2.y = -40.053;
	//start_point2.z = sensor_height;

	//PM_CARTESIAN start_point3;
	//start_point3.x = 451.515;
	//start_point3.y = 464.485;
	//start_point3.z = sensor_height;

	mytime_seconds = get_seconds();

	ulapi_mutex_take(mutex);
	no_arm = client_stat->get_current_no_arm();
	no_cart = client_stat->get_no_cart();
	ulapi_mutex_give(mutex);

	if (no_arm == 0 && no_cart == 0 && ur_robot != NULL)
	{
		cout << "Lowering Feet..." << endl;

		if ((retval = lower_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet extended." << endl;

		if (level_pause)
			pause.waitUntil(15000);

		cout << "Now staging manipulator..." << endl;

		if ((retval = stage_arm_edge(ur_robot, r3_start)) == CANON_SUCCESS)
		{
			cout << "Manipulator staged" << endl;

			pause.waitUntil(robot_settle_time);

			ulapi_mutex_take(mutex);
			client_stat->get_current_pose(cur_ld_pose);

			pm_ld = new PM_POSE(PM_CARTESIAN(cur_ld_pose.robot_x, cur_ld_pose.robot_y, 0), PM_RPY(0, 0, cur_ld_pose.robot_th * TO_RAD));
			err1 = client_stat->get_current_goal(square_ld);
			err2 = client_stat->get_current_edge_start(r);
			ulapi_mutex_give(mutex);

			if (err1 >= 0 && err2 >= 0 && pm_ld != NULL)
			{
				cout << "Lynx Point: (" << cur_ld_pose.robot_x << ", " << cur_ld_pose.robot_y << "," << cur_ld_pose.robot_th << ")" << endl;
				compute_start(r_world, r, square_ld, *pm_ld);

				cout << "Start Point 1: (" << r_world.x << ", " << r_world.y << ", " << r_world.z << ")" << endl;

				cout << "r3_start = (" << r3_start.x << ", " << r3_start.y << ")" << endl;
				setPosition(check_pose, stage_3, sensor_rot);
				ur_robot->GetRobotPose(&cur_pose);

				if (cur_pose.distance(check_pose) < 0.1)
					square_edge_short(ur_robot, rob2cart(cur_pose), r_world.x, r_world.y, 4 * 25.4, 2, 9 * 25.4, numIters, mytime_seconds, client_stat->get_current_dock(), true);
				else
					cout << "Could not start edge registration test. Pose check failed" << endl;
			}//end if
		}//end if
		else
			exit(retval);

		cout << "Now stowing manipulator..." << endl;

		if ((retval = stow_arm(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Manipulator stowed" << endl;

		cout << "Raising Feet..." << endl;

		if ((retval = raise_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet rectracted." << endl;
	}//end if

	else
	{
		cout << "No cart or arm present. Simulating static load. Now Sleeping for 10 seconds..." << endl;
		Sleep(10000);
	}//end else

}//end test_edge

void cart_comm_client::test_edge_cont(CrpiRobot<CrpiUniversal> * ur_robot, bool level_pause)
{
	CanonReturn retval;
	robotPose cur_pose, world_pose, check_pose, r3_start;
	robotPose sq_search_pose1;
	robotPose start_r;
	PM_CARTESIAN r_world;
	PM_POSE square_ld;
	PM_CARTESIAN r;
	PM_POSE* pm_ld = NULL;
	crpi_timer pause;
	int err;
	int err1, err2, err3;
	int no_arm, no_cart;
	ld_msg_pose cur_ld_pose;
	int mytime_seconds;

	//PM_CARTESIAN start_point1;
	//start_point1.x = 180.837;
	//start_point1.y = 90.043;
	//start_point1.z = sensor_height;

	//PM_CARTESIAN start_point2;
	//start_point2.x = 180.837;
	//start_point2.y = -40.053;
	//start_point2.z = sensor_height;

	//PM_CARTESIAN start_point3;
	//start_point3.x = 451.515;
	//start_point3.y = 464.485;
	//start_point3.z = sensor_height;

	mytime_seconds = get_seconds();

	ulapi_mutex_take(mutex);
	no_arm = client_stat->get_current_no_arm();
	no_cart = client_stat->get_no_cart();
	ulapi_mutex_give(mutex);

	if (no_arm == 0 && no_cart == 0 && ur_robot != NULL)
	{
		cout << "Lowering Feet..." << endl;

		if ((retval = lower_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet extended." << endl;

		if (level_pause)
			pause.waitUntil(15000);

		cout << "Now staging manipulator..." << endl;

		if ((retval = stage_arm_edge(ur_robot, r3_start)) == CANON_SUCCESS)
		{
			cout << "Manipulator staged" << endl;

			pause.waitUntil(robot_settle_time);

			ulapi_mutex_take(mutex);
			client_stat->get_current_pose(cur_ld_pose);

			pm_ld = new PM_POSE(PM_CARTESIAN(cur_ld_pose.robot_x, cur_ld_pose.robot_y, 0), PM_RPY(0, 0, cur_ld_pose.robot_th * TO_RAD));
			err1 = client_stat->get_current_goal(square_ld);
			err2 = client_stat->get_current_edge_start(r);
			ulapi_mutex_give(mutex);

			if (err1 >= 0 && err2 >= 0 && pm_ld != NULL)
			{
				cout << "Lynx Point: (" << cur_ld_pose.robot_x << ", " << cur_ld_pose.robot_y << "," << cur_ld_pose.robot_th << ")" << endl;
				compute_start(r_world, r, square_ld, *pm_ld);

				cout << "Start Point 1: (" << r_world.x << ", " << r_world.y << ", " << r_world.z << ")" << endl;

				cout << "r3_start = (" << r3_start.x << ", " << r3_start.y << ")" << endl;
				setPosition(check_pose, stage_3, sensor_rot);
				ur_robot->GetRobotPose(&cur_pose);

				if (cur_pose.distance(check_pose) < 0.1)
					square_edge_short_cont(ur_robot, rob2cart(cur_pose), r_world.x, r_world.y, 4 * 25.4, 2, 9 * 25.4, numIters, mytime_seconds, client_stat->get_current_dock(), true);
				else
					cout << "Could not start edge registration test. Pose check failed" << endl;
			}//end if
		}//end if
		else
			exit(retval);

		cout << "Now stowing manipulator..." << endl;

		if ((retval = stow_arm(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Manipulator stowed" << endl;

		cout << "Raising Feet..." << endl;

		if ((retval = raise_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet rectracted." << endl;
	}//end if

	else
	{
		cout << "No cart or arm present. Simulating static load. Now Sleeping for 10 seconds..." << endl;
		Sleep(10000);
	}//end else

}//end test_edge_cont


//From 07-03-2019 tests, Trial 6 (10:21 AM) found a hard-to-reproduce issue. The manipulator pans back towards the base before incrementing with small steps towards the table.
//This prolonged the regsitration time. After reviewing the footage I think the issue involves the UR Controller response time. Adding additional pauses between IO reads may help.
void cart_comm_client::test_edge_cont2(CrpiRobot<CrpiUniversal> * ur_robot, bool level_pause)
{
	CanonReturn retval;
	robotPose cur_pose, world_pose, check_pose, r3_start;
	robotPose sq_search_pose1;
	robotPose start_r;
	PM_CARTESIAN r_world;
	PM_POSE square_ld;
	PM_CARTESIAN r;
	PM_POSE* pm_ld = NULL;
	crpi_timer pause;
	int err;
	int err1, err2, err3;
	int no_arm, no_cart;
	ld_msg_pose cur_ld_pose;
	int mytime_seconds;

	//PM_CARTESIAN start_point1;
	//start_point1.x = 180.837;
	//start_point1.y = 90.043;
	//start_point1.z = sensor_height;

	//PM_CARTESIAN start_point2;
	//start_point2.x = 180.837;
	//start_point2.y = -40.053;
	//start_point2.z = sensor_height;

	//PM_CARTESIAN start_point3;
	//start_point3.x = 451.515;
	//start_point3.y = 464.485;
	//start_point3.z = sensor_height;

	mytime_seconds = get_seconds();

	ulapi_mutex_take(mutex);
	no_arm = client_stat->get_current_no_arm();
	no_cart = client_stat->get_no_cart();
	ulapi_mutex_give(mutex);

	if (no_arm == 0 && no_cart == 0 && ur_robot != NULL)
	{
		cout << "Lowering Feet..." << endl;

		if ((retval = lower_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet extended." << endl;

		if (level_pause)
			pause.waitUntil(15000);

		cout << "Now staging manipulator..." << endl;

		if ((retval = stage_arm_edge(ur_robot, r3_start)) == CANON_SUCCESS)
		{
			cout << "Manipulator staged" << endl;

			pause.waitUntil(robot_settle_time);

			ulapi_mutex_take(mutex);
			client_stat->get_current_pose(cur_ld_pose);

			pm_ld = new PM_POSE(PM_CARTESIAN(cur_ld_pose.robot_x, cur_ld_pose.robot_y, 0), PM_RPY(0, 0, cur_ld_pose.robot_th * TO_RAD));
			err1 = client_stat->get_current_goal(square_ld);
			err2 = client_stat->get_current_edge_start(r);
			ulapi_mutex_give(mutex);

			if (err1 >= 0 && err2 >= 0 && pm_ld != NULL)
			{
				cout << "Lynx Point: (" << cur_ld_pose.robot_x << ", " << cur_ld_pose.robot_y << "," << cur_ld_pose.robot_th << ")" << endl;
				compute_start(r_world, r, square_ld, *pm_ld);//Can remove this, just use r since r.y is only used to determine which side of the table cart is on.

				cout << "Start Point 1: (" << r_world.x << ", " << r_world.y << ", " << r_world.z << ")" << endl;

				cout << "r3_start = (" << r3_start.x << ", " << r3_start.y << ")" << endl;
				setPosition(check_pose, stage_3, sensor_rot);
				ur_robot->GetRobotPose(&cur_pose);

				if (cur_pose.distance(check_pose) < 0.1)
					square_edge_short_cont2(ur_robot, rob2cart(cur_pose), r_world.y, 4 * 25.4, 2, 9 * 25.4, numIters, mytime_seconds, client_stat->get_current_dock(), true);
				else
					cout << "Could not start edge registration test. Pose check failed" << endl;
			}//end if
		}//end if
		else
			exit(retval);

		cout << "Now stowing manipulator..." << endl;

		if ((retval = stow_arm(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Manipulator stowed" << endl;

		cout << "Raising Feet..." << endl;

		if ((retval = raise_feet(ur_robot)) != CANON_SUCCESS)
			exit(retval);

		cout << "Feet rectracted." << endl;
	}//end if

	else
	{
		cout << "No cart or arm present. Simulating static load. Now Sleeping for 10 seconds..." << endl;
		Sleep(10000);
	}//end else

}//end test_edge_cont


void cart_comm_client::print_info()
{
	ulapi_mutex_take(mutex);
		client_stat->print_status();
	ulapi_mutex_give(mutex);

	return;
}//end print_info