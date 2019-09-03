/**
*\file ld_comm.cpp
*\brief Function implementation for managing ARCL connections to the Omron Adept ld mobile robot platform. \n
*This code is for the outgoing arcl connection, whereby this control program opens \n
*a server connection and the ld core connects as a client. \n

*The Outgoing arcl connection can be configured in Mobile Planner uncer Configuration->Robot Interface->Outgoing ARCL Connection Setup and should \n
*be given the address and port number used on this machine to establish the server socket. \n

*For details about the Outgoing ARCL connection, see the ARCL User Guide by Adept, and personal notes that summarizes communications. \n

*Based on: agv_comm.h by J. Marvel, S. Legowik \n

*"_ftime, _ftime32, _ftime64 from MSDN" https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64

*\author Omar Aboul-Enein
*\date 2018-06-05
*/

#include "ld_comm.h"
#include "ulapi.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>

using namespace std;


/**
*Helper function for printing communication errors.
*\param[in] err Error code returned by ulapi socket
*\param[in] msg Error message the user desires to print.
*/
void ld_comm::comm_err(int err, char* msg)
{
	cout << "Comm Error " << err << " " << msg << endl;
}//end comm_err

ld_comm::ld_comm(ulapi_integer arcl_port, bool log)
{
	char log_name[128];
	time_t log_time;
	tm* log_tm = new tm;
	
	time(&log_time);
	localtime_s(log_tm, &log_time);
	strftime(log_name, sizeof(log_name), "..\\Data\\ld_pose_%d-%m-%Y_%I-%M-%S.csv", log_tm);
	cout << "ld Pose Log file created at: " << log_name << endl;
	
	if (log == TRUE)
	{
		ld_log.open(log_name);
		ld_log << "seconds_since_epoch" << ", " << " Client ID" << ", " << "Goal" << ", " << "robot_x" << ", " << "robot_y" << ", " << "robot_th" << endl;
	}//end if

	port = arcl_port;
}//end constructor

ld_comm::~ld_comm()
{
	int err;
	ld_log.close();

	if ((err = ulapi_socket_close(client_id)) < 0)
		comm_err(err, "client socket close failure");

	if ((err = ulapi_socket_close(server_id)) < 0)
		comm_err(err, "server socket close failure");

}//end destructor

int ld_comm::ld_connect()
{
	int err;
	
	if ((server_id = ulapi_socket_get_server_id(port)) < 0)
	{
		comm_err(server_id, "ulapi socket creation failure");
		return server_id;
	}//end if

	cout << "Outgoing ARCL Server started using port " << port << endl;

	if ((client_id = ulapi_socket_get_connection_id(server_id)) < 0)
	{
		comm_err(client_id, "ulapi socket creation failure");
		return client_id;
	}//end if

	cout << "Outgoing ARCL Client Accepted " << port << endl;

	return 0;
}//end ld_connect

int ld_comm::client_disconnect()
{
	int err;

	if ((err = ulapi_socket_close(client_id)) < 0)
	{
		comm_err(err, "client socket close failure");
		exit(err);
	}//end if

	return err;
}//end int

int ld_comm::arcl_dsfv_pose(ld_msg_pose* pose)
{
	//double pose_buf[4];
	char* arcl_resp;
	char* pose_str;
	int err;
	_timeb recv_time;

	//cout << "Entered" << endl;

	do
	{

		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err(err, "ulapi socket read failure");
			return err;
		}//end if

	} while (strstr(arcl_resp, "GetDataStoreFieldValues: RobotX ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: RobotX ");
	pose->robot_x = atof(pose_str);
	_ftime_s(&recv_time);
	pose->x_recv_time = recv_time;

	if ((err = arcl_read(&arcl_resp)) < 0)
	{
		comm_err(err, "ulapi socket read failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: RobotY ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: RobotY ");
	pose->robot_y = atof(pose_str);
	_ftime_s(&recv_time);
	pose->y_recv_time = recv_time;

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: RobotTh ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: RobotTh ");
	pose->robot_th = atof(pose_str);
	_ftime_s(&recv_time);
	pose->th_recv_time = recv_time;

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err(err, "ulapi socket read failure");
			return err;
		}//end if
	}while (strstr(arcl_resp, "GetDataStoreFieldValues: SecondsSinceEpoch ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: SecondsSinceEpoch ");
	pose->seconds_since_epoch = atof(pose_str);
	_ftime_s(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;


}//end ld_dsfv_pose


int ld_comm::arcl_read_status(char * status_string)
{
	char* arcl_resp;
	int err;

	do
	{

		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err(err, "ulapi socket read failure");
			return err;
		}//end if

		//cout << arcl_resp << endl;

	} while (strstr(arcl_resp, status_string) == nullptr);

	cout << arcl_resp << endl;

	return 0;
}//end read status


int ld_comm::arcl_read(char** arcl_recv_buf)
{
	string arcl_recv_str="";
	char* buf = (char*) malloc(1);
	int err;

	do
	{
		if ((err = ulapi_socket_read(client_id, buf, 1)) < 0)
		{
			comm_err(err, "ulapi socket read failure");
			return err;
		}//end if

		//cout << *buf;

		arcl_recv_str.push_back(*buf);

	}while (*buf != '\n');

	//cout << endl;

	//arcl_recv_buf.append('\0');


	*arcl_recv_buf= (char*)malloc(sizeof(char)*(arcl_recv_str.length()+1));
	strcpy_s(*arcl_recv_buf, sizeof(char)*(arcl_recv_str.length()+1), arcl_recv_str.c_str());

	free(buf);

	return 0;

}//end arcl_read


void ld_comm::print_pose(ld_msg_pose* msg)
{
	cout << fixed << msg->seconds_since_epoch << ", " << msg->robot_x << ", " << msg->robot_y << ", " << msg->robot_th <<", " << msg->server_time.time << "." << msg->server_time.millitm << endl;
}//end print_pose

void ld_comm::log_pose(ld_msg_pose* msg, ulapi_integer client_id, char* current_dock)
{
	ld_log << fixed << msg->seconds_since_epoch << ", " << client_id << ", " << current_dock << ", " << msg->robot_x << ", " << msg->robot_y << ", " << msg->robot_th << endl;
}//end log_pose

void ld_comm::log_comment(char* msg)
{
	ld_log << fixed << msg << endl;
}//end log_pose


