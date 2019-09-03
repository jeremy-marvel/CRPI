/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

lynx_comm.cpp

Description
===========

Function implementation for managing ARCL connections to the Omron Adept Lynx mobile robot platform.
This code is for the outgoing arcl connection, whereby this control program opens 
a server connection and the lynx core connects as a client.

The Outgoing arcl connection can be configured in Mobile Planner uncer Configuration->Robot Interface->Outgoing ARCL Connection Setup and should
be given the address and port number used on this machine to establish the server socket.

For details about the Outgoing ARCL connection, see the ARCL User Guide by Adept, and personal notes that summarizes communications.

Code Citations
==============

Based on:
	agv_comm.h by J. Marvel, S. Legowik

References
==========

"_ftime, _ftime32, _ftime64 from MSDN"
https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64
*/

#include "lynx_comm.h"
#include "ulapi.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>

using namespace std;

/*
comm_err

Description
===============================================================
Helper function for printing communication errors.

Paramaters
===============================================================
err -- error code returned by ulapi socket
msg -- error message the user desires to print.
*/
void comm_err(int err, char* msg)
{
	cout << "Comm Error " << err << " " << msg << endl;
}//end comm_err

 /*
 lynx_comm

 Description
 ===============================================================
 Constructor; initializes variables used to establish ulapi server socket.

 Paramaters
 ===============================================================
 arcl_port -- port number to use for the created server socket
 log -- boolean used to enable or disable logging
 */
lynx_comm::lynx_comm(ulapi_integer arcl_port, bool log)
{
	char log_name[128];
	time_t log_time;
	tm* log_tm;
	
	time(&log_time);
	log_tm = localtime(&log_time);
	strftime(log_name, sizeof(log_name), "..\\Applications\\Application_MobileRobot\\Data\\lynx_pose_%d-%m-%Y_%I-%M-%S.csv", log_tm);
	cout << "Lynx Pose Log file created at: " << log_name << endl;
	
	if (log == TRUE)
	{
		lynx_log.open(log_name);
		lynx_log << "seconds_since_epoch" << ", " << "robot_x" << ", " << "robot_y" << ", " << "robot_th" << endl;
	}//end if

	port = arcl_port;
}//end constructor

 /*
 ~lynx_comm

 Description
 ===============================================================
 Destructor; used to safely close server socket
 */
lynx_comm::~lynx_comm()
{
	int err;
	lynx_log.close();

	if ((err = ulapi_socket_close(client_id)) < 0)
		comm_err(err, "client socket close failure");

	if ((err = ulapi_socket_close(server_id)) < 0)
		comm_err(err, "server socket close failure");

}//end destructor

 /*
 lynx_connect

 Description
 ===============================================================
 Opens the server socket and then waits to accept the lynx to connect as a client.
 */
int lynx_comm::lynx_connect()
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
		comm_err(server_id, "ulapi socket creation failure");
		return server_id;
	}//end if

	cout << "Outgoing ARCL Client Accepted " << port << endl;

	return 0;
}//end lynx_connect

 /*
 client_disconnect

 Description
 ===============================================================
 Used to safely disconnect a currently running client connection.
 */
int lynx_comm::client_disconnect()
{
	int err;

	if ((err = ulapi_socket_close(client_id)) < 0)
	{
		comm_err(err, "client socket close failure");
		exit(err);
	}//end if

	return err;
}//end int

 /*
 arcl_dsfv_pose

 Description
 ===============================================================
 Reads the results of commands that are automatically executed on the vehicle and pushed to the control program.

 Paramaters
 ===============================================================
 pose -- datastructure to hold the pose and timestamp that is returned.
 */
int lynx_comm::arcl_dsfv_pose(lynx_msg_pose* pose)
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
	_ftime(&recv_time);
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
	_ftime(&recv_time);
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
	_ftime(&recv_time);
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
	_ftime(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;


}//end lynx_dsfv_pose

 /*
 arcl_read_status

 Description
 ===============================================================
 Used to pause the program until a specified broadcast message is received from the Lynx.

 Parameters
 =============================================
 status_string -- The broadcast message to wait for.
 */
int lynx_comm::arcl_read_status(char * status_string)
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

 /*
 arcl_read

 Description
 ===============================================================
 Reads responses from the ARCL server, returning responses line by line.

 Paramaters
 ===============================================================
 arcl_recv_buf - string pointer to hold the response recieved from the ARCL server.
 */
int lynx_comm::arcl_read(char** arcl_recv_buf)
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


	*arcl_recv_buf= (char*)malloc(arcl_recv_str.length());
	strcpy(*arcl_recv_buf, arcl_recv_str.c_str());

	free(buf);

	return 0;

}//end arcl_read

/*
print_pose

Description
===============================================================
Outputs the contents of a pose message to the console.

Paramaters
===============================================================
msg -- pose msg structure to be printed.
*/
void lynx_comm::print_pose(lynx_msg_pose* msg)
{
	cout << fixed << msg->seconds_since_epoch << ", " << msg->robot_x << ", " << msg->robot_y << ", " << msg->robot_th <<", " << msg->server_time.time << "." << msg->server_time.millitm << endl;
}//end print_pose

 /*
 log_pose

 Description
 ===============================================================
 Outputs the contents of a pose message to a log file.

 Paramaters
 ===============================================================
 msg -- pose msg structure to be logged
 */
void lynx_comm::log_pose(lynx_msg_pose* msg)
{
	lynx_log << fixed << msg->seconds_since_epoch << ", " << msg->robot_x << ", " << msg->robot_y << ", " << msg->robot_th << endl;
}//end log_pose

 /*
 log_comment

 Description
 ===============================================================
 Enters additional comments into the log. Useful for adding headers to the CSV file.

 Paramaters
 ===============================================================
 msg -- Message to be logged.
 */
void lynx_comm::log_comment(char* msg)
{
	lynx_log << fixed << msg << endl;
}//end log_pose


