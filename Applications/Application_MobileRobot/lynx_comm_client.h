/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

lynx_comm_client.h

Description
===========

Interface for managing ARCL connections to the Omron Adept Lynx mobile robot platform.
Note that this connection is one of two ways to communicate with the lynx. Here the control
program connects to the vehicle as a client.

Code Citations
==============

Based on:
		agv_comm.h by J. Marvel, S. Legowik

References
==========
	"_ftime, _ftime32, _ftime64 from MSDN"
		https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64

*/
#ifndef lynx_comm_client_H
#define lynx_comm_client_H
#include "ulapi.h"
#include "lynx_msg.h"
#include <fstream>

class lynx_comm_client
{
private:
	ulapi_integer id; //descriptor for the ulapi socket
	ulapi_integer port; //port number of the ARCL server
	char* ip_addr; //IP address of the ARCL server
	char* passwd; //Password needed to access the ARCL server
	ofstream lynx_log; //Log file for recording data from ARCL command responses. Typically used to record pose or timestamps for messages.

public:

//ctors and dtors:
	lynx_comm_client(char* arcl_addr, ulapi_integer arcl_port, char* arcl_passwd, bool log);
	~lynx_comm_client();

//Connection management:
	int lynx_connect(); //Connects to the ARCL server, sends the password login, and discards extra text sent back from the server

//Pose retrieval functions: 
//Poses are taken from the ARCL server DataStore and also includes various timestamps. There is one function for each type of DataStore Pose
	int arcl_dsfv_pose(lynx_msg_pose* pose);
	int arcl_dsfv_pose_encoder(lynx_msg_pose* pose);
	int arcl_dsfv_pose_interpolated(lynx_msg_pose* pose);
	int arcl_dsfv_pose_encoder_interpolated(lynx_msg_pose* pose);
	int arcl_dsfv_pose_localization(lynx_msg_pose* pose);
	int arcl_dsfv_pose_laser_localization(lynx_msg_pose* pose);
	int arcl_dsfv_pose_light_localization(lynx_msg_pose* pose);

//Other read and write functions:
	int arcl_read_status(); //Repetedly polls the socket until a goal arrival message is recieved from the ARCL server
	int arcl_read(char **arcl_recv_buf); //Reads ARCL server messages delmited by newlines.
	int arcl_write(char* arcl_cmd); //Sends a command to the ARCL server.

//Output and Logging:
	void print_pose(lynx_msg_pose* msg);
	void log_pose(lynx_msg_pose* msg);
	void log_comment(char* msg);

};//end lynx_comm_client
#endif