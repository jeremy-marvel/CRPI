/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

lynx_comm.h

Description
===========

Interface for managing ARCL connections to the Omron Adept Lynx mobile robot platform.

Code Citations
==============

Based on:
		agv_comm.h by J. Marvel, S. Legowik

References
==========
	"_ftime, _ftime32, _ftime64 from MSDN"
		https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64

*/
#ifndef LYNX_COMM_H
#define LYNX_COMM_H
#include "ulapi.h"
#include "lynx_msg.h"
#include <fstream>

class lynx_comm
{
private:
	ulapi_integer server_id; //descriptor for the ulapi server socket
	ulapi_integer client_id;
	ulapi_integer port; //port number of the ARCL server
	ofstream lynx_log; //Log file for recording data from ARCL command responses. Typically used to record pose or timestamps for messages.

public:

//ctors and dtors:
	lynx_comm(ulapi_integer arcl_port, bool log);
	~lynx_comm();

//Connection management:
	int lynx_connect(); //Creates server socket, accepts lynx connection as client.
	int client_disconnect(); //Disconnects the current outgoing arcl client.
//Pose retrieval functions: 
//Poses are taken from the ARCL server DataStore and also includes various timestamps.
	int arcl_dsfv_pose(lynx_msg_pose* pose);

//Other read and write functions:
	int arcl_read_status(char* status_string); //Repetedly polls the socket until a goal arrival message is recieved from the ARCL server
	int arcl_read(char **arcl_recv_buf); //Reads ARCL server messages delmited by newlines.
	//int arcl_write(char* arcl_cmd); //Sends a command to the ARCL server.

//Output and Logging:
	void print_pose(lynx_msg_pose* msg);
	void log_pose(lynx_msg_pose* msg);
	void log_comment(char* msg);

};//end lynx_comm
#endif