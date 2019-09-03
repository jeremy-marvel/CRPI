/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

lynx_msg.h

Description
===========

Definitions for data structures storing lynx status and pose info.

*/

#ifndef LYNX_MSG_H
#define LYNX_MSG_H

#include <fstream>
#include "ulapi.h"
#include <sys\timeb.h>

using namespace std;

struct lynx_msg_pose
{
//Position (mm):
	double robot_x;
	double robot_y;

//Heading (degrees):
	double robot_th;

//Timestamps:
// All timestamps expressed as number of seconds since 1/1/1970 12:00 AM
	double seconds_since_epoch; //Timestamp from ARCL server

//Following timestamps are taken server-side, and measure the time of Outgoing ARCL message receipt:
	_timeb server_time; //full pose
	_timeb x_recv_time; 
	_timeb y_recv_time; 
	_timeb th_recv_time; //receipt of server-side timestamp
	_timeb time_recv_time;
};//end struct
#endif