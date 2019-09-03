/**
*\file ld_msg.h
*\brief Definitions for data structures storing ld status and pose info.
*\author Omar Aboul-Enein
*\date 2017-06-05

*/

#ifndef ld_MSG_H
#define ld_MSG_H

#include <fstream>
#include "ulapi.h"
#include <sys\timeb.h>

using namespace std;

struct ld_msg_pose
{

	/**
	*\public
	*LD position along the x-axis of vehicle map (mm)
	*/
	double robot_x;

	/**
	*\public
	*LD position along the y-axis of vehicle map (mm)
	*/
	double robot_y;


	/**
	*\public
	*LD heading witin vehicle map (degrees)
	*/
	double robot_th;

	/**
	*\public
	*Timestamp from ARCL server Note that all timestamps are expressed as number of seconds since 1/1/1970 12:00 AM
	*/
	double seconds_since_epoch;


	/**
	*\public
	*Timestamp is taken server side from the vehicle process, measures the time that the full pose message was recieved on the Outgoing ARCL connection.
	*/
	_timeb server_time;

	/**
	*\public
	*Timestamp is taken server side from the vehicle process, measures the time that the x component of the pose message was recieved on the Outgoing ARCL connection.
	*/
	_timeb x_recv_time; 

	/**
	*\public
	*Timestamp is taken server side from the vehicle process, measures the time that the y component of the pose message was recieved on the Outgoing ARCL connection.
	*/
	_timeb y_recv_time; 

	/**
	*\public
	*Timestamp is taken server side from the vehicle process, measures the time that the theta component of the pose message was recieved on the Outgoing ARCL connection.
	*/
	_timeb th_recv_time; 
	
	/**
	*\public
	*Timestamp is taken server side from the vehicle process, measures the time that the server-side timestamp was recieved on the Outgoing ARCL connection.
	*/
	_timeb time_recv_time;
};//end struct
#endif