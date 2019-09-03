/**
*\file ld_msg.h
*\brief Definitions for data structures storing ld status and pose info.
*\author Omar Aboul-Enein
*\date 2018-09-14
*/

#ifndef LD_MSG_H
#define LD_MSG_H

#include <fstream>
#include "ulapi.h"
#include <sys\timeb.h>

using namespace std;

struct ld_msg_pose
{
//Position (mm):
	/**
	*\public
	*X component of LD position expressed in millimeters.
	*/
	double robot_x;

	/**
	*\public
	*Y component of LD position expressed in millimeters.
	*/
	double robot_y;

//Heading (degrees):
	/**
	*\public
	*Heading of LD expressed in degrees.
	*/
	double robot_th;

//Timestamps:
	/**
	*\public
	*Timestamp from ARCL server measured as number of seconds since 1/1/1970 12:00 AM
	*/
	double seconds_since_epoch;

//Following timestamps are taken server-side, and measure the time of Outgoing ARCL message receipt:
	
	
	/**
	*\public
	*Timestamp from local computer upon reciept of the full pose. (Pose must be obtained using ARCL commands for each component).
	*/
	_timeb server_time;

	/**
	*\public
	*Timestamp from local computer upon reciept of the X position component of pose. (Pose must be obtained using ARCL commands for each component).
	*/
	_timeb x_recv_time;

	/**
	*\public
	*Timestamp from local computer upon reciept of the Y position component of pose. (Pose must be obtained using ARCL commands for each component).
	*/
	_timeb y_recv_time; 

	/**
	*\public
	*Timestamp from local computer upon reciept of the heading component of pose. (Pose must be obtained using ARCL commands for each component).
	*/
	_timeb th_recv_time; //receipt of server-side timestamp

	/**
	*\public
	*Timestamp from local computer upon reciept of the X position component of pose. (Pose must be obtained using ARCL commands for each component).
	*/
	_timeb time_recv_time;
};//end struct
#endif