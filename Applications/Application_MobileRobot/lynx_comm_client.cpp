/*
Author:				Omar Aboul-Enein
Creation Date:		6/5/2017
Division:			ISD
Supervisor:			Roger Bostelman

lynx_comm_client.h

Description
===========

Interface for managing ARCL connections to the Omron Adept Lynx mobile robot platform.
This is the standard client / server connection where the lynx core functions as the server and
This control program connects as a client.

Note that ARCL commands must be terminated by a newline ("\n") character when sent to the ARCL server.
See ARCL user guide from Adept or use the 'help' command when connecting via Telnet client to the lynx
to see a list of available commands.

The ARCL Server Connection parameters can be configured via Mobile Planner under Configuration -> Robot Interface -> ARCL Server Setup

Code Citations
==============

Based on:
	agv_comm.h by J. Marvel, S. Legowik

References
==========

"_ftime, _ftime32, _ftime64 from MSDN"
https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64

*/

#include "lynx_comm_client.h"
#include "ulapi.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

/*
comm_err_client

Description
===============================================================
Helper function for printing communication errors.

Paramaters
===============================================================
err -- error code returned by ulapi socket
msg -- error message the user desires to print.
*/
void comm_err_client(int err, char* msg)
{
	cout << "Comm Error " << err << " " << msg << endl;
}//end comm_err_client

 /*
 lynx_comm_client

 Description
 ===============================================================
 Constructor for lynx_comm object. Creates log file for transmitted poses, initializes the 
 address of the lynx ARCl server, and password for logging into the server
 The values passed into this function are determined by the Adept Lynx Configuration Parameters.
 These can be changed from the vehicle under Interface Options in Mobile Planner.
 Then, when calling this constructor, the new parameters can be entered.

 Paramaters
 ===============================================================
 arcl_addr -- IP address of the lynx ARCL server
 arcl_port -- port number used when connecting to ARCL server
 arcl_passwd -- password given to the ARCL server to login.
 log -- boolean used to enable logging of ARCL data.
 */
lynx_comm_client::lynx_comm_client(char* arcl_addr, ulapi_integer arcl_port, char* arcl_passwd, bool log)
{

	if (log == TRUE)
	{
		lynx_log.open("lynx_log.csv");
	}//end if
	
	char* temp_passwd;

	ip_addr = (char*) malloc(strlen(arcl_addr));
	strcpy(ip_addr, arcl_addr);

	port = arcl_port;

	
	temp_passwd = (char*)malloc(strlen(arcl_passwd) + 1);
	strcpy(temp_passwd, arcl_passwd);
	strcat(temp_passwd, "\n");
	passwd = (char*)malloc(strlen(arcl_passwd) + 1);
	passwd = temp_passwd;
}//end constructor

 /*
 ~lynx_comm_client

 Description
 ===============================================================
 Destructor, safely closes the network connection to the lynx.
 */
lynx_comm_client::~lynx_comm_client()
{
	int err;
	lynx_log.close();
	if ((err = ulapi_socket_close(id)) < 0)
		comm_err_client(err, "socket close failure");

}//end destructor

 /*
 lynx_connect

 Description
 ===============================================================
 This function establishes the socket connection to the lynx ARCL server.
 This includes opening the client socket, sending the password to login to the lynx,
 and parsing extra input generated by the ARCL server.
 */

int lynx_comm_client::lynx_connect()
{
	int err;
	char* arcl_recv_buf;
	
	//Open the client socket to the ARCL server
	if ((id = ulapi_socket_get_client_id(port, ip_addr)) < 0)
	{
		comm_err_client(id, "ulapi socket creation failure");
		return id;
	}//end if


	//Read response from server, which prompts for a password
	cout << "Connected to ARCL Server at " << ip_addr << " using port " << port << endl;
	arcl_recv_buf = (char*)malloc(strlen("Enter password:")+1);
	arcl_recv_buf[strlen("Enter password:")] = '\0';

	if ((err = ulapi_socket_read(id, arcl_recv_buf, strlen("Enter password:"))) < 0)
	{
		comm_err_client(err, "ulapi socket read failure");
		return err;
	}//end if

	free(arcl_recv_buf);

	//Send the password to the lynx ARCL server
	if ((err = ulapi_socket_write(id, passwd, strlen(passwd))) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	arcl_recv_buf = (char*)malloc(2);
	arcl_recv_buf[1] = '\0';

	//Parse the response from the lynx. The ARCL server automatically
	//executes the 'help' command, which outputs all commands that can be executed
	//on the server. This response is accepted and then discarded.
	for(int i = 0; i<7537; i++)
	{
		//cout << arcl_recv_buf << endl;
		if ((err = ulapi_socket_read(id, arcl_recv_buf, 1)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}
	}//end while

	return 0;
}//end lynx_connect

 /*
 arcl_dsfv_pose

 Description
 ===============================================================
 Sends multiple commands to get the current X, Y, and Theta pose of the vehicle,
 plus another to get the timestamp. Parses the response and then groups
 the data to store within a datastructure. Access pose and status
 data within the ARCl DataStore. This not the best way to monitor the lynx
 as multiple commands issued in sequence are not guaranteed to return an atomic
 response.

 Paramaters
 ===============================================================
 pose -- datastructure to hold the pose and timestamp that is returned.
 */
int lynx_comm_client::arcl_dsfv_pose(lynx_msg_pose* pose)
{
	//double pose_buf[4];
	char* arcl_resp;
	char* pose_str;
	int err;
	_timeb recv_time;

	//cout << "Entered" << endl;

	//Request the X Position
	if ((err = arcl_write("dsfv RobotX\n") < 0))
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	//Program loops until the appropriate repsonse to the command is recieved.
	do
	{

		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if

	} while (strstr(arcl_resp, "GetDataStoreFieldValues: RobotX ") == nullptr);
	//In the while loop above, if the response contains that particular string, we
	//know that the reposne is associated with the command just sent.

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: RobotX ");
	pose->robot_x = atof(pose_str);
	_ftime(&recv_time);
	pose->x_recv_time = recv_time;

	//Extra read included to get rid of an extra line of response returned by ARCL
	if ((err = arcl_read(&arcl_resp)) < 0)
	{
		comm_err_client(err, "ulapi socket read failure");
		return err;
	}//end if

	//Rest of the code follows the same pattern, but applies commands to get the rest of the pose
	//and timestep.

	if ((err = arcl_write("dsfv RobotY\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: RobotY ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: RobotY ");
	pose->robot_y = atof(pose_str);
	_ftime(&recv_time);
	pose->y_recv_time = recv_time;

	if ((err = arcl_write("dsfv RobotTh\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: RobotTh ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: RobotTh ");
	pose->robot_th = atof(pose_str);
	_ftime(&recv_time);
	pose->th_recv_time = recv_time;

	if ((err = arcl_write("dsfv SecondsSinceEpoch\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
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
 arcl_dsfv_pose_encoder

 Description
 ===============================================================
 Same as arcl_dsfv_pose, but uses a different ARCL command to access an alternate pose in the data store, see arcl_dsfv_pose.
 Not currently used in performance test.

 Paramaters
 ===============================================================
 pose -- datastructure to hold the pose and timestamp that is returned.
 */
int lynx_comm_client::arcl_dsfv_pose_encoder(lynx_msg_pose* pose)
{
	//double pose_buf[4];
	char* arcl_resp;
	char* pose_str;
	int err;
	_timeb recv_time;
	//cout << "Entered" << endl;

	if ((err = arcl_write("dsfv EncoderX\n") < 0))
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: EncoderX ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: EncoderX ");
	pose->robot_x = atof(pose_str);
	_ftime(&recv_time);
	pose->x_recv_time = recv_time;

	if ((err = arcl_write("dsfv EncoderY\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: EncoderY ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: EncoderY ");
	pose->robot_y = atof(pose_str);
	_ftime(&recv_time);
	pose->y_recv_time = recv_time;

	if ((err = arcl_write("dsfv EncoderTh\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{

		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: EncoderTh ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: EncoderTh ");
	pose->robot_th = atof(pose_str);
	_ftime(&recv_time);
	pose->th_recv_time = recv_time;

	if ((err = arcl_write("dsfv SecondsSinceEpoch\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: SecondsSinceEpoch ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: SecondsSinceEpoch ");
	pose->seconds_since_epoch = atof(pose_str);
	_ftime(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;


}//end lynx_dsfv_pose_encoder


 /*
 arcl_dsfv_pose_interpolated

 Description
 ===============================================================
 Same as arcl_dsfv_pose, but uses a different ARCL command to access an alternate pose in the data store, see arcl_dsfv_pose.
 Not currently used in performance test.

 Paramaters
 ===============================================================
 pose -- datastructure to hold the pose and timestamp that is returned.
 */
int lynx_comm_client::arcl_dsfv_pose_interpolated(lynx_msg_pose* pose)
{
	//double pose_buf[4];
	char* arcl_resp;
	char* pose_str;
	int err;
	_timeb recv_time;
	//cout << "Entered" << endl;

	if ((err = arcl_write("dsfv RobotPoseInterpolated_X\n") < 0))
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: RobotPoseInterpolated_X ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: RobotPoseInterpolated_X ");
	pose->robot_x = atof(pose_str);
	_ftime(&recv_time);
	pose->x_recv_time = recv_time;

	if ((err = arcl_write("dsfv RobotPoseInterpolated_Y\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: RobotPoseInterpolated_Y ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: RobotPoseInterpolated_Y ");
	pose->robot_y = atof(pose_str);
	_ftime(&recv_time);
	pose->y_recv_time = recv_time;

	if ((err = arcl_write("dsfv RobotPoseInterpolated_Th\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: RobotPoseInterpolated_Th ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: RobotPoseInterpolated_Th ");
	pose->robot_th = atof(pose_str);
	_ftime(&recv_time);
	pose->th_recv_time = recv_time;

	if ((err = arcl_write("dsfv SecondsSinceEpoch\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: SecondsSinceEpoch ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: SecondsSinceEpoch ");
	pose->seconds_since_epoch = atof(pose_str);
	_ftime(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end lynx_dsfv_pose_encoder

 /*
 arcl_dsfv_pose_encoder_interpolated

 Description
 ===============================================================
 Same as arcl_dsfv_pose, but uses a different ARCL command to access an alternate pose in the data store, see arcl_dsfv_pose.
 Not currently used in performance test.

 Paramaters
 ===============================================================
 pose -- datastructure to hold the pose and timestamp that is returned.
 */
int lynx_comm_client::arcl_dsfv_pose_encoder_interpolated(lynx_msg_pose* pose)
{
	//double pose_buf[4];
	char* arcl_resp;
	char* pose_str;
	int err;
	_timeb recv_time;

	//cout << "Entered" << endl;

	if ((err = arcl_write("dsfv EncoderPoseInterpolated_X\n") < 0))
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: EncoderPoseInterpolated_X ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: EncoderPoseInterpolated_X ");
	pose->robot_x = atof(pose_str);
	_ftime(&recv_time);
	pose->x_recv_time = recv_time;

	if ((err = arcl_write("dsfv EncoderPoseInterpolated_Y\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: EncoderPoseInterpolated_Y ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: EncoderPoseInterpolated_Y ");
	pose->robot_y = atof(pose_str);
	_ftime(&recv_time);
	pose->y_recv_time = recv_time;

	if ((err = arcl_write("dsfv EncoderPoseInterpolated_Th\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: EncoderPoseInterpolated_Th ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: EncoderPoseInterpolated_Th ");
	pose->robot_th = atof(pose_str);
	_ftime(&recv_time);
	pose->th_recv_time = recv_time;

	if ((err = arcl_write("dsfv SecondsSinceEpoch\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: SecondsSinceEpoch ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: SecondsSinceEpoch ");
	pose->seconds_since_epoch = atof(pose_str);
	_ftime(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end lynx_dsfv_pose_encoder

 /*
 arcl_dsfv_pose_localization

 Description
 ===============================================================
 Same as arcl_dsfv_pose, but uses a different ARCL command to access an alternate pose in the data store, see arcl_dsfv_pose.
 Not currently used in performance test.

 Paramaters
 ===============================================================
 pose -- datastructure to hold the pose and timestamp that is returned.
 */
int lynx_comm_client::arcl_dsfv_pose_localization(lynx_msg_pose* pose)
{
	//double pose_buf[4];
	char* arcl_resp;
	char* pose_str;
	int err;
	_timeb recv_time;

	//cout << "Entered" << endl;

	if ((err = arcl_write("dsfv LocalizationManager_X\n") < 0))
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: LocalizationManager_X ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: LocalizationManager_X ");
	pose->robot_x = atof(pose_str);
	_ftime(&recv_time);
	pose->x_recv_time = recv_time;

	if ((err = arcl_write("dsfv LocalizationManager_Y\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: LocalizationManager_Y ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: LocalizationManager_Y ");
	pose->robot_y = atof(pose_str);
	_ftime(&recv_time);
	pose->y_recv_time = recv_time;

	if ((err = arcl_write("dsfv LocalizationManager_Th\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: LocalizationManager_Th ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: LocalizationManager_Th ");
	pose->robot_th = atof(pose_str);
	_ftime(&recv_time);
	pose->th_recv_time = recv_time;

	if ((err = arcl_write("dsfv SecondsSinceEpoch\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: SecondsSinceEpoch ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: SecondsSinceEpoch ");
	pose->seconds_since_epoch = atof(pose_str);
	_ftime(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end lynx_dsfv_pose_localization

 /*
 arcl_dsfv_pose_laser_localization

 Description
 ===============================================================
 Same as arcl_dsfv_pose, but uses a different ARCL command to access an alternate pose in the data store, see arcl_dsfv_pose.
 Not currently used in performance test.

 Paramaters
 ===============================================================
 pose -- datastructure to hold the pose and timestamp that is returned.
 */
int lynx_comm_client::arcl_dsfv_pose_laser_localization(lynx_msg_pose* pose)
{
	//double pose_buf[4];
	char* arcl_resp;
	char* pose_str;
	int err;
	_timeb recv_time;

	//cout << "Entered" << endl;

	if ((err = arcl_write("dsfv LaserLocalization_X\n") < 0))
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: LaserLocalization_X ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: LaserLocalization_X ");
	pose->robot_x = atof(pose_str);
	_ftime(&recv_time);
	pose->x_recv_time = recv_time;

	if ((err = arcl_write("dsfv LaserLocalization_Y\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: LaserLocalization_Y ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: LaserLocalization_Y ");
	pose->robot_y = atof(pose_str);
	_ftime(&recv_time);
	pose->y_recv_time = recv_time;

	if ((err = arcl_write("dsfv LaserLocalization_Th\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: LaserLocalization_Th ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: LaserLocalization_Th ");
	pose->robot_th = atof(pose_str);
	_ftime(&recv_time);
	pose->th_recv_time = recv_time;

	if ((err = arcl_write("dsfv SecondsSinceEpoch\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: SecondsSinceEpoch ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: SecondsSinceEpoch ");
	pose->seconds_since_epoch = atof(pose_str);
	_ftime(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end lynx_dsfv_pose_light_localization

 /*
 arcl_dsfv_pose_encoder

 Description
 ===============================================================
 Same as arcl_dsfv_pose, but uses a different ARCL command to access an alternate pose in the data store, see arcl_dsfv_pose.
 Not currently used in performance test.

 Paramaters
 ===============================================================
 pose -- datastructure to hold the pose and timestamp that is returned.
 */
int lynx_comm_client::arcl_dsfv_pose_light_localization(lynx_msg_pose* pose)
{
	//double pose_buf[4];
	char* arcl_resp;
	char* pose_str;
	int err;
	_timeb recv_time;

	//cout << "Entered" << endl;

	if ((err = arcl_write("dsfv LightLocalization_X\n") < 0))
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: LightLocalization_X ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: LightLocalization_X ");
	pose->robot_x = atof(pose_str);
	_ftime(&recv_time);
	pose->x_recv_time = recv_time;

	if ((err = arcl_write("dsfv LightLocalization_Y\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: LightLocalization_Y ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: LightLocalization_Y ");
	pose->robot_y = atof(pose_str);
	_ftime(&recv_time);
	pose->y_recv_time = recv_time;

	if ((err = arcl_write("dsfv LightLocalization_Th\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: LightLocalization_Th ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: LightLocalization_Th ");
	pose->robot_th = atof(pose_str);
	_ftime(&recv_time);
	pose->th_recv_time = recv_time;

	if ((err = arcl_write("dsfv SecondsSinceEpoch\n")) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	do
	{
		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if
	} while (strstr(arcl_resp, "GetDataStoreFieldValues: SecondsSinceEpoch ") == nullptr);

	pose_str = arcl_resp + strlen("GetDataStoreFieldValues: SecondsSinceEpoch ");
	pose->seconds_since_epoch = atof(pose_str);
	_ftime(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end lynx_dsfv_pose_localization

 /*
 arcl_read_status

 Description
 ===============================================================
 This function is note currently used in performance test, but formed the basis of logic for
 the similarly name function in lynx_comm.cpp.

 Repeatedly polls the ARCL server, waiting for a broadcast message that specifies the vehicle 
 has arrived at a goal point.
 */
int lynx_comm_client::arcl_read_status()
{
	char* arcl_resp;
	int err;

	do
	{

		if ((err = arcl_read(&arcl_resp)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}//end if

	} while (strstr(arcl_resp, "Arrived at ") == nullptr);

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
int lynx_comm_client::arcl_read(char** arcl_recv_buf)
{
	string arcl_recv_str="";
	char* buf = (char*) malloc(1);
	int err;

	do
	{
		if ((err = ulapi_socket_read(id, buf, 1)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
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
 arcl_write

 Description
 ===============================================================
 Sends arcl commands to the ARCL server using ULAPI sockets.

 Paramaters
 ===============================================================
 arcl_cmd -- clear text command to be sent to the ARCL server.
 */
int lynx_comm_client::arcl_write(char* arcl_cmd)
{
	int err;

	if ((err = ulapi_socket_write(id, arcl_cmd, strlen(arcl_cmd))) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	return 0;
}//end arcl_write

 /*
 print_pose

 Description
 ===============================================================
 Outputs the contents of a pose message to the console.

 Paramaters
 ===============================================================
 msg -- pose msg structure to be printed.
 */
void lynx_comm_client::print_pose(lynx_msg_pose* msg)
{
	cout << fixed << msg->seconds_since_epoch << ", " << msg->robot_x << ", " << msg->robot_y << ", " << msg->robot_th <<", "<< endl;
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
void lynx_comm_client::log_pose(lynx_msg_pose* msg)
{
	lynx_log << fixed << msg->seconds_since_epoch << ", " << msg->robot_x << ", " << msg->robot_y << ", " << msg->robot_th << ", "
		<< msg->x_recv_time.time << "." << msg->x_recv_time.millitm << ","
		<< msg->y_recv_time.time << "." << msg->y_recv_time.millitm << ","
		<< msg->th_recv_time.time << "." << msg->th_recv_time.millitm << ","
		<< msg->time_recv_time.time << "." << msg->time_recv_time.millitm << endl;
}//end log_pose


 /*
 log_comment

 Description
 ===============================================================
 Enters additional comments into the log. Useful for adding headers to csv file.

 Paramaters
 ===============================================================
 msg -- Message to be logged.
 */
void lynx_comm_client::log_comment(char* msg)
{
	lynx_log << fixed << msg << endl;
}//end log_pose


