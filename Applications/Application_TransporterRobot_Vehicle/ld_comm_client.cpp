/**
*\file ld_comm_client.cpp

*\brief Interface for managing ARCL connections to the Omron Adept ld mobile robot platform. \n
*This is the standard client / server connection where the ld core functions as the server and \n
*This control program connects as a client. \n

*Note that ARCL commands must be terminated by a newline ("\n") character when sent to the ARCL server. \n
*See ARCL user guide from Adept or use the 'help' command when connecting via Telnet client to the ld \n
*to see a list of available commands. \n

*The ARCL Server Connection parameters can be configured via Mobile Planner under Configuration -> Robot Interface -> ARCL Server Setup \n

*References: \n

*Based on: agv_comm.h by J. Marvel, S. Legowik \n

*"_ftime, _ftime32, _ftime64 from MSDN" https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/ftime-ftime32-ftime64

*\author Omar Aboul-Enein
*\date 2018-06-05

*/

#include "ld_comm_client.h"
#include "ulapi.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

/**
*Helper function for printing communication errors.
*\param[in] err Error code returned by ulapi socket
*\param[in] msg Error message the user desires to print.
*/
void comm_err_client(int err, char* msg)
{
	cout << "Comm Error " << err << " " << msg << endl;
}//end comm_err_client

ld_comm_client::ld_comm_client(char* arcl_addr, ulapi_integer arcl_port, char* arcl_passwd, bool log)
{

	if (log == TRUE)
	{
		ld_log.open("ld_log.csv");
	}//end if
	
	char* temp_passwd;

	ip_addr = (char*) malloc(sizeof(char)*(strlen(arcl_addr)+1));
	strcpy_s(ip_addr, sizeof(char)*(strlen(arcl_addr)+1), arcl_addr);

	port = arcl_port;

	
	temp_passwd = (char*)malloc(sizeof(char)*(strlen(arcl_passwd) + 2));
	strcpy_s(temp_passwd, sizeof(char)*(strlen(arcl_passwd) + 2), arcl_passwd);
	strcat_s(temp_passwd, sizeof(char)*(strlen(arcl_passwd) + 2), "\n");

	passwd = (char*)malloc(sizeof(char)*(strlen(arcl_passwd) + 2));

	passwd = temp_passwd;

}//end constructor

ld_comm_client::~ld_comm_client()
{
	int err;
	ld_log.close();
	if ((err = ulapi_socket_close(id)) < 0)
		comm_err_client(err, "socket close failure");

}//end destructor

int ld_comm_client::ld_connect()
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

	//cout << "FINISHED READING ARCL TEXT" << endl;

	free(arcl_recv_buf);

	//Send the password to the ld ARCL server
	if ((err = ulapi_socket_write(id, passwd, strlen(passwd))) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	//cout << "FINISHED WRITING PASSWORD" << endl;

	arcl_recv_buf = (char*)malloc(2);
	arcl_recv_buf[1] = '\0';

	//Parse the response from the ld. The ARCL server automatically
	//executes the 'help' command, which outputs all commands that can be executed
	//on the server. This response is accepted and then discarded.
	//NOTE THAT THE LENGTH OF THE HELP TEXT MAY VARY BETWEEN VEHICLES WITH DIFFERENT
	//ARCL VERSIONS! THIS MAY CAUSE PROCESS TO BLOCK IF NUMBER IN FOR LOOP IS WRONG!
	for(int i = 0; i<7416; i++)
	{
		//cout << i<<": "<<arcl_recv_buf << endl;
		if ((err = ulapi_socket_read(id, arcl_recv_buf, 1)) < 0)
		{
			comm_err_client(err, "ulapi socket read failure");
			return err;
		}
	}//end while

	//cout << "FINISHED READING HELP TEXT" << endl;

	return 0;
}//end ld_connect

int ld_comm_client::arcl_dsfv_pose(ld_msg_pose* pose)
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;


}//end ld_dsfv_pose

int ld_comm_client::arcl_dsfv_pose_encoder(ld_msg_pose* pose)
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;


}//end ld_dsfv_pose_encoder

int ld_comm_client::arcl_dsfv_pose_interpolated(ld_msg_pose* pose)
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end ld_dsfv_pose_encoder

int ld_comm_client::arcl_dsfv_pose_encoder_interpolated(ld_msg_pose* pose)
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end ld_dsfv_pose_encoder

int ld_comm_client::arcl_dsfv_pose_localization(ld_msg_pose* pose)
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end ld_dsfv_pose_localization


int ld_comm_client::arcl_dsfv_pose_laser_localization(ld_msg_pose* pose)
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end ld_dsfv_pose_light_localization

int ld_comm_client::arcl_dsfv_pose_light_localization(ld_msg_pose* pose)
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
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
	_ftime_s(&recv_time);
	pose->time_recv_time = recv_time;

	return 0;

}//end ld_dsfv_pose_localization

int ld_comm_client::arcl_read_status()
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

int ld_comm_client::arcl_read(char** arcl_recv_buf)
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


	*arcl_recv_buf= (char*)malloc(sizeof(char)*(arcl_recv_str.length()+1));
	strcpy_s(*arcl_recv_buf, sizeof(char)*(arcl_recv_str.length()+1), arcl_recv_str.c_str());

	free(buf);

	return 0;

}//end arcl_read

int ld_comm_client::arcl_write(char* arcl_cmd)
{
	int err;

	if ((err = ulapi_socket_write(id, arcl_cmd, strlen(arcl_cmd))) < 0)
	{
		comm_err_client(err, "ulapi socket write failure");
		return err;
	}//end if

	return 0;
}//end arcl_write

void ld_comm_client::print_pose(ld_msg_pose* msg)
{
	cout << fixed << msg->seconds_since_epoch << ", " << msg->robot_x << ", " << msg->robot_y << ", " << msg->robot_th <<", "<< endl;
}//end print_pose

void ld_comm_client::log_pose(ld_msg_pose* msg)
{
	ld_log << fixed << msg->seconds_since_epoch << ", " << msg->robot_x << ", " << msg->robot_y << ", " << msg->robot_th << ", "
		<< msg->x_recv_time.time << "." << msg->x_recv_time.millitm << ","
		<< msg->y_recv_time.time << "." << msg->y_recv_time.millitm << ","
		<< msg->th_recv_time.time << "." << msg->th_recv_time.millitm << ","
		<< msg->time_recv_time.time << "." << msg->time_recv_time.millitm << endl;
}//end log_pose

void ld_comm_client::log_comment(char* msg)
{
	ld_log << fixed << msg << endl;
}//end log_pose


