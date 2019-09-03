///////////////////////////////////////////////////////////////////////////////
//
//  Original System: ISD CRPI
//  Subsystem:       PiCam_Calibration
//  Workfile:        PiCam_Calibration.cpp
//  Revision:        March 8, 2018
//  Author:          K. Van Wyk
//
//  Description
//  ===========
//  Collects data from KUKA arm and pose of markers from PiCam in order to 
//	perform extrinsic cam calibration. In particular, localizing the coordinate
//	system of the PiCam at the end of the arm
///////////////////////////////////////////////////////////////////////////////
#include "crpi_allegro.h"
#include "../../Libraries/MotionPrims/AssemblyPrims.h"
#include "CoordFrameReg.h"
#include "FT_COP.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "crpi_kuka_lwr.h"
#include "crpi_robotiq.h"
#include <vector>
#include <string>
#include "ulapi.h"
#include <algorithm>
#include <time.h>
#include <math.h>
//Multi-threading using windows processes
#include <process.h>
#include <NumericalMath.h>
#include <conio.h>
#include <process.h>
#include "kuka_IK.h"

#pragma warning (disable: 4996)

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;
using namespace Sensor;
using namespace Registration;

//Global Variables
typedef CrpiKukaLWR robType;
bool keep_running=true;

static unsigned int __stdcall connect_to_Pi_Cam(void * inst)
{
	//Connect to Raspberry Pi 3 camera
	ulapi_integer client;
	client = ulapi_socket_get_client_id (6000, "192.168.1.101");

    if (client < 0)
    {
      cout << "no connection" << endl;
    }
    else
    {
      cout << "connection success for RASPI" << endl;
    }
	int send,get;

	//string message;
	char message[5];
	strcpy(message,"beat\0");

	while (keep_running == true)
	{
		get = ulapi_socket_read(client,message,5);

		cout << string(message) << endl;

		send = ulapi_socket_write(client, "beat\0", 5);

		Sleep(1000);

	}

	return 0;
};

void cart_pose_from_char(char * input_char, vector<double> & tag_pose)
{
	//Copy over into string and replace commas with spaces
	std::string temp(input_char);
	tag_pose.resize(6);

	//Copy back over into existing char array
	//for (int i=0;i<data_size; ++i)
	//{
	//	input_char[i] = temp[i];
	//}

	//Pull out tag Cartesian pose assuming they all exist and are in order, and are in a char array with spaces separating values
	char* pEnd;
	tag_pose[0] = strtod (input_char, &pEnd); //get first angle
	
	for (unsigned short int i=1;i<=4;++i) //gets other angles
	{
		tag_pose[i] = strtod (pEnd, &pEnd);
	}

	tag_pose[5] = strtod (pEnd, NULL); //get last angle
};

//Reads Cartesian poses of robot from file
void get_poses(vector<robotPose> & robot_poses)
{
	std::ifstream pos_file("..\\Applications\\PiCam_Calibration\\robot_poses.csv");

	//Importing data
	std::string line;
	unsigned int row=0,col=0;
	double temp=0;
	robotPose temp_pose;

	while (pos_file.good())
	{
		getline(pos_file,line);
		std::stringstream lineStream(line);
		std::string cell;
		while (col < 6)
		{
			getline(lineStream,cell,',');
			std::stringstream convertor(cell);
			convertor >> temp;

			if (col == 0) {temp_pose.x = temp;}
			else if (col == 1) {temp_pose.y = temp;}
			else if (col == 2) {temp_pose.z = temp;}
			else if (col == 3) {temp_pose.xrot = temp;}
			else if (col == 4) {temp_pose.yrot = temp;}
			else if (col == 5) {temp_pose.zrot = temp;}

			col+=1;
		}

		col=0;
		temp_pose.print();
		
		robot_poses.push_back(temp_pose);
	}

	pos_file.close();
};

int main ()
{
	//Data writing stuff
	ofstream data_recorder;
	data_recorder.open("..\\Applications\\PiCam_Calibration\\kuka_joint_angles_cam_pose.csv");	
	data_recorder << "J1, J2, J3, J4, J5, J6, J7, tag_X, tag_Y, tag_Z, tag_Xrot, tag_Yrot, tag_Zrot" << endl;

	//-----------------------------
	int i = 0;
	crpi_timer timer;
	double time_prev = 0;

	
	//Setting up KUKA arm
	cout << "Connecting to arm" << endl;
	CrpiRobot<CrpiKukaLWR> arm("..\\Applications\\PiCam_Calibration\\kuka_lwr.xml");
	arm.SetAngleUnits("degree");
	arm.SetLengthUnits("mm");
	arm.Couple("allegro");  // doesn't matter what you put here because we are just going to record joint angles
	Sleep(1000);

	//Setting up Allegro hand
	cout << "Connecting to hand" << endl;
	CrpiRobot<CrpiAllegro> allegro("..\\Applications\\PiCam_Calibration\\allegro.xml");
	allegro.SetParameter("gravity_vector","0 0 9.81");
	allegro.SetAbsoluteSpeed(90);
	allegro.SetParameter("touch_stop","off");
	robotAxes open_hand_pose;
	
	//Connect to Raspberry Pi 3 camera
	bool piThreadRun = true;
	uintptr_t piThread = _beginthreadex(NULL, 0, connect_to_Pi_Cam, NULL, 0, NULL); //establishes heartbeat

	//connect another Pi socket for retrieving tag pose
	ulapi_integer client_tag;
	client_tag = ulapi_socket_get_client_id (6001, "192.168.1.101");
    if (client_tag < 0)
    {
      cout << "no connection" << endl;
    }
    else
    {
      cout << "connection success for RASPI" << endl;
    }
	int send,get;

	//AIK stuff
	KUKAIK::KUKA_AIK kuka_ik;
	double closest_angles[7] = { 0 };
	double all_angles[8][7] = { 0 };
	int num_valid = 0;
	kuka_ik.set_TCP_offset(0,0,106,0,-90,-60); //(49, 0, 154.25, 0, 0, 0);

	
	//Open hand and hold
	open_hand_pose.axis[0] = 0;	open_hand_pose.axis[1] = 0; open_hand_pose.axis[2] =  0; open_hand_pose.axis[3] = 0; //Index
	open_hand_pose.axis[4] = 0; open_hand_pose.axis[5] = 0; open_hand_pose.axis[6] = 0; open_hand_pose.axis[7] = 0; //Middle
	open_hand_pose.axis[9] = 0;	open_hand_pose.axis[10] = 0; open_hand_pose.axis[11] = 0; open_hand_pose.axis[12] = 0; //Little
	open_hand_pose.axis[12] = 1.0;  open_hand_pose.axis[13] = 1.0; open_hand_pose.axis[14] = 0; open_hand_pose.axis[15] = 0; //Thumb

	allegro.MoveToAxisTarget(open_hand_pose);
	
		
	//Place robot in impedance control with zero stiffness a manually move around and acquire robot joint and tag pose data

	cout << "Press ENTER to continue" << endl;
	cin.get();
	robotPose curPose;
	vector<robotPose> robot_poses;
	robotAxes curAxes, tarAxes;
	//arm.GetRobotPose(&curPose);

	//Read in poses
	get_poses(robot_poses);
	//robot_poses[0].x = 0; robot_poses[0].y = -560; robot_poses[0].z = 500; robot_poses[0].xrot = 180; robot_poses[0].yrot = 0; robot_poses[0].zrot = -90;

	//arm.MoveTo(curPose);

	arm.GetRobotAxes(&curAxes);
	kuka_ik.set_current_angles_deg(curAxes.axis[0], curAxes.axis[1], curAxes.axis[2], curAxes.axis[3], curAxes.axis[4], curAxes.axis[5], curAxes.axis[6]);

	int is_valid;
	
	int key;
	char tag_pose[100];
	//robotAxes curAxes;
	vector<double> tag_pose_vec;

	cout << "Press space to grab data for marker 90" << endl;
	while (1)
	{
		if (kbhit())
		{
			key = getch();
			
			if(key == 27) //if ESC, then quit
			{
				break;
			}

			else if(key == 13) //ENTER
			{
				send = ulapi_socket_write(client_tag, "90\0", 3);

				//Get pose
				strcpy(tag_pose,"");
				while (ulapi_socket_read(client_tag, tag_pose, 100) <= 0) {Sleep(100);}

				//Print pose
				cout << string(tag_pose) << endl;
			}

			else if(key == 32) //SPACE, then record data
			{
				
				for (int i=0; i<robot_poses.size(); i++)
				{
					//Go to robot pose
					is_valid = kuka_ik.target_Cartesian_pose(robot_poses[i].x, robot_poses[i].y, robot_poses[i].z, robot_poses[i].zrot, robot_poses[i].yrot, robot_poses[i].xrot);
					cout << "Does valid solution exist? 0 indicates yes: " << is_valid << endl;

					if (is_valid == 0)
					{
						//recorder << i << ", ";
						//move_to_all_solutions(arm, closest_angles, 1); //only one solution for closest solution
		
						num_valid = kuka_ik.get_all_solutions(*all_angles);

						cout << "Number valid solutions:" << num_valid << endl;

						cout << "Press Enter to start motion or calculate next set of solutions." << endl;
						//cin.get();

						if (num_valid > 0)
						{
							//Move to closest angles first to keep reflector in sight

							kuka_ik.get_closest_solution(closest_angles);
							arm.GetRobotAxes(&tarAxes);
							//cout << closest_angles.size() << endl;
							for (int j = 0; j < 7; j++)
							{
								tarAxes.axis[j] = closest_angles[j];
							}
							cout << "moving to closest solution" << endl;
							arm.MoveToAxisTarget(tarAxes);
						}
				

						//Tell Pi which marker you want a pose for
						if (key == 13) {send = ulapi_socket_write(client_tag, "1\0", 3);}
						else {send = ulapi_socket_write(client_tag, "90\0", 3);}

						//Get pose
						strcpy(tag_pose,"");
						while (ulapi_socket_read(client_tag, tag_pose, 100) <= 0) {Sleep(100);}

						//Print pose
						cout << string(tag_pose) << endl;

						//Store pose in vector
						cart_pose_from_char(tag_pose,tag_pose_vec);

						//Quickly grab robot joint pose now that tag is localized
						arm.GetRobotAxes(&curAxes);

						//Save data
						for (int i=0;i<7;i++)
						{

							data_recorder << curAxes.axis[i] << ", ";
						}

						for (int i=0;i<5;i++)
						{

							data_recorder << tag_pose_vec[i] << ", ";
						}
						data_recorder << tag_pose_vec[5] << endl;
					}
				}
			}
			else //anything else, send kill command
			{
				//Tell Pi which marker you want a pose for
				send = ulapi_socket_write(client_tag, "-1\0", 3);
			}
			
			//strcpy(message2,"beat\0");
		}
	}

	//Places arm in position control again
	arm.GetRobotPose(&curPose);
	arm.MoveTo(curPose);

	keep_running = false;

	//Close recorder
	data_recorder.close();

	//Close heartbeat connection thread
	cout << "Closing heartbeat" << endl;
	WaitForSingleObject((HANDLE)piThread, INFINITE);
	CloseHandle((HANDLE)piThread);

	return 0;

}