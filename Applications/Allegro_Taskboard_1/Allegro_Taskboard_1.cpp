///////////////////////////////////////////////////////////////////////////////
//
//  Original System: ISD CRPI
//  Subsystem:       Allegro_Taskboard_1
//  Workfile:        Allegro_Taskboard_1.cpp
//  Revision:        March 15, 2018
//  Author:          K. Van Wyk
//
//  Description
//  ===========
//  KUKA, Allegro, Raspberry Pi 3 + camera, perform Task Board 1 
//	
//	
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

//Mapping from gripper to camera - Allegro
matrix allegro_T_cam(4, 4);
long long int tarMarker;
bool getMarker = false;


static unsigned int __stdcall connect_to_Pi(void * inst)
{
	//Connect to Raspberry Pi 3 camera
	ulapi_integer client;
	client = ulapi_socket_get_client_id (6000, "192.168.1.101");

    if (client < 0)
    {
      cout << "no heartbeat connection" << endl;
    }
    else
    {
      cout << "connection success for heartbeat" << endl;
    }
	int send,get;

	//string message;
	char message[5];
	strcpy(message,"beat\0");

	while (keep_running == true)
	{
		get = ulapi_socket_read(client,message,5);

		//cout << string(message) << endl;

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
	std::ifstream pos_file("..\\Applications\\Allegro_Taskboard_1\\waypoints.csv");

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

bool Cart_Move(robotPose & tarPose, CrpiRobot<CrpiKukaLWR> & arm)
{
	//Initialize AIK stuff. Initializes every time this function is called, but it's not heavy so who cares.
	KUKAIK::KUKA_AIK kuka_ik;
	double closest_angles[7] = { 0 };
	int num_valid = 0;

	//Setting TCP to be specified TCP on allegro
	kuka_ik.set_TCP_offset(-17.80,13.2878,78+48+57.86 ,-60,-90,0); //(0,0,106,0,-90,-60); //(49, 0, 154.25, 0, 0, 0);
		
	//Poll current robot joint configuration, and set for AIK. AIK will find closest joint solution, if exists.
	robotAxes curAxes;
	arm.GetRobotAxes(&curAxes);
	//kuka_ik.set_current_angles_deg(curAxes.axis[0], curAxes.axis[1], curAxes.axis[2], curAxes.axis[3], curAxes.axis[4], curAxes.axis[5], curAxes.axis[6]);
	kuka_ik.set_current_angles_deg(-90, 90, 0, 90, -90, 0, 0); //closest to nomimal joint pose for wires

	int is_valid = kuka_ik.target_Cartesian_pose(tarPose.x, tarPose.y, tarPose.z, tarPose.zrot, tarPose.yrot, tarPose.xrot);

	if (is_valid == 0) //valid solution exists, then get and move to closest solution
	{
		//Move to closest angles
		kuka_ik.get_closest_solution(closest_angles);
		robotAxes tarAxes;
		arm.GetRobotAxes(&tarAxes); //construct

		for (int j = 0; j < 7; j++) //fill with solution and move
		{
			tarAxes.axis[j] = closest_angles[j];
		}
		//cout << "moving to closest solution" << endl;
		arm.MoveToAxisTarget(tarAxes);

		return true;

	}

	else {return false;}
};

//Calculates transformation matrix given 3 translations and 3 Euler ZYX rotations
matrix Tmatrix(vector<double> pose_vec)
{
	matrix T_pose;
	T_pose.identity(4);
	double x,y,z,yaw,pitch,roll;
	double pi = 3.14159;

	x = pose_vec[0];
	y = pose_vec[1];
	z = pose_vec[2];
	yaw = pose_vec[3]*(pi/180);
	pitch = pose_vec[4]*(pi/180);
	roll = pose_vec[5]*(pi/180);

	//Rotational components using Euler ZYX
	T_pose.at(0, 0) = cos(pitch)*cos(roll); 
	T_pose.at(0, 1) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
	T_pose.at(0, 2) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);

	T_pose.at(1, 0) = cos(pitch)*sin(roll);
	T_pose.at(1, 1) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);
	T_pose.at(1, 2) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);

	T_pose.at(2, 0) = -sin(pitch);
	T_pose.at(2, 1) = cos(pitch)*sin(yaw);
	T_pose.at(2, 2) = cos(pitch)*cos(yaw);

	//Translational components
	T_pose.at(0,3) = x;
	T_pose.at(1,3) = y;
	T_pose.at(2,3) = z;

	return T_pose;

}

//Look for pose of marker and automatically return its pose as transformation
//matrix expressed in robot base coordinate system.
bool get_marker_pose(long long int marker_id,ulapi_integer & client, matrix & cam_T_tag)
{
	char tag_pose[100];
	vector <double> tag_pose_vec;
	string marker;
	int send;

	marker = to_string(marker_id);
	marker.append("\0");

	cout << "finding marker " << marker << endl;
	//send = ulapi_socket_write(client, "90\0", 3);
	send = ulapi_socket_write(client, marker.c_str(), 3);

	//Get pose
	strcpy(tag_pose,"");

	//If pose found, generate transformation matrix
	ulapi_socket_read(client, tag_pose, 100);
	cout << "MARKER POSE: " << string(tag_pose) << endl;
	//cin.get();
	if (strcmp(tag_pose,"") != 0) //means it actually got a pose
	{
		cart_pose_from_char(tag_pose,tag_pose_vec);
		cam_T_tag = Tmatrix(tag_pose_vec);

		cam_T_tag.print();

		return true;
	}
	else {cam_T_tag.identity(4); return false;}

	
};

//Calculates Cartesian pose from transformation matrix
robotPose pose_from_Tmatrix(matrix matrixPose)
{
	double pi = 3.14159;
	robotPose CartPose;
	double r10,r00,r20,r21,r22;
	r10 = matrixPose.at(1,0);
	r00 = matrixPose.at(0,0);
	r20 = matrixPose.at(2,0);
	r21 = matrixPose.at(2,1);
	r22 = matrixPose.at(2,2);

	//Translations
	CartPose.x = matrixPose.at(0,3);
	CartPose.y = matrixPose.at(1,3);
	CartPose.z = matrixPose.at(2,3);

	//Rotations
	CartPose.xrot = atan2(r21,r22)*(180/pi);
	CartPose.yrot = atan2(-r20,sqrt(pow(r00,2)+pow(r10,2)))*(180/pi);
	CartPose.zrot = atan2(r10,r00)*(180/pi);

	return CartPose;
};

//Kinematically optimizes the approach orientation of the gripper with respect to the tag.
//This is to prevent massive arm reconfigurations (protects cables) and unapproachable orientations
vector<double> optimize_gripper_tag_orientation(matrix & R_T_tag)
{
	double angular_step = 10; //step size for prospective gripper Z angle
	vector<double> tag2hover(6), best_tag2hover(6);
	double total_rotation = 0;
	robotPose evalPose;
	double min_joint_config_distance = 999999; //initialized to some high number

	tag2hover[0] = best_tag2hover[0] = 0;
	tag2hover[1] = best_tag2hover[1] = 200; //Evaluate at final tag y axis offset //500
	tag2hover[2] = best_tag2hover[2] = 0;
	tag2hover[3] = best_tag2hover[3] = 90;
	tag2hover[4] = best_tag2hover[4] = 0; //tag y-axis is pivot axis //-90
	tag2hover[5] = best_tag2hover[5] = 0;

	//Initialize AIK stuff. Initializes every time this function is called, but it's not heavy so who cares.
	KUKAIK::KUKA_AIK kuka_ik;
	double closest_angles[7] = { 0 };
	double nominal_angles[7] = {-90, 90, 0, 90, -90, 0, 0};
	int num_valid = 0;
	int is_valid = 0;

	//Setting TCP to be specified TCP on allegro
	kuka_ik.set_TCP_offset(-17.80,13.2878,78+48+57.86 ,-60,-90,0);
	//Setting nominal angles
	kuka_ik.set_current_angles_deg(-90, 90, 0, 90, -90, 0, 0); //closest to nomimal joint pose for wires

	//Spin 360 deg about tag y-axis and evaluate joint angle distance from nominal
	while ((int)total_rotation < 360)
	{
		evalPose = pose_from_Tmatrix(R_T_tag*Tmatrix(tag2hover));
		is_valid = kuka_ik.target_Cartesian_pose(evalPose.x, evalPose.y, evalPose.z, evalPose.zrot, evalPose.yrot, evalPose.xrot);

		if (is_valid == 0) //valid solution exists, then evaluate
		{
			//Get closest angles
			kuka_ik.get_closest_solution(closest_angles);
			
			//Calculate joint distance from nominal joint config
			int cur_joint_config_dist = 0;
			for (int j = 0; j < 7; j++)
			{
				cur_joint_config_dist += pow(closest_angles[j]-nominal_angles[j],2);
			}

			//If distance is current best, then save angle
			if (cur_joint_config_dist < min_joint_config_distance)
			{
				best_tag2hover[4] = tag2hover[4];
			}
		}

		total_rotation+=angular_step;
		tag2hover[4] = total_rotation;	
	}

	best_tag2hover[1] = 600;  //set good vertical distance from marker as start point

	return best_tag2hover;

};

//Refinement pose of marker once found
matrix refine_marker_pose(CrpiRobot<CrpiKukaLWR> & arm, long long int tag, ulapi_integer & client, matrix & allegro_T_cam, matrix & cam_T_tag)
{
	matrix R_T_hover, R_T_allegro, R_T_tag;
	robotPose approachPose, curPose;
	vector<double> tag2hover(6), curPose_vec(6); //pose from tag to hover point

	//Optimize approach orientation
	arm.GetRobotPose(&curPose);
	curPose_vec[0] = curPose.x;
	curPose_vec[1] = curPose.y;
	curPose_vec[2] = curPose.z;
	curPose_vec[3] = curPose.xrot;
	curPose_vec[4] = curPose.yrot;
	curPose_vec[5] = curPose.zrot;
	R_T_allegro = Tmatrix(curPose_vec);
	R_T_tag = R_T_allegro * allegro_T_cam * cam_T_tag;
	tag2hover = optimize_gripper_tag_orientation(R_T_tag);


	//Approach over pose, descend, and refine tag estimates
	for (int i=0; i<=3; i++)
	{
		//Get arm position, and calculate R_T_tag - initial location of tag in robot coordinates
		arm.GetRobotPose(&curPose);
		curPose_vec[0] = curPose.x;
		curPose_vec[1] = curPose.y;
		curPose_vec[2] = curPose.z;
		curPose_vec[3] = curPose.xrot;
		curPose_vec[4] = curPose.yrot;
		curPose_vec[5] = curPose.zrot;

		R_T_allegro = Tmatrix(curPose_vec);
		R_T_tag = R_T_allegro * allegro_T_cam * cam_T_tag;

		//Move for closer inspection of tag
		tag2hover[1]-=100; //Drop by 100 mm perpedicular to the tag
		R_T_hover = R_T_tag * Tmatrix(tag2hover);
		approachPose = pose_from_Tmatrix(R_T_hover);
		
		cout << Cart_Move(approachPose,arm) << endl;
		cout << "ENTER" << endl;
		cin.get();
		

		//if (Cart_Move(approachPose,arm) == false) //Move kinematically failed - reoptimize approach orientation from current perception of tag location
		//{
		//	double curr_approach_height = tag2hover[1];
		//	tag2hover = optimize_gripper_tag_orientation(R_T_tag);
		//	tag2hover[1] = curr_approach_height;
		//}

		 //If tag located, refine estimate of R_T_tag - location of tag in robot coordinates
		if (get_marker_pose(tag, client, cam_T_tag))
		{
			//Get arm position, and calculate R_T_tag - location of tag in robot coordinates
			arm.GetRobotPose(&curPose);
			curPose_vec[0] = curPose.x;
			curPose_vec[1] = curPose.y;
			curPose_vec[2] = curPose.z;
			curPose_vec[3] = curPose.xrot;
			curPose_vec[4] = curPose.yrot;
			curPose_vec[5] = curPose.zrot;

			R_T_allegro = Tmatrix(curPose_vec);
			R_T_tag = R_T_allegro * allegro_T_cam * cam_T_tag;
		}
	}

	return R_T_tag;
};

bool localize_board_kit(CrpiRobot<CrpiKukaLWR> & arm, ulapi_integer & client, matrix & R_T_board, matrix & R_T_kit)
{
	//Success indicators
	bool board_tag_located = false;
	bool kit_tag_located = false;
	
	//Markers
	long long int board_tag, kit_tag;
	board_tag = 70;
	kit_tag = 80;

	//Import pose search pattern for KUKA
	vector<robotPose> robot_poses;

	//Pose matrices
	//Transformation from robot to gripper
	matrix R_T_allegro(4,4);

	//Transformation from gripper to camera
	allegro_T_cam.at(0,0) = -0.9941; allegro_T_cam.at(0,1) = -0.1072; allegro_T_cam.at(0,2) = -0.0179; allegro_T_cam.at(0,3) = 18.0337;
	allegro_T_cam.at(1,0) = 0.1082;  allegro_T_cam.at(1,1) = -0.9915; allegro_T_cam.at(1,2) = -0.0724; allegro_T_cam.at(1,3) = -11.4773;
	allegro_T_cam.at(2,0) = -0.0100; allegro_T_cam.at(2,1) = -0.0739; allegro_T_cam.at(2,2) = 0.9972;  allegro_T_cam.at(2,3) = 22.4356;
	allegro_T_cam.at(3,0) = 0;       allegro_T_cam.at(3,1) = 0;       allegro_T_cam.at(3,2) = 0;       allegro_T_cam.at(3,3) = 1;

	//Transformation from cam to tag
	matrix cam_T_tag(4,4);
	
	//Read in poses
	get_poses(robot_poses);

	//Current robot pose
	robotPose curPose;
	vector<double> curPose_vec(6);

	//Engage search routine by stopping at waypoint Cartesian poses loaded from file
	//At each waypoint, query marker location. If failed, then move to next waypoint.
	//Continue until success or utter failure

	for (int i=0; i<robot_poses.size(); i++)
	{
		if (Cart_Move(robot_poses[i],arm)) //move successful, then look for markers
		{
			//cout << "ROBOT MOVED TO WAYPOINT. PRESS ENTER TO CONTINUE" << endl;
			//cin.get();

			//Locate board
			if (get_marker_pose(board_tag, client, cam_T_tag) && board_tag_located == false) //board located
			{
				cout << "Board Located!" << endl;
				board_tag_located = true;

				//Calculate and refine R_T_board - transformation from robot to board
				R_T_board = refine_marker_pose(arm,board_tag,client,allegro_T_cam,cam_T_tag);
			}

			//We know that Cartesian move is possible at this point, so we go back to it to find kit
			Cart_Move(robot_poses[i],arm);

			//Locate kit
			if (get_marker_pose(kit_tag, client, cam_T_tag) && kit_tag_located == false) //board located
			{
				cout << "Kit Located!" << endl;
				kit_tag_located = true;

				//Calculate and refine R_T_kit - transformation from robot to kit
				R_T_kit = refine_marker_pose(arm,kit_tag,client,allegro_T_cam,cam_T_tag);
				
			}

			//If both board and kit are located, then return true (success)
			if (board_tag_located == true && kit_tag_located == true) {return true;}
		}
	}

	return false; //board and kit localization failed
};

int main ()
{
	//Home Pose
	robotPose homePose;
	homePose.x = -35; homePose.y = -540; homePose.z = 700; homePose.xrot = 180; homePose.yrot = 0; homePose.zrot = -90;

	//Data writing stuff
	//ofstream data_recorder;
	//data_recorder.open("..\\Applications\\PiCam_Calibration\\kuka_joint_angles_cam_pose.csv");	
	//data_recorder << "J1, J2, J3, J4, J5, J6, J7, tag_X, tag_Y, tag_Z, tag_Xrot, tag_Yrot, tag_Zrot" << endl;

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
	
	//Establish heartbeat with Raspberry Pi 3
	bool piThreadRun = true;
	uintptr_t piThread = _beginthreadex(NULL, 0, connect_to_Pi, NULL, 0, NULL); //establishes heartbeat

	//Connect to Pi Camera for retrieving tag pose
	ulapi_integer client;
	client = ulapi_socket_get_client_id (6001, "192.168.1.101");
	
    if (client < 0)
    {
      cout << "no Pi Cam Pose connection" << endl;
    }
    else
    {
      cout << "connection success for Pi Cam Pose" << endl;
    }
	

	//Open hand and hold, move robot to homePose
	cout << "Opening hand and moving to Home Position" << endl;
	open_hand_pose.axis[0] = 0;	open_hand_pose.axis[1] = 0; open_hand_pose.axis[2] =  0; open_hand_pose.axis[3] = 0; //Index
	open_hand_pose.axis[4] = 0; open_hand_pose.axis[5] = 0; open_hand_pose.axis[6] = 0; open_hand_pose.axis[7] = 0; //Middle
	open_hand_pose.axis[9] = 0;	open_hand_pose.axis[10] = 0; open_hand_pose.axis[11] = 0; open_hand_pose.axis[12] = 0; //Little
	open_hand_pose.axis[12] = 0.3;  open_hand_pose.axis[13] = 1.0; open_hand_pose.axis[14] = 0; open_hand_pose.axis[15] = 0; //Thumb
	allegro.MoveToAxisTarget(open_hand_pose);
	
	cout << "press enter to move to home pose" << endl;
	Cart_Move(homePose,arm);
		
	//Move robot around and localize board and kit via markers
	matrix R_T_board, R_T_kit;
	localize_board_kit(arm, client, R_T_board,R_T_kit);

	R_T_board.print();
	cout << endl;
	R_T_kit.print();
	//Align gripper with some orientation with marker
	
	

	cout << "Moving back home" << endl;
	
	Cart_Move(homePose,arm);

	//Places arm in position control again
	//arm.GetRobotPose(&curPose);
	//arm.MoveTo(curPose);

	keep_running = false;

	//Close recorder
	//data_recorder.close();

	//Close heartbeat connection thread
	cout << "Closing heartbeat" << endl;
	WaitForSingleObject((HANDLE)piThread, INFINITE);
	CloseHandle((HANDLE)piThread);

	return 0;

}