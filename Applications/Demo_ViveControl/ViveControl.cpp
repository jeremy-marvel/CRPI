///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Virtual Reality Control for Dual Armed Robot
//  Subsystem:       Hand Control Demonstration
//  Workfile:        ViveControl.cpp
//  Revision:        13 January, 2017
//  Author:          M. Zimmerman
//
//  Description
//  ===========
//
//   Hand control program using Lighthouse tracking, steam VR must be
//	 running concurently. 
//
///////////////////////////////////////////////////////////////////////////////

//CRPI includes
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <time.h>
#include <ctime>
#include "crpi_robot.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "crpi_abb.h"
#include "ulapi.h"
#include "NumericalMath.h" 
#include "MatrixMath.h"
#include "../../Libraries/MotionPrims/AssemblyPrims.h"
#include "Vicon.h"
#include "OptiTrack.h"
#include "MoCapTypes.h"
#include "../../Libraries/RegistrationKit/CoordFrameReg.h"
#include "ATI_Wired.h"
#include "FT_COP.h"
#include "LeapMotion.h"
#include "MYO.h"

//VR Includes
#include "stdincludes.h"
#include "LighthouseTracking.h"
#include <conio.h>


#pragma warning (disable: 4996)

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;
using namespace Registration;

string G_POSE = "";
string G_TIME = "";

vector<double> ParseTrackingFrame1(LighthouseTracking *lighthouseTracking, vector<double>& right, vector<double>& left);
//toTokens
//Preconditions: Hashed splays object must have been created, given valid line and stream
//Postconditions: breaks the string into a vector by whitespace
vector<string> toTokens(vector<string>* line, string buffer, stringstream& stream)
{
	if (stream >> buffer)
	{
		line->push_back(buffer);
		toTokens(line, buffer, stream);
	}

	return *line;
}

int mainTran() 
{

	CrpiRobot<CrpiAbb> armR("abb_irb14000_right.xml");
	armR.SetAngleUnits("degree");
	armR.SetLengthUnits("mm");
	cout << "Connected" << endl;

	//! Calibration Variables
	vector<robotPose> pur;
	vector<point> world, rob;
	point wldpt[4], robpt[4], wldpt1[4], robpt1[4], wldpt2[4], robpt2[4], wldpt3[4], robpt3[4];
	//vector<point[4]> worldpts, robpts;
	vector<point> v_wldpt, v_robpt;
	//vector<matrix> transmats;
	matrix W_T_R(4,4); 
	matrix W_T_Ri(4,4);
	matrix W_T_Rx(4, 4);
	matrix W_T_Ry(4, 4);
	matrix W_T_Rz(4, 4);


	string val;
	robotPose w_pose, r_pose;
	vector<string> tokens;

	ifstream rfile("yumi_pose.txt");
	if (rfile.is_open())
	{
		int i = 0;
		int j = 0;
		while (getline(rfile, val))
		{
			stringstream stream(val);
			string buffer;
			vector<string> tokens;
			tokens = toTokens(&tokens, buffer, stream);
			point p;

			p.x = stod(tokens[0]);
			p.y = stod(tokens[1]);
			p.z = stod(tokens[2]);
			
			v_robpt.push_back(p);

			//output for sanity check
			cout << v_robpt[i].x << " ";
			cout << v_robpt[i].y << " ";
			cout << v_robpt[i].z << endl;

			i++;

			
		}
		cout << i << endl;
	}
	rfile.close();

	ifstream vfile("v_pose.txt");
	if (vfile.is_open())
	{
		int i = 0;
		int j = 0;
		while (getline(vfile, val))
		{
			stringstream stream(val);
			string buffer;
			vector<string> tokens;
			tokens = toTokens(&tokens, buffer, stream);
			point p;
			// RX = VZ, RY= VX, RZ = VY
			p.x = stod(tokens[2]) * 1000.0;
			p.y = stod(tokens[0]) * 1000.0;
			p.z = stod(tokens[1]) * 1000.0;
			v_wldpt.push_back(p);

			//output for sanity check
			cout << v_wldpt[i].x << " ";
			cout << v_wldpt[i].y << " ";
			cout << v_wldpt[i].z << endl;

			i++;
		}
		cout << i << endl;
	}
	cout << "Len wldpt" << size(wldpt) << endl;
	vfile.close();
	cout << "v do" << endl;
	std::ofstream outstream;
	std::ofstream outstream1;
	matrix avg(4, 4);

	avg.at(0,0) = 0; avg.at(0, 1) = 0; avg.at(0, 2) = 0; avg.at(0, 3) = 0; avg.at(1, 0) = 0; avg.at(1, 1) = 0; avg.at(1, 2) = 0; avg.at(1, 3) = 0; avg.at(2, 0) = 0; avg.at(2, 1) = 0; avg.at(2, 2) = 0; avg.at(2, 3) = 0; avg.at(3, 0) = 0; avg.at(3, 1) = 0; avg.at(3, 2) = 0; avg.at(3, 3) = 0;

	//read set permutations file
	ifstream sfile("sets.txt");
	if (sfile.is_open())
	{
		int i = 0;
		int p1,p2,p3;
		while (getline(sfile, val))
		{
			outstream.open("trmat.csv", std::ios::app);

			stringstream stream(val);
			string buffer;
			vector<string> tokens;
			tokens = toTokens(&tokens, buffer, stream);
			p1 = stoi(tokens[0]);
			p2 = stoi(tokens[1]);
			p3 = stoi(tokens[2]);
			/*
			cout << i+1 << "th itteration --------------------------------" << endl;
			cout << "possition Numbers:  " << p1 << " " << p2 << " " << p3 << endl;
			cout << "Init R1 Val " << v_robpt[p1].x << ", " << v_robpt[p1].y << ", " << v_robpt[p1].z << endl;
			cout << "Init V1 Val " << v_wldpt[p1].x << ", " << v_wldpt[p1].y << ", " << v_wldpt[p1].z << endl;
			*/
			world.clear();
			world.push_back(v_wldpt[p1]);
			world.push_back(v_wldpt[p2]);
			world.push_back(v_wldpt[p3]);

			w_pose.x = v_wldpt[p1].x;
			w_pose.y = v_wldpt[p1].y;
			w_pose.z = v_wldpt[p1].z;

			rob.clear();
			rob.push_back(v_robpt[p1]);
			rob.push_back(v_robpt[p2]);
			rob.push_back(v_robpt[p3]);

			Registration::reg2target(world, rob, W_T_R);

			armR.UpdateWorldTransform(W_T_R.inv());

			W_T_Ri = W_T_R.inv();
			cout << "Tansform Matrix : " << endl;
			W_T_Ri.print();

			cout << "Translation Results on possition numbers" << endl;
			armR.FromWorld(&w_pose, &r_pose);
			cout << w_pose.x << ", " << w_pose.y << ", " << w_pose.z << endl;
			cout << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;
			cout << " " << endl;

			avg.at(0, 0) += W_T_Ri.at(0, 0); avg.at(0, 1) += W_T_Ri.at(0, 1); avg.at(0, 2) += W_T_Ri.at(0, 2); avg.at(0, 3) += W_T_Ri.at(0, 3);
			avg.at(1, 0) += W_T_Ri.at(1, 0); avg.at(1, 1) += W_T_Ri.at(1, 1); avg.at(1, 2) += W_T_Ri.at(1, 2); avg.at(1, 3) += W_T_Ri.at(1, 3);
			avg.at(2, 0) += W_T_Ri.at(2, 0); avg.at(2, 1) += W_T_Ri.at(2, 1); avg.at(2, 2) += W_T_Ri.at(2, 2); avg.at(2, 3) += W_T_Ri.at(2, 3);
			avg.at(3, 0) += W_T_Ri.at(3, 0); avg.at(3, 1) += W_T_Ri.at(3, 1); avg.at(3, 2) += W_T_Ri.at(3, 2); avg.at(3, 3) += W_T_Ri.at(3, 3);

			//saving best matrix for x position
			if (i == 119)
			{
				W_T_Rx = W_T_Ri;
			}

			//saving best matrix for y position
			if (i == 45)
			{
				W_T_Rz = W_T_Ri;
			}

			//saving best matrix for z position
			if (i == 406)
			{
				W_T_Rz = W_T_Ri;
			}

			/*
			for (int i = 0; i < rows; ++i)
			{
			printf ("| ");
			for (int j = 0; j < cols; ++j)
			{
			printf ("%f ", at(i, j));
			}
			printf ("|\n");
			}
			}
			*/

			outstream1.open("trresults.csv", std::ios::app);
			outstream1 << p1 << "," << p2 << "," << p3 << "," << v_robpt[p1].x << "," << v_robpt[p1].y << "," << v_robpt[p1].z << "," << v_wldpt[p1].x << ", " << v_wldpt[p1].y << ", " << v_wldpt[p1].z << "," << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;
			outstream1.close();

			outstream.open("trmat.csv", std::ios::app);
			outstream << W_T_R.at(0, 0) << "," << W_T_R.at(0, 1) << "," << W_T_R.at(0, 2) << "," << W_T_R.at(0, 3) << "," << W_T_R.at(1, 0) << "," << W_T_R.at(1, 1) << "," << W_T_R.at(1, 2) << "," << W_T_R.at(1, 3) << "," << W_T_R.at(2, 0) << "," << W_T_R.at(2, 1) << "," << W_T_R.at(2, 2) << "," << W_T_R.at(2, 3) << "," << W_T_R.at(3, 0) << "," << W_T_R.at(3, 1) << "," << W_T_R.at(3, 2) << "," << W_T_R.at(3, 3) << endl;
			outstream.close();

			i++;
		}
		cout << i << " translation interations" << endl;
	}
	sfile.close();

	outstream.open("xeval2.csv", std::ios::app);
	//x evaluation
	for (int i = 0; i < size(v_wldpt); i++) 
	{
		armR.UpdateWorldTransform(W_T_Rx);
		w_pose.x = v_wldpt[i].x;
		w_pose.y = v_wldpt[i].y;
		w_pose.z = v_wldpt[i].z;

		armR.FromWorld(&w_pose, &r_pose);

		cout << "Translation Results on possition numbers" << endl;
		cout << w_pose.x << ", " << w_pose.y << ", " << w_pose.z << endl;
		cout << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;
		cout << " " << endl;

		outstream << i << "," << v_robpt[i].x << "," << v_robpt[i].y << "," << v_robpt[i].z << "," << v_wldpt[i].x << ", " << v_wldpt[i].y << ", " << v_wldpt[i].z << "," << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;


	}
	outstream.close();
	cout << "x done" << endl;
	outstream.open("yeval2.csv", std::ios::app);
	//y evaluation
	for (int i = 0; i < size(v_wldpt); i++)
	{
		armR.UpdateWorldTransform(W_T_Ry);
		w_pose.x = v_wldpt[i].x;
		w_pose.y = v_wldpt[i].y;
		w_pose.z = v_wldpt[i].z;

		armR.FromWorld(&w_pose, &r_pose);

		cout << "Translation Results on possition numbers" << endl;
		cout << w_pose.x << ", " << w_pose.y << ", " << w_pose.z << endl;
		cout << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;
		cout << " " << endl;

		outstream << i << "," << v_robpt[i].x << "," << v_robpt[i].y << "," << v_robpt[i].z << "," << v_wldpt[i].x << ", " << v_wldpt[i].y << ", " << v_wldpt[i].z << "," << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;


	}
	cout << "y done" << endl;
	outstream.close();
	outstream.open("zeval2.csv", std::ios::app);
	//z evaluation
	for (int i = 0; i < size(v_wldpt); i++)
	{
		armR.UpdateWorldTransform(W_T_Rx);
		w_pose.x = v_wldpt[i].x;
		w_pose.y = v_wldpt[i].y;
		w_pose.z = v_wldpt[i].z;

		armR.FromWorld(&w_pose, &r_pose);

		cout << "Translation Results on possition numbers" << endl;
		cout << w_pose.x << ", " << w_pose.y << ", " << w_pose.z << endl;
		cout << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;
		cout << " " << endl;

		outstream << i << "," << v_robpt[i].x << "," << v_robpt[i].y << "," << v_robpt[i].z << "," << v_wldpt[i].x << ", " << v_wldpt[i].y << ", " << v_wldpt[i].z << "," << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;


	}
	outstream.close();
	cout << "z done" << endl;

	// calc avgs
	avg.at(0, 0) = avg.at(0, 0) / 560.0; avg.at(0, 1) = avg.at(0, 1) / 560.0; avg.at(0, 2) = avg.at(0, 2) / 560.0; avg.at(0, 3) = avg.at(0, 3) / 560.0;
	avg.at(1, 0) = avg.at(1, 0) / 560.0; avg.at(1, 1) = avg.at(1, 1) / 560.0; avg.at(1, 2) = avg.at(1, 2) / 560.0; avg.at(1, 3) = avg.at(1, 3) / 560.0;
	avg.at(2, 0) = avg.at(2, 0) / 560.0; avg.at(2, 1) = avg.at(2, 1) / 560.0; avg.at(2, 2) = avg.at(2, 2) / 560.0; avg.at(2, 3) = avg.at(2, 3) / 560.0;
	avg.at(3, 0) = avg.at(3, 0) / 560.0; avg.at(3, 1) = avg.at(3, 1) / 560.0; avg.at(3, 2) = avg.at(3, 2) / 560.0; avg.at(3, 3) = avg.at(3, 3) / 560.0;

	outstream.open("avgeval.csv", std::ios::app);
	//avg evaluation
	for (int i = 0; i < size(v_wldpt); i++)
	{
		armR.UpdateWorldTransform(avg);
		w_pose.x = v_wldpt[i].x;
		w_pose.y = v_wldpt[i].y;
		w_pose.z = v_wldpt[i].z;

		armR.FromWorld(&w_pose, &r_pose);

		cout << "Translation Results on possition numbers" << endl;
		cout << w_pose.x << ", " << w_pose.y << ", " << w_pose.z << endl;
		cout << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;
		cout << " " << endl;

		outstream << i << "," << v_robpt[i].x << "," << v_robpt[i].y << "," << v_robpt[i].z << "," << v_wldpt[i].x << ", " << v_wldpt[i].y << ", " << v_wldpt[i].z << "," << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;


	}
	outstream.close();
	cout << "avg done" << endl;
	/*
	//Save coordinates of three points measured in both world and robot to a vector
	world.clear();
	world.push_back(v_wldpt[0]);
	world.push_back(v_wldpt[1]);
	world.push_back(v_wldpt[2]);

	w_pose.x = v_wldpt[0].x;
	w_pose.y = v_wldpt[0].y;
	w_pose.z = v_wldpt[0].z;

	rob.clear();
	rob.push_back(v_robpt[0]);
	rob.push_back(v_robpt[1]);
	rob.push_back(v_robpt[2]);

	Registration::reg2target(world, rob, W_T_R);


	armR.UpdateWorldTransform(W_T_R.inv());
	W_T_Ri = W_T_R.inv();

	cout << "Tansform Matrix : " << endl;
	W_T_Ri.print();

	armR.FromWorld(&w_pose, &r_pose);
	cout << w_pose.x << ", " << w_pose.y << ", " << w_pose.z << endl;
	cout << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;

	string yvfile = "yumi_vive_right1.xml";
	armR.SaveConfig(yvfile.c_str());
	cout << "done" << endl;
	*/
	//Convert poses from "world" to robot coordinate system
	//armR.FromWorld(&w_pose, &r_pose);
	//cout << w_pose.x << ", " << w_pose.y << ", " << w_pose.z << endl;
	//cout << r_pose.x << ", " << r_pose.y << ", " << r_pose.z << endl;
	//armR.MoveStraightTo(r_pose);
	return 0;
}

int main() 
{
	//If false we'll parse tracking data continously, if true we parse when an openvr event fires
	bool wait_ev = false;

	LighthouseTracking *lighthouseTracking = new LighthouseTracking();
	if (lighthouseTracking) {


		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Starting tracking, Press 'q' to quit\n");
		printf_s(buf);

		//2 second buffer
		Sleep(2000);

		while (lighthouseTracking->RunProcedure(wait_ev)) {

			// Windows quit routine - adapt as you need
			if (_kbhit()) {
				char ch = _getch();
				if ('q' == ch) {

					char buf[1024];
					sprintf_s(buf, sizeof(buf), "User pressed 'q' - exiting...");
					printf_s(buf);

					break;
				}
			}
			Sleep(1);
		}

		delete lighthouseTracking;
	}
	return 0;
}


int main1()
{


	CrpiRobot<CrpiAbb> armL("abb_irb14000_left.xml");
	armL.SetAngleUnits("degree");
	armL.SetLengthUnits("mm");
	CrpiRobot<CrpiAbb> armR("abb_irb14000_right.xml");
	armR.SetAngleUnits("degree");
	armR.SetLengthUnits("mm");
	armL.Couple("Yumi_Parallel");
	armR.Couple("Yumi_Parallel");

	cout << "Arms Connected and Initialized" << endl;

	robotPose poseMe, curPose, curPose_r, poseR, poseL;
	robotAxes curAxes, curAxesR, tarAxes, tarAxesR;

	//center robot L
	tarAxes.axis[0] = -128.393;	tarAxes.axis[1] = -126.96;	tarAxes.axis[2] = 21.8281;	tarAxes.axis[3] = 24.3266;
	tarAxes.axis[4] = 182.648;	tarAxes.axis[5] = -71.2129;	tarAxes.axis[6] = -26.6042;

	//center robot R
	tarAxesR.axis[0] = 120.111;	tarAxesR.axis[1] = -124.582; tarAxesR.axis[2] = -16.7288; tarAxesR.axis[3] = 23.7964;
	tarAxesR.axis[4] = -175.136; tarAxesR.axis[5] = -58.5175; tarAxesR.axis[6] = 105.083;
	
	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}

	char buf[1024];
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	bool running = true; 
	vr::VREvent_t event;
	LighthouseTracking *lighthouseTracking = new LighthouseTracking();

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str(buffer);
	std::cout << "Centered at " << str << endl;
	
	vector<double>right;
	vector<double>left;
	double rx, ry, rz, lx, ly, lz = 0;
	while (running == true) 
	{
		//get current event
		while (lighthouseTracking->m_pHMD->PollNextEvent(&event, sizeof(event))) 
		{
			//attempt to process
			if (!lighthouseTracking->ProcessVREvent(event)) 
			{
				sprintf_s(buf, sizeof(buf), "(OpenVR) service quit\n");
				printf_s(buf);
				running = false;
				break;
			}

			ParseTrackingFrame1(lighthouseTracking, right, left);
			/*
			if (right.size >= 2) 
			{
				rx = (right[2] * 1600) - 60;
				ry = right[0] * -204;
				rz = (((right[1] - 1) * 100) *7.235) - 340;
			}
			if (left.size >= 2) 
			{
				lx = (left[2] * 1600) - 60;
				ly = left[0] * -204;
				lz = (((left[1] - 1) * 100) *7.235) - 340;
			}
			*/
			//validate within bounds

			//file out the time

			//move left
			if (armL.MoveStraightTo(poseL) != CANON_SUCCESS)
			{
				std::cout << "motion error" << endl;
			}
			//move right
			if (armR.MoveStraightTo(poseR) != CANON_SUCCESS)
			{
				std::cout << "motion error" << endl;
			}


		}

	}
	return 0;
}



//! @brief Convert possitional data from vive coordinate system to robot coordinate system
//!
//! @param Vector of possition data as floats vx,vy,vz,vqw,vqx,vqy,vqz
//!
//! @return Vector of possition data as doubles as x,y,z,rx,ry,rz
//!
vector<double> translateCoordinates(vector<double> incoord);
vector<double> translateCoordinates(vector<double> incoord) 
{
	vector<double> coordinates;
	double x, y, z, rx, ry, rz;
	
	//cartesian coordinate translation
	if (incoord.size() >= 3) 
	{
		x = (incoord[2] *1600) - 60;
		y = incoord[0] * -204;
		z = (((incoord[1] - 1) * 100) *7.235) - 340;
		coordinates.push_back(x);
		coordinates.push_back(y);
		coordinates.push_back(z);
	}

	//case for if rotational data is present
	if (incoord.size() == 7) 
	{
		vector<double> quaternion;
		double w, wx, wy, wz;
		w = incoord[3];
		wx = incoord[6];
		wy = incoord[4];
		wz = incoord[5];
		quaternion.push_back(w);
		quaternion.push_back(wx);
		quaternion.push_back(wy);
		quaternion.push_back(wz);

		Math::matrix rot(3, 3);
		
		rot.rotQuaternionMatrixConvert(quaternion);
		rot.print();
		
		rx = rot.at(0, 1);
		ry = rot.at(1, 0);
		ry = rot.at(2, 0);

		coordinates.push_back(rx);
		coordinates.push_back(ry);
		coordinates.push_back(rz);

	}
	return coordinates;
}

//Just a print function
void printCoord(vector<double> coord) 
{
	std::cout << " x: " << coord[0] << ", y: " << coord[1] << ", z: " << coord[2] << endl;
}

//! @brief Process current tracking frame, returns current cartesian points for the right and left controllers
//!
//! @param LighthouseTracking frame, vectors for both right and left arms 
//!
//! @return Coordinate values for the right and left arms
//!

vector<double> ParseTrackingFrame1(LighthouseTracking *lighthouseTracking, vector<double>& right, vector<double>& left) 
{

	vector<double> possitiondata;

	// Process SteamVR device states
	for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
	{
		if (!lighthouseTracking->m_pHMD->IsTrackedDeviceConnected(unDevice))
			continue;

		vr::VRControllerState_t state;


		//For device
		vr::TrackedDevicePose_t trackedDevicePose;
		vr::TrackedDevicePose_t *devicePose = &trackedDevicePose;

		//For controller, of same type of device
		vr::TrackedDevicePose_t trackedControllerPose;
		vr::TrackedDevicePose_t *controllerPose = &trackedControllerPose;

		//Controllerstate type holds a struct for button pressed and touched, axis and last packet
		vr::VRControllerState_t controllerState;
		vr::VRControllerState_t *ontrollerState_ptr = &controllerState;

		vr::HmdVector3_t vector;
		vr::HmdQuaternion_t quaternion;

		if (vr::VRSystem()->IsInputFocusCapturedByAnotherProcess()) {
			char buf[1024];

			sprintf_s(buf, sizeof(buf), "\nInput Focus by Another Process\n");
			printf_s(buf);
		}

		bool bPoseValid = trackedDevicePose.bPoseIsValid;
		vr::HmdVector3_t vVel;
		vr::HmdVector3_t vAngVel;
		vr::ETrackingResult eTrackingResult;
		char buf[1024];
		std::ofstream mystream;

		vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);
		switch (trackedDeviceClass) {
		case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
			vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);

			vector = lighthouseTracking->GetPosition(devicePose->mDeviceToAbsoluteTracking);
			quaternion = lighthouseTracking->GetRotation(devicePose->mDeviceToAbsoluteTracking);

			// print stuff for the HMD here, see controller example below
			sprintf_s(buf, sizeof(buf), "\nHMD\nx: %.2f y: %.2f z: %.2f\n", vector.v[0], vector.v[1], vector.v[2]);
			printf_s(buf);
			sprintf_s(buf, sizeof(buf), "qw: %.2f qx: %.2f qy: %.2f qz: %.2f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
			printf_s(buf);

			break;

		case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
				vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState, sizeof(controllerState), &trackedDevicePose);

				vector = lighthouseTracking->GetPosition(controllerPose->mDeviceToAbsoluteTracking);
				quaternion = lighthouseTracking->GetRotation(controllerPose->mDeviceToAbsoluteTracking);
				
				
				/*cout << "Rotational matrix: " << endl;
				cout << controllerPose->mDeviceToAbsoluteTracking.m[0][0] << controllerPose->mDeviceToAbsoluteTracking.m[0][1] <<controllerPose->mDeviceToAbsoluteTracking.m[0][2] << controllerPose->mDeviceToAbsoluteTracking.m[0][3] << endl;
				cout << controllerPose->mDeviceToAbsoluteTracking.m[1][0] << controllerPose->mDeviceToAbsoluteTracking.m[1][1] << controllerPose->mDeviceToAbsoluteTracking.m[1][2] << controllerPose->mDeviceToAbsoluteTracking.m[1][3] << endl;
				cout << controllerPose->mDeviceToAbsoluteTracking.m[2][0] << controllerPose->mDeviceToAbsoluteTracking.m[2][1] << controllerPose->mDeviceToAbsoluteTracking.m[2][2] << controllerPose->mDeviceToAbsoluteTracking.m[2][3] << endl;
				cout << controllerPose->mDeviceToAbsoluteTracking.m[3][0] << controllerPose->mDeviceToAbsoluteTracking.m[3][1] << controllerPose->mDeviceToAbsoluteTracking.m[3][2] << endl;
				*/

				switch (vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(unDevice)) {
				case vr::TrackedControllerRole_Invalid:
					// invalid hand... 
					printf("hand invalid \n");
					break;

				case vr::TrackedControllerRole_LeftHand:
				

					sprintf_s(buf, sizeof(buf), "\nLeft Controller\nx: %.2f y: %.2f z: %.2f\n", vector.v[0], vector.v[1], vector.v[2]);
					printf_s(buf);

					left.push_back(vector.v[0]); left.push_back(vector.v[1]); left.push_back(vector.v[2]);

					possitiondata.push_back(vector.v[0]);
					possitiondata.push_back(vector.v[1]);
					possitiondata.push_back(vector.v[2]);

					sprintf_s(buf, sizeof(buf), "qw: %.2f qx: %.2f qy: %.2f qz: %.2f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
					printf_s(buf);

					left.push_back(quaternion.w); left.push_back(quaternion.x), left.push_back(quaternion.y); left.push_back(quaternion.z);

					possitiondata.push_back(quaternion.w);
					possitiondata.push_back(quaternion.x);
					possitiondata.push_back(quaternion.y);
					possitiondata.push_back(quaternion.z);
					possitiondata.push_back(11);
					break;

				case vr::TrackedControllerRole_RightHand:

					sprintf_s(buf, sizeof(buf), "\nRight Controller\nx: %.2f y: %.2f z: %.2f\n", vector.v[0], vector.v[1], vector.v[2]);
					printf_s(buf);
					right.push_back(vector.v[0]); right.push_back(vector.v[1]); right.push_back(vector.v[2]);

					possitiondata.push_back(vector.v[0]);
					possitiondata.push_back(vector.v[1]);
					possitiondata.push_back(vector.v[2]);

					sprintf_s(buf, sizeof(buf), "qw: %.2f qx: %.2f qy: %.2f qz: %.2f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
					printf_s(buf);

					right.push_back(quaternion.w); right.push_back(quaternion.x), right.push_back(quaternion.y); right.push_back(quaternion.z);

					possitiondata.push_back(quaternion.w);
					possitiondata.push_back(quaternion.x);
					possitiondata.push_back(quaternion.y);
					possitiondata.push_back(quaternion.z);
					possitiondata.push_back(10);
					break;

				}

				break;
			}

		
	}
	cout << "posdata size " << possitiondata.size() << endl;
	return possitiondata;
}


//callibrate main
int gomain() 
{

	vector<CrpiRobot<CrpiAbb>> arms;

	CrpiRobot<CrpiAbb> armL("abb_irb14000_left.xml");
	std::cout << "left arm connected" << endl;

	std::cout << "initializing" << endl;
	armL.SetAngleUnits("degree");
	armL.SetLengthUnits("mm");

	CrpiRobot<CrpiAbb> armR("abb_irb14000_right.xml");
	std::cout << "right arm connected" << endl;

	std::cout << "initializing" << endl;
	armR.SetAngleUnits("degree");
	armR.SetLengthUnits("mm");

	std::cout << "coupling tools" << endl;
	armL.Couple("Yumi_Parallel");
	armR.Couple("Yumi_Parallel");
	std::cout << "coupling tools done" << endl;

	//Pose variables and initialization
	robotPose poseMe, curPose, curPose_r, poseR, poseR_center, poseL_center, poseL_up, poseR_up, poseL_down, poseR_down, poseL_L, poseR_L, poseL_R, poseR_R;
	robotAxes curAxes, curAxesR, tarAxes, tarAxesR;

	//Left
	curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
	poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;

	//Right
	curPose_r.x = curPose_r.y = curPose_r.z = curPose_r.xrot = curPose_r.yrot = curPose_r.zrot = 0.0f;
	poseR.x = poseR.y = poseR.z = poseR.xrot = poseR.yrot = poseR.zrot = 0.0f;

	//Center arm possitions
	poseR_center.x = 411.715;
	poseR_center.y = -1.65271;
	poseR_center.z = 237.495;

	poseL_center.x = 366.163;
	poseL_center.y = 124.383;
	poseL_center.z = 230.711;

	//Down arm possitions
	poseR_down.x = 411.715;
	poseR_down.y = -1.65271;
	poseR_down.z = 107.905;

	poseL_down.x = 366.163;
	poseL_down.y = 124.383;
	poseL_down.z = 107.905;

	//Up arm possitions
	poseR_up.x = 411.715;
	poseR_up.y = -1.65271;
	poseR_up.z = 407.79;

	poseL_up.x = 366.821;
	poseL_up.y = 124.187;
	poseL_up.z = 407.79;

	//ESTABLISH
/*	
	//Left arm possitions
	poseL_L.x = 4;
	poseL_L.y = 4;
	poseL_L.z = 4;

	poseR_L.x = 4;
	poseR_L.y = 4;
	poseR_L.z = 4;

	//Right arm possitions
	poseL_R.x = 4;
	poseL_R.y = 4;
	poseL_R.z = 4;

	poseR_R.x = 4;
	poseR_R.y = 4;
	poseR_R.z = 4;
*/

	//Start tracking event
	LighthouseTracking *lighthouseTracking = new LighthouseTracking();
	crpi_timer timer;
	//Variable initialization
	double track = 0;
	bool right = false;
	bool left = false;
	bool open = false;
	double gripper = 0;
	time_t start = time(0);
	cout << start << endl;
	vector<double> lastvivedata, lefthand, righthand;
	ofstream outfile;
	outfile.open("VR_coords.csv");

	//initialization of VR variables
	vr::IVRSystem *m_pHMD; 
	vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
	Matrix4 m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];
	vr::VREvent_t event;
	bool center, up, down, done = false;
	center = false;
	up = false;
	down = false;
	while (!done) 
	{
		//Get current event
		while (lighthouseTracking->m_pHMD->PollNextEvent(&event, sizeof(event)))
		{
			// Process event
			if (!lighthouseTracking->ProcessVREvent(event))
			{
				char buf[1024];
				sprintf_s(buf, sizeof(buf), "(OpenVR) service quit\n");
				printf_s(buf);
				return 0;
			}

			// Parse current frame
			ParseTrackingFrame1(lighthouseTracking, righthand, lefthand);

			//left hand input validation
			cout << "left parsed size: " << lefthand.size() << endl;
			for (int i = 0; i < lefthand.size() - 1; i++)
			{
				left = true;
				cout << lefthand[i] << ",";
			}
			cout << lefthand[lefthand.size() - 1] << endl;

			//right hand input validation
			cout << "right parsed size: " << righthand.size() << endl;
			if (righthand.size() == 7)
			{
				right = true;
			}

			for (int i = 0; i < righthand.size() - 1; i++)
			{
				cout << righthand[i] << ",";
			}
			cout << righthand[righthand.size() - 1] << endl;
			cout << "Input Parse Complete ----------------------------------------------" << endl;
		
			if (!center) 
			{
				cout << "---Center ------------------" << endl;
				outfile << "vx, vy, vz, rx, ry, rz, orientation" << endl;

				/*----------------Center------------------*/
				//move left arm
				//outfile << lefthand[0] << ", " << lefthand[1] << ", " << lefthand[2] << "," << poseL_center.x << ", " << poseL_center.y << ", " << poseL_center.z << ", " << "center left" << endl;
				//lefthand = translateCoordinates(lefthand);
				G_POSE = "Center";


				if (armL.GetRobotPose(&curPose) == CANON_SUCCESS)
				{
					armL.GetRobotPose(&poseMe);
					armL.GetRobotAxes(&curAxes);

					std::cout << "robo initial left: (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;
					poseMe = poseL_center;
					if (armL.MoveStraightTo(poseMe) != CANON_SUCCESS)
					{
						std::cout << "motion error" << endl;
					}

				}
				lefthand.clear();


				

				//move right arm
				//outfile << righthand[0] << ", " << righthand[1] << ", " << righthand[2] << "," << poseR_center.x << ", " << poseR_center.y << ", " << poseR_center.z << ", " << "center right" << endl;

				if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
				{
					std::cout << "robo initial right (" << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << ")" << endl;
					armR.GetRobotPose(&poseR);
					armR.GetRobotAxes(&curAxesR);

					poseR = poseR_center;

					if (armR.MoveStraightTo(poseR) != CANON_SUCCESS)
					{
						std::cout << "motion error" << endl;
					}
					std::cout << "New right (" << poseR.x << ", " << poseR.y << ", " << poseR.z << ", " << poseR.xrot << ", " << poseR.yrot << ", " << poseR.zrot << ")" << endl;
					righthand.clear();
				}
				timer.waitUntil(100);
				center = true;
			}
			/*---------------UP--------------*/
			if (!up)
			{
				//move left arm
				//outfile << lefthand[0] << ", " << lefthand[1] << ", " << lefthand[2] << "," << poseL_up.x << ", " << poseL_up.y << ", " << poseL_up.z << ", " << "up left" << endl;
				//lefthand = translateCoordinates(lefthand);

				G_POSE = "Center Up";
				if (armL.GetRobotPose(&curPose) == CANON_SUCCESS)
				{
					armL.GetRobotPose(&poseMe);
					armL.GetRobotAxes(&curAxes);

					poseMe = poseL_up;
					if (armL.MoveStraightTo(poseMe) != CANON_SUCCESS)
					{
						std::cout << "motion error" << endl;
					}

				}
				lefthand.clear();

				//move right arm
				//outfile << righthand[0] << ", " << righthand[1] << ", " << righthand[2] << "," << poseR_up.x << ", " << poseR_up.y << ", " << poseR_up.z << ", " << "up right" << endl;

				if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
				{
					std::cout << "robo initial right (" << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << ")" << endl;
					armR.GetRobotPose(&poseR);
					armR.GetRobotAxes(&curAxesR);

					poseR = poseR_up;

					if (armR.MoveStraightTo(poseR) != CANON_SUCCESS)
					{
						std::cout << "motion error" << endl;
					}
					std::cout << "New right (" << poseR.x << ", " << poseR.y << ", " << poseR.z << ", " << poseR.xrot << ", " << poseR.yrot << ", " << poseR.zrot << ")" << endl;
					righthand.clear();
				}
				timer.waitUntil(100);
				up = true;
			}
			/*-------------DOWN-------------*/
			if (!down)
			{
				//Move left arm
				//outfile << lefthand[0] << ", " << lefthand[1] << ", " << lefthand[2] << "," << poseL_down.x << ", " << poseL_down.y << ", " << poseL_down.z << ", " << "down left" << endl;
				//lefthand = translateCoordinates(lefthand);
				G_POSE = "Center Down";

				if (armL.GetRobotPose(&curPose) == CANON_SUCCESS)
				{
					armL.GetRobotPose(&poseMe);
					armL.GetRobotAxes(&curAxes);

					poseMe = poseL_up;
					if (armL.MoveStraightTo(poseMe) != CANON_SUCCESS)
					{
						std::cout << "motion error" << endl;
					}

				}
				lefthand.clear();


				//Move right arm
				//outfile << righthand[0] << ", " << righthand[1] << ", " << righthand[2] << "," << poseR_down.x << ", " << poseR_down.y << ", " << poseR_down.z << ", " << "down right" << endl;

				if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
				{
					std::cout << "robo initial right (" << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << ")" << endl;
					armR.GetRobotPose(&poseR);
					armR.GetRobotAxes(&curAxesR);

					poseR = poseR_up;

					if (armR.MoveStraightTo(poseR) != CANON_SUCCESS)
					{
						std::cout << "motion error" << endl;
					}
					std::cout << "New right (" << poseR.x << ", " << poseR.y << ", " << poseR.z << ", " << poseR.xrot << ", " << poseR.yrot << ", " << poseR.zrot << ")" << endl;
					righthand.clear();
				}
				timer.waitUntil(100);
				down = true;
			}
			/*---------------Re_CENTER----------------------*/
			//move left arm
			//outfile << lefthand[0] << ", " << lefthand[1] << ", " << lefthand[2] << "," << poseL_center.x << ", " << poseL_center.y << ", " << poseL_center.z << ", " << "center left" << endl;
			//lefthand = translateCoordinates(lefthand);


			if (armL.GetRobotPose(&curPose) == CANON_SUCCESS)
			{
				armL.GetRobotPose(&poseMe);
				armL.GetRobotAxes(&curAxes);

				poseMe = poseL_center;
				if (armL.MoveStraightTo(poseMe) != CANON_SUCCESS)
				{
					std::cout << "motion error" << endl;
				}

			}
			lefthand.clear();


			timer.waitUntil(500);

			//move right arm
			//outfile << righthand[0] << ", " << righthand[1] << ", " << righthand[2] << "," << poseR_center.x << ", " << poseR_center.y << ", " << poseR_center.z << ", " << "center right" << endl;

			if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
			{
				std::cout << "robo initial right (" << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << ")" << endl;
				armR.GetRobotPose(&poseR);
				armR.GetRobotAxes(&curAxesR);

				poseR = poseR_center;

				if (armR.MoveStraightTo(poseR) != CANON_SUCCESS)
				{
					std::cout << "motion error" << endl;
				}
				std::cout << "New right (" << poseR.x << ", " << poseR.y << ", " << poseR.z << ", " << poseR.xrot << ", " << poseR.yrot << ", " << poseR.zrot << ")" << endl;
				righthand.clear();
			}
			timer.waitUntil(500);	
			
		}		
		
		track = difftime(time(0), start);
		
		if (up == true && down == true && center == true) 
		{
			done = true;
			break;
		}                             

	}
	outfile.close();
	return 0;
}

//Moves robot to center position
int ddmain() 
{
	char buf[1024];

	CrpiRobot<CrpiAbb> armL("abb_irb14000_left.xml");
	armL.SetAngleUnits("degree");
	armL.SetLengthUnits("mm");
	sprintf_s(buf, sizeof(buf), "Left connected and init");
	printf_s(buf);

	CrpiRobot<CrpiAbb> armR("abb_irb14000_right.xml");
	armR.SetAngleUnits("degree");
	armR.SetLengthUnits("mm");
	sprintf_s(buf, sizeof(buf), "Right connected and init");
	printf_s(buf);

	armL.Couple("Yumi_Parallel");
	armR.Couple("Yumi_Parallel");
	sprintf_s(buf, sizeof(buf), "Tool coupling done");
	printf_s(buf);

	//! Pose Variables
	robotPose poseMe, curPose, curPose_r, poseR;
	robotAxes curAxes, curAxesR, tarAxes, tarAxesR;

	//center robot L
	tarAxes.axis[0] = -128.393;
	tarAxes.axis[1] = -126.96;
	tarAxes.axis[2] = 21.8281;
	tarAxes.axis[3] = 24.3266;
	tarAxes.axis[4] = 182.648;
	tarAxes.axis[5] = -71.2129;
	tarAxes.axis[6] = -26.6042;

	//center robot R
	tarAxesR.axis[0] = 120.111;
	tarAxesR.axis[1] = -124.582;
	tarAxesR.axis[2] = -16.7288;
	tarAxesR.axis[3] = 23.7964;
	tarAxesR.axis[4] = -175.136;
	tarAxesR.axis[5] = -58.5175;
	tarAxesR.axis[6] = 105.083;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	return 0;
}

int dmain()
{
	//initialize robot
	vector<CrpiRobot<CrpiAbb>> arms;
	char buf[1024];

	CrpiRobot<CrpiAbb> armL("abb_irb14000_left.xml");
	//std::cout << "left arm connected" << endl;
	sprintf_s(buf, sizeof(buf), ":left arm connect");
	printf_s(buf);

	//std::cout << "initializing" << endl;
	armL.SetAngleUnits("degree");
	armL.SetLengthUnits("mm");
	sprintf_s(buf, sizeof(buf), "initializing");
	printf_s(buf);

	CrpiRobot<CrpiAbb> armR("abb_irb14000_right.xml");
	sprintf_s(buf, sizeof(buf), "right arm connect");
	printf_s(buf);

	armR.SetAngleUnits("degree");
	armR.SetLengthUnits("mm");
	sprintf_s(buf, sizeof(buf), "right arm calib");
	printf_s(buf);

	sprintf_s(buf, sizeof(buf), "tool coupling");
	armL.Couple("Yumi_Parallel");
	armR.Couple("Yumi_Parallel");
	sprintf_s(buf, sizeof(buf), "coupling done");
	printf_s(buf);

	//Pose variables and initialization
	robotPose poseMe, curPose, curPose_r, poseR;
	robotAxes curAxes, curAxesR, tarAxes, tarAxesR;
	
	//callibration poses ----------------------------------------------------------------------------------------------------
	robotPose poseR_center, poseL_center, poseL_up, poseR_up, poseL_down, poseR_down, poseL_L, poseR_L, poseL_R, poseR_R;
	//Center arm possitions
	poseR_center.x = 411.715;
	poseR_center.y = -1.65271;
	poseR_center.z = 237.495;

	poseL_center.x = 366.163;
	poseL_center.y = 124.383;
	poseL_center.z = 230.711;

	//Down arm possitions
	poseR_down.x = 411.715;
	poseR_down.y = -1.65271;
	poseR_down.z = 107.905;

	poseL_down.x = 366.163;
	poseL_down.y = 124.383;
	poseL_down.z = 107.905;

	//Up arm possitions
	poseL_up.x = 366.821;
	poseL_up.y = 124.187;
	poseL_up.z = 407.79;

	poseR_up.x = 411.715;
	poseR_up.y = -1.65271;
	poseR_up.z = 407.79;

	//end callibration poses ------------------------------------------------------------------------------------------------


	//Left
	curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
	poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
	//Right
	curPose_r.x = curPose_r.y = curPose_r.z = curPose_r.xrot = curPose_r.yrot = curPose_r.zrot = 0.0f;
	poseR.x = poseR.y = poseR.z = poseR.xrot = poseR.yrot = poseR.zrot = 0.0f;


	//Start tracking event
	//LighthouseTracking *lighthouseTracking = new LighthouseTracking();
	
	//Variable initialization
	double track = 0;
	bool right = false;
	bool left = false;
	bool open = false;
	double gripper = 0;
	time_t start = time(0);
	bool done = false;
	cout << start << endl;
	vector<double> lastvivedata, lefthand, righthand;
	if (armL.GetRobotPose(&curPose) == CANON_SUCCESS)
	{
		std::cout << "robo initial left: (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;
		armL.GetRobotPose(&poseMe);

		armL.GetRobotAxes(&curAxes);

		//- goes inwards, + goes outwards
		//poseMe.x = curPose.x  - 39.098;
		//- goes right, + goes left
		//poseMe.y = curPose.y + 10.0;
		//- goes down, + goes up
		//poseMe.z = curPose.z + 10.0;
		//cout << "goal values: " << lefthand[0] << ", " << lefthand[1] << ", " << lefthand[2] << endl;
		//poseMe.x = lefthand[0];
		//poseMe.y = lefthand[1];
		//poseMe.z = lefthand[2];

		if (armL.MoveStraightTo(poseMe) != CANON_SUCCESS)
		{
			std::cout << "motion error" << endl;
		}
		//if (Math::rotQuaternionMatrixConvert()) {
		std::cout << "New left(" << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " << poseMe.yrot << ", " << poseMe.zrot << ")" << endl;
		//}

	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		std::cout << "robo initial right (" << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << ")" << endl;
		armR.GetRobotPose(&poseR);

		armR.GetRobotAxes(&curAxesR);

		//- goes inwards, + goes outwards
		//poseR.x = curPose_r.x + 15.0;
		//- goes right, + goes left
		//poseR.y = curPose_r.y - 20.0;
		//- goes down, + goes up
		//poseR.z = curPose_r.z - 50.0;

		//poseR.x = righthand[0];
		//poseR.y = righthand[1];
		//poseR.z = righthand[2];
		if (armR.MoveStraightTo(poseR) != CANON_SUCCESS)
		{
			std::cout << "motion error" << endl;
		}
		std::cout << "New right (" << poseR.x << ", " << poseR.y << ", " << poseR.z << ", " << poseR.xrot << ", " << poseR.yrot << ", " << poseR.zrot << ")" << endl;
	}

	return 0;
	
	/*
	//Adjust this for longer running time
	while (track < 90)
	{
		//initialization of VR variables
		vr::IVRSystem *m_pHMD;
		vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
		Matrix4 m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];
		vr::VREvent_t event;
		
		//Get current event
		while (lighthouseTracking->m_pHMD->PollNextEvent(&event, sizeof(event)))
		{
			// Process event
			if (!lighthouseTracking->ProcessVREvent(event)) 
			{
				char buf[1024];
				sprintf_s(buf, sizeof(buf), "(OpenVR) service quit\n");
				printf_s(buf);
				return 0;
			}
			track = difftime(time(0), start);

			//adjust this for longer time
			if (track > 100)
			{
				break;
			}

			// Parse current frame
			ParseTrackingFrame1(lighthouseTracking, righthand, lefthand);

			//left hand input validation
			cout << "left parsed size: " << lefthand.size() << endl;
			for (int i = 0; i < lefthand.size()-1; i++)
			{
				left = true;
				cout << lefthand[i] << ",";
			}
			cout << lefthand[lefthand.size()-1] << endl;
			
			//right hand input validation
			cout << "right parsed size: " << righthand.size() << endl;
			if (righthand.size() == 7) 
			{
				right = true;
			}

			for (int i = 0; i < righthand.size()-1; i++)
			{
				cout << righthand[i] << ",";
			}
			cout << righthand[righthand.size() - 1]<< endl;

			cout << "Input Parse Complete ----------------------------------------------" << endl;

			//move left arm
			if (left)
			{
				lefthand = translateCoordinates(lefthand);
				cout << "Left goal values: " << lefthand[0] << ", " << lefthand[1] << ", " << lefthand[2] << endl;

				
				if (armL.GetRobotPose(&curPose) == CANON_SUCCESS)
				{
					std::cout << "robo initial left: (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;
					armL.GetRobotPose(&poseMe);

					armL.GetRobotAxes(&curAxes);

					//- goes inwards, + goes outwards
					//poseMe.x = curPose.x  - 39.098;
					//- goes right, + goes left
					//poseMe.y = curPose.y + 10.0;
					//- goes down, + goes up
					//poseMe.z = curPose.z + 10.0;
					cout << "goal values: " << lefthand[0] << ", " << lefthand[1] << ", " << lefthand[2] << endl;
					//poseMe.x = lefthand[0];
					//poseMe.y = lefthand[1];
					//poseMe.z = lefthand[2];

					if (armL.MoveStraightTo(poseMe) != CANON_SUCCESS)
					{
						std::cout << "motion error" << endl;
					}
					//if (Math::rotQuaternionMatrixConvert()) {
					std::cout << "New left(" << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " << poseMe.yrot << ", " << poseMe.zrot << ")" << endl;
					//}

				}
				lefthand.clear();
				left = false;

			}

			//move right arm
			if (right)
			{
				righthand = translateCoordinates(righthand);
				cout << "Right goal values: " << righthand[0] << ", " << righthand[1] << ", " << righthand[2] << endl;

				
				if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
				{
					std::cout << "robo initial right (" << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << ")" << endl;
					armR.GetRobotPose(&poseR);

					armR.GetRobotAxes(&curAxesR);

					//- goes inwards, + goes outwards
					//poseR.x = curPose_r.x + 15.0;
					//- goes right, + goes left
					//poseR.y = curPose_r.y - 20.0;
					//- goes down, + goes up
					//poseR.z = curPose_r.z - 50.0;

					//poseR.x = righthand[0];
					//poseR.y = righthand[1];
					//poseR.z = righthand[2];
					if (armR.MoveStraightTo(poseR) != CANON_SUCCESS)
					{
						std::cout << "motion error" << endl;
					}
					std::cout << "New right (" << poseR.x << ", " << poseR.y << ", " << poseR.z << ", " << poseR.xrot << ", " << poseR.yrot << ", " << poseR.zrot << ")" << endl;
				}
				righthand.clear();
				right = false;
			}


		}
		track = difftime(time(0), start);
		//cout << track << endl;

	}
	cout << "time elapsed " << track << endl;
	return 0;*/

}

	