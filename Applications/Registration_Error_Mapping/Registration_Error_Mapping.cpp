///////////////////////////////////////////////////////////////////////////////
//
//  Original System: ISD CRPI
//  Subsystem:       Registration_Error_Mapping
//  Workfile:        Registration_Error_Mapping.cpp
//  Revision:        7 November, 2016
//  Author:          K. Van Wyk
//
//  Description
//  ===========
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
#include <sstream>
#include <conio.h> //keyboard inputs

#pragma warning (disable: 4996)

//#define NOISY
//#define SUPERNOISY
//#define ALTTEXT

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;
using namespace Sensor;
using namespace Registration;

//Global Variables
typedef CrpiUniversal robType;

vector<double> ground_truth(robotPose & curPose)
{
	//Since optical board is with 25 mm space, find closest ground truth
	//Vertical height is static at 11 mm.
	vector<double> ground_truth(3);
	ground_truth[2] = 11;
	double multiple=0;

	//check X data
	if (fmod(abs(curPose.x),25.0) <= 12.5) //round down
	{
		multiple = floor(abs(curPose.x)/25.0);

		//Signage
		if (curPose.x > 0)
		{
			ground_truth[0] = multiple*25.0;
		}

		else
		{
			ground_truth[0] = -multiple*25.0;
		}
	}

	else //round up
	{
		multiple = ceil(abs(curPose.x)/25.0);

		//Signage
		if (curPose.x > 0)
		{
			ground_truth[0] = multiple*25.0;
		}

		else
		{
			ground_truth[0] = -multiple*25.0;
		}
	}

	//check Y data
	if (fmod(abs(curPose.y),25.0) <= 12.5) //round down
	{
		multiple = floor(abs(curPose.y)/25.0);

		//Signage
		if (curPose.y > 0)
		{
			ground_truth[1] = multiple*25.0;
		}

		else
		{
			ground_truth[1] = -multiple*25.0;
		}
	}

	else //round up
	{
		multiple = ceil(abs(curPose.y)/25.0);

		//Signage
		if (curPose.y > 0)
		{
			ground_truth[1] = multiple*25.0;
		}

		else
		{
			ground_truth[1] = -multiple*25.0;
		}
	}

	return ground_truth;
};

int main (int argc, char * argv[])
{
	if (argc != 2) {cout << "Incorrect command line parameters! Give only desired filename!" <<endl; return 0;}

	string filename(argv[1]);

	ofstream recorder;
	char full_path[2000];
	strcpy(full_path,"");
	//strcat(full_path,"..\\Applications\\Registration_Error_Mapping\\");
	strcat(full_path,filename.c_str());
	strcat(full_path,".csv");
	//string pathy(full_path);
	//std::cout << pathy << std::endl;
	//cin.get();

  int i = 0;
  crpi_timer timer;

  //Setting up robots
  CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");

  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");
  arm.Couple("flange_ring");
  Sleep(1000);

  robotPose curPose, tarPose, W_curPose;
  robotAxes curAxes, tarAxes, homeAxes;
  bool poseDefined = false;
  bool tool = false;

  curPose.x = 10;
  curPose.y = 10;
  curPose.z = 10;
  curPose.xrot = 10;
  curPose.yrot = 10;
  curPose.zrot = 10;
  
  homeAxes.axis[0] = 135;
  homeAxes.axis[1] = -90;
  homeAxes.axis[2] = 90;
  homeAxes.axis[3] = 180;
  homeAxes.axis[4] = 0;
  homeAxes.axis[5] = 0;

  arm.SetRelativeSpeed(0.3);
  arm.MoveToAxisTarget(homeAxes);
  arm.GetRobotAxes(&curAxes);

  /*
  arm.GetRobotPose(&curPose);
  curPose.print();
  arm.ToWorld(&curPose,&W_curPose);
  W_curPose.print();

  arm.Couple("default");
  Sleep(1000);
  arm.GetRobotPose(&curPose);
  curPose.print();

  cin.get();
  */

  int register_robot;
  cout << "Register robot? 1) yes, 2) no" << endl;
  cin >> register_robot;
  cin.ignore();

  //Robot registration
  if (register_robot == 1)
  {
	  //arm.Couple("flange_ring");
	  //Sleep(1000);
	  vector<robotPose> regposes;
	  vector<robotPose>::iterator regiter;
	  point wldpt[4], robpt[4];
	  string color[4];

	  robotPose pur[4], pworld[4];

	  matrix W_T_R1(4, 4), W_T_R2(4, 4), W_T_R3(4, 4), W_T_R4(4, 4);
	  matrix W_T_R(4, 4); //average

	  vector<point> world1, world2, world3, world4; 
	  vector<point> rob1,rob2,rob3,rob4;

	  //Reading in hard coded world poses for large points from .dat...
	  ifstream WorldVals("Config.large.dat");

	  //! Load 
	  for (i = 0; i < 4; ++i)
	  {
		WorldVals >> color[i] >> pworld[i].x >> pworld[i].y >> pworld[i].z;
		cout << color[i] << " " << pworld[i].x << " " << pworld[i].y << " " << pworld[i].z << endl;
	  }
  
	  WorldVals.close();

	  int choice;
	  char in;
	  for (i = 0; i < 4; ++i)
		{
			cout << "Move ur to defined point " << (i+1) << " (" << color[i] << ") and press ENTER";
			cin.get();
		
			arm.GetRobotPose(&pur[i]);
			robpt[i].x = pur[i].x;
			robpt[i].y = pur[i].y;
			robpt[i].z = pur[i].z;

			pur[i].print();

			wldpt[i].x = pworld[i].x;
			wldpt[i].y = pworld[i].y;
			wldpt[i].z = pworld[i].z;

			pworld[i].print();
		}

		//Calculate Registration - Four combinations of four points in sets of 3
		world1.clear();
		world1.push_back(wldpt[0]);
		world1.push_back(wldpt[1]);
		world1.push_back(wldpt[2]);

		world2.clear();
		world2.push_back(wldpt[1]);
		world2.push_back(wldpt[2]);
		world2.push_back(wldpt[3]);

		world3.clear();
		world3.push_back(wldpt[2]);
		world3.push_back(wldpt[3]);
		world3.push_back(wldpt[0]);

		world4.clear();
		world4.push_back(wldpt[3]);
		world4.push_back(wldpt[0]);
		world4.push_back(wldpt[1]);

		rob1.clear();
		rob1.push_back(robpt[0]);
		rob1.push_back(robpt[1]);
		rob1.push_back(robpt[2]);

		rob2.clear();
		rob2.push_back(robpt[1]);
		rob2.push_back(robpt[2]);
		rob2.push_back(robpt[3]);

		rob3.clear();
		rob3.push_back(robpt[2]);
		rob3.push_back(robpt[3]);
		rob3.push_back(robpt[0]);

		rob4.clear();
		rob4.push_back(robpt[3]);
		rob4.push_back(robpt[0]);
		rob4.push_back(robpt[1]);

		reg2target(rob1, world1, W_T_R1);
		W_T_R1.print();
		reg2target(rob2, world2, W_T_R2);
		W_T_R2.print();
		reg2target(rob3, world3, W_T_R3);
		W_T_R3.print();
		reg2target(rob4, world4, W_T_R4);
		W_T_R4.print();

		for (int ii=0; ii<4; ++ii)
		{
			for (int jj=0; jj<4; ++jj)
			{
				W_T_R.at(ii,jj) = (W_T_R1.at(ii,jj) + W_T_R2.at(ii,jj) + W_T_R3.at(ii,jj) + W_T_R4.at(ii,jj))/4;
			}
		}

		
		W_T_R.print();

		arm.UpdateWorldTransform(W_T_R.inv());
		string urfile = "universal_ur5.xml";
		arm.SaveConfig(urfile.c_str());

		arm.Couple("flange_ring");
		Sleep(1000);

		cout << "Move robot out of registration seat and into the open. Press ENTER when done." << std::endl;
		cin.get();
  }

  //arm.GetRobotAxes(&curAxes);
  //curAxes.print();
  
  //arm.MoveToAxisTarget(homeAxes);
  //cin.get();

  /*
  //Test vertical accuracy
  arm.Couple("flange_ring");
  Sleep(1000);
  arm.GetRobotPose(&curPose);
  robotPose w_curPose;
  arm.ToWorld(&curPose,&w_curPose);
  w_curPose.print();
  cin.get();
  */

  /*
  //Test with pointer
  arm.Couple("flange_ring");
  Sleep(1000);
  arm.SetAbsoluteSpeed(.1);
  
  robotPose w_pose, r_pose;
  w_pose.x = 5*25;
  w_pose.y = 12*25;
  w_pose.z = 50;
  w_pose.xrot = 0;
  w_pose.yrot = 180;
  w_pose.zrot = 0;

  arm.FromWorld(&w_pose,&r_pose);

  r_pose.print();

  arm.MoveTo(r_pose);

  w_pose.z = -1;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);
  cin.get();
  w_pose.z = 50;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);

  w_pose.x = -1*25;
  w_pose.y = 25*3;
  w_pose.z = 50;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);

  w_pose.z = -1;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);
  cin.get();
  w_pose.z = 50;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);

  w_pose.x = 25*22;
  w_pose.y = 25*3;
  w_pose.z = 50;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);

  w_pose.z = -1;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);
  cin.get();
  w_pose.z = 50;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);

  w_pose.x = 25*22;
  w_pose.y = 25*18;
  w_pose.z = 50;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);

  w_pose.z = -1;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);
  cin.get();
  w_pose.z = 50;

  arm.FromWorld(&w_pose,&r_pose);

  arm.MoveStraightTo(r_pose);
  */

  
  //------------------------------------------------------------------

	//Drag robot to different locations, poll current position and express in world
	//Save position and estimated ground truth to CSV file
	//Hit "Enter" to grab a position sample
	//Hit "Esc" to terminate programs

	//Data writing stuff
	
	recorder.open(full_path);	
	recorder << "raw_robot_X, raw_robot_Y, raw_robot_Z, registered_robot_X, registered_robot_Y, registered_robot_Z, truth_X, truth_Y, truth_Z" << std::endl;
  
	robotPose w_curPose;
	vector<double> truth(3);
	int key=0;
	int iteration=1;

	while (1)
	{
		if (kbhit())
		{
			key = getch(); //get keystroke
			//std::cout << key << std::endl;
			if (key == 13)
			{
				arm.GetRobotPose(&curPose);
				arm.ToWorld(&curPose,&w_curPose);
				truth = ground_truth(w_curPose);
				std::cout << "Iteration: " << iteration << std::endl;
				std::cout << "Raw Robot Pose:        " << curPose.x << ", " << curPose.y << ", " << curPose.z << std::endl;
				std::cout << "Registered Robot Pose: " << w_curPose.x << ", " << w_curPose.y << ", " << w_curPose.z << std::endl;
				std::cout << "Ground Truth Pose:     " << truth[0] << ", " << truth[1] << ", " << truth[2] << std::endl;
				recorder << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << w_curPose.x << ", " << w_curPose.y << ", " << w_curPose.z << ", " << truth[0] << ", " << truth[1] << ", " << truth[2] << std::endl;
				iteration+=1;
			}

			else if (key == 27) //Esc
			{break;}
		}
		Sleep(100);
	}

	recorder.close();

	return 0;

}