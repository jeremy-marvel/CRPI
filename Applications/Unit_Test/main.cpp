///////////////////////////////////////////////////////////////////////////////
//
//  Original System: ISD CRPI
//  Subsystem:       Unit Test
//  Workfile:        main.cpp
//  Revision:        23 July, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
//  
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <conio.h>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_demo_hack.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#include "NumericalMath.h" 
#include "../../Libraries/MotionPrims/AssemblyPrims.h"
#include "Vicon.h"
#include "CoordFrameReg.h"
#include "ati_wired.h"
#include "FT_COP.h"
#include "crpi_allegro.h"
#include "LeapMotion.h"

#pragma warning (disable: 4996)

//#define TESTROBOTIQ
//#define TESTSDH
//#define MANUALDEMO
//#define COGNEXDEMO
//#define ROBOTDATADEMO
//#define ROBOTSYNCDEMO
//#define NEWCOGNEXDEMO
//#define TOOLCHANGERDEMO
//#define FINGERFORCETEST
//#define SDHTEST
//#define ASSEMBLYDEMO
//#define ASSEMBLYTEST1
//#define PEGTEST
//#define MAPVISERR
//#define ASSEMBLYTEST1
//#define MOTIONDEMO
//#define XMLDEMO
//#define REGISTRATIONTEST
//#define JOINTDEMO
//#define MATHTEST
//#define SENSORTEST
//#define FT_TEST
//#define FORCE_REGISTER
//#define REGISTERTEST
//#define FIXED_3POINT_REGISTER
//#define ALLEGRO_CONTROL //main one
//#define ALLEGRO_DEXTERITY_DEMO
//#define SerialTest
#define DC_Demo

//#define NOISY
//#define SUPERNOISY
//#define ALTTEXT

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;
using namespace Sensor;
using namespace Registration;

//typedef CrpiUniversal robType;
typedef CrpiKukaLWR robType;
//typedef CrpiDemoHack robType;


//! @brief Remove whitespace from the messages from the Cognex system (used with COGNEXDEMO)
//!
//! @param linein The plaintext string sent via TCP from the Cognex system
//!
//! @note This function was added because of an issue with "fixing" the Cognex system
//!       to recognize the gearbox housing at all orientations.  To call this function
//!       define ALTTEXT at compile time
//!
void removeCognexWhitespace (char *linein);

//! @brief Parse the part names and locations from the plaintext string sent from the Cognex system (used with COGNEX DEMO)
//!
//! @param line The plaintext string sent via TCP from the Cognex system
//!
void parseCognexFrame (char *line);

//! @brief Parse the pose information sent from an external application (used with ROBOTSYNCDEMO)
//!
//! @param line The plaintext string containing the 6DOF pose (X, Y, Z, Xrot, Yrot, Zrot)
//! @param pose The robot target pose to be populated by the function
//!
void parseCommandPose (char *line, robotPose *pose);

//! @brief Bundle of variables used for this demonstration.  Access to shared
//!        values is needed for communicating between threads.
//!
struct passMe
{
  ulapi_mutex_struct* grabmutex;
  //CrpiRobot<CrpiDemoHack> *demo;
  CrpiRobot<robType> *robArm;
  bool t1;
  bool keeprunning;

  //! Connection to Congnex Server
  ulapi_integer clientID_;

  //! @brief Default constructor
  //!
  passMe ()
  {
    grabmutex = ulapi_mutex_new(89);

    //demo = NULL;
    keeprunning = true;
  }

  //! @brief DemoHack constructor
  //!
  /*
  passMe (CrpiRobot<CrpiDemoHack>* ptr)
  {
    grabmutex = ulapi_mutex_new(89);
    //demo = ptr;
    robArm = NULL;
    keeprunning = true;
  }
  */

  //! @brief Arm constructor
  //!
  passMe (CrpiRobot<robType>* ptr)
  {
    grabmutex = ulapi_mutex_new(89);
    robArm = ptr;
    //demo = NULL;
    keeprunning = true;
  }

};

//! @brief Bundle of variables used for the Cognex system demonstration of kitting gearbox parts
//!
struct cognexFrameData
{
  robotPose rp[5];
  robotPose rt_pickOffset[5], pa_pickOffset[5];
  double rt_hypOffset[5], pa_hypOffset[5];
  bool found[5];
  bool picked[5];

  double xadd;
  double yadd;
  double xmult;
  double ymult;
  double zrotadd;
  double zrotmult;
};


//! @brief Primary thread to maintain connection with the robotiq
//!
//! @param param Arguments passed to the thread to avoid needing the use of
//!              global variables
//!
void CognexThread (void *param);

//! Global variables (boo! hiss!)
cognexFrameData frameData;
bool userobotiq;


void main ()
{
  int i = 0;
  crpi_timer timer;

#ifdef TESTROBOTIQ
  CrpiRobot<CrpiRobotiq> riq("robotiq.xml");
  int param;

    //Grasp a gear (any size)
  for(i=0; i<33; i++)
  {

  timer.waitUntil(2000);

  //PreGrasp
  param = 1;
  riq.SetParameter("ADVANCED_CONTROL", &param);
  //riq.SetParameter("SCISSOR_CONTROL", &param);
  param=100;
  riq.SetParameter("SPEED_FINGER_A", &param);
  riq.SetParameter("SPEED_FINGER_B", &param);
  riq.SetParameter("SPEED_FINGER_C", &param);
  //riq.SetParameter("SPEED_SCISSOR", &param);
  
  param=0;
  riq.SetParameter("FORCE_FINGER_A", &param);
  riq.SetParameter("FORCE_FINGER_B", &param);
  riq.SetParameter("FORCE_FINGER_C", &param);
  //riq.SetParameter("FORCE_SCISSOR", &param);
  //param=200;
 // riq.SetParameter("POSITION_SCISSOR", &param);
  param=1;
  riq.SetParameter("GRIP", &param);
 
  //Grasp
  param=255;
  riq.SetParameter("POSITION_FINGER_A", &param);
  param=75;
  riq.SetParameter("POSITION_FINGER_B", &param);
  riq.SetParameter("POSITION_FINGER_C", &param);
  param=1;
  riq.SetParameter("GRIP", &param);
  timer.waitUntil(1000);

  param=75;
  riq.SetParameter("POSITION_FINGER_A", &param);
  riq.SetParameter("POSITION_FINGER_B", &param);
  riq.SetParameter("POSITION_FINGER_C", &param);
  param=1;
  riq.SetParameter("GRIP", &param);
  timer.waitUntil(1000);

  }

  //Release
  param=0;
  riq.SetParameter("POSITION_FINGER_A", &param);
  riq.SetParameter("POSITION_FINGER_B", &param);
  riq.SetParameter("POSITION_FINGER_C", &param);
  param=1;
  riq.SetParameter("GRIP", &param);
  timer.waitUntil(2000);

#elif defined SDHTEST
	CrpiRobot<CrpiSchunkSDH> sdh("dummy text");

	int param;
	double percent;

	while (1)
	{
		std::cout << "Enter pre-grasp configuration: 1) Tripod, 2) Lateral, 3) Acute Tripod -1) quit : ";
		std::cin >> i;

		if (i==1)
		{
			param = 1;
			sdh.SetParameter("GRIP_TYPE", &param);
		}

		if (i==2)
		{
			param = 2;
			sdh.SetParameter("GRIP_TYPE", &param);
		}

		if (i==3)
		{
			param = 3;
			sdh.SetParameter("GRIP_TYPE", &param);
		}

		if (i==4)
		{
		}

		std::cout << "Engage Grasping? 1) yes 2) Shut Down SDH ";
		std::cin >> i;
		if (i==1)
		{
			percent = 100;
			sdh.SetTool(percent);
		}

		if (i==2)
		{
			percent = -100;
			sdh.SetTool(percent);
		}

		std::cout << "Continue Grasping? " << endl;
		std::cin >> i;
		if (i==0) {break;}
	}

	std::cout << "Enter pre-grasp configuration: 1) Tripod, 2) Lateral, 3) Acute Tripod -1) quit : ";
	std::cin >> i;

	if (i==1)
	{
		param = 1;
		sdh.SetParameter("GRIP_TYPE", &param);
	}

	if (i==2)
	{
		param = 2;
		sdh.SetParameter("GRIP_TYPE", &param);
	}

	if (i==3)
	{
		param = 3;
		sdh.SetParameter("GRIP_TYPE", &param);
	}

	if (i==4)
	{
	}

	std::cout << "Engage Grasping? 1) yes 2) Shut Down SDH ";
	std::cin >> i;
	if (i==1)
	{
		percent = 100;
		sdh.SetTool(percent);
	}

	if (i==2)
	{
		percent = -100;
		sdh.SetTool(percent);
	}

	std::cin.ignore();
	std::cin.get();

#elif defined FINGERFORCETEST

  CrpiRobot<CrpiDemoHack> arm("kuka_lwr.xml");
  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->Couple("gripper_finger_test_a");
  robotPose poseMe, curPose;
  bool poseDefined = false;
  bool tool = false;
  int param;

   cout << "1) Test Finger A 2) Finger B, 3) Finger C, 4) Demo Mode, -1) quit : ";
  cin >> i;
  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      ulapi_mutex_give(pm.grabmutex);

      poseDefined = true;
    }

    if (i==1 || i==5)
    {
      poseMe.x = 412.5;
      poseMe.y = -350;
      poseMe.z = 350;
      poseMe.xrot = -90;
      poseMe.yrot = 0;
      poseMe.zrot = 180;

      ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
        cout << "Configuring finger A test." << endl;
      }

      ulapi_mutex_give(pm.grabmutex);
      poseMe = curPose;
      ulapi_mutex_take(pm.grabmutex);
      param = 1;
      pm.demo->SetParameter("ADVANCED_CONTROL", &param);
      pm.demo->SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      pm.demo->SetParameter("SPEED_FINGER_A", &param);
      pm.demo->SetParameter("SPEED_FINGER_B", &param);
      pm.demo->SetParameter("SPEED_FINGER_C", &param);
      pm.demo->SetParameter("SPEED_SCISSOR", &param);
      param=100;
      pm.demo->SetParameter("FORCE_FINGER_A", &param);
      pm.demo->SetParameter("FORCE_FINGER_B", &param);
      pm.demo->SetParameter("FORCE_FINGER_C", &param);
      pm.demo->SetParameter("FORCE_SCISSOR", &param);
      param=200;
      pm.demo->SetParameter("POSITION_SCISSOR", &param);
      param=10;
      pm.demo->SetParameter("POSITION_FINGER_A", &param);
      param=30;
      pm.demo->SetParameter("POSITION_FINGER_B", &param);
      pm.demo->SetParameter("POSITION_FINGER_C", &param);
      pm.demo->SetTool (0.1);
      ulapi_mutex_give(pm.grabmutex);
      timer.waitUntil(1000);
    }

    if (i==1 || i==2 || i==3)
    {
      cout << "1) Test Finger A 2) Finger B, 3) Finger C, 4) Demo Mode, -1) quit : ";
      cin >> i;
    }
  }// while (i != -1)

#elif defined MANUALDEMO
<<<<<<< .mine
  CrpiRobot<CrpiUniversal> arm("universal_ur10_right.xml");
//  CrpiRobot<CrpiKukaLWR> arm("kuka_lwr.xml");
  arm.InitCanon();
=======
  CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");

  //CrpiRobot<CrpiUniversal> arm("universal_ur10_right.xml");
  //CrpiRobot<CrpiKukaLWR> arm("kuka_lwr.xml");
>>>>>>> .r245
  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");
  arm.Couple("flange_ring");
  robotPose poseMe, curPose;
  robotAxes curAxes;
  bool poseDefined = false;
  bool tool = false;
  int param;

  double theta = 0.0f;
  matrix pin(3,1), pout(3,1);

  cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Move To Perch, 4) Move To Approach, 5) Cartesian feedback, 6) Pre-Grasp Gear, 7) Grasp Gear, 8) PreGrasp Bottom, 9) Grasp Bottom, 10) Pregrasp Top, 11) Grasp Top, 12) Release -1) quit : ";
  cin >> i;
  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
//      ulapi_mutex_take(pm.grabmutex);
      arm.GetRobotPose(&curPose);
//      ulapi_mutex_give(pm.grabmutex);

      poseDefined = true;
    }

    if (i >= 0 && i < 5)
    {
      poseMe = curPose;

      pin.at(0,0) = poseMe.x;
      pin.at(1,0) = poseMe.y;
      pin.at(2,0) = poseMe.z;

      switch (i)
      {
      case 0:
        poseMe.xrot = 180.0f;
        poseMe.yrot = 0.0f;
        poseMe.zrot = 0.0f;
        break;
      case 1:
        poseMe.x += 20.0;
        break;
      case 2:
        poseMe.x += -20.0;
        break;
      case 3:
        poseMe.y += 20.0;
//        poseMe.z = 500;
//        poseMe.y = -150;
        break;
      case 4:
        poseMe.y -= 20.0f;
//        poseMe.z = 200;
//        poseMe.y = -690;
      default:
        break;
      }

      pin.at(0,0) = poseMe.x;
      pin.at(1,0) = poseMe.y;
      pin.at(2,0) = poseMe.z;

//      ulapi_mutex_take(pm.grabmutex);
      if (arm.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
        cout << "curPose=" << curPose.z << endl;
      }
//      ulapi_mutex_give(pm.grabmutex);
    }

    else if (i == 5)
    {
      poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
//      ulapi_mutex_take(pm.grabmutex);
      arm.GetRobotPose(&poseMe);
//      ulapi_mutex_give(pm.grabmutex);

      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << " S:" << poseMe.status << " T:" << poseMe.turns << endl;
      arm.GetRobotAxes(&curAxes);
      cout << " >> J1:" << curAxes.axis[0] << " >> J2:" << curAxes.axis[1] << " >> J3:" << curAxes.axis[2] << " >> J4:" << curAxes.axis[3] << " >> J5:" << curAxes.axis[4] << " >> J6:" << curAxes.axis[5] << " >> J7:" << curAxes.axis[6] << endl; 
    }

    else if (i == 6) //Pre-Grasp Gears
    {  
      /*
      poseMe = curPose;
      ulapi_mutex_take(pm.grabmutex);
      param = 1;
      pm.demo->SetParameter("ADVANCED_CONTROL", &param);
      pm.demo->SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      pm.demo->SetParameter("SPEED_FINGER_A", &param);
      pm.demo->SetParameter("SPEED_FINGER_B", &param);
      pm.demo->SetParameter("SPEED_FINGER_C", &param);
      pm.demo->SetParameter("SPEED_SCISSOR", &param);
      param=100;
      pm.demo->SetParameter("FORCE_FINGER_A", &param);
      pm.demo->SetParameter("FORCE_FINGER_B", &param);
      pm.demo->SetParameter("FORCE_FINGER_C", &param);
      pm.demo->SetParameter("FORCE_SCISSOR", &param);
      param=200;
      pm.demo->SetParameter("POSITION_SCISSOR", &param);
      param=10;
      pm.demo->SetParameter("POSITION_FINGER_A", &param);
      param=30;
      pm.demo->SetParameter("POSITION_FINGER_B", &param);
      pm.demo->SetParameter("POSITION_FINGER_C", &param);
      pm.demo->SetTool (0.1);
      ulapi_mutex_give(pm.grabmutex);
	  */
      timer.waitUntil(1000);
      poseMe.z = 20;
      // Move to grasp position
//      ulapi_mutex_take(pm.grabmutex);
      if (arm.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
      }
//      ulapi_mutex_give(pm.grabmutex);
    }

    else if (i == 7) //Grasp Gears
    {
      poseMe = curPose;
	  /*
      ulapi_mutex_take(pm.grabmutex);
      param=100;
      pm.demo->SetParameter("POSITION_FINGER_A", &param);
      param=130;
      pm.demo->SetParameter("POSITION_FINGER_B", &param);
      pm.demo->SetParameter("POSITION_FINGER_C", &param);
      pm.demo->SetTool (0.1);
      ulapi_mutex_give(pm.grabmutex);
	  */

      poseMe.z = 200;
//      ulapi_mutex_take(pm.grabmutex);
      if (arm.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
      }
//      ulapi_mutex_give(pm.grabmutex);
    }

    else if (i == 8) //Pre-Grasp Bottom
    {  
      poseMe = curPose;
	  /*
      ulapi_mutex_take(pm.grabmutex);
      param = 1;
      pm.demo->SetParameter("ADVANCED_CONTROL", &param);
      pm.demo->SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      pm.demo->SetParameter("SPEED_FINGER_A", &param);
      pm.demo->SetParameter("SPEED_FINGER_B", &param);
      pm.demo->SetParameter("SPEED_FINGER_C", &param);
      pm.demo->SetParameter("SPEED_SCISSOR", &param);
      param=100;
      pm.demo->SetParameter("FORCE_FINGER_A", &param);
      pm.demo->SetParameter("FORCE_FINGER_B", &param);
      pm.demo->SetParameter("FORCE_FINGER_C", &param);
      pm.demo->SetParameter("FORCE_SCISSOR", &param);
      param=190;
      pm.demo->SetParameter("POSITION_SCISSOR", &param);
      param=0;
      pm.demo->SetParameter("POSITION_FINGER_A", &param);
      param=30;
      pm.demo->SetParameter("POSITION_FINGER_B", &param);
      pm.demo->SetParameter("POSITION_FINGER_C", &param);
      pm.demo->SetTool (0.1);
      ulapi_mutex_give(pm.grabmutex);
	  */
       
    }

    else if (i == 9) //Grasp Bottom
    {
      poseMe = curPose;
	  /*
      ulapi_mutex_take(pm.grabmutex);
      param=100;
      pm.demo->SetParameter("POSITION_FINGER_A", &param);
      param=130;
      pm.demo->SetParameter("POSITION_FINGER_B", &param);
      pm.demo->SetParameter("POSITION_FINGER_C", &param);
      pm.demo->SetTool (0.1);
      ulapi_mutex_give(pm.grabmutex);
	  */
    }
    
    else if (i == 10) //Pre-Grasp Top
    {  
      poseMe = curPose;
	  /*
      ulapi_mutex_take(pm.grabmutex);
      param = 1;
      pm.demo->SetParameter("ADVANCED_CONTROL", &param);
      pm.demo->SetParameter("SCISSOR_CONTROL", &param);
      param=100;
      pm.demo->SetParameter("SPEED_FINGER_A", &param);
      pm.demo->SetParameter("SPEED_FINGER_B", &param);
      pm.demo->SetParameter("SPEED_FINGER_C", &param);
      pm.demo->SetParameter("SPEED_SCISSOR", &param);
      param=100;
      pm.demo->SetParameter("FORCE_FINGER_A", &param);
      pm.demo->SetParameter("FORCE_FINGER_B", &param);
      pm.demo->SetParameter("FORCE_FINGER_C", &param);
      pm.demo->SetParameter("FORCE_SCISSOR", &param);
      param=200;
      pm.demo->SetParameter("POSITION_SCISSOR", &param);
      param=0;
      pm.demo->SetParameter("POSITION_FINGER_A", &param);
      param=30;
      pm.demo->SetParameter("POSITION_FINGER_B", &param);
      pm.demo->SetParameter("POSITION_FINGER_C", &param);
      pm.demo->SetTool (0.1);
      ulapi_mutex_give(pm.grabmutex);
	  */
    }

    else if (i == 11) //Grasp Top
    {
      poseMe = curPose;
	  /*
      ulapi_mutex_take(pm.grabmutex);
      param=100;
      pm.demo->SetParameter("POSITION_FINGER_A", &param);
      param=130;
      pm.demo->SetParameter("POSITION_FINGER_B", &param);
      pm.demo->SetParameter("POSITION_FINGER_C", &param);
      pm.demo->SetTool (0.1);
      ulapi_mutex_give(pm.grabmutex);
	  */
    }

    else if (i == 12)  // Release
    {
//      ulapi_mutex_take(pm.grabmutex);
      arm.SetTool (0.9);
//      ulapi_mutex_give(pm.grabmutex);
    }

    else
    {
      cout << "Invalid entry" << endl;
    }

    cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Move To Perch, 4) Move To Approach, 5) Cartesian feedback, 6) Pre-Grasp Gear, 7) Grasp Gear, 8) PreGrasp Bottom, 9) Grasp Bottom, 10) Pregrasp Top, 11) Grasp Top, 12)Release -1) quit : ";
    cin >> i;
  } // while (i != -1)

#elif defined COGNEXDEMO
  CrpiRobot<CrpiDemoHack> demo("kuka_lwr.xml");
  passMe pm (&demo); //! State variable used to communicate with the two threads
  robotPose poseMe, curPose, hover, tray, placeOffset[5];
  bool poseDefined = false;
  bool connected = false;
  bool tool = false;
  int param, get, c;
  void *task;

  char inbuffer[1024];
  cout << " " << endl;

  //cout << "Connect to which address (xxx.xxx.xxx.xxx)? : ";
  //cin >> inbuffer;

  //clientID_ = ulapi_socket_get_client_id (456, inbuffer);
  pm.clientID_ = ulapi_socket_get_client_id (456, "169.254.152.200");
  ulapi_socket_set_nonblocking(pm.clientID_);

  cout << "connected to client " << pm.clientID_ << endl;

  for (c = 0; c < 5; ++c)
  {
    frameData.pa_hypOffset[c] = frameData.pickOffset[c].x = frameData.pickOffset[c].y = frameData.pickOffset[c].z = frameData.pickOffset[c].xrot = frameData.pickOffset[c].yrot = frameData.pickOffset[c].zrot = 0.0f;
    placeOffset[c].x = placeOffset[c].y = placeOffset[c].z = placeOffset[c].xrot = placeOffset[c].yrot = placeOffset[c].zrot = 0.0f;
  }

  ifstream infile ("poses.dat");
  infile >> hover.x >> hover.y >> hover.z >> hover.xrot >> hover.yrot >> hover.zrot;
  infile >> tray.x >> tray.y >> tray.zrot;

  infile.close();
  infile.open("PlaceOffsets.dat");

  for (c = 0; c < 5; ++c)
  {
    infile >> placeOffset[c].x >> placeOffset[c].y >> placeOffset[c].z >> placeOffset[c].zrot << placeOffset[c].yrot << placeOffset[c].xrot;
  }

  infile.close();
  infile.open("PickOffsets.dat");

  for (c = 0; c < 5; ++c)
  {
    infile >> frameData.pickOffset[c].x >> frameData.pickOffset[c].y >> frameData.pickOffset[c].z >> 
              frameData.pickOffset[c].zrot >> frameData.pickOffset[c].yrot >> frameData.pickOffset[c].xrot >>;
  }
  
  infile.close();
  infile.open ("Config.dat");
  infile >> frameData.xadd >> frameData.xmult;
  infile >> frameData.yadd >> frameData.ymult;
  infile >> frameData.zrotadd >> frameData.zrotmult;

  infile.close();

  task = ulapi_task_new();

  ulapi_task_start((ulapi_task_struct*)task, CognexThread, &pm, ulapi_prio_lowest(), 0);
  bool cleared;

  if (!poseDefined)
  {
    curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
    ulapi_mutex_take(pm.grabmutex);
    pm.demo->GetRobotPose(&curPose);
    ulapi_mutex_give(pm.grabmutex);
    poseDefined = true;
    cout << " >> X:" << curPose.x << " Y:" << curPose.y << " Z:" << curPose.z << " XR:" << curPose.xrot << " YR:" << curPose.yrot << " ZR:" << curPose.zrot << endl;
  }

  cout << "moving to hover" << endl;
  if (pm.demo->MoveTo (hover) == CANON_SUCCESS)
  {
    curPose = hover;
  }
  else
  {
    cout << "could not move robot" << endl;
  }

  for (c = 0; c < 5; ++c)
  {
    frameData.picked[c] = false;
  }

  cout << "1) Get Vision Data 2) Run Pick-n-Place -1) quit : ";
  cin >> i;
  while (i != -1)
  {
    cleared = false;

    if (i == 1)
    {
      //! Read data from Congnex server
      ulapi_mutex_take(pm.grabmutex);
      get = ulapi_socket_read (pm.clientID_, inbuffer, 1024);
      ulapi_mutex_give(pm.grabmutex);

      if (get > 0)
      {
#ifdef NOISY
		cout << inbuffer << endl;
#endif
#ifdef ALTTEXT
    removeCognexWhitespace (inbuffer);
#ifdef NOISY
    cout << inbuffer << endl;
#endif
#endif

        parseCognexFrame(inbuffer);
      }
    }
    else if (i == 2)
    {
      timer.waitUntil(5000);
      //! Read data from Congnex server
      ulapi_mutex_take(pm.grabmutex);
      get = ulapi_socket_read (pm.clientID_, inbuffer, 1024);
      ulapi_mutex_give(pm.grabmutex);

      if (get > 0)
      {

        for (c = 4; c >= 0; --c)
        {
          //! Read data from Congnex server
          ulapi_mutex_take(pm.grabmutex);
          get = ulapi_socket_read (pm.clientID_, inbuffer, 1024);
          ulapi_mutex_give(pm.grabmutex);

#ifdef ALTTEXT
          removeCognexWhitespace (inbuffer);
#endif
          parseCognexFrame (inbuffer);

          if (frameData.found[c] && !frameData.picked[c])
          {
            poseMe = curPose;
            poseMe.x = frameData.rp[c].x;
            poseMe.y = frameData.rp[c].y;
            poseMe.zrot = frameData.rp[c].zrot;

            if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
            {
              curPose = poseMe;

              if (c >= 0 && c < 3)
              {
                //! Configure gripper for pre-grasp
                poseMe = curPose;
                ulapi_mutex_take(pm.grabmutex);
                param = 1;
                pm.demo->SetParameter("ADVANCED_CONTROL", &param);
                pm.demo->SetParameter("SCISSOR_CONTROL", &param);
                param=100;
                pm.demo->SetParameter("SPEED_FINGER_A", &param);
                pm.demo->SetParameter("SPEED_FINGER_B", &param);
                pm.demo->SetParameter("SPEED_FINGER_C", &param);
                pm.demo->SetParameter("SPEED_SCISSOR", &param);
                param=100;
                pm.demo->SetParameter("FORCE_FINGER_A", &param);
                pm.demo->SetParameter("FORCE_FINGER_B", &param);
                pm.demo->SetParameter("FORCE_FINGER_C", &param);
                pm.demo->SetParameter("FORCE_SCISSOR", &param);
                param=200;
                pm.demo->SetParameter("POSITION_SCISSOR", &param);
                param=10;
                pm.demo->SetParameter("POSITION_FINGER_A", &param);
                param=30;
                pm.demo->SetParameter("POSITION_FINGER_B", &param);
                pm.demo->SetParameter("POSITION_FINGER_C", &param);
                pm.demo->SetTool (0.1);
                ulapi_mutex_give(pm.grabmutex);

                poseMe.z = 17.5;
                // Move to grasp position
                ulapi_mutex_take(pm.grabmutex);
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }
                ulapi_mutex_give(pm.grabmutex);

                //! Grasp part
                poseMe = curPose;
                ulapi_mutex_take(pm.grabmutex);
                param=100;
                pm.demo->SetParameter("POSITION_FINGER_A", &param);
                param=130;
                pm.demo->SetParameter("POSITION_FINGER_B", &param);
                pm.demo->SetParameter("POSITION_FINGER_C", &param);
                pm.demo->SetTool (0.1);
                ulapi_mutex_give(pm.grabmutex);

                //! Pick up part
                poseMe.z = 200;
                ulapi_mutex_take(pm.grabmutex);
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }
                ulapi_mutex_give(pm.grabmutex);

                //! Move to tray
                poseMe.x = tray.x + placeOffset[c].x;
                poseMe.y = tray.y + placeOffset[c].y;
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }

                //! Put part down
                poseMe.z = 45.0f;
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }

                //! Release part
                ulapi_mutex_take(pm.grabmutex);
                pm.demo->SetTool (0.9);
                ulapi_mutex_give(pm.grabmutex);

                //! Move away
                poseMe.z = 200.0f;
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }

                frameData.picked[c] = true;
              } // if (c >= 0 && c < 3)
              else
              {
                if (c == 3)
                {
                  //! Top cover
                  //! Pre-grasp top
                  poseMe = curPose;
                  ulapi_mutex_take(pm.grabmutex);
                  param = 1;
                  pm.demo->SetParameter("ADVANCED_CONTROL", &param);
                  pm.demo->SetParameter("SCISSOR_CONTROL", &param);
                  param=100;
                  pm.demo->SetParameter("SPEED_FINGER_A", &param);
                  pm.demo->SetParameter("SPEED_FINGER_B", &param);
                  pm.demo->SetParameter("SPEED_FINGER_C", &param);
                  pm.demo->SetParameter("SPEED_SCISSOR", &param);
                  param=100;
                  pm.demo->SetParameter("FORCE_FINGER_A", &param);
                  pm.demo->SetParameter("FORCE_FINGER_B", &param);
                  pm.demo->SetParameter("FORCE_FINGER_C", &param);
                  pm.demo->SetParameter("FORCE_SCISSOR", &param);
                  param=230;
                  pm.demo->SetParameter("POSITION_SCISSOR", &param);
                  param=0;
                  pm.demo->SetParameter("POSITION_FINGER_A", &param);
                  param=30;
                  pm.demo->SetParameter("POSITION_FINGER_B", &param);
                  pm.demo->SetParameter("POSITION_FINGER_C", &param);
                  pm.demo->SetTool (0.1);
                  ulapi_mutex_give(pm.grabmutex);

                  poseMe.z = 21.0f;
                  // Move to grasp position
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Grasp top
                  poseMe = curPose;
                  ulapi_mutex_take(pm.grabmutex);
                  param=100;
                  pm.demo->SetParameter("POSITION_FINGER_A", &param);
                  param=130;
                  pm.demo->SetParameter("POSITION_FINGER_B", &param);
                  pm.demo->SetParameter("POSITION_FINGER_C", &param);
                  pm.demo->SetTool (0.1);
                  ulapi_mutex_give(pm.grabmutex);

                  //! Pick up part
                  poseMe.z = 200;
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Move to tray
                  poseMe.x = tray.x + placeOffset[c].x;
                  poseMe.y = tray.y + placeOffset[c].y;
                  poseMe.zrot = tray.zrot + placeOffset[c].zrot;
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }

                  //! Put part down
                  poseMe.z = 45.0f;
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }

                  //! Release part
                  ulapi_mutex_take(pm.grabmutex);`
                  pm.demo->SetTool (0.9);
                  ulapi_mutex_give(pm.grabmutex);

                  //! Move away
                  poseMe.z = 200.0f;
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                } // if (c == 3)
                else if (c == 4)
                {
                  //! Bottom cover
                  //! Pre-grasp bottom
                  poseMe = curPose;
                  ulapi_mutex_take(pm.grabmutex);
                  param = 1;
                  pm.demo->SetParameter("ADVANCED_CONTROL", &param);
                  pm.demo->SetParameter("SCISSOR_CONTROL", &param);
                  param=100;
                  pm.demo->SetParameter("SPEED_FINGER_A", &param);
                  pm.demo->SetParameter("SPEED_FINGER_B", &param);
                  pm.demo->SetParameter("SPEED_FINGER_C", &param);
                  pm.demo->SetParameter("SPEED_SCISSOR", &param);
                  param=100;
                  pm.demo->SetParameter("FORCE_FINGER_A", &param);
                  pm.demo->SetParameter("FORCE_FINGER_B", &param);
                  pm.demo->SetParameter("FORCE_FINGER_C", &param);
                  pm.demo->SetParameter("FORCE_SCISSOR", &param);
                  param=190;
                  pm.demo->SetParameter("POSITION_SCISSOR", &param);
                  param=0;
                  pm.demo->SetParameter("POSITION_FINGER_A", &param);
                  param=30;
                  pm.demo->SetParameter("POSITION_FINGER_B", &param);
                  pm.demo->SetParameter("POSITION_FINGER_C", &param);
                  pm.demo->SetTool (0.1);
                  ulapi_mutex_give(pm.grabmutex);

                  poseMe.z = 25.0f;
                  //! Move to grasp position
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Grasp bottom
                  poseMe = curPose;
                  ulapi_mutex_take(pm.grabmutex);
                  param=100;
                  pm.demo->SetParameter("POSITION_FINGER_A", &param);
                  param=130;
                  pm.demo->SetParameter("POSITION_FINGER_B", &param);
                  pm.demo->SetParameter("POSITION_FINGER_C", &param);
                  pm.demo->SetTool (0.1);
                  ulapi_mutex_give(pm.grabmutex);

                  //! Pick up part
                  poseMe.z = 200;
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Move to tray
                  poseMe.x = tray.x + placeOffset[c].x;
                  poseMe.y = tray.y + placeOffset[c].y;
                  poseMe.zrot = tray.zrot + placeOffset[c].zrot;
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }

                  //! Put part down
                  poseMe.z = 45.0f;
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }

                  //! Release part
                  ulapi_mutex_take(pm.grabmutex);
                  pm.demo->SetTool (0.9);
                  ulapi_mutex_give(pm.grabmutex);

                  //! Move away
                  poseMe.z = 200.0f;
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                } // else if (c == 4)

                frameData.picked[c] = true;
              } // if (c >= 0 && c < 3) ... else
            } // if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
            else
            {
              cout << "cannot move" << endl;
            }
            cout << " >> X:" << curPose.x << " Y:" << curPose.y << " Z:" << curPose.z << " XR:" << curPose.xrot << " YR:" << curPose.yrot << " ZR:" << curPose.zrot << endl;
          } // if (found[c] && !picked[c])
        } //for (int c = 0; c < 3; ++c)

        if (pm.demo->MoveTo (hover) == CANON_SUCCESS)
        {
          curPose = hover;
        }

        for (c = 0; c < 5; ++c)
        {
          frameData.picked[c] = false;
        }
      } // if (get > 0)
      inbuffer[0] = '/0';

      //! Move back to hover position
      if (pm.demo->MoveTo (hover) == CANON_SUCCESS)
      {
        curPose = hover;
      }
    } // if (i == 1)
    
    cout << "1) Get Vision Data 2) Run Pick-n-Place -1) quit : ";
    cin >> i;
  } // while (i != -1)
#elif defined ROBOTDATADEMO
  //! Open COM port
  serialStruct serialData;
  int numlines = 5;
  int num = (5 + (6 * 11)) * numlines;
  char *inbuffer = new char[num];
  char *pch;
  ofstream datalogger("robotlog.csv");
  clock_t t1;

  datalogger << "ts,X,Y,Z,RZ,RY,RX,A1,A2,A3,A4,A5,A6,TA1,TA2,TA3,TA4,TA5,TA6,FX,FY,FZ,TRZ,TRY,TRX,EA1,TEA1,NULL,NULL,NULL,NULL" << endl;

  bool test;

  test = serialData.setBaud (57600);
  test &= serialData.setChannel (1);
  Network::serial *serial_ = new Network::serial();
  if (serial_->attach(serialData))
  {
    //! Connected
    cout << "connected to robot" << endl;
  }
  else
  {
    //! Failed to connect
    cout << "could not connect to robot" << endl;
  }

  while (true)
  {
    //! Read data
    if (serial_->getData (inbuffer, serialData, num))
    {
#ifdef SUPERNOISY
      cout << inbuffer << endl;
#endif
      t1 = clock();
      datalogger << t1 << ", ";
      for (int x = 0; x < numlines; ++x)
      {
        if (x == 0)
        {
          //! First time through, reset the buffer
          pch = strtok (inbuffer, ",\n "); //! Name
        }
        else
        {
          pch = strtok (NULL, ",\n ");
        }
#ifdef NOISY
        cout << pch << " : Data received";
#else
        cout << "." ;
#endif
                                     //!            POSE      AXES      TORQ        FORC
        pch = strtok (NULL, ",\n "); //! Value 1    X         A1        A1          X
        datalogger << pch << ", ";
        pch = strtok (NULL, ",\n "); //! Value 2    Y         A2        A2          Y
        datalogger << pch << ", ";
        pch = strtok (NULL, ",\n "); //! Value 3    Z         A3        A3          Z
        datalogger << pch << ", ";
        pch = strtok (NULL, ",\n "); //! Value 4    A         A4        A4          A
        datalogger << pch << ", ";
        pch = strtok (NULL, ",\n "); //! Value 5    B         A5        A5          B
        datalogger << pch << ", ";
        pch = strtok (NULL, ",\n "); //! Value 6    C         A6        A6          C
        datalogger << pch << ", ";
      }
      datalogger << endl;
    } // if (serial_->getData (inbuffer, serialData, 285))
    else
    {
#ifdef NOISY
      cout << "no data..." << endl;
#endif
    }
  }
#elif defined ROBOTSYNCDEMO
  CrpiRobot<CrpiKukaLWR> robarm("kuka_lwr.xml");

  passMe pm (&robarm); //! State variable used to communicate with the two threads
  
  robotPose hover;

  ulapi_integer client;
  char linein[67];
  int get, c;
  
  clock_t t1, t2;

  cout << "waiting for connection...";
  pm.clientID_ = ulapi_socket_get_server_id(5248);
  ulapi_socket_set_blocking(pm.clientID_);
  ofstream outfile ("datalog.dat");
  outfile << "X,Y,Z,XR,YR,ZR,T" << endl;

  //! Wait for remote application to connect
  client = ulapi_socket_get_connection_id(pm.clientID_);
  cout << " connected." << endl;

  while (true)
  {
    get = ulapi_socket_read (client, linein, 66);
    
#ifdef NOISY
    cout << get << " bytes read" << endl;
#endif
    if (get > 0)
    {
#ifdef NOISY
      cout << linein << endl;
#endif
      parseCommandPose (linein, &hover);
      t1 = clock();
      
      if (pm.robArm->MoveTo (hover) == CANON_SUCCESS)
      {
        t2 = clock() - t1;

        c = ulapi_socket_write(client, "1\n", 1);
        outfile << hover.x << "," << hover.y << "," << hover.z << "," << hover.xrot << "," << hover.yrot << "," << hover.zrot << "," << t2 << endl;
      }
      else
      {
        c = ulapi_socket_write(client, "0\n", 1);
        cout << "could not move robot" << endl;
      }
    }
  }
#elif defined NEWCOGNEXDEMO
  CrpiRobot<CrpiDemoHack> demo("kuka_lwr.xml");
  passMe pm (&demo); //! State variable used to communicate with the two threads
  robotPose poseMe, curPose, hover, tray, rt_placeOffset[5], pa_placeOffset[5];
  bool poseDefined = false;
  bool connected = false;
  bool tool = false;
  int param, get, c;
  void *task;
  int oldgripper = 0;
  char curtool[32];
  userobotiq = true;

  char inbuffer[1024];
  cout << " " << endl;

  //cout << "Connect to which address (xxx.xxx.xxx.xxx)? : ";
  //cin >> inbuffer;

  //clientID_ = ulapi_socket_get_client_id (456, inbuffer);
  pm.clientID_ = ulapi_socket_get_client_id (123, "169.254.152.200");
  ulapi_socket_set_nonblocking(pm.clientID_);

  cout << "connected to Cognex client " << pm.clientID_ << endl;

  for (c = 0; c < 5; ++c)
  {
    frameData.rt_hypOffset[c] = frameData.rt_pickOffset[c].x = frameData.rt_pickOffset[c].y =
                                frameData.rt_pickOffset[c].z = frameData.rt_pickOffset[c].xrot =
                                frameData.rt_pickOffset[c].yrot = frameData.rt_pickOffset[c].zrot = 0.0f;
    frameData.pa_hypOffset[c] = frameData.pa_pickOffset[c].x = frameData.pa_pickOffset[c].y =
                                frameData.pa_pickOffset[c].z = frameData.pa_pickOffset[c].xrot =
                                frameData.pa_pickOffset[c].yrot = frameData.pa_pickOffset[c].zrot = 0.0f;
    rt_placeOffset[c].x = rt_placeOffset[c].y = rt_placeOffset[c].z = rt_placeOffset[c].xrot = rt_placeOffset[c].yrot = rt_placeOffset[c].zrot = 0.0f;
    pa_placeOffset[c].x = pa_placeOffset[c].y = pa_placeOffset[c].z = pa_placeOffset[c].xrot = pa_placeOffset[c].yrot = pa_placeOffset[c].zrot = 0.0f;
  }

  ifstream infile ("poses.dat");
  infile >> hover.x >> hover.y >> hover.z >> hover.xrot >> hover.yrot >> hover.zrot;
  infile >> tray.x >> tray.y >> tray.zrot;

  infile.close();
  infile.open("PlaceOffsets.dat");

  for (c = 0; c < 5; ++c)
  {
    infile >> rt_placeOffset[c].x >> rt_placeOffset[c].y >> rt_placeOffset[c].z >> rt_placeOffset[c].zrot >> rt_placeOffset[c].yrot >> rt_placeOffset[c].xrot;
  }
  for (c = 0; c < 5; ++c)
  {
    infile >> pa_placeOffset[c].x >> pa_placeOffset[c].y >> pa_placeOffset[c].z >> pa_placeOffset[c].zrot >> pa_placeOffset[c].yrot >> pa_placeOffset[c].xrot;
  }

  infile.close();
  infile.open("PickOffsets.dat");

  for (c = 0; c < 5; ++c)
  {
    infile >> frameData.rt_pickOffset[c].x >> frameData.rt_pickOffset[c].y >> frameData.rt_pickOffset[c].z >>
              frameData.rt_pickOffset[c].zrot >> frameData.rt_pickOffset[c].yrot >> frameData.rt_pickOffset[c].xrot >>
              frameData.rt_hypOffset[c];
  }
  for (c = 0; c < 5; ++c)
  {
    infile >> frameData.pa_pickOffset[c].x >> frameData.pa_pickOffset[c].y >> frameData.pa_pickOffset[c].z >>
              frameData.pa_pickOffset[c].zrot >> frameData.pa_pickOffset[c].yrot >> frameData.pa_pickOffset[c].xrot >>
              frameData.pa_hypOffset[c];
  }
  
  infile.close();
  infile.open ("Config.dat");
  infile >> frameData.xadd >> frameData.xmult;
  infile >> frameData.yadd >> frameData.ymult;
  infile >> frameData.zrotadd >> frameData.zrotmult;

  infile.close();

  task = ulapi_task_new();

  ulapi_task_start((ulapi_task_struct*)task, CognexThread, &pm, ulapi_prio_lowest(), 0);
  bool cleared;

  if (userobotiq)
  {
    strcpy(curtool, "gripper_gear");
  }
  else
  {
    strcpy(curtool, "gripper_parallel");
  }
  ulapi_mutex_take(pm.grabmutex);
  pm.demo->Couple(curtool);
  ulapi_mutex_give(pm.grabmutex);

  if (!poseDefined)
  {
    curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
    ulapi_mutex_take(pm.grabmutex);
    pm.demo->GetRobotPose(&curPose);
    ulapi_mutex_give(pm.grabmutex);
    poseDefined = true;
    cout << " >> X:" << curPose.x << " Y:" << curPose.y << " Z:" << curPose.z << " XR:" << curPose.xrot << " YR:" << curPose.yrot << " ZR:" << curPose.zrot << endl;
  }

  cout << "moving to hover" << endl;
  ulapi_mutex_take(pm.grabmutex);
  if (pm.demo->MoveTo (hover) == CANON_SUCCESS)
  {
    curPose = hover;
  }
  else
  {
    cout << "could not move robot" << endl;
  }
  ulapi_mutex_give(pm.grabmutex);

  for (c = 0; c < 5; ++c)
  {
    frameData.picked[c] = false;
  }

  cout << "1) Get Vision Data 2) Run Pick-n-Place 3) Set Tool -1) quit : ";
  cin >> i;
  while (i != -1)
  {
    cleared = false;

    if (i == 1)
    {
      //! Read data from Congnex server
      ulapi_mutex_take(pm.grabmutex);
      get = ulapi_socket_read (pm.clientID_, inbuffer, 1024);
      ulapi_mutex_give(pm.grabmutex);

      if (get > 0)
      {
#ifdef NOISY
		cout << inbuffer << endl;
#endif
#ifdef ALTTEXT
    removeCognexWhitespace (inbuffer);
#ifdef NOISY
    cout << inbuffer << endl;
#endif
#endif

        parseCognexFrame(inbuffer);
      }
    }
    else if (i == 2)
    {
      //! Read data from Congnex server
      ulapi_mutex_take(pm.grabmutex);
      get = ulapi_socket_read (pm.clientID_, inbuffer, 1024);
      ulapi_mutex_give(pm.grabmutex);

      if (get > 0)
      {

        for (c = 4; c >= 0; --c)
        {
          //! Read data from Congnex server
          ulapi_mutex_take(pm.grabmutex);
          get = ulapi_socket_read (pm.clientID_, inbuffer, 1024);
          ulapi_mutex_give(pm.grabmutex);

#ifdef ALTTEXT
          removeCognexWhitespace (inbuffer);
#endif
          parseCognexFrame (inbuffer);

          if (frameData.found[c] && !frameData.picked[c])
          {
            poseMe = curPose;
            poseMe.x = frameData.rp[c].x;
            poseMe.y = frameData.rp[c].y;
            poseMe.zrot = frameData.rp[c].zrot;
            ulapi_mutex_take(pm.grabmutex);
            if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
            {
              ulapi_mutex_give(pm.grabmutex);
              curPose = poseMe;

              if (c >= 0 && c < 3)
              {
                //! Configure gripper for pre-grasp
                poseMe = curPose;
                ulapi_mutex_take(pm.grabmutex);
                if (userobotiq)
                {
                  pm.demo->Decouple(curtool);
                  strcpy(curtool, "gripper_gear");
                  pm.demo->Couple(curtool);
                }
                //! Open tool to grasp gear
                pm.demo->SetTool (0.9);
                ulapi_mutex_give(pm.grabmutex);

                poseMe.z = (userobotiq ? frameData.rt_pickOffset[c].z : frameData.pa_pickOffset[c].z);

                // Move to grasp position
                ulapi_mutex_take(pm.grabmutex);
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }
                ulapi_mutex_give(pm.grabmutex);

                //! Grasp part
                poseMe = curPose;
                ulapi_mutex_take(pm.grabmutex);
                pm.demo->SetTool (0.1);
                ulapi_mutex_give(pm.grabmutex);

                //! Pick up part
                poseMe.z += 190.0f;
                ulapi_mutex_take(pm.grabmutex);
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }
                ulapi_mutex_give(pm.grabmutex);

                //! Move to tray
                poseMe.x = tray.x + (userobotiq ? rt_placeOffset[c].x : pa_placeOffset[c].x);
                poseMe.y = tray.y + (userobotiq ? rt_placeOffset[c].y : pa_placeOffset[c].y);
                ulapi_mutex_take(pm.grabmutex);
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }
                ulapi_mutex_give(pm.grabmutex);

                //! Put part down
                poseMe.z = (userobotiq ? rt_placeOffset[c].z : pa_placeOffset[c].z);
                ulapi_mutex_take(pm.grabmutex);
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }
                ulapi_mutex_give(pm.grabmutex);

                //! Release part
                ulapi_mutex_take(pm.grabmutex);
                pm.demo->SetTool (0.9);
                ulapi_mutex_give(pm.grabmutex);

                //! Move away
                poseMe.z += 150.0f;
                ulapi_mutex_take(pm.grabmutex);
                if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                {
                  curPose = poseMe;
                }
                ulapi_mutex_give(pm.grabmutex);

                frameData.picked[c] = true;
              } // if (c >= 0 && c < 3)
              else
              {
                if (c == 3)
                {
                  //! Top cover
                  //! Pre-grasp top
                  poseMe = curPose;
                  ulapi_mutex_take(pm.grabmutex);
                  if (userobotiq)
                  {
                    pm.demo->Decouple(curtool);
                    strcpy(curtool, "gripper_top_cover");
                    pm.demo->Couple(curtool);
                  }
                  else
                  {
                    //! parallel gripper grasps part from inside hole
                    //pm.demo->SetTool(0.1);
                  }
                  pm.demo->SetTool(0.9);
                  ulapi_mutex_give(pm.grabmutex);
                  
                  poseMe.z = (userobotiq ? frameData.rt_pickOffset[c].z : frameData.pa_pickOffset[c].z);
 
                  // Move to grasp position
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Grasp top
                  poseMe = curPose;
                  ulapi_mutex_take(pm.grabmutex);
                  pm.demo->SetTool (0.1);
                  ulapi_mutex_give(pm.grabmutex);

                  //! Pick up part
                  poseMe.z += 190.0f;
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Move to tray
                  poseMe.x = tray.x + (userobotiq ? rt_placeOffset[c].x : pa_placeOffset[c].x);
                  poseMe.y = tray.y + (userobotiq ? rt_placeOffset[c].y : pa_placeOffset[c].y);
                  poseMe.zrot = tray.zrot + (userobotiq ? rt_placeOffset[c].zrot : pa_placeOffset[c].zrot);
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Put part down
                  poseMe.z = (userobotiq ? rt_placeOffset[c].z : pa_placeOffset[c].z);
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Release part
                  ulapi_mutex_take(pm.grabmutex);
                  pm.demo->SetTool (0.9);
                  ulapi_mutex_give(pm.grabmutex);

                  //! Move away
                  poseMe.z += 150.0f;
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);
                } // if (c == 3)
                else if (c == 4)
                {
                  //! Bottom cover
                  //! Pre-grasp bottom
                  poseMe = curPose;
                  ulapi_mutex_take(pm.grabmutex);
                  if (userobotiq)
                  {
                    pm.demo->Decouple(curtool);
                    strcpy(curtool, "gripper_bottom_cover");
                    pm.demo->Couple(curtool);
                  }
                  pm.demo->SetTool (0.9);
                  ulapi_mutex_give(pm.grabmutex);

                  poseMe.z = (userobotiq ? frameData.rt_pickOffset[c].z : frameData.pa_pickOffset[c].z);

                  //! Move to grasp position
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Grasp bottom
                  poseMe = curPose;
                  ulapi_mutex_take(pm.grabmutex);
                  pm.demo->SetTool (0.1);
                  ulapi_mutex_give(pm.grabmutex);

                  //! Pick up part
                  poseMe.z += 190;
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Move to tray
                  poseMe.x = tray.x + (userobotiq ? rt_placeOffset[c].x : pa_placeOffset[c].x);
                  poseMe.y = tray.y + (userobotiq ? rt_placeOffset[c].y : pa_placeOffset[c].y);
                  poseMe.zrot = tray.zrot + (userobotiq ? rt_placeOffset[c].zrot : pa_placeOffset[c].zrot);
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Put part down
                  poseMe.z = (userobotiq ? rt_placeOffset[c].z : pa_placeOffset[c].z);
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);

                  //! Release part
                  ulapi_mutex_take(pm.grabmutex);
                  pm.demo->SetTool (0.9);
                  ulapi_mutex_give(pm.grabmutex);

                  //! Move away
                  poseMe.z += 150.0f;
                  ulapi_mutex_take(pm.grabmutex);
                  if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
                  {
                    curPose = poseMe;
                  }
                  ulapi_mutex_give(pm.grabmutex);
                } // else if (c == 4)

                frameData.picked[c] = true;
              } // if (c >= 0 && c < 3) ... else
            } // if (pm.demo->MoveTo (poseMe) == CANON_SUCCESS)
            else
            {
              ulapi_mutex_give(pm.grabmutex);
              cout << "cannot move" << endl;
            }
            cout << " >> X:" << curPose.x << " Y:" << curPose.y << " Z:" << curPose.z << " XR:" << curPose.xrot << " YR:" << curPose.yrot << " ZR:" << curPose.zrot << endl;
          } // if (found[c] && !picked[c])
        } //for (int c = 0; c < 3; ++c)

        if (pm.demo->MoveTo (hover) == CANON_SUCCESS)
        {
          curPose = hover;
        }

        for (c = 0; c < 5; ++c)
        {
          frameData.picked[c] = false;
        }
      } // if (get > 0)
      inbuffer[0] = '/0';

      //! Move back to hover position
      if (pm.demo->MoveTo (hover) == CANON_SUCCESS)
      {
        curPose = hover;
      }
    } // if (i == 1) else if (i == 2)
    else if (i == 3)
    {
      cout << "1) Robotiq 2) Parallel : ";
      cin >> i;

      ulapi_mutex_take(pm.grabmutex);
      pm.demo->Decouple(curtool);
      if (i == 1)
      {
        userobotiq = true;
        strcpy(curtool, "gripper_gear");
        pm.demo->Couple(curtool);
      }
      else if (i == 2)
      {
        userobotiq = false;
        strcpy(curtool, "gripper_parallel");
        pm.demo->Couple(curtool);
      }
      else
      {
        cout << "Unknown entry, no change made" << endl;
      }
      ulapi_mutex_give(pm.grabmutex);
    } // if (i == 1) ... else if (i == 3)
    
    cout << "1) Get Vision Data 2) Run Pick-n-Place 3) Set Tool -1) quit : ";
    cin >> i;
  } // while (i != -1)
#elif defined TOOLCHANGERDEMO
  CrpiRobot<CrpiDemoHack> demo("kuka_lwr.xml");
  passMe pm (&demo); //! State variable used to communicate with the two threads
  int c = 0;
  bool toolclosed = false;
  char curtool[32];
  double val;

  while (i >= 0)
  {
    if (c == 0)
    {
      cout << "Select new tool: 1) parallel, 2) gear, 3) top_cover, 4) bottom_cover: ";
      cin >> c;
      if (c < 1 || c > 5)
      {
        cout << "Invalid tool selection." << endl;
        c = 0;
      }
      else
      {
        ulapi_mutex_take(pm.grabmutex);
        pm.demo->Decouple(curtool);
        toolclosed = false;
        switch (c)
        {
        case 1:
          strcpy(curtool, "gripper_parallel");
          break;
        case 2:
          strcpy(curtool, "gripper_gear");
          break;
        case 3:
          strcpy(curtool, "gripper_top_cover");
          break;
        case 4:
          strcpy(curtool, "gripper_bottom_cover");
          break;
        default:
          strcpy(curtool, "nothing");
          c = 0;
          break;
        }
        pm.demo->Couple(curtool);
        ulapi_mutex_give(pm.grabmutex);
      }
    } // if (c == 0)
    else if (i > 0)
    {
      if (i == 1)
      {
        c = 0;
        continue;
      }
      else
      {
        val = (i == 2) ? 0.9 : 0.1;
        ulapi_mutex_take(pm.grabmutex);
        pm.demo->SetTool(val);
        ulapi_mutex_give(pm.grabmutex);
      }
    }

    cout << "1) Change tool, 2) open tool, 3) close tool -1) quit : ";
    cin >> i;

  } while (i >= 0)

#elif defined ASSEMBLYDEMO
  //CrpiRobot<robType> arm("universal_ur10_right.xml");
  CrpiRobot<robType> arm("kuka_lwr.xml");
  Assembly asbly;
  char curtool[32];
  bool toolopen = true;

  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->SetAngleUnits("degree");
  pm.robArm->SetLengthUnits("mm");

  //strcpy(curtool, "gripper_gear");
  //strcpy(curtool, "gripper_parallel");
  strcpy(curtool, "schunk_hand");
  pm.robArm->Couple(curtool);

  asbly.AddSearchSpiral(5, 50.0f, 45.0f);
  //asbly.AddSearchLinear(200.0, 0.0f, 0.0f, 10.0f);
  //asbly.AddSearchHop(20.0, 10.0f);
  asbly.AddTerminatorTimer(CANON_FAILURE, 60.0);

  robotPose poseMe, curPose;
  robotIO io;
  bool poseDefined = false;
  bool tool = false;
  int param;
  int counter; 
  ofstream out("sprial.csv");

  double theta = 0.0f;
  matrix pin(3,1), pout(3,1);

  cout << "int:  " << sizeof(int) << " double:  " << sizeof(double) << endl;

  cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Move +X 20 mm, 4) Move -X 20 mm, 5) Move +Y 20 mm, 6) Move -Y 20 mm, 7) Get Pose, 8) Gripper, 9) Test Spiral -1) quit : ";
  cin >> i;
  AssemblyTimer asblytimer;
  double tim;
  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      ulapi_mutex_give(pm.grabmutex);

      poseDefined = true;
    }

    poseMe = curPose;

    switch (i)
    {
    case 1:
      poseMe.z += 20.0f;
      break;
    case 2:
      poseMe.z += -20.0f;
      break;
    case 3:
      poseMe.x += 20.0f;
      break;
    case 4:
      poseMe.x += -20.0f;
      break;
    case 5:
      poseMe.y += 20.0f;
      break;
    case 6:
      poseMe.y += -20.0f;
      break;
    case 7:
      poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&poseMe);
      ulapi_mutex_give(pm.grabmutex);

      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << endl;
      break;
    case 8:
      /*
      double dval;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotIO(&io);
      ulapi_mutex_give(pm.grabmutex);
      cout << " >> DIO: ";
      for (i = 0; i < CRPI_IO_MAX; ++i)
      {
        cout << io.dio[i];
      }
      cout << endl;
      */

      /*
      //! Force control tesl
      poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->MoveAttractor(poseMe);
      ulapi_mutex_give(pm.grabmutex);
      */

      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->SetTool((toolopen ? 0.0 : 1.0));
      ulapi_mutex_give(pm.grabmutex);
      toolopen = !toolopen;
      break;
    case 9:
      //! TODO
      counter = 0;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      pm.robArm->GetRobotIO(&io);
      ulapi_mutex_give(pm.grabmutex);

      tim = asblytimer.startTimer();
      while (asbly.RunAssemblyStep (counter++, curPose, poseMe, io) == CANON_RUNNING)
      {
        cout << "X: " << poseMe.x << " Y: " << poseMe.y << " Z: " << poseMe.z << endl;
        out << poseMe.x << ", " << poseMe.y << endl;
        ulapi_mutex_take(pm.grabmutex);
        pm.robArm->MoveTo(poseMe);
        ulapi_mutex_give(pm.grabmutex);
        timer.waitUntil(100);
      }
      tim = asblytimer.timeElapsed();
      asblytimer.stopTimer();
      cout << "timer:  " << tim << endl;
      break;
    default:
      break;
    }

    if (i > 0 && i < 7)
    {
      ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
        cout << "curPose=" << curPose.z << endl;
      }
      ulapi_mutex_give(pm.grabmutex);
    }

    cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Move +X 20 mm, 4) Move -X 20 mm, 5) Move +Y 20 mm, 6) Move -Y 20 mm, 7) Get Pose, 8) Gripper, 9) Test Spiral -1) quit : ";
    cin >> i;
  } // while (i != -1)

#elif defined ASSEMBLYTEST1
  //CrpiRobot<robType> arm("universal_ur10_right.xml");
  CrpiRobot<robType> arm("kuka_lwr.xml");
  Assembly asbly;
  char curtool[32];
  bool toolopen = true;

  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->SetAngleUnits("degree");
  pm.robArm->SetLengthUnits("mm");

  strcpy(curtool, "gripper_parallel");

  pm.robArm->Couple(curtool);
  //
  asbly.AddSearchSpiral(5, 30.0f, 150.0f);
  asbly.AddSearchConstOffset(0.0, 0.0, -23.0f);
  //asbly.AddSearchLinear(0.0, 0.0f, -1.0f, 10.0f);
  //asbly.AddSearchHop(20.0, 10.0f);
  asbly.AddTerminatorTimer(CANON_FAILURE, 60.0);
  asbly.AddTerminatorDistance(CANON_SUCCESS,-1,-1,10,-1);

  robotPose poseMe, curPose;
  robotIO io;
  bool poseDefined = false;
  bool tool = false;
  int param;
  int counter; 
  ofstream out("sprial.csv");

  double theta = 0.0f;
  matrix pin(3,1), pout(3,1);

  cout << "int:  " << sizeof(int) << " double:  " << sizeof(double) << endl;

  cout << "1) Hole 1 Approach, 2) Hole 1, 3) Hole 2 Approach, 4) Hole 2 , 5) Gripper, 6) Get Pose, 7) Test Spiral, -1) quit : ";
  cin >> i;
  AssemblyTimer asblytimer;
  double tim;
  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      ulapi_mutex_give(pm.grabmutex);
      poseDefined = true;
    }

    poseMe = curPose;

    switch (i)
    {
    case 1:
      poseMe.x = -161.84f;
      poseMe.y = -613.26f;
      poseMe.z = 100.0f;
      poseMe.xrot = 180.0f;
      poseMe.yrot = 0.0f;
      poseMe.zrot = 0.0f;
      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << endl;
      break;
    case 2:
	  poseMe.x = -163.84f;
      poseMe.y = -613.26f;
      poseMe.z = 25.0f;
      poseMe.xrot = 180.0f;
      poseMe.yrot = 0.0f;
      poseMe.zrot = 0.0f;
      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << endl;
      break;
    case 3:
      poseMe.x = 138.44f;
      poseMe.y = -613.07f;
      poseMe.z = 100.0f;
      poseMe.xrot = 180.0f;
      poseMe.yrot = 0.0f;
      poseMe.zrot = 0.0f;
      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << endl;
	  break;
    case 4:
      poseMe.x = 138.44f;
      poseMe.y = -613.07f;
      poseMe.z = 25.0f;
      poseMe.xrot = 180.0f;
      poseMe.yrot = 0.0f;
      poseMe.zrot = 0.0f;
      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << endl;
      break;
    case 5:
      /*
      double dval;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotIO(&io);
      ulapi_mutex_give(pm.grabmutex);
      cout << " >> DIO: ";
      for (i = 0; i < CRPI_IO_MAX; ++i)
      {
        cout << io.dio[i];
      }
      cout << endl;
      */

      /*
      //! Force control tesl
      poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->MoveAttractor(poseMe);
      ulapi_mutex_give(pm.grabmutex);
      */

      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->SetTool((toolopen ? 0.0 : 1.0));
      ulapi_mutex_give(pm.grabmutex);
      toolopen = !toolopen;
      break;

    case 6:
      poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&poseMe);
      ulapi_mutex_give(pm.grabmutex);

      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << endl;
      break;

    
    case 7:
      //! TODO
      counter = 0;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      pm.robArm->GetRobotIO(&io);
      ulapi_mutex_give(pm.grabmutex);

      tim = asblytimer.startTimer();
      while (asbly.RunAssemblyStep (counter++, curPose, poseMe, io) == CANON_RUNNING)
      {
        cout << "cur X: " << curPose.x << " cur Y: " << curPose.y << " cur Z: " << curPose.z << endl;
        cout << "tar X: " << poseMe.x << " tar Y: " << poseMe.y << " tar Z: " << poseMe.z << endl << endl;
        out << poseMe.x << ", " << poseMe.y << endl;
        ulapi_mutex_take(pm.grabmutex);
        pm.robArm->MoveAttractor(poseMe);
		pm.robArm->GetRobotPose(&curPose);
        pm.robArm->GetRobotIO(&io);
        ulapi_mutex_give(pm.grabmutex);
        timer.waitUntil(100);
      }
      tim = asblytimer.timeElapsed();
      asblytimer.stopTimer();
      cout << "timer:  " << tim << endl;
      break;
    default:
      break;
    }

    if (i > 0 && i < 5)
    {
      ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
        cout << "curPose=" << curPose.z << endl;
      }
      ulapi_mutex_give(pm.grabmutex);
    }

    cout << "1) Hole 1 Approach, 2) Hole 1, 3) Hole 2 Approach, 4) Hole 2 , 5) Gripper, 6) Get Pose, 7) Test Spiral, -1) quit : ";
    cin >> i;
  } // while (i != -1)

#elif defined PEGTEST
  CrpiRobot<robType> arm("kuka_lwr.xml");
  Assembly asbly;
  char curtool[32];
  bool toolopen = true;
 
  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->SetAngleUnits("degree");
  pm.robArm->SetLengthUnits("mm");

  strcpy(curtool, "gripper_parallel");

  pm.robArm->Couple(curtool);
  
  asbly.AddSearchSpiral(10, 20.0f, 150.0f);
  asbly.AddSearchConstOffset(0.0, 0.0, -10.0f);//23 10 5
  asbly.AddTerminatorTimer(CANON_FAILURE, 60.0);
  asbly.AddTerminatorDistance(CANON_SUCCESS,-1,-1,5,-1);//10 5 3

  robotPose poseMe, curPose, offsetPose;

  robotPose hole_1, hole_2;

  robotIO io;
  bool poseDefined = false;
  bool tool = false;
  int param;
  int counter; 
  

  double theta = 0.0f;
  matrix pin(3,1), pout(3,1);

  // << "int:  " << sizeof(int) << " double:  " << sizeof(double) << endl;

  cout << "1) Run Spiral Insertion, 2) Tool, -1) quit : ";
  cin >> i;
  AssemblyTimer asblytimer;
  double tim;
  CanonReturn peg_in_hole = CANON_RUNNING;

  int const insertions = 150;
  char delim;
  double visErrorXYR[6][insertions];
  ifstream infile ("perception_spoof_1mm.csv");
  ofstream results("percep_spoof_1mm_kuka.csv");
  
 /* // hole 1 nominal position - Table Front
  hole_1.x = -161.84f;
  hole_1.y = -613.26f;
  hole_1.z = 25.0f;
  hole_1.xrot = 180.0f;
  hole_1.yrot = 0.0f;
  hole_1.zrot = 0.0f;

  // hole 2 nominal postion - Table Front
  hole_2.x = 138.44f;
  hole_2.y = -613.07f;
  hole_2.z = 25.0f;
  hole_2.xrot = 180.0f;
  hole_2.yrot = 0.0f;
  hole_2.zrot = 0.0f;*/

  hole_1.x = -588.00f;
  hole_1.y = 33.71f;
  hole_1.z = 25.0f;
  hole_1.xrot = 180.0f;
  hole_1.yrot = 0.0f;
  hole_1.zrot = 180.0f;

  hole_2.x = -588.35f;
  hole_2.y = -265.34f;
  hole_2.z = 25.0f;
  hole_2.xrot = 180.0f;
  hole_2.yrot = 0.0f;
  hole_2.zrot = 180.0f;

  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      ulapi_mutex_give(pm.grabmutex);
      poseDefined = true;
    }

    poseMe = curPose;

    switch (i)
    {
      case 1:
	  
        for(int j = 1; j <= insertions; j++)
	    {
		  offsetPose.x = offsetPose.y = offsetPose.z = offsetPose.xrot  = offsetPose.yrot = offsetPose.zrot = 0.0f;

		  // Grab data from file
		  infile >> offsetPose.x >> delim >> offsetPose.y >> delim >> offsetPose.z >> delim >> offsetPose.xrot >> delim >> offsetPose.yrot >> delim >> offsetPose.zrot;
		  //cout << "offsets: " << offsetPose.x << ", " << offsetPose.y << endl;
	      // Alternate hole 1 and 2 aquire peg

		  if (peg_in_hole == CANON_SUCCESS || j == 1)
		  {
		    if (j % 2 > 0) {poseMe = hole_1;}
		    else {poseMe = hole_2;}

  		  // Hole approach++
        poseMe.z = 50.0f;
        ulapi_mutex_take(pm.grabmutex);
        if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS){ curPose = poseMe; }
        ulapi_mutex_give(pm.grabmutex);
        //cout << poseMe.x << " " << poseMe.y << " " << poseMe.z;
		
		
        // Open Gripper
        ulapi_mutex_take(pm.grabmutex);
        pm.robArm->SetTool(1.0);
        ulapi_mutex_give(pm.grabmutex);
		    //cout << poseMe.x << " " << poseMe.y << " " << poseMe.z;
		    timer.waitUntil(100);

		    // peg location
		    poseMe.z = 12.0f;
		    ulapi_mutex_take(pm.grabmutex);
        if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS){ curPose = poseMe; }
        ulapi_mutex_give(pm.grabmutex);
		    //cout << poseMe.x << " " << poseMe.y << " " << poseMe.z;
		  

		    // Close Gripper
        ulapi_mutex_take(pm.grabmutex);
        pm.robArm->SetTool(0.0);
        ulapi_mutex_give(pm.grabmutex);
		    tim = asblytimer.startTimer();
		    //cout << poseMe.x << " " << poseMe.y << " " << poseMe.z;
		    timer.waitUntil(100);

		    // Hole 1 retract
		    poseMe.z = 50.0f;
		    ulapi_mutex_take(pm.grabmutex);
        if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS){ curPose = poseMe; }
        ulapi_mutex_give(pm.grabmutex);
		  }
		  // Alternate hole 1 and 2 insertions
		  if (j % 2 > 0) {poseMe = hole_2;}
		  else {poseMe = hole_1;}

		  //apply error and retract to nominal poses
		  poseMe.x = poseMe.x + offsetPose.x;
	    poseMe.y = poseMe.y + offsetPose.y;
		  poseMe.z = 50.0f;	  

		  //move to approach pose
		  ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
      }
      ulapi_mutex_give(pm.grabmutex);

		  //move to start
		  poseMe.z = 32.0f;

		  ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
      }
      ulapi_mutex_give(pm.grabmutex);

      counter = 0;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      pm.robArm->GetRobotIO(&io);
      ulapi_mutex_give(pm.grabmutex);
		  
		  peg_in_hole = asbly.RunAssemblyStep (counter++, curPose, poseMe, io);
      while (peg_in_hole == CANON_RUNNING)
      {

        ulapi_mutex_take(pm.grabmutex);
        pm.robArm->MoveAttractor(poseMe);
        pm.robArm->GetRobotPose(&curPose);
        pm.robArm->GetRobotIO(&io);
        ulapi_mutex_give(pm.grabmutex);
        peg_in_hole = asbly.RunAssemblyStep (counter++, curPose, poseMe, io);
      }
		  if (peg_in_hole == CANON_SUCCESS)
		  {
			  poseMe.z = 12.0f;
			  ulapi_mutex_take(pm.grabmutex);
        if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS)
        {
          curPose = poseMe;
        }
        ulapi_mutex_give(pm.grabmutex);

        // Open Gripper
        ulapi_mutex_take(pm.grabmutex);
        pm.robArm->SetTool(1.0);
        ulapi_mutex_give(pm.grabmutex);
        timer.waitUntil(100);
		  }
		  tim = asblytimer.timeElapsed();
      asblytimer.stopTimer();
      cout << j << " " << tim << " " << offsetPose.x << " " << offsetPose.y << " " << endl;
		  cout << poseMe.x << " " << poseMe.y << " " << poseMe.z << endl;
		  // write to file
		  results << tim << endl;

		  //retract from current pos
		  poseMe = curPose;
		  poseMe.z += 50.0f;
		
		  ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
      }
      ulapi_mutex_give(pm.grabmutex);
		
	  	}
  	    infile.close();
	      results.close();
        break;
	  case 2:
		  // Open Gripper
		  ulapi_mutex_take(pm.grabmutex);
      pm.robArm->SetTool(1.0);
      ulapi_mutex_give(pm.grabmutex);
		  break;
	  case 3:
		  // Close Gripper
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->SetTool(0.0);
      ulapi_mutex_give(pm.grabmutex);
		break;
      default:
        break;
	  }

    cout << "1) Run Spiral Insertion, 2) Tool, -1) quit : ";
    cin >> i;
  } // while (i != -1)

#elif defined MAPVISERR

  CrpiRobot<robType> arm("kuka_lwr.xml");
  Assembly asbly;
  char curtool[32];

  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->SetAngleUnits("degree");
  pm.robArm->SetLengthUnits("mm");

  strcpy(curtool, "gripper_parallel");

  pm.robArm->Couple(curtool);
  
  robotPose poseMe, curPose, calPose;

  robotIO io;
  bool poseDefined = false;
  int pauseTime, testPositions, testRepetitions;
  ifstream infile ("Perception_DOE.csv");
  char delim;
  double testXY[2][31];

  testPositions = 31;
  testRepetitions = 1;
  pauseTime = 5000;

  // Read in datafile
 
  for(int k = 0; k < testPositions; k++)
   {
      infile >> testXY[0][k] >> delim >> testXY[1][k];
      //cout << "X: " << testXY[0][k] << "   Y: " << testXY[1][k] << endl;
	  //timer.waitUntil(500);
   }
  
  cout << "1) Run, -1) quit : ";
  cin >> i;

  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      ulapi_mutex_give(pm.grabmutex);
      poseDefined = true;
    }

	calPose = curPose;
    poseMe = curPose;

    switch (i)
    {
      case 1:
		for (int n = 0; n < testRepetitions; n++)
        {
		  for(int k = 0; k < testPositions; k++)
		  {
		    poseMe.x = testXY[0][k];
		    poseMe.y = testXY[1][k];
		    ulapi_mutex_take(pm.grabmutex);
		    if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS){ curPose = poseMe; }
            ulapi_mutex_give(pm.grabmutex);
		    timer.waitUntil(pauseTime);
		  }
		}
		
		infile.close();
		/*poseMe = calPose;
		ulapi_mutex_take(pm.grabmutex);
        if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS){ curPose = poseMe; }
        ulapi_mutex_give(pm.grabmutex);*/
	    break;

      default:
        break;
	}
    cout << "1) Run, -1) quit : ";
    cin >> i;
  } // while (i != -1)

#elif defined MOTIONDEMO
  CrpiRobot<robType> arm("universal_ur10_right.xml");
  //CrpiRobot<robType> arm("kuka_lwr.xml");
  char curtool[32];
  bool toolopen = true;

  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->SetAngleUnits("degree");
  pm.robArm->SetLengthUnits("mm");

  //strcpy(curtool, "gripper_gear");
  strcpy(curtool, "gripper_parallel");
  pm.robArm->Couple(curtool);

  robotPose poseMe, curPose;
  robotIO io;
  bool poseDefined = false;
  bool tool = false;
  int param;
  int counter; 
  ofstream out("sprial.csv");

  double theta = 0.0f;
  matrix pin(3,1), pout(3,1);

  cout << "int:  " << sizeof(int) << " double:  " << sizeof(double) << endl;

  cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Move +X 20 mm, 4) Move -X 20 mm, 5) Move +Y 20 mm, 6) Move -Y 20 mm, 7) Get Pose, 8) Gripper, 9) Test Spiral -1) quit : ";
  cin >> i;
  AssemblyTimer asblytimer;
  double tim;
  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      ulapi_mutex_give(pm.grabmutex);

      poseDefined = true;
    }

    poseMe = curPose;

    switch (i)
    {
    case 1:
      poseMe.z += 20.0f;
      break;
    case 2:
      poseMe.z += -20.0f;
      break;
    case 3:
      poseMe.x += 20.0f;
      break;
    case 4:
      poseMe.x += -20.0f;
      break;
    case 5:
      poseMe.y += 20.0f;
      break;
    case 6:
      poseMe.y += -20.0f;
      break;
    case 7:
      poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&poseMe);
      ulapi_mutex_give(pm.grabmutex);

      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << endl;
      break;
    case 8:
      /*
      double dval;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotIO(&io);
      ulapi_mutex_give(pm.grabmutex);
      cout << " >> DIO: ";
      for (i = 0; i < CRPI_IO_MAX; ++i)
      {
        cout << io.dio[i];
      }
      cout << endl;
      */

      /*
      //! Force control tesl
      poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->MoveAttractor(poseMe);
      ulapi_mutex_give(pm.grabmutex);
      */

      ulapi_mutex_take(pm.grabmutex);
      pm.robArm->SetTool((toolopen ? 0.0 : 1.0));
      ulapi_mutex_give(pm.grabmutex);
      toolopen = !toolopen;
      break;
    case 9:
      if (!poseDefined)
      {
        curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
        ulapi_mutex_take(pm.grabmutex);
        pm.robArm->GetRobotPose(&curPose);
        ulapi_mutex_give(pm.grabmutex);

        poseDefined = true;
      }

      while (true)
      {
      poseMe.z += 100.0f;
      ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
        cout << "curPose=" << curPose.z << endl;
      }
      ulapi_mutex_give(pm.grabmutex);

      poseMe.y += 100.0f;
      ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
        cout << "curPose=" << curPose.z << endl;
      }
      ulapi_mutex_give(pm.grabmutex);

      poseMe.z -= 100.0f;
      ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
        cout << "curPose=" << curPose.z << endl;
      }
      ulapi_mutex_give(pm.grabmutex);

      poseMe.y -= 100.0f;
      ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
        cout << "curPose=" << curPose.z << endl;
      }
      ulapi_mutex_give(pm.grabmutex);
      }

      break;
    default:
      break;
    }

    if (i > 0 && i < 7)
    {
      ulapi_mutex_take(pm.grabmutex);
      if (pm.robArm->MoveTo (poseMe) == CANON_SUCCESS)
      {
        curPose = poseMe;
        cout << "curPose=" << curPose.z << endl;
      }
      ulapi_mutex_give(pm.grabmutex);
    }

    cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Move +X 20 mm, 4) Move -X 20 mm, 5) Move +Y 20 mm, 6) Move -Y 20 mm, 7) Get Pose, 8) Gripper, 9) Test Spiral -1) quit : ";
    cin >> i;
  } // while (i != -1)
#elif defined XMLDEMO
  //CrpiRobot<robType> arm("universal_ur10_right.xml");
  CrpiRobot<robType> arm("kuka_lwr.xml");
  char curtool[32];
  char buffer[1024];
  bool toolopen = true;

  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->SetAngleUnits("degree");
  pm.robArm->SetLengthUnits("mm");

  //strcpy(curtool, "gripper_gear");
  //strcpy(curtool, "gripper_parallel");
  //pm.robArm->Couple(curtool);



  robotPose poseMe, curPose;
  robotIO io;
  bool poseDefined = false;
  bool tool = false;
  int param;
  int counter; 

  std::string str1, str2, str3, str4;

  str1 = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLCommandInstance xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"../xmlSchemas/CRCLCommandInstance.xsd\"><CRCLCommand xsi:type=\"MoveToType\"><CommandID>2</CommandID><MoveStraight>false</MoveStraight><Point><X>2.5</X> <Y>1</Y> <Z>1</Z></Point><XAxis><I>1</I><J>0</J><K>0</K></XAxis><ZAxis><I>0</I><J>0</J><K>-1</K></ZAxis></CRCLCommand></CRCLCommandInstance>";
  str2 = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLCommandInstance xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"><CRCLCommand xsi:type\"SetEndEffectorType\"><CommandID>123</CommandID><NumPositions>0.5</NumPositions></CRCLCommand></CRCLCommandInstance>"; 
  str3 = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLStatus xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"../xmlSchemas/CRCLStatus.xsd\"><CommandStatus><CommandID>1</CommandID><StatusID>1</StatusID><CommandState>Working</CommandState></CommandStatus><Pose><Point><X>1.5</X> <Y>1</Y> <Z>1</Z></Point><XAxis><I>1</I> <J>0</J> <K>0</K></XAxis><ZAxis><I>0</I> <J>0</J> <K>-1</K></ZAxis></Pose></CRCLStatus>";

  timer.waitUntil(2000);

  curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
  ulapi_mutex_take(pm.grabmutex);
  pm.robArm->GetRobotPose(&curPose);
  ulapi_mutex_give(pm.grabmutex);

  cout << "(" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;

  matrix m(3, 3);

//  curPose.xrot = 90.0f;
//  curPose.yrot = 0.0f;
//  curPose.zrot = 0.0f;

  poseMe = curPose;

  //poseMe.xrot *= (3.141592654f / 180.0f);
  //poseMe.yrot *= (3.141592654f / 180.0f);
  //poseMe.zrot *= (3.141592654f / 180.0f);
  pm.robArm->RPYMatrixConvert(poseMe, m);

  cout << "Test orientation conversion: " << endl;
  cout << "Robot representation (deg): (" << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;
  cout << "Robot representation (rad): (" << poseMe.xrot << ", " << poseMe.yrot << ", " << poseMe.zrot << ")" << endl;
  cout << "CRCL representation (rad):  |" << m.at(0, 0) << " " << m.at(0, 1) << " " << m.at(0,2) << "|" << endl;
  cout << "                            |" << m.at(1, 0) << " " << m.at(1, 1) << " " << m.at(1,2) << "|" << endl;
  cout << "                            |" << m.at(2, 0) << " " << m.at(2, 1) << " " << m.at(2,2) << "|" << endl;

  pm.robArm->matrixRPYconvert(m, poseMe);

  poseMe.xrot *= (180.0f / 3.141592654f);
  poseMe.yrot *= (180.0f / 3.141592654f);
  poseMe.zrot *= (180.0f / 3.141592654f);

  cout << "Back to robot representation (deg): (original, reverted)" << endl;
  cout << "xrot(" << curPose.xrot << ", " << poseMe.xrot << "), yrot(" << curPose.yrot << ", " << poseMe.yrot << "), zrot(" << curPose.zrot << ", " << poseMe.zrot << ")" << endl;

  poseMe.z += 20.0f;

  sprintf (buffer, "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLCommandInstance xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"../xmlSchemas/CRCLCommandInstance.xsd\"><CRCLCommand xsi:type=\"MoveToType\"><CommandID>2</CommandID><MoveStraight>false</MoveStraight><Point><X>%f</X><Y>%f</Y><Z>%f</Z></Point><XAxis><I>%f</I><J>%f</J><K>%f</K></XAxis><ZAxis><I>%f</I><J>%f</J><K>%f</K></ZAxis></CRCLCommand></CRCLCommandInstance>",
           poseMe.x,
           poseMe.y,
           poseMe.z,
           m.at(0, 0),
           m.at(1, 0),
           m.at(2, 0),
           m.at(0, 2),
           m.at(1, 2),
           m.at(2, 2));

  cout << buffer << endl;

  str4 = buffer;

  pm.robArm->XmlHandler(str4);

  timer.waitUntil(2000);
  pm.robArm->MoveTo(curPose);

  pm.robArm->XmlResponse(buffer);
  cout << buffer << endl;
#elif defined REGISTRATIONTEST
  CrpiRobot<CrpiUniversal> arm1("universal_ur10_right.xml");
  CrpiRobot<CrpiKukaLWR> arm2("kuka_lwr.xml");
  char curtool[32];
  bool toolopen = true;
  char in;

  arm1.SetAngleUnits("degree");
  arm1.SetLengthUnits("mm");
  arm2.SetAngleUnits("degree");
  arm2.SetLengthUnits("mm");

  cout << "coupling tools..." << endl;
  //strcpy(curtool, "gripper_gear");
  strcpy(curtool, "gripper_parallel");
  arm1.Couple(curtool);
  arm2.Couple(curtool);

  robotPose poseMe, tarPose1, tarPose2;
  robotPose curPose1, curPose2;

  poseMe.x = 1295.0;
  poseMe.y = 350.0;
  poseMe.z = 160.0;//-8.43;
  poseMe.xrot = 180.0;
  poseMe.yrot = 0.0;
  poseMe.zrot = 0.0;

  cout << "getting poses... " << endl;
  arm1.GetRobotPose(&curPose1);
  arm2.GetRobotPose(&curPose2);

  arm1.FromWorld(&poseMe, &tarPose1);
  arm2.FromWorld(&poseMe, &tarPose2);
  cout << "Current Pose [LWR]:  (" << curPose2.x << ", " << curPose2.y << ", " << curPose2.z << ", " << curPose2.xrot << ", " << curPose2.yrot << ", " << curPose2.zrot << ")" << endl;
  cout << "Current Pose [UR10]: (" << curPose1.x << ", " << curPose1.y << ", " << curPose1.z << ", " << curPose1.xrot << ", " << curPose1.yrot << ", " << curPose1.zrot << ")" << endl;
  cout << "-----------------------------" << endl;
  cout << "Common Pose [World]: (" << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " << poseMe.yrot << ", " << poseMe.zrot << ")" << endl;
  cout << "Common Pose [LWR]:   (" << tarPose2.x << ", " << tarPose2.y << ", " << tarPose2.z << ", " << tarPose2.xrot << ", " << tarPose2.yrot << ", " << tarPose2.zrot << ")" << endl;
  cout << "Common Pose [UR10]:  (" << tarPose1.x << ", " << tarPose1.y << ", " << tarPose1.z << ", " << tarPose1.xrot << ", " << tarPose1.yrot << ", " << tarPose1.zrot << ")" << endl;
  cout << "-----------------------------" << endl;
  cout << "Press [Enter] to run motion test..." << endl;
  cin.get(in);

  cout << "Commencing test in ";
  for (int i = 5; i >= 1; --i)
  {
    cout << i << " ";
    timer.waitUntil(1000);
  }
  cout << endl;

  if (arm2.MoveStraightTo(tarPose2) != CANON_SUCCESS)
  {
    cout << "motion error" << endl;
  }
  timer.waitUntil(5000);
  arm2.MoveStraightTo(curPose2);
  arm1.MoveStraightTo(tarPose1);
  timer.waitUntil(5000);
  arm1.MoveStraightTo(curPose1);
#elif defined JOINTDEMO
  CrpiRobot<CrpiKukaLWR> arm("kuka_lwr.xml");
  char curtool[32];
  bool toolopen = true;
  char in;

  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");

  cout << "coupling tools..." << endl;
  strcpy(curtool, "gripper_parallel");
  arm.Couple(curtool);

  robotPose curPose, tarPose;
  robotAxes curAxes, curAxes2, tarAxes;

  cout << "getting pose... " << endl;
  arm.GetRobotPose(&curPose);
  arm.GetRobotAxes(&curAxes);

  cout << "Current Pose:  (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ", " << curPose.status << " " << curPose.turns << ")" << endl;
  cout << "Current Axes:  (";
  for (int q = 0; q < 7; ++q)
  {
    cout << "J" << (q+1) << ": " << curAxes.axis[q] << (q < 6 ? ", " : ")");
  }
  cout << endl;

  cout << "Press [Enter] to run motion test..." << endl;
  cin.get(in);

  tarAxes = curAxes;
  tarAxes.axis[0] += 25.0f; //! Base joint (A1, J1)
  tarAxes.axis[2] += 40.0f; //! External axis (E1, J3)

  if (arm.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
  {
    cout << "motion error" << endl;
  }

  arm.GetRobotPose(&curPose);
  arm.GetRobotAxes(&curAxes2);

  cout << "New Pose:  (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;
  cout << "New Axes:  (";
  for (int q = 0; q < 7; ++q)
  {
    cout << "J" << (q+1) << ": " << curAxes2.axis[q] << (q < 6 ? ", " : ")");
  }
  cout << endl;

  //! Go back to where we started
  timer.waitUntil(5000);
  arm.MoveToAxisTarget(curAxes);
#elif defined MATHTEST
  matrix m1(3, 3), m2(3, 3), mout(3, 3);
  m1.at(0, 0) = 1;
  m1.at(1, 1) = 1;
  m1.at(2, 2) = 1;
  m2.at(0, 1) = 2;
  m2.at(1, 2) = 2;
  m2.at(2, 0) = 2;

  mout = m1 + m2;

  cout << endl;
  mout.print();

#elif defined SENSORTEST
/*
  LeapMotion lm;
  Hand handy;
  int iii;

  HandList hl;
  matrix mat;
  mat.resize(400, 400);
  while (true)
  {
    lm.getHands(&hl);

    iii = 0;
    for (HandList::const_iterator hli = hl.begin(); hli != hl.end(); ++hli, ++iii)
    {
      handy = *hli;
      cout << "hand #" << iii << " : " << (handy.isLeft() ? "Left" : "Right") << endl;
    }

    lm.getDepthMap(mat);
  }
*/

  Vicon vtest ("129.6.35.25");
  vector<ViconSubject> vec;
  vector<ViconSubject>::iterator iter;

  vector<point> mvec;
  vector<point>::iterator miter;

  while (true)
  {
    vtest.GetCurrentSubjects(vec);
    
    i = 0;
    for (iter = vec.begin(); iter != vec.end(); ++iter, ++i)
    {
      cout << i << ": " << iter->name << " (" << iter->pose.x << ", " << iter->pose.y << ", " << iter->pose.z << ", " << iter->pose.xrot << ", " << iter->pose.yrot << ", " << iter->pose.zrot << ")" << endl;
    }

    /*
    vtest.GetUnlabeledMarkers(mvec);
    i = 0;
    for (miter = mvec.begin(); miter != mvec.end(); ++miter, ++i)
    {
      cout << i << ": (" << miter->x << ", " << miter->y << ", " << miter->z << ")" << endl;
    }
    */
    Sleep (200);
  }
  //! JAM TODO

 #elif defined FT_TEST	
	//ATI_Wired ft_sensor;
	FT_COP COP_sensor('z',0.008);
	vector<double> COP1(3);
	vector<double> COP2(3);
	vector<double> COP3(3);

	/*
	for(int i=0;i<200;++i)
	{
		//ft_sensor.Get_FT();
		COP_sensor.Calc_COP();
		std::cout << COP_sensor.COP[0] << " " << COP_sensor.COP[1] << " " << COP_sensor.COP[2] << std::endl;
		cin.get();
	}
	*/
	
	
	COP_sensor.Calc_COP();
	COP1 = COP_sensor.COP;
	std::cout << "Move to another location and press any key to sample another COP" << std::endl;
	cin.get();
	COP_sensor.Calc_COP();
	COP2 = COP_sensor.COP;
	std::cout << "Move to another location and press any key to sample another COP" << std::endl;
	cin.get();
	COP_sensor.Calc_COP();
	COP3 = COP_sensor.COP;

	std::cout << "COP1: " << COP1[0] << " " << COP1[1] << " " << COP1[2] << std::endl;
	std::cout << "COP2: " << COP2[0] << " " << COP2[1] << " " << COP2[2] << std::endl;
	std::cout << "COP3: " << COP3[0] << " " << COP3[1] << " " << COP3[2] << std::endl;
	
	std::cout << "press any key to quit" << std::endl;
	
	cin.get();


 #elif defined	FORCE_REGISTER

  //INITIALIZATIONS-------------------------------------------------------
  CrpiRobot<CrpiKukaLWR> arm("kuka_lwr.xml");
  char curtool[32];
  bool toolopen = true;

  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");

  cout << "coupling tools..." << endl;
  strcpy(curtool, "point");
  arm.Couple(curtool);

  robotPose curPose,curPose1, curPose2, curPose3, tarPose, firstPose;
  robotPose homePose;

  FT_COP COP_sensor('z',0.008); //FT_COP COP_sensor('z',0.033);
  vector<double> COP1(3);
  vector<double> COP2(3);
  vector<double> COP3(3);

  //MOVING TO CALIBRATE--------------------------------------------------------

  arm.GetRobotPose(&homePose);
  arm.GetRobotPose(&tarPose);
  arm.GetRobotPose(&firstPose);

  cout << "Going to approach point... " << endl;
  homePose.x = -535;
  homePose.y = -460;
  homePose.z = 114;
  homePose.xrot = -180;
  homePose.yrot = 0;
  homePose.zrot = 30;
  while (arm.MoveAttractor(homePose) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  //! TARE
  COP_sensor.Set_FT_Data();

  //Define one touch point for tool end
  firstPose.x = -580;
  firstPose.y = -470;
  firstPose.z = 43;
  firstPose.xrot = -180;
  firstPose.yrot = 0;
  firstPose.zrot = 44;

  cout << "Going to first point... " << endl;
  tarPose = firstPose;
  while (arm.MoveAttractor(tarPose) != CANON_SUCCESS) {}
  timer.waitUntil(1000);
  
  COP_sensor.Calc_COP();
  arm.GetRobotPose(&curPose1);
  COP1 = COP_sensor.COP;

  cout << "Going to approach point... " << endl;
  while (arm.MoveAttractor(homePose) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  cout << "Going to second point... " << endl;
  tarPose.x = firstPose.x+60;
  tarPose.y = firstPose.y-28;
  tarPose.z = firstPose.z;

  while (arm.MoveAttractor(tarPose) != CANON_SUCCESS) {}
  timer.waitUntil(1000);
  
  COP_sensor.Calc_COP();
  arm.GetRobotPose(&curPose2);
  COP2 = COP_sensor.COP;

  cout << "Going to approach point... " << endl;
  while (arm.MoveAttractor(homePose) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  cout << "Going to third point... " << endl;
  tarPose.x = firstPose.x+50;
  tarPose.y = firstPose.y+48;
  tarPose.z = 43;

  while (arm.MoveAttractor(tarPose) != CANON_SUCCESS) {}
  timer.waitUntil(1000);
  
  COP_sensor.Calc_COP();
  arm.GetRobotPose(&curPose3);
  COP3 = COP_sensor.COP;

  cout << "Going to approach point... " << endl;
  while (arm.MoveAttractor(homePose) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  //CRUNCH THROUGH COLLECTED DATA------------------------------------
 
  //Transform Load Cell points into World
  //W = World coordinate system
  //LC = Load Cell coordinate system
  //R = Robot coordinate system
  matrix W_T_LC(4,4);
  W_T_LC.at(0,0) = W_T_LC.at(1,1) = W_T_LC.at(2,2) = W_T_LC.at(3,3) = 1;
  W_T_LC.at(0,3) = 14*25;//-2.5;
  W_T_LC.at(1,3) = 17*25;//+1;
  W_T_LC.at(2,3) = 53.3-8;

  matrix LC_p1mat,LC_p2mat,LC_p3mat,W_p1mat,W_p2mat,W_p3mat;

  point LC_p1(COP1[0]*1000, COP1[1]*1000, COP1[2]*1000),
        LC_p2(COP2[0]*1000, COP2[1]*1000, COP2[2]*1000),
        LC_p3(COP3[0]*1000, COP3[1]*1000, COP3[2]*1000);

  LC_p1mat.fromPoint(LC_p1);
  LC_p2mat.fromPoint(LC_p2);
  LC_p3mat.fromPoint(LC_p3);
  //LC_p1.print();
  //LC_p1mat.print();

  W_p1mat = W_T_LC*LC_p1mat;
  W_p2mat = W_T_LC*LC_p2mat;
  W_p3mat = W_T_LC*LC_p3mat;
   
  point W_p1(W_p1mat.at(0,0),W_p1mat.at(1,0),W_p1mat.at(2,0)),
	    W_p2(W_p2mat.at(0,0),W_p2mat.at(1,0),W_p2mat.at(2,0)),
		W_p3(W_p3mat.at(0,0),W_p3mat.at(1,0),W_p3mat.at(2,0));

  //cout << "W_p1: ";
  //W_p1.print();
  //cout << "target: " << 14*25 << " " << 17*25 << " ~" << 55 << endl;

  vector<point> W_robot_touch_points;
  W_robot_touch_points.push_back(W_p1);
  W_robot_touch_points.push_back(W_p2);
  W_robot_touch_points.push_back(W_p3);

  //Transform load cell points into Robot coordinate system
  point R_p1(curPose1.x,curPose1.y,curPose1.z),
	    R_p2(curPose2.x,curPose2.y,curPose2.z),
		R_p3(curPose3.x,curPose3.y,curPose3.z);

  vector<point> R_robot_touch_points;
  R_robot_touch_points.push_back(R_p1);
  R_robot_touch_points.push_back(R_p2);
  R_robot_touch_points.push_back(R_p3);

  //Calculate transformation from Robot to World coordinate system
  matrix W_T_R(4, 4), R_T_W(4, 4);
  reg2target(W_robot_touch_points, R_robot_touch_points, R_T_W);

  R_T_W.print();

  

  //Define a few points in established World coordinate system, transform into Robot coordinate system, command robot to those points and do sanity check
  robotPose W_target, R_target;
  matrix W_XYZ(4,1), R_XYZ(4,1);
  vector<double> Euler_Angles;
  
  matrix W_Rotation_Matrix(3,3);
  W_Rotation_Matrix.at(0,0) = 0;
  W_Rotation_Matrix.at(1,0) = -1;
  W_Rotation_Matrix.at(2,0) = 0;
  W_Rotation_Matrix.at(0,1) = -1;
  W_Rotation_Matrix.at(1,1) = 0;
  W_Rotation_Matrix.at(2,1) = 0;
  W_Rotation_Matrix.at(0,2) = 0;
  W_Rotation_Matrix.at(1,2) = 0;
  W_Rotation_Matrix.at(2,2) = -1;


  W_Rotation_Matrix.rotMatrixEulerConvert(Euler_Angles);
  
  cout << Euler_Angles[0] << " " << Euler_Angles[1] << " " << Euler_Angles[2] << endl;

  arm.GetRobotPose(&curPose);
  R_target = curPose;
  //curPose = homePose;

  //TEST POINT 1 
  W_target.x = 23*25;
  W_target.y = 17*25;
  W_target.z = 50;
  W_target.xrot = curPose.xrot;
  W_target.yrot = curPose.yrot;
  W_target.zrot = curPose.zrot;

  //Convert to math matrix
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;

  //Transform target point to robot point
  R_XYZ = R_T_W * W_XYZ;
  
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  R_target.xrot = W_target.xrot;
  R_target.yrot = W_target.yrot;
  R_target.zrot = W_target.zrot;

  R_target.print();

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  //arm.GetRobotPose(&R_target);
    
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = -1;
  W_XYZ.at(3,0) = 1;
  R_XYZ = R_T_W * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p1" << endl;
  R_target.print();
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  cin.get();

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;
  R_XYZ = R_T_W * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move up" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  
  //TEST POINT 2 
  W_target.x = 13*25;
  W_target.y = 35*25;
  W_target.z = 50;
  W_target.xrot = curPose.xrot;
  W_target.yrot = curPose.yrot;
  W_target.zrot = curPose.zrot;

  //Convert to math matrix
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;

  //Transform target point to robot point
  R_XYZ = R_T_W * W_XYZ;
  
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  R_target.xrot = W_target.xrot;
  R_target.yrot = W_target.yrot;
  R_target.zrot = W_target.zrot;

  R_target.print();

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  //arm.GetRobotPose(&R_target);
    
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = -1;
  W_XYZ.at(3,0) = 1;
  R_XYZ = R_T_W * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p1" << endl;
  R_target.print();
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  cin.get();

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;
  R_XYZ = R_T_W * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move up" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  cin.get();
  //Further refine registeration by going to other points (countersinks)

  cout << "Going to countersink touch point... " << endl;

  //hover over
  W_XYZ.at(0,0) = 23*25;
  W_XYZ.at(1,0) = 10*25;
  W_XYZ.at(2,0) = 10 + 50;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W*W_XYZ;

  R_target = homePose;

  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  //insert
  W_XYZ.at(0,0) = 23*25;
  W_XYZ.at(1,0) = 10*25;
  W_XYZ.at(2,0) = 10 - 5;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W*W_XYZ;

  R_target = homePose;

  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  timer.waitUntil(1000);
  arm.GetRobotPose(&curPose1); //grab new registration position

  //hover over
  W_XYZ.at(0,0) = 23*25;
  W_XYZ.at(1,0) = 10*25;
  W_XYZ.at(2,0) = 10 + 50;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W*W_XYZ;

  R_target = homePose;

  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  cout << "going to countersink 2" << endl;

  //hover over
  W_XYZ.at(0,0) = 18*25;
  W_XYZ.at(1,0) = 32*25;
  W_XYZ.at(2,0) = 10 + 50;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W*W_XYZ;

  R_target = homePose;

  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  //insert
  W_XYZ.at(0,0) = 18*25;
  W_XYZ.at(1,0) = 32*25;
  W_XYZ.at(2,0) = 10 - 5;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W*W_XYZ;

  R_target = homePose;

  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  timer.waitUntil(1000);
  arm.GetRobotPose(&curPose2); //grab new registration position

  //hover over
  W_XYZ.at(0,0) = 18*25;
  W_XYZ.at(1,0) = 32*25;
  W_XYZ.at(2,0) = 10 + 50;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W*W_XYZ;

  R_target = homePose;

  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  cout << "going to countersink 3" << endl;

  //hover over
  W_XYZ.at(0,0) = 7*25;
  W_XYZ.at(1,0) = 32*25;
  W_XYZ.at(2,0) = 10 + 50;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W*W_XYZ;

  R_target = homePose;

  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  //insert
  W_XYZ.at(0,0) = 7*25;
  W_XYZ.at(1,0) = 32*25;
  W_XYZ.at(2,0) = 10 - 5;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W*W_XYZ;

  R_target = homePose;

  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  timer.waitUntil(1000);
  arm.GetRobotPose(&curPose3); //grab new registration position

  //hover over
  W_XYZ.at(0,0) = 7*25;
  W_XYZ.at(1,0) = 32*25;
  W_XYZ.at(2,0) = 10 + 50;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W*W_XYZ;

  R_target = homePose;

  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  timer.waitUntil(1000);

  //---------------------------------------

  //Registering new transformation matrix

  point W_p4(23*25,10*25,11); //fixed coordinates on table...add
  point W_p5(18*25,32*25,11);
  point W_p6(7*25,32*25,11);

  point R_p4(curPose1.x,curPose1.y,curPose1.z),
	    R_p5(curPose2.x,curPose2.y,curPose2.z),
		R_p6(curPose3.x,curPose3.y,curPose3.z);

  W_robot_touch_points[0] = W_p4;
  W_robot_touch_points[1] = W_p5;
  W_robot_touch_points[2] = W_p6;
  
  R_robot_touch_points[0] = R_p4;
  R_robot_touch_points[1] = R_p5;
  R_robot_touch_points[2] = R_p6;

  //Calculate transformation from Robot to World coordinate system
  //matrix R_T_W(4, 4);
  reg2target(W_robot_touch_points, R_robot_touch_points, R_T_W);

  R_T_W.print();


  /*
  //World defined robot pose
  
  //Robot taught point
  matrix R_W_T_fixed(4,4), R_point_taught(4,1), W_point_taught(4,1);

  R_W_T_fixed.at(0,0) = 0.999952; R_W_T_fixed.at(0,1) = -0.001475; R_W_T_fixed.at(0,2) = 0.009657; R_W_T_fixed.at(0,3) = -887.186289;
  R_W_T_fixed.at(1,0) = 0.001382; R_W_T_fixed.at(1,1) = 0.999952; R_W_T_fixed.at(1,2) = 0.009689; R_W_T_fixed.at(1,3) = -890.264954;
  R_W_T_fixed.at(2,0) = -0.009671; R_W_T_fixed.at(2,1) = -0.009675; R_W_T_fixed.at(2,2) = 0.999906; R_W_T_fixed.at(2,3) = -0.526560;
  R_W_T_fixed.at(3,0) = 0; R_W_T_fixed.at(3,1) = 0; R_W_T_fixed.at(3,2) = 0; R_W_T_fixed.at(3,3) = 1;

  R_point_taught.at(0,0) = -512.51;
  R_point_taught.at(1,0) = -165.38;
  R_point_taught.at(2,0) = 209.24;
  R_point_taught.at(3,0) = 1;

  W_point_taught = R_W_T_fixed.inv()*R_point_taught;
 

//0.999952 -0.001475 0.009657 -887.186289
//0.001382 0.999952 0.009689 -890.264954 |
//-0.009671 -0.009675 0.999906 -0.526560 |
//-0.000000 -0.000000 0.000000 1.000000 |

  
  //Taught Test point
  //Centered Test point 
  W_target.x = W_point_taught.at(0,0); //23*25;
  W_target.y = W_point_taught.at(1,0); //17*25;
  W_target.z = W_point_taught.at(2,0)+50; //53.3+50;
  W_target.xrot = curPose.xrot;
  W_target.yrot = curPose.yrot;
  W_target.zrot = curPose.zrot;

  //Convert to math matrix
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;

  //Transform target point to robot point
  R_XYZ = W_T_R * W_XYZ;
  
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  R_target.xrot = W_target.xrot;
  R_target.yrot = W_target.yrot;
  R_target.zrot = W_target.zrot;

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  arm.GetRobotPose(&R_target);
    
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z-50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p1" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move up" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);


  //Centered Test point 
  W_target.x = 14*25; //23*25;
  W_target.y = 17*25;
  W_target.z = 53.3+50;
  W_target.xrot = curPose.xrot;
  W_target.yrot = curPose.yrot;
  W_target.zrot = curPose.zrot;

  //Convert to math matrix
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;

  //Transform target point to robot point
  R_XYZ = W_T_R * W_XYZ;
  
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  R_target.xrot = W_target.xrot;
  R_target.yrot = W_target.yrot;
  R_target.zrot = W_target.zrot;

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  arm.GetRobotPose(&R_target);
    
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z-50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p1" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move up" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  */

  curPose = homePose;

  //TEST POINT 1 
  W_target.x = 23*25;
  W_target.y = 17*25;
  W_target.z = 50;
  W_target.xrot = curPose.xrot;
  W_target.yrot = curPose.yrot;
  W_target.zrot = curPose.zrot;

  //Convert to math matrix
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;

  //Transform target point to robot point
  R_XYZ = R_T_W * W_XYZ;
  
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  R_target.xrot = W_target.xrot;
  R_target.yrot = W_target.yrot;
  R_target.zrot = W_target.zrot;

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  //arm.GetRobotPose(&R_target);
    
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = -1;
  W_XYZ.at(3,0) = 1;
  R_XYZ = R_T_W * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p1" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  cin.get();

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;
  R_XYZ = R_T_W * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move up" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  
  //TEST POINT 2 
  W_target.x = 13*25;
  W_target.y = 35*25;
  W_target.z = 50;
  W_target.xrot = curPose.xrot;
  W_target.yrot = curPose.yrot;
  W_target.zrot = curPose.zrot;

  //Convert to math matrix
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;

  //Transform target point to robot point
  R_XYZ = R_T_W * W_XYZ;
  
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  R_target.xrot = W_target.xrot;
  R_target.yrot = W_target.yrot;
  R_target.zrot = W_target.zrot;

  R_target.print();

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  //arm.GetRobotPose(&R_target);
    
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = -1;
  W_XYZ.at(3,0) = 1;
  R_XYZ = R_T_W * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p1" << endl;
  R_target.print();
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  cin.get();

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;
  R_XYZ = R_T_W * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move up" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}


  /*
  //TEST POINT 2 
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y-25;
  W_XYZ.at(2,0) = 50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p2" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y-25;
  W_XYZ.at(2,0) = 0;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p2" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y-25;
  W_XYZ.at(2,0) = 50.0;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move up" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  //TEST POINT 3 
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y+25;
  W_XYZ.at(2,0) = 50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p3" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y+25;
  W_XYZ.at(2,0) = 0;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p3" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y+25;
  W_XYZ.at(2,0) = 50.0;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move up" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  //TEST POINT 4 
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y+50;
  W_XYZ.at(2,0) = 50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p4" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y+50;
  W_XYZ.at(2,0) = 0;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move p4" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y+50;
  W_XYZ.at(2,0) = 50.0;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  cout << "move up" << endl;
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);


  
  //----------------------------------------------------------------------
  //Line two

  //World defined robot pose
  
  //TEST POINT 1 
  W_target.x = 15*25; //10*25;
  W_target.y = 29*25; //35*25;
  W_target.z = 220+50; //50;
  W_target.xrot = curPose.xrot;
  W_target.yrot = curPose.yrot;
  W_target.zrot = curPose.zrot;

  //Convert to math matrix
  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;

  //Transform target point to robot point
  R_XYZ = W_T_R * W_XYZ;
  
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  R_target.xrot = W_target.xrot;
  R_target.yrot = W_target.yrot;
  R_target.zrot = W_target.zrot;

  while (arm.MoveAttractor(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z-50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = W_target.z;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);
  
  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  /*
  //Test point 2
  W_XYZ.at(0,0) = W_target.x-25;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = 50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x-25;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = 0;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x-25;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = 50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  //Test point 3
  W_XYZ.at(0,0) = W_target.x-50;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = 50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x-50;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = 0;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x-50;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = 50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  //Test point 4
  W_XYZ.at(0,0) = W_target.x-75;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = 50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x-75;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = 0;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);

  W_XYZ.at(0,0) = W_target.x-75;
  W_XYZ.at(1,0) = W_target.y;
  W_XYZ.at(2,0) = 50;
  W_XYZ.at(3,0) = 1;
  R_XYZ = W_T_R * W_XYZ;
  R_target.x = R_XYZ.at(0,0);
  R_target.y = R_XYZ.at(1,0);
  R_target.z = R_XYZ.at(2,0);

  while (arm.MoveTo(R_target) != CANON_SUCCESS) {}
  Sleep(1000);
  */
  timer.waitUntil(3000);

 #elif defined	REGISTERTEST
  CrpiRobot<CrpiKukaLWR> arm("kuka_lwr.xml");
  robotPose ps_in, ps_out;

  vector<point> world;
  vector<point> rob1;
  vector<point> rob2;

  vector<point>::const_iterator iter;

  int k = 1;

  matrix w2r1_1(4, 4), w2r2_1(4, 4);
  matrix rot(3, 3);
  matrix posin(4, 4), posout(4, 4);

  vector<double> outs;

  point temp;
  //! Set world coordinates
  //! World Origin:  0, 0, 0
  point w1(575.0f, 0.0f, 0.0f),
        w2(575.0f, 10.0f, 0.0f),
        w3(585.0f, 0.0f, 0.0f),
        w4(585.0f, 10.0f, 0.0f);

  point r1(-315.0f, -890.0f, 0.0f),
        r2(-315.0f, -880.0f, 0.0f),
        r3(-305.0f, -890.0f, 0.0f),
        r4(-305.0f, -880.0f, 0.0f);

  point s1(-30.0f, 790.0f, -10.0f),
        s2(-20.0f, 790.0f, -10.0f),
        s3(-30.0f, 780.0f, -10.0f),
        s4(-20.0f, 780.0f, -10.0f);

  world.push_back(w1);
  world.push_back(w2);
  world.push_back(w3);
  world.push_back(w4);
  
  rob1.push_back(r1);
  rob1.push_back(r2);
  rob1.push_back(r3);
  rob1.push_back(r4);
  
  rob2.push_back(s1);
  rob2.push_back(s2);
  rob2.push_back(s3);
  rob2.push_back(s4);
  
  bool state = reg2target(rob1, world, w2r1_1);

  posin.at(0, 0) = posin.at(1, 1) = posin.at(2, 2) = posin.at(3, 3) = 1.0f;
  posin.at(0, 3) = world.at(k).x;
  posin.at(1, 3) = world.at(k).y;
  posin.at(2, 3) = world.at(k).z;

  matrix posin_inv(4, 4);
  posin_inv = w2r1_1.inv();

  ps_in.x = world.at(k).x;
  ps_in.y = world.at(k).y;
  ps_in.z = world.at(k).z;
  ps_in.xrot = ps_in.yrot = ps_in.zrot = 0.0f;
  arm.FromWorld(&ps_in, &ps_out);


  posout = posin_inv * posin;
  cout << "looking at target " << k << " for robot 1" << endl;
  cout << "origin:  [[" << world.at(k).x << ", " << world.at(k).y  << ", " << world.at(k).z << "]]" << endl;
  cout << "output:  [[" << posout.at(0, 3) << ", " << posout.at(1, 3) << ", " << posout.at(2, 3) << "]]" << endl;
  cout << "target:  [[" << rob1.at(k).x << ", " << rob1.at(k).y << ", " << rob1.at(k).z << "]]" << endl;
  cout << "target2: [[" << ps_out.x << ", " << ps_out.y << ", " << ps_out.z << "]]" << endl;

  cout << endl << endl << endl;

  state = reg2target(rob2, world, w2r2_1);
  posin.at(0, 0) = posin.at(1, 1) = posin.at(2, 2) = posin.at(3, 3) = 1.0f;
  posin.at(0, 3) = world.at(k).x;
  posin.at(1, 3) = world.at(k).y;
  posin.at(2, 3) = world.at(k).z;

  posin_inv = w2r2_1.inv();
  posout = posin_inv * posin;
  cout << "looking at target " << k << " for robot 2" << endl;
  cout << "origin:  [[" << world.at(k).x << ", " << world.at(k).y  << ", " << world.at(k).z << "]]" << endl;
  cout << "output:  [[" << posout.at(0, 3) << ", " << posout.at(1, 3) << ", " << posout.at(2, 3) << "]]" << endl;
  cout << "target:  [[" << rob2.at(k).x << ", " << rob2.at(k).y << ", " << rob2.at(k).z << "]]" << endl;

 #elif defined FIXED_3POINT_REGISTER
  
  //INITIALIZATIONS-------------------------------------------------------
  CrpiRobot<CrpiKukaLWR> arm("kuka_lwr.xml");
  char curtool[32];
  bool toolopen = true;


  arm.InitCanon();
  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");

  cout << "coupling tools..." << endl;
  strcpy(curtool, "point");
  arm.Couple(curtool);

  robotPose curPose, tarPose;
  robotPose homePose;


  //REGISTRATION----------------------------
  vector<point> world;
  vector<point> rob;

  /*
  point green_w1(11*25, 22*25, 0.0f),
        silver_w2(12*25, 24*25, 0.0f),
        blue_w3(14*25, 23*25, 0.0f);

  point green_r1(-612.94,-340.48,-8.46),
		silver_r2(-588.11,-290.68,-8.85),
		blue_r3(-537.53,-315.98,-9.49);
  */

  point green_w1(21*25, 29*25, 0.0f),
        silver_w2(6*25, 32*25, 0.0f),
		blue_w3(23*25, 9*25, 0.0f);

  point green_r1(-363.91,-163.53,-11.32),
		silver_r2(-739.5,-88.98,-7.2),
		blue_r3(-311.87,-664,-8.25);

  world.push_back(green_w1);
  world.push_back(silver_w2);
  world.push_back(blue_w3);

  rob.push_back(green_r1);
  rob.push_back(silver_r2);
  rob.push_back(blue_r3);

  matrix R_T_W(4,4);

  reg2target(world, rob, R_T_W);

  //Move Robot around
  
  matrix W_XYZ(4,1), R_XYZ(4,1);

  W_XYZ.at(0,0) = 23*25;
  W_XYZ.at(1,0) = 17*25;
  W_XYZ.at(2,0) = 0;
  W_XYZ.at(3,0) = 1;

  R_XYZ = R_T_W * W_XYZ;
  
  arm.GetRobotPose(&curPose);

  R_XYZ.print();

  curPose.print();
  
  tarPose = curPose;
  tarPose.x = R_XYZ.at(0,0);
  tarPose.y = R_XYZ.at(1,0);
  tarPose.z = R_XYZ.at(2,0)+50;
  tarPose.xrot = curPose.xrot;
  tarPose.yrot = curPose.yrot;
  tarPose.zrot = curPose.zrot;

  

  while (arm.MoveTo(tarPose) != CANON_SUCCESS) {}
  Sleep(1000);

  tarPose = curPose;
  tarPose.x = R_XYZ.at(0,0);
  tarPose.y = R_XYZ.at(1,0);
  tarPose.z = R_XYZ.at(2,0);
  tarPose.xrot = curPose.xrot;
  tarPose.yrot = curPose.yrot;
  tarPose.zrot = curPose.zrot;

  while (arm.MoveTo(tarPose) != CANON_SUCCESS) {}
  Sleep(1000);

  tarPose = curPose;
  tarPose.x = R_XYZ.at(0,0);
  tarPose.y = R_XYZ.at(1,0);
  tarPose.z = R_XYZ.at(2,0)+50;
  tarPose.xrot = curPose.xrot;
  tarPose.yrot = curPose.yrot;
  tarPose.zrot = curPose.zrot;

  while (arm.MoveTo(tarPose) != CANON_SUCCESS) {}
  Sleep(1000);

#elif defined ALLEGRO_CONTROL

  int key = 0;

  //Testing connection and control to Allegro Hand
  //CrpiRobot<CrpiSchunkSDH> sdh("dummy text");

  CrpiRobot<CrpiAllegro> allegro("allegro.xml");

  
  //just to read joints
  //allegro.SetParameter("Control","read_only");
  //allegro.SetParameter("Plan","set_point");
  
  
  
  robotAxes jointAngles, homePose;

  jointAngles.axis[0] = 0; jointAngles.axis[1] = .4; jointAngles.axis[2] = .4; jointAngles.axis[3] = .4;
  jointAngles.axis[4] = 0; jointAngles.axis[5] = .4; jointAngles.axis[6] = .4; jointAngles.axis[7] = .4;
  jointAngles.axis[8] = 0; jointAngles.axis[9] = .4; jointAngles.axis[10] = .4; jointAngles.axis[11] = .4;
  jointAngles.axis[12] = .6; jointAngles.axis[13] = 1; jointAngles.axis[14] = 0.2; jointAngles.axis[15] = 0.2;

  homePose = jointAngles;

  homePose.axis[1] = homePose.axis[2] = homePose.axis[3] = 0;
  homePose.axis[5] = homePose.axis[6] = homePose.axis[7] = 0;
  homePose.axis[9] = homePose.axis[10] = homePose.axis[11] = 0;

  jointAngles = homePose;

  allegro.SetAbsoluteSpeed(60);
  allegro.SetParameter("touch_stop","off");
  allegro.MoveToAxisTarget(homePose);
  Sleep(4000);
  
  /*
  //Touch Sensitivity Testing
  robotAxes preTouchAngles;
  preTouchAngles = homePose;
  //preTouchAngles.axis[1] = 1.5;
  //preTouchAngles.axis[3] = 0.65;
  preTouchAngles.axis[5] = 1.5;
  preTouchAngles.axis[7] = 0.65;
  allegro.SetParameter("touch_stop","on");
  allegro.MoveToAxisTarget(preTouchAngles);
  Sleep(2000);
  //jointAngles.axis[1] = 2; jointAngles.axis[2] = 2; jointAngles.axis[3] = 2;
  jointAngles.axis[5] = 2; jointAngles.axis[6] = 2; jointAngles.axis[7] = 2;

  for (int k = 0; k < 32; ++k)
  {
	  allegro.SetAbsoluteSpeed(50);
	  allegro.MoveToAxisTarget(jointAngles);
	  Sleep(3000);
	  allegro.SetAbsoluteSpeed(60);
	  allegro.MoveToAxisTarget(preTouchAngles);
	  Sleep(2000);
	  std::cout << k << std::endl;
  }
  */

  /*
  //Finger Strength Testing
  robotAxes preTouchAngles;
  preTouchAngles = homePose;
  
  //Finger1
  preTouchAngles.axis[1] = 1.5;
  preTouchAngles.axis[3] = 0.65;
  jointAngles.axis[1] = 5; jointAngles.axis[2] = 5; jointAngles.axis[3] = 0.65;

  //Finger2
  //preTouchAngles.axis[5] = 1.5;
  //preTouchAngles.axis[7] = 0.65;
  //jointAngles.axis[5] = 5; jointAngles.axis[6] = 5; jointAngles.axis[7] = 0.65;

  //Finger3
  //preTouchAngles.axis[9] = 1.5;
  //preTouchAngles.axis[11] = 0.65;
  //jointAngles.axis[9] = 5; jointAngles.axis[10] = 5; jointAngles.axis[11] = 0.65;

  allegro.SetParameter("touch_stop","off");
  allegro.MoveToAxisTarget(preTouchAngles);
  Sleep(2000);  

  for (int k = 0; k < 32; ++k)
  {
	  allegro.MoveToAxisTarget(jointAngles);
	  Sleep(3000);
	  //allegro.SetAbsoluteSpeed(60);
	  allegro.MoveToAxisTarget(preTouchAngles);
	  Sleep(2000);
	  std::cout << k << std::endl;
  }
  */

  
  //GRASP TESTING-----------------------------------------------

  
  allegro.MoveToAxisTarget(homePose);
  Sleep(2000);

  jointAngles.axis[12] = 1.1;
  allegro.SetAbsoluteSpeed(30);
  allegro.SetParameter("touch_stop","on");
  allegro.MoveToAxisTarget(jointAngles);
  Sleep(3000);

  
  //GRASP TESTING-----------------------------------------
  
  //Close fingers feeling for contact
  jointAngles.axis[12] = 1.15;  jointAngles.axis[14] = .95; jointAngles.axis[15] = .1;
  allegro.MoveToAxisTarget(jointAngles);
  Sleep(2000);

  /*
  //for three fingers
  jointAngles.axis[12] = 1.7;
  allegro.MoveToAxisTarget(jointAngles);
  Sleep(2000);
  jointAngles.axis[0] = -.1;
  jointAngles.axis[4] = .25;
  jointAngles.axis[1] = jointAngles.axis[5] = .5;
  jointAngles.axis[2] = jointAngles.axis[6] = .8;
  jointAngles.axis[3] = jointAngles.axis[7] = .8;
  allegro.MoveToAxisTarget(jointAngles);
  cin.get();
  */
  
  
  //for two fingers
  jointAngles.axis[0] = .25;
  jointAngles.axis[1] = .3;
  jointAngles.axis[2] = 1.4;
  jointAngles.axis[3] = .5;
  //jointAngles.axis[12] = 1.37;
  allegro.MoveToAxisTarget(jointAngles);
  cin.get();
  

  /*
  allegro.GetRobotAxes(&jointAngles);

  jointAngles.axis[1] = jointAngles.axis[1] + 0.1;
  jointAngles.axis[2] = jointAngles.axis[2] + 0.1;
  jointAngles.axis[5] = jointAngles.axis[5] + 0.1;
  jointAngles.axis[6] = jointAngles.axis[6] + 0.1;
  jointAngles.axis[14] = jointAngles.axis[14] + 0.1;
  jointAngles.axis[15] = jointAngles.axis[15] + 0.1;

  allegro.SetParameter("touch_stop","off");
  allegro.MoveToAxisTarget(jointAngles);
  Sleep(1000);
  */
  
  //Issue grasp command
  robotPose pose_d;
  pose_d.x = 0.0; pose_d.y = 0.0; pose_d.z = 50; 
  pose_d.xrot = pose_d.yrot = pose_d.zrot = 0.0;
  allegro.MoveAttractor(pose_d);

  robotPose curPose;
  allegro.GetRobotPose(&curPose);
  //curPose.print();
  cout << "Hit Esc to quit..." << endl;
  //cin.get();

  pose_d = curPose;
  //pose_d.y = curPose.y + 10;
  //allegro.MoveAttractor(pose_d);
  //cin.get();

  while (1)
  {
	  //pose_d.y = pose_d.y + 10;
	  //allegro.MoveAttractor(pose_d);
	  //cin.get();
	  if (kbhit())
	  {
		key = getch();
		if(key == 27)
		{
			break;
		}

		else if (key == 75)
		{
			pose_d.y = pose_d.y + 1;
			allegro.MoveAttractor(pose_d);
		}

		else if (key == 77)
		{
			pose_d.y = pose_d.y - 1;
			allegro.MoveAttractor(pose_d);
		}

		else if (key == 72)
		{
			pose_d.z = pose_d.z - 1;
			allegro.MoveAttractor(pose_d);
		}

		else if (key == 80)
		{
			pose_d.z = pose_d.z + 1;
			allegro.MoveAttractor(pose_d);
		}

		else if (key == 119)
		{
			pose_d.yrot = pose_d.yrot + .025;
			allegro.MoveAttractor(pose_d);
		}

		else if (key == 115)
		{
			pose_d.yrot = pose_d.yrot - .025;
			allegro.MoveAttractor(pose_d);
		}

		else if (key == 97)
		{
			pose_d.zrot = pose_d.zrot + .025;
			allegro.MoveAttractor(pose_d);
		}

		else if (key == 100)
		{
			pose_d.zrot = pose_d.zrot - .025;
			allegro.MoveAttractor(pose_d);
		}

		else if (key == 224) {}

		else {cout << key << endl;}
	  }
	  Sleep(100);
  }

  //------------------------------------------------------------------------------------------
  
  

  /*
  //Building dynamic model-------------------------------------------------------------------
  //std::string thumb_filename = "thumb_joint_angles.csv";
  allegro.SetParameter("touch_stop","on");
  std::string thumb_filename = "middle_joint_angles.csv";
  std::ifstream thumb_file(thumb_filename.c_str());

  //Importing data
  robotAxes currAngles;
  std::string line;
  unsigned int row=0,col=0;
  double speed = 0;
  double temp=0;
  double sum= 0;

  while (row < 200)
  {
	getline(thumb_file,line);
	std::stringstream lineStream(line);
	std::string cell;
	while (col < 17)
	{
		getline(lineStream,cell,',');
		std::stringstream convertor(cell);
        convertor >> temp;
		
		//speed
		if (col == 16) 	{speed = temp;}

		//joint angles
		else	{jointAngles.axis[col] =temp;}
		col+=1;
			//std::cout << col << " " << temp << std::endl;

	}

	allegro.SetAbsoluteSpeed(speed);  
    allegro.MoveToAxisTarget(jointAngles);
	while (1)
	{
		allegro.GetRobotAxes(&currAngles);
		for (int jj=0; jj < 16; ++jj)
		{
			sum = abs(jointAngles.axis[jj] - currAngles.axis[jj]) + sum;
		}
		if (sum < .2) {break;}

		Sleep(500);
		sum = 0;
	}
	

	std::cout << row << std::endl;
	//std::cin.get();
	col=0;
	row+=1;
	sum = 0;
  }

  thumb_file.close();
  //--------------------------------------------------------------------------------------------------------------
  */

/*
  //FINGER FORCE CONTROL-----------------------------------------------
  
  //jointAngles.axis[5] = 1;
  //jointAngles.axis[6] = jointAngles.axis[6] + .5;
  //jointAngles.axis[7] = jointAngles.axis[7] + .5;
  jointAngles.axis[5] = 1.1;
  jointAngles.axis[6] = 1;
  jointAngles.axis[7] = 0.25;
  allegro.MoveToAxisTarget(jointAngles);
  cin.get();

  robotPose forcetorque;
  forcetorque.x = -1;
  forcetorque.y = 0;
  forcetorque.z = 0;
  vector<bool> filler;
  vector<bool> active_fingers(4);
  active_fingers[0] = false;
  active_fingers[1] = true;
  active_fingers[2] = false;
  active_fingers[3] = false;
  

  allegro.ApplyCartesianForceTorque (forcetorque, filler, active_fingers);
  cin.get();
  */

  //FORCES FROM FILE
  /*
  allegro.SetParameter("Control","finger_force");
  allegro.SetParameter("Plan","file");
  allegro.SetParameter("forces.csv","forces.csv");

  Sleep(10000);

  //allegro.SetParameter("touch_stop","dummy");
  */
  

  /*
  //jointAngles.axis[6] = 1.62;

  //allegro.MoveToAxisTarget(jointAngles);
  
  
	double temp=0;
	//std::vector<double> temp_states;
	//std::vector<std::vector<double>> q_d_array;

	//Importing motion data
	std::ifstream states_from_file("lhs_data.csv");

	std::string line;
	unsigned int row=0,col=0;

	while (getline(states_from_file,line))
	{
		std::stringstream lineStream(line);
		std::string cell;

		while (col < 16)
		{
			getline(lineStream,cell,',');
			std::stringstream convertor(cell);
			convertor >> temp;
			//temp_states.push_back(temp);
			jointAngles.axis[col] = temp;
			col+=1;
			//std::cout << temp << " ";
		}

		std::cout << row << std::endl;

		allegro.MoveToAxisTarget(jointAngles);
		Sleep(1000);

		col=0;
		row+=1;
	}

	states_from_file.close();

	

  //std::cout << "HIZZAHH" << std::endl;

  //allegro.SetParameter("Control","joint_pos");
  //allegro.SetParameter("Plan","set_point_smooth");


  /*
  //Sleep(5000);
  while (1)
  {
	  if (kbhit())
	  {
		key = getch();
		if(key == 27)
		{
			break;
		}
	  }
	  Sleep(100);
  }
  */
  
  //when sending pos command, wait for confirmation that move is finished from hand
  //be able to get pose from hand
  //look at touch activation and make it so that it doesn't replace the current joint angles over and over again
  
  allegro.SetAbsoluteSpeed(60);
  allegro.SetParameter("touch_stop","off");
  allegro.MoveToAxisTarget(homePose);
  Sleep(4000);

  robotAxes curAxes;
  allegro.GetRobotAxes(&curAxes);
  curAxes.print();

  allegro.SetParameter("Control","shutdown");

#elif defined ALLEGRO_DEXTERITY_DEMO

  int key = 0;

  //Testing connection and control to Allegro Hand
  //CrpiRobot<CrpiSchunkSDH> sdh("dummy text");

  CrpiRobot<CrpiAllegro> allegro("allegro.xml");

  robotAxes jointAngles, homePose;

  jointAngles.axis[0] = 0; jointAngles.axis[1] = .4; jointAngles.axis[2] = .4; jointAngles.axis[3] = .4;
  jointAngles.axis[4] = 0; jointAngles.axis[5] = .4; jointAngles.axis[6] = .4; jointAngles.axis[7] = .4;
  jointAngles.axis[8] = 0; jointAngles.axis[9] = .4; jointAngles.axis[10] = .4; jointAngles.axis[11] = .4;
  jointAngles.axis[12] = .5; jointAngles.axis[13] = 1; jointAngles.axis[14] = 0.2; jointAngles.axis[15] = 0.2;

  homePose = jointAngles;

  homePose.axis[1] = homePose.axis[2] = homePose.axis[3] = 0;
  homePose.axis[5] = homePose.axis[6] = homePose.axis[7] = 0;
  homePose.axis[9] = homePose.axis[10] = homePose.axis[11] = 0;

  jointAngles = homePose;

  allegro.SetAbsoluteSpeed(90);
  allegro.SetParameter("touch_stop","on");

  allegro.MoveToAxisTarget(homePose);
  Sleep(2000);

  for (unsigned int k=0; k<1000; ++k)
  {
	  jointAngles = homePose;
	  std::cout << k << std::endl;
	  //Move Thumb
	  jointAngles.axis[12] = 1.5;
	  allegro.MoveToAxisTarget(jointAngles);
	  Sleep(1000);

	  jointAngles.axis[14] = jointAngles.axis[15] = 1.25;
	  allegro.MoveToAxisTarget(jointAngles);
	  Sleep(2000);

	  allegro.MoveToAxisTarget(homePose);
	  Sleep(2000);

	  //Move Index Finger
	  jointAngles = homePose;

	  jointAngles.axis[1] = jointAngles.axis[2] = 1.6;
	  jointAngles.axis[3] = 1;
	  allegro.MoveToAxisTarget(jointAngles);
	  Sleep(2000);
	  allegro.MoveToAxisTarget(homePose);
	  Sleep(2000);

	  //Move Middle Finger
	  jointAngles = homePose;

	  jointAngles.axis[5] = jointAngles.axis[6] = 1.6;
	  jointAngles.axis[7] = 1;
	  allegro.MoveToAxisTarget(jointAngles);
	  Sleep(2000);
	  allegro.MoveToAxisTarget(homePose);
	  Sleep(2000);

	  //Move Little Finger
	  jointAngles = homePose;

	  jointAngles.axis[9] = jointAngles.axis[10] = 1.6;
	  jointAngles.axis[11] = 1;
	  allegro.MoveToAxisTarget(jointAngles);
	  Sleep(2000);
	  allegro.MoveToAxisTarget(homePose);
	  Sleep(2000);

	  //Move all
	  jointAngles = homePose;

	  jointAngles.axis[1] = jointAngles.axis[2] = jointAngles.axis[5] = jointAngles.axis[6] = jointAngles.axis[9] = jointAngles.axis[10] = 1.6;
	  jointAngles.axis[3] = jointAngles.axis[7] = jointAngles.axis[11] = 1;
	  allegro.MoveToAxisTarget(jointAngles);
	  Sleep(2000);
	  allegro.MoveToAxisTarget(homePose);
	  Sleep(2000);

	  //Peace!
	  jointAngles = homePose;

	  jointAngles.axis[9] = jointAngles.axis[10] = 1.6;
	  jointAngles.axis[11] = 1;
	  jointAngles.axis[12] = 1.5;
	  jointAngles.axis[14] = jointAngles.axis[15] = 1.25;
	  allegro.MoveToAxisTarget(jointAngles);
	  Sleep(2000);
	  allegro.MoveToAxisTarget(homePose);
	  Sleep(2000);
  }


  allegro.SetParameter("touch_stop","off");
  allegro.MoveToAxisTarget(homePose);
  Sleep(2000);

  allegro.SetParameter("Control","shutdown");

#elif defined SerialTest	
	//AssemblyTimer asblytimer;
	
	serialStruct data;
	char inbuffer[17] = "";
	//strcpy(inbuffer,"");

	cout << data.setBaud(115200) << endl;
	cout << data.setChannel(22) << endl;

	Network::serial *serinator_ = new Network::serial();
	
	cout << "Establishing connection!" << endl;

	//while (serinator_->attach(data) == 0) {};

	vector<char *>test;
	int msg_size=0;

	char collect[10000];
	strcpy(collect,"");

	if (serinator_->attach(data))
	{
		cout << "Success!" << endl;

		cout << "Getting Data" << endl;
		//Grab data
		for(int jj=0; jj<100; jj++) 
		{
			cout << "sending" << endl;
		serinator_->sendData("1",data);
		cout << "after" << endl;
		
		//strcpy(inbuffer,"");
		for(int j=0; j<sizeof(inbuffer); ++j) {inbuffer[j] = '\0';}
		serinator_->getData(inbuffer,data,sizeof(inbuffer));
		//cout << "success" << endl;

		for (int j=0; j<sizeof(inbuffer); ++j)
		{
			cout << inbuffer[j];
		}

		cout << endl;
		cin.get();
		//cout << inbuffer << endl;
		//msg_size = atoi(inbuffer);
		//cout << msg_size << endl;

		//strcpy(inbuffer,"");
		//serinator_->getData(inbuffer,data,msg_size);

		//cout << inbuffer << endl;

		//strcat(collect,inbuffer);
		stringstream ss(inbuffer);
		double tempy;
		for(int j=0; j<2; j++) {ss >> tempy; cout << tempy << endl;}
		
		}

		//Print data
		//cout << collect << endl;

		//Print conversion
		stringstream ss(collect);
		double tempy;
		for(int j=0; j<40; j++) {ss >> tempy; cout << tempy << endl;}

	}

	

	else {cout << "UTTER FAILURE" << endl;}

	serinator_->closeConnection(data);

	delete serinator_;
	
#elif defined DC_Demo
  
  //Connecting to robots and initialization------------------------------------------------------
  cout << "Connecting to UR5" << endl;
  CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");

  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");
  arm.Couple("robotiq");
  robotPose poseMe, curPose;
  robotAxes curAxes, tarAxes;
  bool poseDefined = false;
  bool tool = false;

  cout << "Connecting to Robotiq" << endl;
  CrpiRobot<CrpiRobotiq> riq("robotiq.xml");
  int param, paramA, paramB, paramC, paramS;

  param = 1;
  riq.SetParameter("ADVANCED_CONTROL", &param);
  riq.SetParameter("SCISSOR_CONTROL", &param);
  param=255;
  riq.SetParameter("SPEED_FINGER_A", &param);
  riq.SetParameter("SPEED_FINGER_B", &param);
  riq.SetParameter("SPEED_FINGER_C", &param);
  riq.SetParameter("SPEED_SCISSOR", &param);
  
  param = 255;
  riq.SetParameter("FORCE_FINGER_A", &param);
  riq.SetParameter("FORCE_FINGER_B", &param);
  riq.SetParameter("FORCE_FINGER_C", &param);
  riq.SetParameter("FORCE_SCISSOR", &param);
  //----------------------------------------------------------------------------

  int choice=0;

  while(1)
  {
	//Make choice
	cout << "ENTER CHOICE: 1) HAND MOTION, 2) GRASP STRENGTH, ENTER ANY OTHER KEY TO QUIT" << endl; 
	cin >> choice;

	if (choice == 1)
	{

	  //Move arm to presentation pose #1
	  arm.GetRobotAxes(&curAxes);
	  tarAxes = curAxes;

	  tarAxes.axis[0] = 74.182257;
	  tarAxes.axis[1] = -60.821892;
	  tarAxes.axis[2] = -114.773089;
	  tarAxes.axis[3] = -161.866039;
	  tarAxes.axis[4] = 19.641343;
	  tarAxes.axis[5] = -38.580694;
	  arm.MoveToAxisTarget(tarAxes);
  
	  while(1)
	  {
		  arm.GetRobotAxes(&curAxes);
		  if (abs(curAxes.axis[0]-74.182257) < 1) {break;}
	  }

	  cout << "Arm move #1 done!" << endl;

	  //Articulate Robotiq

	  //Scissor Move
	  paramA=0;
	  paramB=0;
	  paramC=0;
	  paramS=255;
	  riq.SetParameter("POSITION_FINGER_A", &paramA);
	  riq.SetParameter("POSITION_FINGER_B", &paramB);
	  riq.SetParameter("POSITION_FINGER_C", &paramC);
	  riq.SetParameter("POSITION_SCISSOR", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);

	  paramS=0;
	  riq.SetParameter("POSITION_SCISSOR", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);
	  //timer.waitUntil(2000);

	  //Finger A move
	  paramA=255;
	  paramB=0;
	  paramC=0;
	  paramS=0;
	  riq.SetParameter("POSITION_FINGER_A", &paramA);
	  riq.SetParameter("POSITION_FINGER_B", &paramB);
	  riq.SetParameter("POSITION_FINGER_C", &paramC);
	  riq.SetParameter("POSITION_SCISSOR", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);
	  //timer.waitUntil(2000);

	  paramA=0;
	  riq.SetParameter("POSITION_FINGER_A", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);
	  //timer.waitUntil(2000);

	  //Finger B move
	  paramA=0;
	  paramB=255;
	  paramC=0;
	  paramS=0;
	  riq.SetParameter("POSITION_FINGER_A", &paramA);
	  riq.SetParameter("POSITION_FINGER_B", &paramB);
	  riq.SetParameter("POSITION_FINGER_C", &paramC);
	  riq.SetParameter("POSITION_SCISSOR", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);
	  //timer.waitUntil(2000);

	  paramB=0;
	  riq.SetParameter("POSITION_FINGER_B", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);
	  //timer.waitUntil(2000);

	  //Finger C move
	  paramA=0;
	  paramB=0;
	  paramC=255;
	  paramS=0;
	  riq.SetParameter("POSITION_FINGER_A", &paramA);
	  riq.SetParameter("POSITION_FINGER_B", &paramB);
	  riq.SetParameter("POSITION_FINGER_C", &paramC);
	  riq.SetParameter("POSITION_SCISSOR", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);
	  //timer.waitUntil(2000);

	  paramC=0;
	  riq.SetParameter("POSITION_FINGER_C", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);
	  //timer.waitUntil(2000);
	}
  
	else if (choice == 2)
	{
	  //Arm move #2
	  tarAxes.axis[0] = 55.559641;
	  tarAxes.axis[1] = -101.700518;
	  tarAxes.axis[2] = -127.923634;
	  tarAxes.axis[3] = 58.554341;
	  tarAxes.axis[4] = 58.457694;
	  tarAxes.axis[5] = -98.375227;
	  arm.MoveToAxisTarget(tarAxes);
  
	  while(1)
	  {
		  arm.GetRobotAxes(&curAxes);
		  if (abs(curAxes.axis[0]-55.559641) < 1) {break;}
	  }
  
	  //Grip for strength
	  paramA=255;
	  paramB=255;
	  paramC=255;
	  paramS=100;
	  riq.SetParameter("POSITION_FINGER_A", &paramA);
	  riq.SetParameter("POSITION_FINGER_B", &paramB);
	  riq.SetParameter("POSITION_FINGER_C", &paramC);
	  riq.SetParameter("POSITION_SCISSOR", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);
	  timer.waitUntil(2000);

	  paramA=0;
	  paramB=0;
	  paramC=0;
	  paramS=100;
	  riq.SetParameter("POSITION_FINGER_A", &paramA);
	  riq.SetParameter("POSITION_FINGER_B", &paramB);
	  riq.SetParameter("POSITION_FINGER_C", &paramC);
	  riq.SetParameter("POSITION_SCISSOR", &paramS);
	  param=1;
	  riq.SetParameter("GRIP", &param);
	  timer.waitUntil(2000);
	}

	else {break;}

  }

#endif

  cout << "All done" << endl;

}


void CognexThread (void *param)
{
  //! Explicitly cast your shared data back from a void* to the proper structure
  passMe *pm = (passMe*)param;
  char inbuffer[1024];
  crpi_timer timer;

  //! Inform the main function that this thread is running.
  ulapi_mutex_take(pm->grabmutex);
  pm->t1 = true;
  ulapi_mutex_give(pm->grabmutex);

  while (pm->keeprunning)
  {
    ulapi_mutex_take(pm->grabmutex);
    ulapi_socket_read (pm->clientID_, inbuffer, 1024);
    ulapi_mutex_give(pm->grabmutex);

    //! Don't slam your processor!  You don't need to poll at full speed.
    timer.waitUntil(500);
  } // while (true)

  return;
} // void CognexThread (void *param)



void removeCognexWhitespace (char *linein)
{
  int c_in, c_out;
  char lineout[1024];
  
  c_out = 0;
  for (c_in = 0; c_in < 1024; ++c_in)
  {
    if (linein[c_in] != ' ')
    {
      lineout[c_out++] = linein[c_in];
    }
    linein[c_in] = ' ';
  }

  for (c_in = 0 ; c_in < c_out; ++c_in)
  {
    linein[c_in] = lineout[c_in];
  }
}


void parseCognexFrame (char *line)
{
  int c;
  char *pch;

  for (c = 0; c < 5; ++c)
  {
#ifdef SUPERNOISY
    cout << line << endl;
#endif
    if (c == 0)
    {
      pch = strtok (line, ",\n"); //! Name
#ifdef ALTTEXT
      while (strcmp(pch, "BigGear") != 0 && strcmp(pch, "\nBigGear") != 0)
      {
        pch = strtok (NULL, ",\n");
      }
#else
      while (strcmp(pch, "Big_Gear") != 0 && strcmp(pch, "\nBig_Gear") != 0)
      {
        pch = strtok (NULL, ",\n");
      }
#endif
    }
    else
    {
      pch = strtok (NULL, ","); //! Name
    }
    pch[strlen(pch)] = '\0';

#ifdef NOISY
    cout << pch << " " << strlen(pch) << endl;
#endif

    pch = strtok (NULL,","); //! Found
    frameData.found[c] = (atoi(pch) == 1) ? true : false;
    if (frameData.found[c])
    {
      pch = strtok (NULL,","); //! Zrot
      cout << "--" << c << "-- zrot:" << pch;

      frameData.rp[c].zrot = frameData.zrotadd + (frameData.zrotmult * ((atof(pch)/3.14159265) * 180.0f));
      frameData.rp[c].zrot += (userobotiq ? frameData.rt_pickOffset[c].zrot : frameData.pa_pickOffset[c].zrot);
      //! Correct out-of-range angles for the KUKA LWR
      frameData.rp[c].zrot = (frameData.rp[c].zrot > 180.0f) ? (frameData.rp[c].zrot - 360.0f) : frameData.rp[c].zrot;
      frameData.rp[c].zrot = (frameData.rp[c].zrot < -90.0f) ? (frameData.rp[c].zrot + 360.0f) : frameData.rp[c].zrot;

      cout << "(" << frameData.rp[c].zrot << ")";

      //! Absolute offsets in X and Y
      pch = strtok (NULL,","); //! X
      cout << " x:" << pch;
      frameData.rp[c].x = frameData.xadd + (frameData.xmult * atof(pch));
      frameData.rp[c].x += (userobotiq ? frameData.rt_pickOffset[c].x : frameData.pa_pickOffset[c].x);
      cout << "(" << frameData.rp[c].x << ")";
      
      pch = strtok (NULL,","); //! Y
      cout << " y:" << pch;
      frameData.rp[c].y = frameData.yadd + (frameData.ymult * atof(pch));
      frameData.rp[c].y += (userobotiq ? frameData.rt_pickOffset[c].y : frameData.pa_pickOffset[c].y);
      cout << "(" << frameData.rp[c].y << ")" << endl;

      //! Offset along axis of rotation
      frameData.rp[c].x -= ((userobotiq ? frameData.rt_hypOffset[c] : frameData.pa_hypOffset[c]) *
                           sin(frameData.rp[c].zrot * (3.14159265 / 180.0f)));
      frameData.rp[c].y += ((userobotiq ? frameData.rt_hypOffset[c] : frameData.pa_hypOffset[c]) *
                           cos(frameData.rp[c].zrot * (3.14159265 / 180.0f)));

      pch = strtok (NULL,","); //! Conf
    } // if (frameData.found[c])
  } // for (int c = 0; c < 5; ++c)

  for (c = 0; c < 5; ++c)
  {
    if (frameData.found[c])
    {
      cout << c << ": X=" << frameData.rp[c].x << " Y=" << frameData.rp[c].y << " ZRot=" << frameData.rp[c].zrot << endl;
    }
    else
    {
      cout << c << " not found" << endl;
    }
  } // for (int c = 0; c < 5; ++c)
} // void parseCognexFrame (char *line)



void parseCommandPose (char *line, robotPose *pose)
{
  char *pch;

  pch = strtok (line, " "); //! X
  pose->x = atof(pch);
  pch = strtok (NULL, " "); //! Y
  pose->y = atof(pch);
  pch = strtok (NULL, " "); //! Z
  pose->z = atof(pch);
  pch = strtok (NULL, " "); //! A
  pose->xrot = atof(pch);
  pch = strtok (NULL, " "); //! B
  pose->yrot = atof(pch);
  pch = strtok (NULL, " "); //! C
  pose->zrot = atof(pch);
} // void parseCommandPose (char *line, robotPose *pose)