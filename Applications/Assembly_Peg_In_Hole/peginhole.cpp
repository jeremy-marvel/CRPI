

///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Peg In Hole
//  Workfile:        peginhole.cpp
//  Revision:        16 July, 2015
//  Author:          J. Falco
//
//  Description
//  ===========
//  Applicatons to support LWR and UR10 Assembly work
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#include "../../Libraries/Math/NumericalMath.h" 
#include "../../Libraries/MotionPrims/AssemblyPrims.h"

#pragma warning (disable: 4996)

#define PEGTEST
//#define MAPVISERR //This code used for Tsai Cognex assessment

#define ALTTEXT

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;

//typedef CrpiUniversal robType;
typedef CrpiKukaLWR robType;


struct passMe
{
  ulapi_mutex_struct* grabmutex;
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


void main ()
{
  int i = 0;
  

#ifdef PEGTEST
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
  //asbly.AddSearchRandom (8.0, false);
  asbly.AddSearchConstOffset(0.0, 0.0, -10.0f); //-23 previous
  asbly.AddTerminatorTimer(CANON_FAILURE, 60.0); 
  asbly.AddTerminatorDistance(CANON_SUCCESS,-1,-1,5,-1); //10 previous

  robotPose poseMe, curPose, offsetPose;

  robotPose hole_1, hole_2;

  robotIO io;
  bool poseDefined = false;
  bool tool = false;
  int counter; 
  
  double theta = 0.0f;
  matrix pin(3,1), pout(3,1);
  //

  // Start
  cout << "1) Run Spiral Insertion, 2) Tool, -1) quit : ";
  cin >> i;
  AssemblyTimer timer;
  double tim;
  CanonReturn peg_in_hole = CANON_RUNNING;

  int const insertions = 150;
  char delim;

  ifstream infile ("perception_spoof_1mm.csv");
  ofstream results("percep_spoof_1mm_kuka_spiral_cool.csv");
  
  // Nominal Hole Positions

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
      Sleep(100);

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
      tim = timer.startTimer();
      //cout << poseMe.x << " " << poseMe.y << " " << poseMe.z;
      Sleep(100);

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
        Sleep(100);

      }
      tim = timer.timeElapsed();
          timer.stopTimer();
          cout << j << " " << tim << " " << offsetPose.x << " " << offsetPose.y << " " << endl;
      cout << poseMe.x << " " << poseMe.y << " " << poseMe.z << endl;
      // write to file
      results << tim << endl;

      //retract from current pos
      poseMe = curPose;
      poseMe.z = 50.0f;
    
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

#elif defined MAPVISERR  //This code used for Tsai Cognex assessment

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
    //Sleep(500);
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
        Sleep(pauseTime);
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

#endif
}