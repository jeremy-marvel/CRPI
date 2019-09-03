////////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Arm-Hand Demonstration
//  Workfile:        main.cpp
//  Revision:        21 September, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Simple application for demonstrating robot functionality using the Universal
//  Robots UR10 and the Robotiq 3-fingered gripper
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"

#pragma warning (disable: 4996)

//#define NOISY
//#define SUPERNOISY
//#define REPROGRAM_POSES

using namespace crpi_robot;
using namespace std;

void main ()
{
  int i = 0;
  bool firstRun = true;
  robotPose tarPose[8], curPose;
  int curStep = -1;
  int idleCount;
  crpi_timer timmy;
  double totDist, lastDist, dist;
  int handopen[8];

  lastDist = 0.0f;

  CrpiRobot<CrpiUniversal> arm("universal_ur10.xml");
  CrpiRobot<CrpiRobotiq> hand("robotiq.xml");
  
  //arm.Couple("robotiq");
  hand.Couple("gripper_gear");

#ifdef REPROGRAM_POSES

  char c;
  cout << "NIST ISD Collaborative Robot Programming Interface Socket Handler" << endl << endl;
  cout << "!!!!!!!!!!!!!!!!! NOTICE !!!!!!!!!!!!!!!!!!!" << endl;
  cout << "!    REPROGRAMMING DEMONSTRATION POSES     !" << endl;
  cout << "!!!!!!!!!!!!!!!!! NOTICE !!!!!!!!!!!!!!!!!!!" << endl << endl;

  arm.GetRobotPose(&curPose);
  cout << "Current Pose:  (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " 
          << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;

  cout << "Move to P1 hover.  Press any key to train." << endl;
  cin.get(c);
  arm.GetRobotPose(&curPose);
  tarPose[0] = tarPose[3] = curPose;

  cout << "Move to P1 pick.  Press any key to train." << endl;
  cin.get(c);
  arm.GetRobotPose(&curPose);
  tarPose[1] = tarPose[2] = curPose;

  cout << "Move to P2 hover.  Press any key to train." << endl;
  cin.get(c);
  arm.GetRobotPose(&curPose);
  tarPose[4] = tarPose[7] = curPose;

  cout << "Move to P2 place.  Press any key to train." << endl;
  cin.get(c);
  arm.GetRobotPose(&curPose);
  tarPose[5] = tarPose[6] = curPose;

  handopen[0] = handopen[1] = handopen[6] = handopen[7] = 1;
  handopen[2] = handopen[3] = handopen[4] = handopen[5] = 0;

  ofstream outfile("settings.dat");
  for (i = 0; i < 8; ++i)
  {
    outfile << tarPose[i].x << " " << tarPose[i].y << " " << tarPose[i].z << " " 
            << tarPose[i].xrot << " " << tarPose[i].yrot << " " << tarPose[i].zrot 
            << " " << handopen[i] << endl;
  }
  outfile.close();
  return;
#endif

  ifstream infile("settings.dat");
  for (i = 0; i < 8; ++i)
  {
    infile >> tarPose[i].x >> tarPose[i].y >> tarPose[i].z >> tarPose[i].xrot >> tarPose[i].yrot >> tarPose[i].zrot >> handopen[i];
  }
  infile.close();

  arm.GetRobotPose(&curPose);
  arm.SetRelativeSpeed(0.2);
  int param;
  //PreGrasp
  param = 1;
  hand.SetParameter("ADVANCED_CONTROL", &param);
  param=100;
  hand.SetParameter("SPEED_FINGER_A", &param);
  hand.SetParameter("SPEED_FINGER_B", &param);
  hand.SetParameter("SPEED_FINGER_C", &param);
  param=0;
  hand.SetParameter("FORCE_FINGER_A", &param);
  hand.SetParameter("FORCE_FINGER_B", &param);
  hand.SetParameter("FORCE_FINGER_C", &param);
  param=1;
  hand.SetParameter("GRIP", &param);

  while (true)
  {
    arm.GetRobotPose(&curPose);
    dist = curPose.distance(tarPose[curStep]);

    if (firstRun || dist <= 1.0f)
    {
      firstRun = false;
      //! At target, move to next pose
      //! Cycle through steps until pick-and-place operation is complete.  Repeat.
      curStep = (curStep >= 7) ? 0 : (curStep+1);

      //! Move to target location
      arm.MoveStraightTo(tarPose[curStep]);

      if (handopen[curStep] == 1)
      {
        //Grasp
        param=255;
        hand.SetParameter("POSITION_FINGER_A", &param);
        //param=75;
        hand.SetParameter("POSITION_FINGER_B", &param);
        hand.SetParameter("POSITION_FINGER_C", &param);
        param=1;
        hand.SetParameter("GRIP", &param);
        //timer.waitUntil(1000);
        /*
        param=75;
        hand.SetParameter("POSITION_FINGER_A", &param);
        hand.SetParameter("POSITION_FINGER_B", &param);
        hand.SetParameter("POSITION_FINGER_C", &param);
        param=1;
        hand.SetParameter("GRIP", &param);
        //timer.waitUntil(1000); 
        */
      }
      else
      {

        param=20;
        hand.SetParameter("POSITION_FINGER_A", &param);
        hand.SetParameter("POSITION_FINGER_B", &param);
        hand.SetParameter("POSITION_FINGER_C", &param);
        param=1;
        hand.SetParameter("GRIP", &param);
      }

      //! Reset distance traveled
      totDist = 0.0f;
    } //  if (firstRun || handle.curPose.distance(tarPose[iter]) <= 1.0f)
    else
    {
      if (fabs(dist - lastDist) < 0.1)
      {
        //! We haven't moved
        ++idleCount;

        if (idleCount > 20)
        {
          //! 10 seconds with no motion.  Reissue move command.
          arm.MoveStraightTo(tarPose[curStep]);
        }
      }
      else
      {
        //! We're moving.  Reset counter;
        idleCount = 0;
        lastDist = dist;
      }
      timmy.waitUntil(500);
    }

  } // while (true)

  cout << "All done" << endl;

}