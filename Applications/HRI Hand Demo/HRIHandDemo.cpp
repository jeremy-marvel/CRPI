///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Human-Robot Interaction Demo:  HMI for grasping
//  Workfile:        5RobotAssembly.cpp
//  Revision:        1.0    5/14/2018
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Simple demo showing off different HRI technologies
///////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>

#include <string>
#include <fstream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#include "LeapMotion.h"

#pragma warning (disable: 4996)

//#define NOISY
//#define SUPERNOISY

using namespace crpi_robot;
using namespace std;
using namespace Sensor;


struct hriHandle
{
  ulapi_mutex_struct *handle;
  bool runThread;
  int left;
  int right;

  hriHandle()
  {
    handle = ulapi_mutex_new(89);
    left = 0;
    right = 0;
  }

  ~hriHandle()
  {
  }
};



void HandThread(void *hhparam)
{
  hriHandle *hH = (hriHandle*)hhparam;
  int param;

  CrpiRobot<CrpiRobotiq> robotiql("robotiq_lab.xml");
  param = 1;
  robotiql.SetParameter("ADVANCED_CONTROL", &param);
  param = 255;
  robotiql.SetParameter("SPEED_FINGER_A", &param);
  robotiql.SetParameter("SPEED_FINGER_B", &param);
  robotiql.SetParameter("SPEED_FINGER_C", &param);
  param = 30;
  robotiql.SetParameter("FORCE_FINGER_A", &param);
  robotiql.SetParameter("FORCE_FINGER_B", &param);
  robotiql.SetParameter("FORCE_FINGER_C", &param);
  param = 0;
  robotiql.SetParameter("POSITION_FINGER_A", &param);
  robotiql.SetParameter("POSITION_FINGER_B", &param);
  robotiql.SetParameter("POSITION_FINGER_C", &param);
  param = 1;
  robotiql.SetParameter("GRIP", &param);

  while (hH->runThread)
  {
    param = hH->right;
    robotiql.SetParameter("POSITION_FINGER_A", &param);
    robotiql.SetParameter("POSITION_FINGER_B", &param);
    robotiql.SetParameter("POSITION_FINGER_C", &param);
    param = 1;
    robotiql.SetParameter("GRIP", &param);
  }
  hH = NULL;
  return;
}


//! @brief Thread method for controlling a robotiq gripper using a Leap Motion
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void LeapThread(void *hhparam)
{
  hriHandle *hH = (hriHandle*)hhparam;
  LeapMotion LM;
  HandList *HL = new HandList();
  Hand H;
  HandList::const_iterator HLi;
  int x = 0;

  while (hH->runThread)
  {
    LM.getHands(HL);
    //system("CLS");
    if (!HL->isEmpty())
    {
      for (x = 0; x < HL->count(); ++x)
      {
        H = (*HL)[x];
        //cout << (H.isLeft() ? "Left" : "Right") << " : " << H.grabStrength() << endl;
        if (H.isLeft())
        {
          hH->left = H.grabStrength() * 255;
        }
        else
        {
          hH->right = H.grabStrength() * 255;
        }
      }
    }
    else
    {
      //cout << "No Hands" << endl;
    }
    Sleep(100);
  }

  hH = NULL;
  return;
}


void main ()
{
  int i = 0;
  crpi_timer timer;
  hriHandle hH;
  hH.runThread = true;
  char temp;

  void* handtask = ulapi_task_new();
  void* leaptask = ulapi_task_new();
  ulapi_task_start((ulapi_task_struct*)leaptask, LeapThread, &hH, ulapi_prio_lowest(), 0);
  Sleep(100);
  ulapi_task_start((ulapi_task_struct*)handtask, HandThread, &hH, ulapi_prio_lowest(), 0);

  cout << "PRESS ENTER TO QUIT";
  cin.get(temp);

  ulapi_task_stop((ulapi_task_struct*)handtask);
  ulapi_task_stop((ulapi_task_struct*)leaptask);

  cout << "All done" << endl;
}


