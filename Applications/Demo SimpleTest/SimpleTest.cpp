///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       CRPI
//  Workfile:        SimpleTest.cpp
//  Revision:        1.0 - 20 September, 2016
//
//  Description
//  ===========
//  Simple test to verify using the CRPI to move the robots.
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include "crpi_robot.h"
#include "crpi_abb.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"

#define XMLINTERFACE_DEBUGTEST

using namespace std;
using namespace crpi_robot;

typedef CrpiUniversal robArmType;

void main()
{
  cout << "Create robot" << endl;
  //CrpiRobot<CrpiUniversal> arm("universal_ur5_table.xml");
  //CrpiRobot<CrpiAbb> arm("abb_irb14000_left.xml");
  //CrpiRobot<CrpiAbb> arm("abb_irb14000_right.xml");
  CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");
  //CrpiRobot<CrpiUniversal> arm("universal_ur10_right.xml");
  //CrpiRobot<CrpiUniversal> arm("universal_ur10_left.xml");
  //CrpiRobot<CrpiKukaLWR> arm("kuka_lwr.xml");
  //CrpiRobot<CrpiUniversal> arm("universal_ur10_agv.xml");

  cout << "initializing" << endl;
  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");

  cout << "coupling tool" << endl;
  arm.Couple("robotiq");

  robotPose poseMe, curPose;
  robotAxes curAxes, tarAxes;
  bool poseDefined = false;
  bool tool = false;

  double theta = 0.0f;
  matrix pin(3, 1), pout(3, 1);

  int i;

  cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Pose feedback 4) DO 0 High 5) DO 0 Low 6) Move J1 -10 deg 7) Move J1 +10 deg -1) quit : ";
  cin >> i;
  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      arm.GetRobotPose(&curPose);

      poseDefined = true;
    }

    if (i == 0)
    {
      curAxes.axis.at(0) = 134.524;
      curAxes.axis.at(1) = -76.6487;
      curAxes.axis.at(2) = 117.04;
      curAxes.axis.at(3) = -162.435;
      curAxes.axis.at(4) = 61.8142;
      curAxes.axis.at(5) = 105.0;
      curAxes.axis.at(6) = 0.0;
      arm.MoveToAxisTarget(curAxes);
    }
    else if (i >= 1 && i < 3)
    {
      poseMe = curPose;

      pin.at(0, 0) = poseMe.x;
      pin.at(1, 0) = poseMe.y;
      pin.at(2, 0) = poseMe.z;

      switch (i)
      {
      case 1:
        poseMe.z += 20.0;
        break;
      case 2:
        poseMe.z += -20.0;
        break;
      default:
        break;
      }

      pin.at(0, 0) = poseMe.x;
      pin.at(1, 0) = poseMe.y;
      pin.at(2, 0) = poseMe.z;

      if (arm.MoveStraightTo(poseMe) == CANON_SUCCESS)
      {
      }
      // curPose = poseMe;
      arm.GetRobotPose(&curPose);
      cout << "curPose=" << curPose.z << endl;
      // }
    } // if (i >= 1 && i < 3)
    else if (i == 3)
    {
      poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
      arm.GetRobotPose(&poseMe);
      curPose = poseMe;
      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << " S:" << poseMe.status << " T:" << poseMe.turns << endl;
      arm.GetRobotAxes(&curAxes);
      cout << " >> J1:" << curAxes.axis.at(0) << " >> J2:" << curAxes.axis.at(1) << " >> J3:" << curAxes.axis.at(2) << " >> J4:"
        << curAxes.axis.at(3) << " >> J5:" << curAxes.axis.at(4) << " >> J6:" << curAxes.axis.at(5) << " >> J7:" << curAxes.axis.at(6) << endl;
      arm.GetRobotForces(&poseMe);
      cout << " >> XF:" << poseMe.x << " YF:" << poseMe.y << " ZF:" << poseMe.z << " XT:" << poseMe.xrot << " YT:" << poseMe.yrot << " ZT:" << poseMe.zrot << " S:" << poseMe.status << " T:" << poseMe.turns << endl;

    }
    else if (i == 4)  // DO 0 High
    {
      arm.SetRobotDO(0, 1);
    }
    else if (i == 5) // DO 0 Low
    {
      arm.SetRobotDO(0, 0);
    }
    else if (i == 6) // Move J1 -10 deg
    {
      arm.GetRobotAxes(&curAxes);
      tarAxes = curAxes;
      tarAxes.axis.at(1) -= 10.0f;
      arm.MoveToAxisTarget(tarAxes);
    }
    else if (i == 7) // Move J1 +10 deg
    {
      arm.GetRobotAxes(&curAxes);
      tarAxes = curAxes;
      tarAxes.axis.at(1) += 10.0f;
      arm.MoveToAxisTarget(tarAxes);
    }
    else if (i == 8)
    {
      cout << "Test freedrive" << endl;
      if (arm.SetParameter("freedrive", NULL) != CANON_SUCCESS)
      {
        cout << "Error!" << endl;
      };
    }
    else if (i == 9)
    {
      arm.SetParameter("endfreedrive", NULL);
    }
    else
    {
      cout << "Invalid entry" << endl;
    }

    cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Pose feedback 4) DO 0 High 5) DO 0 Low 6) Move J1 -10 deg 7) Move J1 +10 deg -1) quit : ";
    cin >> i;
  } // while (i != -1)
}