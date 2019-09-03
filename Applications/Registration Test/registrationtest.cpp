///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Registration Test
//  Workfile:        registrationtest.cpp
//  Revision:        29 May, 2019
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Evaluation of registration methodologies using the UR5/10
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ulapi.h"
#include "../../Libraries/Math/NumericalMath.h" 
#include "../../Libraries/MotionPrims/AssemblyPrims.h"

#pragma warning (disable: 4996)

//#define TESTOFFLINE

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;


void main ()
{
#ifndef TESTOFFLINE
  CrpiRobot<CrpiUniversal> arm("universal_ur10_left.xml");
#endif

  stringstream sstr;
  sstr.str(string());
  sstr << "regtest_log_";
  sstr << getCurrentTime();
  sstr << ".csv";

  vector<robotAxes> axes;
  robotAxes *axisTemp = new robotAxes;
  vector<robotAxes>::iterator axisIter;
  vector<robotPose> poses;
  robotPose *poseTemp = new robotPose;
  vector<robotPose>::iterator poseIter;
  double dTemp;

  ofstream out(sstr.str().c_str());

  string instr;
  cout << "Enter file name: ";
  cin >> instr;

  ifstream in(instr.c_str());

  if (!in)
  {
    cout << "Could not open file " << instr.c_str() << endl;
    exit;
  }
  else
  {
    cout << "Opened " << instr.c_str() << endl;
  }

  while (in >> dTemp)
  {
#ifdef READAXES
    axisTemp.axis.at(0) = dTemp;
    for (int x = 1; x < axisTemp.axes; ++x)
    {
      in >> dTemp;
      axisTemp->axis.at(x) = dTemp;
    }
    axes.push_back(*axisTemp);
#else
    poseTemp->x = dTemp;
    in >> poseTemp->y;
    in >> poseTemp->z;
    in >> poseTemp->xrot;
    in >> poseTemp->yrot;
    in >> poseTemp->zrot;
    poses.push_back(*poseTemp);
#endif
  } // while (in >> dTemp)

#ifndef TESTOFFLINE
#ifdef READAXES
  for (axisIter = axes.begin(); axisIter != axes.end(); ++axisIter)
  {
    arm.MoveToAxisTarget(*axisIter);
  }
#else
  for (poseIter = poses.begin(); poseIter != poses.end(); ++poseIter)
  {
    arm.MoveTo(*poseIter);
  }
#endif

  //! Pause for 2 seconds;
  Sleep(2000);

  arm.GetRobotAxes(axisTemp);
  arm.GetRobotPose(poseTemp);

  out << getCurrentTime() << ", " << poseTemp->x << ", " << poseTemp->y << ", " << poseTemp->z << ", "
    << poseTemp->xrot << ", " << poseTemp->yrot << ", " << poseTemp->zrot;
  for (int x = 0; x < axisTemp->axes; ++x)
  {
    out << ", " << axisTemp->axis.at(x);
  }
  out << endl;
#endif

  Sleep(1000);


}