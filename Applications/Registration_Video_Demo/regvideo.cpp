///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Registration Video Demo
//  Workfile:        RegVideo.cpp
//  Revision:        1.0 1 June, 2019
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
#include <vector>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "crpi_abb.h"
#include "ulapi.h"
#include "NumericalMath.h" 
#include "MatrixMath.h"

#include "Vicon.h"
#include "OptiTrack.h"


#pragma warning (disable: 4996)

//#define NOISY
//#define SUPERNOISY
//#define ENABLEARM

using namespace crpi_robot;
using namespace std;
using namespace Math;
using namespace Sensor;

typedef CrpiUniversal robType;

void main()
{
  int i = 0;
  crpi_timer timer;

#ifdef ENABLEARM
  cout << "Create robot" << endl;
  CrpiRobot<robType> arm("universal_ur5.xml");

  cout << "initializing" << endl;
  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");

  cout << "coupling tool" << endl;
  arm.Couple("flange_ring");
#endif 

  robotPose poseMe, tarPose, curPose, forceMe;
  robotAxes curAxes, tarAxes;

  vector<robotPose> posVec;
  vector<robotPose> tempPosVec;
  vector<robotPose>::iterator posVecIter;

  string str, otip, vip;
  ifstream in;
  ofstream out;

  OptiTrack *ottest;
  Vicon *vtest;

  int samplesize;
  int option;
  int mocapoption;

  //! Read configuration file
  in.open("RegVidConfig.dat");
  if (!in)
  {
    cout << "Could not open configuration file RegVidConfig.dat.  Exiting." << endl;
    return;
  }

  do
  {
    cout << "Using 0) None 1) Vicon or 2) OptiTrack: ";
    cin >> mocapoption;
    if (mocapoption < 0 || mocapoption > 2)
    {
      cout << "Invalid option." << endl;
    }
  } while (mocapoption < 1 || mocapoption > 2);

  if (mocapoption == 1)
  {
    vtest = new Vicon(vip.c_str());
  }
  else if (mocapoption == 2)
  {
    ottest = new OptiTrack(otip.c_str());
  }

  in >> curAxes.axis[0] >> curAxes.axis[1] >> curAxes.axis[2] >> curAxes.axis[3] >> curAxes.axis[4] >> curAxes.axis[5];
  int repeats;
  in >> repeats;
  double insertDepth;
  in >> insertDepth;
  double forceThresh;
  in >> forceThresh;
  in >> otip;
  in >> vip;
  in.close();
  
  do
  {
    cout << "Select runtime option:  1) Execute assembly test, 2) Collect robot and mocap data: ";
    cin >> option;
    if (option < 1 || option > 2)
    {
      cout << "Invalid option." << endl;
    }
  } while (option < 1 || option > 2);

  if (option == 1)
  {
    //! Run assembly
#ifdef ENABLEROBOT

#endif

    double dtemp;
    int startNum;

    while (true)
    {

      //!  Ask for input file name
      str = string();
      cout << "Enter name of pose input file: ";
      cin >> str;

      //!  Read init poses and store to vector
      in.open(str.c_str());
      if (in)
      {
        posVec.clear();

        while (in >> str)
        {
          //! X
          dtemp = atof(str.c_str());
          poseMe.x = dtemp;

          //! Y
          in >> str;
          dtemp = atof(str.c_str());
          poseMe.y = dtemp;

          //! Z
          in >> str;
          dtemp = atof(str.c_str());
          poseMe.z = dtemp;

          //! XRot
          in >> str;
          dtemp = atof(str.c_str());
          poseMe.xrot = dtemp;

          //! YRot
          in >> str;
          dtemp = atof(str.c_str());
          poseMe.yrot = dtemp;

          //! ZRot
          in >> str;
          dtemp = atof(str.c_str());
          poseMe.zrot = dtemp;

          posVec.push_back(poseMe);
        }

        cout << "Read " << posVec.size() << " poses from file." << endl;

        //!  Ask for first pose
        cout << "Begin test at which pose number (1 is first): ";
        cin >> startNum;

        if (startNum > posVec.size())
        {
          startNum = posVec.size();
        }
        else if (startNum < 1)
        {
          startNum = 1;
        }
        posVecIter = posVec.begin();
        for (int x = 1; x < startNum; ++x, ++posVecIter)
        {
        }
        cout << "Queued to index " << startNum;

        //!  Ask for output file name
        cout << "Output results to which file: ";
        cin >> str;

        str += ".csv";
        cout << "Storing test results to " << str.c_str();
        out.open(str.c_str());

        //! TODO:  Add MoCap to the output
        out << "Trial, TarX, TarY, TarZ, TarXRot, TarYRot, TarZRot, CurX, CurY, CurZ, CurXRot, CurYRot, CurZRot, CurJ1, CurJ2, CurJ3, CurJ4, CurJ5, CurJ6, ActX, ActY, ActZ, ActXRot, ActYRot, ActZRot, J1, J2, J3, J4, J5, J6, Result" << endl;

        //!  Move to first pose
        while (posVecIter != posVec.end())
        {
          tarPose = *posVecIter;

#ifdef ENABLEROBOT
          arm.moveStraightTo(tarPose);
          arm.GetRobotPose(&curPose);
          arm.GetRobotAxes(&curAxes);
#endif
          //! Record record number, target pose, read pose, read joints
          out << startNum << ", " << tarPose.x << ", " << tarPose.y << ", " << tarPose.z << ", " << tarPose.xrot << ", " << tarPose.yrot << ", "
              << tarPose.zrot << ", " << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", "
              << curPose.zrot << ", " << curAxes.axis[0] << ", " << curAxes.axis[1] << ", " << curAxes.axis[2] << ", " << curAxes.axis[3] << ", "
              << curAxes.axis[4] << ", " << curAxes.axis[5] << ", ";

          bool flag;

          poseMe = tarPose;
          flag = true;
#ifdef ENABLEROBOT
          do
          {
            //! Move until touch or insertion
            poseMe.z -= 0.5; // Move down in 0.5 mm increments
            arm.moveStraightTo(poseMe);
            arm.GetRobotForces(&forceMe);
            arm.GetRobotPose(&curPose);
            arm.GetRobotAxes(&curAxes);
            flag = (fabs(forceMe.z) <= forceThresh);
          } while (flag && curPose.z > (tarPose.z - insertDepth));

          //! Retract
          arm.moveStraightTo(tarPose);
#endif

          //!  Record final pose, final joints, result
          out << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot
              << ", " << curAxes.axis[0] << ", " << curAxes.axis[1] << ", " << curAxes.axis[2] << ", " << curAxes.axis[3] << ", "
              << curAxes.axis[4] << ", " << curAxes.axis[5] << ", " << (flag ? "true" : "false");

          if (mocapoption != 0)
          {
            //! JAM: TODO
          }

          out << endl;

          //!  Increase counter
          startNum++;
        } // while (posVecIter != posVec.end())
        out.close();
      } // if (in)
      else
      {
        cout << "Could not find file " << str.c_str() << endl;
      }
      in.close();
    } // while (true)

    std::cout << "All done" << endl;
  } // if (option == 1)
  else
  {
    //! Capture robot & mocap data
    vector<MoCapSubject> vec;
    vector<MoCapSubject>::iterator iter;
    double curtime;

    cout << "Enter name of output file: ";
    cin >> str;
    str += ".csv";

    ofstream out;
    out.open(str.c_str());

    timer.start();
    if (mocapoption == 0)
    {
      out << "timestamp, rob_x, rob_y, rob_z, rob_rx, rob_ry, rob_rz, rob_j1, rob_j2, rob_j3, rob_j4, rob_j5, rob_j6" << endl;
    }
    else
    {
      out << "timestamp, rob_x, rob_y, rob_z, rob_rx, rob_ry, rob_rz, rob_j1, rob_j2, rob_j3, rob_j4, rob_j5, rob_j6, mocap_obj, mocap_x, mocap_y, mocap_z, mocap_rx, mocap_ry, mocap_rz, mocap objects XYZ" << endl;
    }

    vector<MoCapSubject>::iterator vec_iter;

    //! Query number of samples to take
    do
    {
      cout << "How many samples at each pose would you like to take? ";
      cin >> samplesize;
      if (samplesize < 1)
      {
        cout << "Invalid entry." << endl;
      }
    } while (samplesize < 1);

    while (true)
    {
      curtime = timer.elapsedTime();

#ifdef ENABLEROBOT
      arm.GetRobotPose(&curPose);
      arm.GetRobotAxes(&curAxes);
#endif

      if (mocapoption == 1)
      {
        //! Vicon
        vtest->GetCurrentSubjects(vec);
      }
      else if (mocapoption == 2)
      {
        //! OptiTrack
        ottest->GetCurrentSubjects(vec);
      }
      else
      {
        vec.clear();
      }

      if (!vec.empty())
      {
        //! Print robot and mocap information
        for (vec_iter = vec.begin(); vec_iter != vec.end(); ++vec_iter)
        {
          out << curtime << ", " << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", "
              << curPose.yrot << ", " << curPose.zrot << ", " << curAxes.axis.at(0) << ", " << curAxes.axis.at(1) << ", "
              << curAxes.axis.at(2) << ", " << curAxes.axis.at(3) << ", " << curAxes.axis.at(4) << ", "
              << curAxes.axis.at(5) << ", ";

          out << vec_iter->name << ", " << vec_iter->pose.x << ", " << vec_iter->pose.y << ", "
              << vec_iter->pose.z << ", " << vec_iter->pose.xr << ", " << vec_iter->pose.yr << ", "
              << vec_iter->pose.zr;
          
          //! TODO:  Spit out individual markers here

          out << endl;
        }
      } // if (!vec.empty())
      else
      {
        //! No mocap data, just output robot pose
        out << curtime << ", " << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", "
          << curPose.yrot << ", " << curPose.zrot << ", " << curAxes.axis.at(0) << ", " << curAxes.axis.at(1) << ", "
          << curAxes.axis.at(2) << ", " << curAxes.axis.at(3) << ", " << curAxes.axis.at(4) << ", "
          << curAxes.axis.at(5) << ", ";

        out << "no_data, 0, 0, 0, 0, 0, 0" << endl;
      } // if (!vec.empty()) ... else

    } // while (true)

  } // if (option == 1) ... else
}

