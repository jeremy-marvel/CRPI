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
#define ENABLEARM

using namespace crpi_robot;
using namespace std;
using namespace Math;
using namespace Sensor;

void main()
{
  int i = 0;
  crpi_timer timer;

  CrpiRobot<CrpiUniversal> *ur5;
  CrpiRobot<CrpiKukaLWR> *lwr;

  robotPose poseMe, tarPose, startPose, curPose, forceMe;
  robotAxes curAxes, homeAxes, tarAxes;

  vector<robotPose> posVec;
  vector<robotPose> tempPosVec;
  vector<robotPose>::iterator posVecIter;

  vector<Math::point> markers;
  vector<Math::point>::iterator markerIter;

  vector<MoCapSubject> subjects;
  vector<MoCapSubject>::iterator subjectIter;

  string str, otip, vip;
  ifstream in;
  ofstream out;

  OptiTrack *ottest;
  Vicon *vtest;

  int samplesize;
  int option;
  int mocapoption;

  int robChoice;
  do
  {
    cout << "Using which robot? 0) None, 1) UR5, 2) LWR: ";
    cin >> robChoice;
  } while (robChoice < 0 || robChoice > 2);

  cout << "Create robot" << endl;
  if (robChoice == 1)
  {
    ur5 = new CrpiRobot<CrpiUniversal>("universal_ur5.xml");
    cout << "initializing" << endl;
    ur5->SetAngleUnits("degree");
    ur5->SetLengthUnits("mm");

    cout << "coupling tool" << endl;
    ur5->Couple("flange_ring");
  }
  else if (robChoice == 2)
  {
    lwr = new CrpiRobot<CrpiKukaLWR>("kuka_lwr.xml");
    cout << "initializing" << endl;
    lwr->SetAngleUnits("degree");
    lwr->SetLengthUnits("mm");

    cout << "coupling tool" << endl;
    lwr->Couple("flange_ring");
  }

  //! Read configuration file
  in.open("RegVidConfig.dat");
  if (!in)
  {
    cout << "Could not open configuration file RegVidConfig.dat." << endl;
    cout << "Expecting a data file with the following format:" << endl;
    if (robChoice == 2)
    {
      cout << "# # # # # # # (Home joint axes J1 through J7)" << endl;
    }
    else
    {
      cout << "# # # # # # (Home joint axes J1 through J6)" << endl;
    }
    cout << "# (number of repeats)" << endl;
    cout << "# (insertion depth, mm)" << endl;
    cout << "# (force threshold, N)" << endl;
    cout << "#.#.#.# (OptiTrack IP address)" << endl;
    cout << "#.#.#.# (Vicon IP address)" << endl;
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
  } while (mocapoption < 0 || mocapoption > 2);

  in >> homeAxes.axis[0] >> homeAxes.axis[1] >> homeAxes.axis[2] >> homeAxes.axis[3] >> homeAxes.axis[4] >> homeAxes.axis[5];
  if (robChoice == 2)
  {
    in >> homeAxes.axis[6];
  }
  int repeats;
  in >> repeats;
  double insertDepth;
  in >> insertDepth;
  double forceThresh;
  in >> forceThresh;
  in >> otip;
  in >> vip;
  in.close();

  if (mocapoption == 1)
  {
    vtest = new Vicon(vip.c_str());
  }
  else if (mocapoption == 2)
  {
    ottest = new OptiTrack(otip.c_str());
  }

  //! Move to home position
  if (robChoice == 1)
  {
    ur5->MoveToAxisTarget(homeAxes);
  }
  else if (robChoice == 2)
  {
    lwr->MoveToAxisTarget(homeAxes);
  }
  
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
    double dtemp;
    int startNum;

    while (true)
    {
      bool runAssembly = false;

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

          poseMe.print();

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
        cout << "Queued to index " << startNum << endl;

        //!  Ask for output file name
        cout << "Output results to which file: ";
        cin >> str;

        str += ".csv";
        cout << "Storing test results to " << str.c_str() << endl;;
        out.open(str.c_str());

        //! Write labels to csv file
        out << "Trial, TarX, TarY, TarZ, TarXRot, TarYRot, TarZRot, CurX, CurY, CurZ, CurXRot, CurYRot, CurZRot, CurJ1, CurJ2, CurJ3, CurJ4, CurJ5, CurJ6, ";
        if (robChoice == 2)
        {
          out << "CurJ7, ";
        }
        out << "Result";

        if (mocapoption != 0)
        {
          out << ", MoCapPoses";
        }
        out << endl;
        
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

        do
        {
          cout << "Run the assembly portion?  0 No, 1 Yes: ";
          cin >> option;
          if (option < 0 || option > 1)
          {
            cout << "Invalid option." << endl;
          }
        } while (option < 0 || option > 1);

        if (option == 1)
        {
          runAssembly = true;
        }

        //!  Move to first pose
        while (posVecIter != posVec.end())
        {
          tarPose = *posVecIter;

          if (robChoice == 1)
          {
            ur5->MoveStraightTo(tarPose);
          }
          else if (robChoice == 2)
          {
            lwr->MoveStraightTo(tarPose);
          }

          for (int x = 0; x < samplesize; ++x)
          {
            if (robChoice == 1)
            {
              ur5->GetRobotPose(&startPose);
              ur5->GetRobotAxes(&curAxes);
            }
            else if (robChoice == 2)
            {
              lwr->GetRobotPose(&startPose);
              lwr->GetRobotAxes(&curAxes);
            }

            //! Record record number, target pose, read pose, read joints
            out << startNum << ", " << tarPose.x << ", " << tarPose.y << ", " << tarPose.z << ", " << tarPose.xrot << ", " << tarPose.yrot << ", "
                << tarPose.zrot << ", " << startPose.x << ", " << startPose.y << ", " << startPose.z << ", " << startPose.xrot << ", " << startPose.yrot << ", "
                << startPose.zrot << ", " << curAxes.axis[0] << ", " << curAxes.axis[1] << ", " << curAxes.axis[2] << ", " << curAxes.axis[3] << ", "
                << curAxes.axis[4] << ", " << curAxes.axis[5];
            if (robChoice == 2)
            {
              out << curAxes.axis[6];
            }
            out << ", null";

            markers.clear();
            if (mocapoption != 0)
            {
              if (mocapoption == 1)
              {
                //! Vicon
                vtest->GetUnlabeledMarkers(markers);
              }
              else if (mocapoption == 2)
              {
                //! OptiTrack
                ottest->GetUnlabeledMarkers(markers);
              }
            }

            //! Spit out individual markers
            for (markerIter = markers.begin(); markerIter != markers.end(); ++markerIter)
            {
              out << ", " << markerIter->x << ", " << markerIter->y << ", " << markerIter->z;
            }

            out << endl;

          } // for (x = 0; x < samplesize; ++x)

          //! If we're not running the assembly, don't bother with this.
          if (runAssembly)
          {
            bool flag;

            poseMe = tarPose;
            flag = true;

            do
            {
              //! Move until touch or insertion
              poseMe.z -= 0.5; // Move down in 0.5 mm increments
              if (robChoice == 1)
              {
                ur5->MoveStraightTo(poseMe);
                ur5->GetRobotForces(&forceMe);
                ur5->GetRobotPose(&curPose);
                ur5->GetRobotAxes(&curAxes);
              }
              else if (robChoice == 2)
              {
                lwr->MoveStraightTo(poseMe);
                lwr->GetRobotForces(&forceMe);
                lwr->GetRobotPose(&curPose);
                lwr->GetRobotAxes(&curAxes);
              }
              flag = (fabs(forceMe.z) <= forceThresh);
            } while (flag && curPose.z > (tarPose.z - insertDepth));
            for (int x = 0; x < samplesize; ++x)
            {
              if (robChoice == 1)
              {
                ur5->GetRobotPose(&curPose);
                ur5->GetRobotAxes(&curAxes);
              }
              else if (robChoice == 2)
              {
                lwr->GetRobotPose(&curPose);
                lwr->GetRobotAxes(&curAxes);
              }

              //! Record record number, target pose, read pose, read joints
              out << startNum << ", " << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " << poseMe.yrot << ", "
                << poseMe.zrot << ", " << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", "
                << curPose.zrot << ", " << curAxes.axis[0] << ", " << curAxes.axis[1] << ", " << curAxes.axis[2] << ", " << curAxes.axis[3] << ", "
                << curAxes.axis[4] << ", " << curAxes.axis[5] << ", ";
              if (robChoice == 2)
              {
                out << curAxes.axis[6] << ", ";
              }
              out << (flag ? "true" : "false");

              markers.clear();
              if (mocapoption != 0)
              {
                if (mocapoption == 1)
                {
                  //! Vicon
                  vtest->GetUnlabeledMarkers(markers);
                }
                else if (mocapoption == 2)
                {
                  //! OptiTrack
                  ottest->GetUnlabeledMarkers(markers);
                }
              }

              //! Spit out individual markers
              for (markerIter = markers.begin(); markerIter != markers.end(); ++markerIter)
              {
                out << ", " << markerIter->x << ", " << markerIter->y << ", " << markerIter->z;
              }
            } // for (int x = 0; x < samplesize; ++x)

            out << endl;

            //! Retract
            if (robChoice == 1)
            {
              ur5->MoveStraightTo(tarPose);
            }
            else if (robChoice == 2)
            {
              lwr->MoveStraightTo(tarPose);
            }
          } // if (runAssembly)

          //!  Increase counter
          startNum++;
          posVecIter++;
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
    double curtime;

    cout << "Enter name of output file: ";
    cin >> str;
    str += ".csv";

    ofstream out;
    out.open(str.c_str());

    timer.start();
    if (mocapoption == 0)
    {
      out << "timestamp, rob_x, rob_y, rob_z, rob_rx, rob_ry, rob_rz, rob_j1, rob_j2, rob_j3, rob_j4, rob_j5, rob_j6";
      if (robChoice == 2)
      {
        out << ", rob_j7";
      }
      out << endl;
    }
    else
    {
      out << "timestamp, rob_x, rob_y, rob_z, rob_rx, rob_ry, rob_rz, rob_j1, rob_j2, rob_j3, rob_j4, rob_j5, rob_j6,";
      if (robChoice == 2)
      {
        out << "rob_j7, ";
      }
      out << "mocap_obj, mocap_x, mocap_y, mocap_z, mocap_rx, mocap_ry, mocap_rz, MoCapPoses" << endl;
    }

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
      cout << "enter 1 to take samples: ";
      cin >> option;

      curtime = timer.elapsedTime();

      for (int x = 0; x < samplesize; ++x)
      {
        if (robChoice == 1)
        {
          ur5->GetRobotPose(&curPose);
          ur5->GetRobotAxes(&curAxes);
        }
        else if (robChoice == 2)
        {
          lwr->GetRobotPose(&curPose);
          lwr->GetRobotAxes(&curAxes);
        }

        subjects.clear();
        markers.clear();
        if (mocapoption == 1)
        {
          //! Vicon
          vtest->GetCurrentSubjects(subjects);
          vtest->GetUnlabeledMarkers(markers);
        }
        else if (mocapoption == 2)
        {
          //! OptiTrack
          ottest->GetCurrentSubjects(subjects);
          ottest->GetUnlabeledMarkers(markers);
        }

        if (!subjects.empty())
        {
          //! Print robot and mocap information
          for (subjectIter = subjects.begin(); subjectIter != subjects.end(); ++subjectIter)
          {
            out << curtime << ", " << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", "
                << curPose.yrot << ", " << curPose.zrot << ", " << curAxes.axis.at(0) << ", " << curAxes.axis.at(1) << ", "
                << curAxes.axis.at(2) << ", " << curAxes.axis.at(3) << ", " << curAxes.axis.at(4) << ", "
                << curAxes.axis.at(5) << ", ";
            if (robChoice == 2)
            {
              out << curAxes.axis.at(6) << ", ";
            }

            out << subjectIter->name << ", " << subjectIter->pose.x << ", " << subjectIter->pose.y << ", "
                << subjectIter->pose.z << ", " << subjectIter->pose.xr << ", " << subjectIter->pose.yr << ", "
                << subjectIter->pose.zr;

            //! Spit out individual markers here
            for (markerIter = markers.begin(); markerIter != markers.end(); ++markerIter)
            {
              out << ", " << markerIter->x << ", " << markerIter->y << ", " << markerIter->z;
            }

            out << endl;
          }
        } // if (!subjects.empty())
        else
        {
          //! No mocap data, just output robot pose
          out << curtime << ", " << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", "
              << curPose.yrot << ", " << curPose.zrot << ", " << curAxes.axis.at(0) << ", " << curAxes.axis.at(1) << ", "
              << curAxes.axis.at(2) << ", " << curAxes.axis.at(3) << ", " << curAxes.axis.at(4) << ", "
              << curAxes.axis.at(5) << ", ";
          if (robChoice == 2)
          {
            out << curAxes.axis.at(6) << ", ";
          }

          if (mocapoption != 0)
          {
            out << "no_data, 0, 0, 0, 0, 0, 0";
          }

          //! Spit out individual markers here
          for (markerIter = markers.begin(); markerIter != markers.end(); ++markerIter)
          {
            out << ", " << markerIter->x << ", " << markerIter->y << ", " << markerIter->z;
          }

          out << endl;
        } // if (!subjects.empty()) ... else
      } // for (int x = 0; x < samplesize; ++x)
    } // while (true)

  } // if (option == 1) ... else
}

