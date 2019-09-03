///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       LabVIEW Interface
//  Workfile:        LabVIEWInterface.cpp
//  Revision:        1.0 - 24 June, 2015
//
//  Description
//  ===========
//  Interface to an external LabVIEW visualization for commanding robot motions
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include "crpi_robot.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#include "MotionPrims/AssemblyPrims.h"
#include "Math/MatrixMath.h"
#include "RegistrationKit/CoordFrameReg.h"

using namespace std;
using namespace crpi_robot;
using namespace MotionPrims;
using namespace Math;
using namespace Registration;

#define LABVIEW_NOISY
#define OFFLINE_TEST

typedef CrpiUniversal robArmType;
//typedef CrpiKukaLWR robArmType;
typedef CrpiRobotiq robHandType;


struct globalHandle
{
  ulapi_mutex_struct *handle; char buffer[REQUEST_MSG_SIZE]; bool runThread;
  bool remoteHandConnected, whtrbtobj, remoteArmConnected;
  ulapi_integer remoteHandID, watchdogID, remoteHandClient,
                remoteArmID, remoteWatchdogClient, remoteArmClient;
  CrpiRobot<robArmType> *robArm; CrpiRobot<robHandType> *robHand;

  ulapi_integer feedbackID, commandID; ulapi_integer feedbackClient, commandClient; int ap, hp;

  globalHandle (CrpiRobot<robArmType>* armptr, CrpiRobot<robHandType>* handptr)
  {
    robArm = armptr; robHand = handptr;
  }

  globalHandle (CrpiRobot<robArmType>* armptr)
  {
    robArm = armptr; robHand = NULL;
  }

  ~globalHandle ()
  {
    robArm = NULL; robHand = NULL;
  }
};


void main ()
{
  ulapi_integer clientID_;
  char curtool[32];

  Assembly asbly;

  //CrpiRobot<CrpiKukaLWR> robarm("kuka_lwr.xml");
  CrpiRobot<CrpiUniversal> robarm("universal_ur10_table.xml");
#ifdef LABVIEW_NOISY
  cout << "init cannon" << endl;
  cout << "set angle units" << endl;
#endif
  robarm.SetAngleUnits("degree");
#ifdef LABVIEW_NOISY
  cout << "set length units" << endl;
#endif
  robarm.SetLengthUnits("mm");
  strcpy(curtool, "laser_pointer");
#ifdef LABVIEW_NOISY
  cout << "set tool" << endl;
#endif
  robarm.Couple(curtool);
  
  robotPose hover, curPose, poseMe;
  robotIO io;
  crpi_timer timer;

  ulapi_integer client;
  char linein[67];
  int get, c, i, j;
#ifndef OFFLINE_TEST
  int counter;
  CanonReturn retVal;
  clock_t t1, t2;
#endif

  cout << "setting up spiral search...";
  asbly.AddSearchSpiral(10, 5, 50);
  asbly.AddTerminatorSignal(CANON_SUCCESS, 8);
  asbly.AddTerminatorTimer(CANON_FAILURE, 120);
  cout << " done" << endl;

  cout << "loading registration data... ";
  matrix robotToWorld(4, 4), worldToRobot(4, 4);

  robotPose W_target, R_target;
  matrix W_XYZ(4,1), R_XYZ(4,1);

  ifstream inreg("register.txt");
  double val;

  for (i = 0; i < 4; ++i)
  {
    for (j = 0; j < 4; ++j)
    {
      inreg >> val;
      robotToWorld.at(i, j) = val;
    }
  }
  inreg.close();
  cout << " done." << endl;

  c = 0; 
  while (c != 1 && c != 2)
  {
    cout << "1) retrain registry 2) use loaded registry : ";
    cin >> c;
  }

  char ch;

  if (c == 1)
  {
    point p1[3], p2[3];
    robotPose pose;

    p2[0].x = 0.0f;
    p2[0].y = 7 * 25.0f;
    p2[0].z = 14.0f;

    p2[1].x = 3 * 25.0f;
    p2[1].y = 2 * 25.0f;
    p2[1].z = 0.0f;

    p2[2].x = 6 * 25.0f;
    p2[2].y = 5 * 25.0f;
    p2[2].z = 14.0f;
    vector<point> worldPoints, robotPoints;

    for (i = 0; i < 3; ++i)
    {
      cout << "Move robot to pose " << i+1 << ", and press <enter>";
      cin.get(ch);
      robarm.GetRobotPose(&pose);
      p1[i].x = pose.x;
      p1[i].y = pose.y;
      p1[i].z = pose.z;

      worldPoints.push_back(p2[i]);
      robotPoints.push_back(p1[i]);
    }
    reg2target(worldPoints, robotPoints, robotToWorld);
  }
  ofstream outreg("register.txt");
  for (i = 0; i < 4; ++i)
  {
    for (j = 0; j < 4; ++j)
    {
      outreg << robotToWorld.at(i, j) << " ";
    }
  }
  outreg.close();
  worldToRobot = robotToWorld.inv();

  robarm.GetRobotPose(&curPose);
  cout << curPose.x << " " << curPose.y << " " << curPose.z << endl;

  cout << "waiting for LabVIEW client connection...";
  clientID_ = ulapi_socket_get_server_id(5248);
  ofstream outfile ("datalog.dat");
//  outfile << "T,Target X,Target Y,Target Z,Target XR,Target YR,Target ZR,Act X,Act Y,Act Z,Act XR,Act YR,Act ZR,J1,J2,J3,J4,J5,J6,J7" << endl;

  //! Wait for remote application to connect
  client = ulapi_socket_get_connection_id(clientID_);
  ulapi_socket_set_blocking(client);
  cout << " connected." << endl;


  robotPose target;
  target.x = 4.5f * 25.0f;
  target.y = 5.5f * 25.0f;
  target.z = 1.0f;


  double randx, randy;
  double d1, d2, d3, d4;
  while (true)
  {
    cout << "Moving to initial seach position... " << endl;
    randx = ((rand() % 10000) - 5000) / 5000.0f;
    randy = ((rand() % 10000) - 5000) / 5000.0f;
    hover = target;
    hover.x += randx;
    hover.y += randy;

    d4 = 0.0f;

#ifdef OFFLINE_TEST
    d1 = hover.x;             //! X position
    d2 = hover.y;             //! Y position
    d3 = target.distance(hover);     //! Error
    c = 1;
#else
    //! Set hover pose to random position
    W_target.x = hover.x;
    W_target.y = hover.y;
    W_target.z = 50;
    W_target.xrot = curPose.xrot;
    W_target.yrot = curPose.yrot;
    W_target.zrot = curPose.zrot;

    //! Convert to math matrix
    W_XYZ.zero();
    W_XYZ.at(0, 0) = W_target.x;
    W_XYZ.at(1, 0) = W_target.y;
    W_XYZ.at(2, 0) = W_target.z;
    W_XYZ.at(3, 0) = 1;

    //! Transform target point to robot point
    R_XYZ = robotToWorld * W_XYZ;

    //! Set robot point to target location
    hover = W_target;
    hover.x = R_XYZ.at(0, 0);
    hover.y = R_XYZ.at(1, 0);
    hover.z = R_XYZ.at(2, 0);

    //! Move to initial hover pose
    if (robarm.MoveStraightTo (hover) == CANON_SUCCESS)
    {
      robarm.GetRobotPose(&curPose);
      robarm.GetRobotIO(&io);
      timer.waitUntil (250);

      //! Run spiral search at hover pose
      counter = 0;
      do
      {
        robarm.GetRobotIO(&io);
        io.dio[8] = !io.dio[8];
        retVal = asbly.RunAssemblyStep (counter++, curPose, poseMe, io);

        if (retVal != CANON_RUNNING)
        {
          robarm.GetRobotPose(&poseMe);
          break;
        }

        robarm.MoveStraightTo (poseMe);
        timer.waitUntil (100);
      } while (retVal == CANON_RUNNING);

      c = (retVal == CANON_SUCCESS) ? 1 : 0;
    } // if (robarm.MoveStraightTo (hover) == CANON_SUCCESS)
    d1 = poseMe.x;
    d2 = poseMe.x;

    R_XYZ.zero();
    R_XYZ.at(0, 0) = poseMe.x;
    R_XYZ.at(1, 0) = poseMe.y;
    R_XYZ.at(2, 0) = poseMe.z;
    R_XYZ.at(3, 0) = 1;

    //! Transform target point to robot point
    W_XYZ = worldToRobot * R_XYZ;
    hover.x = W_XYZ.at(0, 0);
    hover.y = W_XYZ.at(1, 0);
    hover.z = W_XYZ.at(2, 0);
    d3 = target.distance(hover);
#endif
    for (i = 0; i < 67; ++i)
    {
      linein[i] = ' ';
    }
    sprintf (linein, "%d, %f, %f, %f, %f\0", c, d1, d2, d3, d4);
    get = ulapi_socket_write (client, linein, 67);
    if (get < 0)
    {
      cout << "Connection lost, waiting for reconnect..." << endl;
      client = ulapi_socket_get_connection_id(clientID_);
      ulapi_socket_set_blocking(client);
      cout << "Client connected." << endl;
    }

#ifdef OFFLINE_TEST
    c = rand() % 2000;
    Sleep(c);
#endif
  } // while (true)
} // main


/*
//! @brief Parse the pose information sent from an external application (used with ROBOTSYNCDEMO)
//!
//! @param line The plaintext string containing the 6DOF pose (X, Y, Z, Xrot, Yrot, Zrot)
//! @param pose The robot target pose to be populated by the function
//!
void parseCommandPose (char *line, robotPose *pose);


void main ()
{
  ulapi_integer clientID_;
  char buffer[2048];
  char curtool[32];

  //CrpiRobot<CrpiKukaLWR> robarm("kuka_lwr.xml");
  CrpiRobot<CrpiUniversal> robarm("universal_ur10_table.xml");
#ifdef LABVIEW_NOISY
  cout << "init cannon" << endl;
  cout << "set angle units" << endl;
#endif
  robarm.SetAngleUnits("degree");
#ifdef LABVIEW_NOISY
  cout << "set length units" << endl;
#endif
  robarm.SetLengthUnits("mm");
  strcpy(curtool, "gripper_parallel");
#ifdef LABVIEW_NOISY
  cout << "set tool" << endl;
#endif
  robarm.Couple(curtool);
  
  robotPose hover, curPose;
  robotAxes curAxes;
  curAxes.axes = 6;

  ulapi_integer client;
  char linein[67];
  int get, c;
  
  clock_t t1, t2;

  cout << "waiting for LabVIEW client connection...";
  clientID_ = ulapi_socket_get_server_id(5248);
  ofstream outfile ("datalog.dat");
  outfile << "T,Target X,Target Y,Target Z,Target XR,Target YR,Target ZR,Act X,Act Y,Act Z,Act XR,Act YR,Act ZR,J1,J2,J3,J4,J5,J6,J7" << endl;

  //! Wait for remote application to connect
  client = ulapi_socket_get_connection_id(clientID_);
  ulapi_socket_set_blocking(client);
  cout << " connected." << endl;

  while (true)
  {
    get = ulapi_socket_read (client, linein, 66);

#ifdef LABVIEW_NOISY
    cout << get << " bytes read" << endl;
#endif
    if (get > 0)
    {
#ifdef LABVIEW_NOISY
      cout << linein << endl;
#endif
      try
      {
        cout << linein << endl;
        parseCommandPose (linein, &hover);
      }
      catch (...)
      {
        cout << "parse error" << endl;
        //! Is this where things are breaking?
      }
      t1 = clock();
      cout << "target:  " << hover.x << "," << hover.y << "," << hover.z << "," << hover.xrot << "," << hover.yrot << "," << hover.zrot << endl;
      //cout << "current: " << curPose.x << "," << curPose.y << "," << curPose.z << "," << curPose.xrot << "," << curPose.yrot << "," << curPose.zrot << endl;
      //cout << "axes:    " << curAxes.axis.at(0) << "," << curAxes.axis.at(1) << "," << curAxes.axis.at(2) << "," << curAxes.axis.at(3) << "," << curAxes.axis.at(4) << "," << curAxes.axis.at(5) << endl;
      if (robarm.MoveStraightTo (hover) == CANON_SUCCESS)
      {
#ifdef LABVIEW_NOISY
        cout << "Motion command okay" << endl;
#endif
        t2 = clock() - t1;
        robarm.GetRobotPose(&curPose);
        robarm.GetRobotAxes(&curAxes);
        sprintf (buffer, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                 1, hover.x, hover.y, hover.z, hover.xrot, hover.yrot, hover.zrot,
                 curPose.x, curPose.y, curPose.z, curPose.xrot, curPose.yrot, curPose.zrot,
                 curAxes.axis.at(0), curAxes.axis.at(1), curAxes.axis.at(2), curAxes.axis.at(3), curAxes.axis.at(4), curAxes.axis.at(5), curAxes.axis.at(6));
        c = ulapi_socket_write(client, buffer, 1024);//strlen(buffer)+1);
        outfile << t2 << hover.x << "," << hover.y << "," << hover.z << "," << hover.xrot << "," << hover.yrot << "," << hover.zrot << ","
                << curPose.x << "," << curPose.y << "," << curPose.z << "," << curPose.xrot << "," << curPose.yrot << "," << curPose.zrot << ","
                << curAxes.axis.at(0) << "," << curAxes.axis.at(1) << "," << curAxes.axis.at(2) << "," << curAxes.axis.at(3) << "," << curAxes.axis.at(4) << ","
                << curAxes.axis.at(5) << "," << curAxes.axis.at(6) << endl;
#ifdef LABVIEW_NOISY
        cout << "Finished writing logs" << endl;
#endif
      }
      else
      {
        c = ulapi_socket_write(client, "0\n", 1024);//1);
        cout << "could not move robot" << endl;
      }
    }
  }
}
*/

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