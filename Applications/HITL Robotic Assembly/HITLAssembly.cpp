///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Human-in-the-loop Robotic Assembly
//  Workfile:        5RobotAssembly.cpp
//  Revision:        1.0    3/20/2018
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Demo in which a human and a robot work together to assemble a sphere and
//  block artifact assembly.
//  Robot :  Acquire sphere and act as a compliant fixture for the human
//  Human :  Attach block assembly to the sphere, and then guide the completed
//           assembly into place to be connected to the table/base
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#include "NumericalMath.h" 

#pragma warning (disable: 4996)

//#define verifysteps

#define enableUR5
#define enableRobotiq

#define hololens

//#define NOISY
//#define SUPERNOISY

using namespace crpi_robot;
using namespace std;
using namespace Math;

typedef enum
{
  HITL_PICK = 0,  //! Move to pick pose
  HITL_PLACE,     //! Move to place pose
  HITL_GRAVON,    //! Gravity compensation on
  HITL_GRAVOFF,   //! Gravity compensation off
  HITL_NONE       //! Null or invalid entry
} hitlMssg;

struct hitlHandle
{
  ulapi_mutex_struct *handle;
  char buffer[1024];
  bool runThread;
  ulapi_integer remoteID;
  int port;
  bool t1;
  hitlMssg msg;
  bool newmsg;

  hitlHandle()
  {
    handle = ulapi_mutex_new(89);
    newmsg = false;
  }

  ~hitlHandle()
  {
  }
};


//! @brief Thread method for communicating with a UR robot
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void hololensThread(void *param)
{
  hitlHandle *hH = (hitlHandle*)param;
  ulapi_integer server, client;
  bool clientConnected = false;

  crpi_timer timer;
  char buffer[1024];
  string str;
  ulapi_integer rec, sent;

  //! Create socket connection
  server = ulapi_socket_get_server_id(hH->port);
  ulapi_socket_set_blocking(server);

  while (hH->runThread)
  {
    if (!clientConnected)
    {
      cout << "Waiting for Hololens client on port " << hH->port << "." << endl;
      client = ulapi_socket_get_connection_id(server);
      ulapi_socket_set_blocking(client);
      clientConnected = true;
      cout << "Remote Hololens client connected..." << endl;
    }

    while (clientConnected && hH->runThread)
    {
      rec = ulapi_socket_read(client, buffer, 2048);
      if (rec > 0)
      {
        buffer[rec] = '\0';
        str = buffer;
#ifdef NOISY
        cout << rec << " bytes received" << endl;
        cout << "buffer: " << buffer << endl;
        cout << "str: " << str.c_str() << endl;
#endif 
        if (!hH->newmsg)
        {
          if (str == "pick")
          {
#ifdef NOISY
            cout << "sending pick" << endl;
#endif
            hH->msg = HITL_PICK;
          }
          else if (str == "place")
          {
#ifdef NOISY
            cout << "sending place" << endl;
#endif
            hH->msg = HITL_PLACE;
          }
          else if (str == "gravcomp_on")
          {
#ifdef NOISY
            cout << "sending gravcomp_on" << endl;
#endif
            hH->msg = HITL_GRAVON;
          }
          else if (str == "gravcomp_off")
          {
#ifdef NOISY
            cout << "sending gravcomp_off" << endl;
#endif
            hH->msg = HITL_GRAVOFF;
          }
          else
          {
#ifdef NOISY
            cout << "sending none" << endl;
#endif
            hH->msg = HITL_NONE;
          }
          hH->newmsg = true;
        }
//        sent = ulapi_socket_write(client, buffer, strlen(buffer));
      }
      else
      {
        timer.waitUntil(5);
      }
    } // while (clientConnected && gH->runThread)
  } // while (gH->runThread)

  hH = NULL;
  return;
}


void SerialAssembly()
{
  int index;
  string str;
  char type;
  bool okay = true;
  double speed = 0.5;
  hitlHandle hH;
  hH.port = 20602;

  cout << "Initializing robots..." << endl;

#ifdef enableUR5
  cout << "Connecting to UR5... ";
  CrpiRobot<CrpiUniversal> ur5("universal_ur5.xml");
  ur5.Couple("robotiq");
  ur5.SetRelativeSpeed(speed);
  ur5.SetAngleUnits("degree");
  ur5.SetLengthUnits("mm");
  cout << "done." << endl;
#endif
#ifdef enableRobotiq
  cout << "Connecting to Robotiq... ";
  CrpiRobot<CrpiRobotiq> robotiq("robotiq.xml");
  robotiq.Couple("gripper_peg");
  cout << "done." << endl;
#endif

  int option;

  cout << "Populating poses..." << endl;
  robotAxes jtemp, curjoints;
  robotPose ctemp, ctar, curpose;

  vector<robotAxes> assembleAxes;
  vector<robotPose> assemblePoses;

  int param;

  ifstream in;
  in.open("hitlposes.dat");

  int store = 0;
  int hover1 = 0, acquire = 1, grasp = 2, hover2 = 3, assemble = 4, hover3 = 5, secure = 6;

  assembleAxes.resize(1);
  assemblePoses.resize(7);

  //! Read poses from disk
  int cindex = 0, jindex = 0;
  while (in >> type)
  {
    if (type == 'c')
    {
      //! Cartesian pose
      in >> ctemp.x >> ctemp.y >> ctemp.z >> ctemp.xrot >> ctemp.yrot >> ctemp.zrot >> ctemp.status >> ctemp.turns;
      assemblePoses.at(cindex) = ctemp;
      ++cindex;
    }
    else if (type == 'j')
    {
      in >> jtemp.axis.at(0) >> jtemp.axis.at(1) >> jtemp.axis.at(2) >> jtemp.axis.at(3) >> jtemp.axis.at(4) >> jtemp.axis.at(5);
      assembleAxes.at(jindex) = jtemp;
      ++jindex;
    }
  }
  in.close();

#ifdef hololens
  cout << "Initializing server thread." << endl;
  hH.runThread = true;

  void* task = ulapi_task_new();
  //! Start new thread to listen for Hololens
  ulapi_task_start((ulapi_task_struct*)task, hololensThread, &hH, ulapi_prio_lowest(), 0);
#else
  cout << "Bypassing hololens interface." << endl;
#endif

  do
  {
    cout << "Moving robots to stow positions." << endl;
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    //!  Move to stow positions...
#ifdef enableUR5
    if (ur5.MoveToAxisTarget(assembleAxes.at(store)) == CANON_SUCCESS)
    {
      ur5.GetRobotAxes(&curjoints);
    }
#endif
#ifdef enableRobotiq
    //! Open hand
    param = 1;
    robotiq.SetParameter("ADVANCED_CONTROL", &param);
    robotiq.SetParameter("SCISSOR_CONTROL", &param);
    param = 100;
    robotiq.SetParameter("SPEED_FINGER_A", &param);
    robotiq.SetParameter("SPEED_FINGER_B", &param);
    robotiq.SetParameter("SPEED_FINGER_C", &param);
    robotiq.SetParameter("SPEED_SCISSOR", &param);
    param = 236;
    robotiq.SetParameter("FORCE_FINGER_A", &param);
    robotiq.SetParameter("FORCE_FINGER_B", &param);
    robotiq.SetParameter("FORCE_FINGER_C", &param);
    robotiq.SetParameter("FORCE_SCISSOR", &param);
    param = 0;
    robotiq.SetParameter("POSITION_FINGER_A", &param);
    robotiq.SetParameter("POSITION_FINGER_B", &param);
    robotiq.SetParameter("POSITION_FINGER_C", &param);
    robotiq.SetParameter("POSITION_SCISSOR", &param);
    param = 1;
    robotiq.SetParameter("GRIP", &param);
#endif

    cout << "Ready." << endl;

    //! Wait for user input
#ifndef hololens
    cout << "Enter 1 to begin: ";
    cin >> option;
#else
    hH.newmsg = false;
    do
    {
      if (hH.newmsg)
      {
        if (hH.msg == HITL_PICK)
        {
          hH.newmsg = false;
          break;
        }
        else
        {
          hH.newmsg = false;
        }
      }
      Sleep(100);
    } while (true);
#endif

    index = 0;
    cout << "Acquire sphere artifact..." << endl;
    //!   Get sphere
    //! Move to hover in position control
#ifdef enableUR5
    if (ur5.MoveTo(assemblePoses.at(hover1)) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

#ifdef enableRobotiq
    //! Open hand
    param = 1;
    robotiq.SetParameter("ADVANCED_CONTROL", &param);
    param = 100;
    robotiq.SetParameter("SPEED_FINGER_A", &param);
    robotiq.SetParameter("SPEED_FINGER_B", &param);
    robotiq.SetParameter("SPEED_FINGER_C", &param);
    param = 236;
    robotiq.SetParameter("FORCE_FINGER_A", &param);
    robotiq.SetParameter("FORCE_FINGER_B", &param);
    robotiq.SetParameter("FORCE_FINGER_C", &param);
    param = 0;
    robotiq.SetParameter("POSITION_FINGER_A", &param);
    robotiq.SetParameter("POSITION_FINGER_B", &param);
    robotiq.SetParameter("POSITION_FINGER_C", &param);
    param = 1;
    robotiq.SetParameter("GRIP", &param);
#endif 

    //! Move to acquire in position control
#ifdef enableUR5
    if (ur5.MoveTo(assemblePoses.at(acquire)) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Move down in force control
#ifdef enableUR5
    if (ur5.MoveAttractor(assemblePoses.at(grasp)) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    Sleep(4000);

#ifdef enableRobotiq
    //! Close hand
    param = 255;
    robotiq.SetParameter("POSITION_FINGER_A", &param);
    robotiq.SetParameter("POSITION_FINGER_B", &param);
    robotiq.SetParameter("POSITION_FINGER_C", &param);
    param = 1;
    robotiq.SetParameter("GRIP", &param);
#endif

    //! Move to hover1 in position control
#ifdef enableUR5
    if (ur5.MoveStraightTo(assemblePoses.at(hover1)) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    cout << "Moving to assembly READY position." << endl;
    //! Move to hover2 in position control
#ifdef enableUR5
    if (ur5.MoveTo(assemblePoses.at(hover2)) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    
    //! SLOW DOWN!
#ifdef enableUR5
    ur5.SetRelativeSpeed(0.50);

    //! Move to first assembly pose
    if (ur5.MoveStraightTo(assemblePoses.at(assemble)) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Wait for user input
    cout << "Waiting for user input." << endl;
#ifndef hololens
    cout << "Enter 1 to continue: ";
    cin >> option;
#else
    hH.newmsg = false;
    do
    {
      if (hH.newmsg)
      {
        if (hH.msg == HITL_PLACE)
        {
          hH.newmsg = false;
          break;
        }
        else
        {
          hH.newmsg = false;
        }
      }
      Sleep(100);
    } while (true);
#endif

    //! Okay, speed up again.
#ifdef enableUR5
    ur5.SetRelativeSpeed(1.0);

    //! Move to hover2 in position control
    if (ur5.MoveStraightTo(assemblePoses.at(hover2)) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    cout << "Moving to secure READY position." << endl;
    //! Move to hover3 in position control
#ifdef enableUR5
    if (ur5.MoveTo(assemblePoses.at(hover3)) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Move to secure position in position control
#ifdef enableUR5
    if (ur5.MoveTo(assemblePoses.at(secure)) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Wait for user input
    cout << "Waiting for user input." << endl;
#ifndef hololens
    cout << "Enter 1 to continue: ";
    cin >> option;
#else
    hH.newmsg = false;
    do
    {
      if (hH.newmsg)
      {
        if (hH.msg == HITL_GRAVON)
        {
          hH.newmsg = false;
          break;
        }
        else
        {
          hH.newmsg = false;
        }
      }
      Sleep(100);
    } while (true);
#endif

    //! Go to grav comp mode
#ifdef enableUR5
    if (ur5.SetParameter("freedrive", NULL) != CANON_SUCCESS)
    {
      cout << "Error entering freedrive mode!" << endl;
    };
#endif

    //! Wait for user input
    cout << "Waiting for user input." << endl;
#ifndef hololens
    cout << "Enter 1 to continue: ";
    cin >> option;
#else
    hH.newmsg = false;
    do
    {
      if (hH.newmsg)
      {
        if (hH.msg == HITL_GRAVOFF)
        {
          hH.newmsg = false;
          break;
        }
        else
        {
          hH.newmsg = false;
        }
      }
      Sleep(100);
    } while (true);
#endif

#ifdef enableUR5
    //! Cancel grav comp
    ur5.SetParameter("endfreedrive", NULL);
#endif

    //! Release ball
#ifdef enableRobotiq
    //! Open hand
    param = 1;
    robotiq.SetParameter("ADVANCED_CONTROL", &param);
    param = 100;
    robotiq.SetParameter("SPEED_FINGER_A", &param);
    robotiq.SetParameter("SPEED_FINGER_B", &param);
    robotiq.SetParameter("SPEED_FINGER_C", &param);
    param = 236;
    robotiq.SetParameter("FORCE_FINGER_A", &param);
    robotiq.SetParameter("FORCE_FINGER_B", &param);
    robotiq.SetParameter("FORCE_FINGER_C", &param);
    param = 0;
    robotiq.SetParameter("POSITION_FINGER_A", &param);
    robotiq.SetParameter("POSITION_FINGER_B", &param);
    robotiq.SetParameter("POSITION_FINGER_C", &param);
    param = 1;
    robotiq.SetParameter("GRIP", &param);
#endif

#ifdef enableUR5
    ur5.GetRobotPose(&curpose);
    curpose.z += 140;

    //! Move up in position control
    if (ur5.MoveStraightTo(curpose) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    else
    {
      //! Error
    }
#endif

#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
  } while (true);
}


void main ()
{
  int i = 0;
  crpi_timer timer;

  int option;

  SerialAssembly();

  cout << "All done" << endl;

}


