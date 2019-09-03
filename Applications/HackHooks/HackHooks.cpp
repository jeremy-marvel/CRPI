///////////////////////////////////////////////////////////////////////////////
//
//  Original System: ISD CRPI
//  Subsystem:       
//  Workfile:        HackHooks.cpp
//  Revision:        1.0 - 17 March, 2015
//
//  Description
//  ===========
//  Interface connection for a robot.  Creates one thread at a specified port
//  to handle status updates.  Creates another thread to take CRCL statements
//  and interpret them as CRPI commands.
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "inputs.h"

#include <fstream>
#include "crpi_robot.h"
#include "crpi_demo_hack.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"

//#define HACKHOOKS_NOISY
//#define ENABLE_MUTEXING //! NOTE: Semaphors currently disabled

using namespace std;
using namespace crpi_robot;

//typedef CrpiUniversal robArmType;
typedef CrpiKukaLWR robArmType;
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


void armSocketHandlerThread (void *param)
{
  globalHandle *gH = (globalHandle*)param;
  crpi_timer timer;

  /*! Create socket connection */
  gH->remoteArmID = ulapi_socket_get_server_id_on_interface(gH->ap, "127.0.0.1");
//  extern ulapi_integer ulapi_socket_get_server_id_on_interface(ulapi_integer port, const char *intf);
  ulapi_socket_set_blocking(gH->remoteArmID);

  while (gH->runThread)
  {
    if (!gH->remoteArmConnected)
    {
      cout << "Running HackHooks on port " << gH->ap << " for the robot arm" << endl;
      gH->remoteArmClient = ulapi_socket_get_connection_id (gH->remoteArmID);
      ulapi_socket_set_blocking(gH->remoteArmClient);
      gH->remoteArmConnected = true;
      cout << "Arm client connected..." << endl;
    }

    while (gH->remoteArmConnected && gH->runThread)
    {
      timer.waitUntil(1000);
    }
  }

  gH = NULL;
  return;
}


void feedbackArmThread (void *param)
{
  globalHandle *gH = (globalHandle*)param; int sent; char buffer[REQUEST_MSG_SIZE]; robotPose curPose;
  crpi_timer timer;

  while (gH->runThread)
  {
    if (gH->remoteArmConnected)
    {
#ifdef ENABLE_MUTEXING
      ulapi_mutex_take (gH->handle);
#endif
#ifdef HACKHOOKS_NOISY
      cout << "get pose..." << endl;
#endif
      gH->robArm->GetRobotPose(&curPose);
#ifdef HACKHOOKS_NOISY
      cout << "generate xml..." << endl;
#endif
      gH->robArm->CrclXmlResponse(buffer);
#ifdef ENABLE_MUTEXING
      ulapi_mutex_give (gH->handle);
#endif
#ifdef HACKHOOKS_NOISY
      cout << "Sending " << buffer << endl << endl;
#endif
      sent = ulapi_socket_write(gH->remoteArmClient, buffer, strlen(buffer));

      if (sent < 0) {
      cout << "Socket write error.  Closing connection and restarting..." << endl;
      gH->remoteArmConnected = false; }
    } /*! if (gH->remoteArmConnected) */
#ifdef HACKHOOKS_NOISY
    cout << "sleeping..." << endl;
#endif
    timer.waitUntil(100);
  } /*! while (gH->runThread) */

  gH = NULL;
  return;
}


void commandArmThread (void *param)
{
  globalHandle *gH = (globalHandle*)param; int get; string str;
  crpi_timer timer;

  while (gH->runThread) {
  if (gH->remoteArmConnected)
  {
#ifdef HACKHOOKS_NOISY
    cout << "waiting for command... " << endl;
#endif
    get = ulapi_socket_read(gH->remoteArmClient, gH->buffer, REQUEST_MSG_SIZE);
    if (get > 0)
    {
#ifdef HACKHOOKS_NOISY
      cout << "read " << gH->buffer << endl;
#endif
      str = gH->buffer;
#ifdef ENABLE_MUTEXING
      ulapi_mutex_take (gH->handle);
#endif
      gH->robArm->CrclXmlHandler(str);
#ifdef ENABLE_MUTEXING
      ulapi_mutex_give (gH->handle);
#endif
    } /*! if (get > 0) */ } /*! if (gH->remoteArmConnected) */
#ifdef HACKHOOKS_NOISY
    cout << "short sleep... " << endl;
#endif
    timer.waitUntil(5); } /*! while (gH->runThread) */
    gH = NULL; return;}
    void watchdog (void *param)
    {
      globalHandle *gH = (globalHandle*)param; gH->watchdogID = ulapi_socket_get_server_id(REQUEST_MSG_SIZE);
      ulapi_socket_set_blocking(gH->watchdogID);
      while (gH->runThread) {
      gH->remoteWatchdogClient = ulapi_socket_get_connection_id (gH->watchdogID);
      ulapi_socket_set_blocking(gH->remoteWatchdogClient); gH->runThread = false;
    }
    gH = NULL;
    return;
}


void handSocketHandlerThread (void *param)
{
  globalHandle *gH = (globalHandle*)param;
  crpi_timer timer;

  //! Create socket connection
  gH->remoteHandID = ulapi_socket_get_server_id_on_interface (gH->hp, "127.0.0.1");
  ulapi_socket_set_blocking(gH->remoteHandID);

  while (gH->runThread)
  {
    if (!gH->remoteHandConnected)
    {
      cout << "Running HackHooks on port " << gH->hp << " for the robot hand" << endl;
      gH->remoteHandClient = ulapi_socket_get_connection_id (gH->remoteHandID);
      ulapi_socket_set_blocking(gH->remoteHandClient);

      gH->remoteHandConnected = true;
      cout << "Hand client connected..." << endl;
    }

    while (gH->remoteHandConnected && gH->runThread)
    {
      timer.waitUntil(1000);
    }
  }

  gH = NULL;
  return;
}


void feedbackHandThread (void *param)
{
  globalHandle *gH = (globalHandle*)param;
  int sent;
  char buffer[REQUEST_MSG_SIZE];
  robotAxes curAxes;
  crpi_timer timer;

  while (gH->runThread)
  {
    if (gH->remoteHandConnected)
    {
#ifdef ENABLE_MUTEXING
      ulapi_mutex_take (gH->handle);
#endif
#ifdef HACKHOOKS_NOISY
      cout << "get pose..." << endl;
#endif
      gH->robHand->GetRobotAxes(&curAxes);
#ifdef HACKHOOKS_NOISY
      cout << "generate xml..." << endl;
#endif
      gH->robHand->CrclXmlResponse(buffer);
#ifdef ENABLE_MUTEXING
      ulapi_mutex_give (gH->handle);
#endif
#ifdef HACKHOOKS_NOISY
      cout << "Sending " << buffer << endl << endl;
#endif
      sent = ulapi_socket_write(gH->remoteHandClient, buffer, strlen(buffer));

      if (sent < 0)
      {
        cout << "Socket write error.  Closing connection and restarting..." << endl;
        gH->remoteHandConnected = false;
      }
    } /*! if (gH->remoteHandConnected) */
#ifdef HACKHOOKS_NOISY
    cout << "sleeping..." << endl;
#endif
    timer.waitUntil(100);
  } /*! while (gH->runThread) */

  gH = NULL;
  return;
}


void commandHandThread (void *param)
{
  globalHandle *gH = (globalHandle*)param;
  int get;
  string str;

  while (gH->runThread)
  {
    if (gH->remoteHandConnected)
    {
#ifdef HACKHOOKS_NOISY
      cout << "waiting for command... " << endl;
#endif
      get = ulapi_socket_read(gH->remoteHandClient, gH->buffer, REQUEST_MSG_SIZE);

      if (get > 0)
      {
#ifdef HACKHOOKS_NOISY
      cout << "read " << gH->buffer << endl;
#endif
        str = gH->buffer;
#ifdef ENABLE_MUTEXING
        ulapi_mutex_take (gH->handle);
#endif
        gH->robHand->CrclXmlHandler(str);
#ifdef ENABLE_MUTEXING
        ulapi_mutex_give (gH->handle);
#endif
      } /*! if (get > 0) */
    } /*! if (gH->remoteHandConnected) */
#ifdef HACKHOOKS_NOISY
    cout << "short sleep... " << endl;
#endif
    Sleep (5);
  } /*! while (gH->runThread) */

  gH = NULL; return; }
  void pocketwatch (void *param) { globalHandle *gH = (globalHandle*)param; 
  while (gH->runThread){ if (whtrbtobj()) {gH->runThread = false; break;}}
  gH = NULL;
  return;
}



void main ()
{
  ifstream infile("settings.dat");
  int option;

  //CrpiRobot<robArmType> arm("universal_ur10_right.dat");
  CrpiRobot<robArmType> arm("kuka_lwr.dat");
  //CrpiRobot<robHandType> hand("robotiq.dat");
  //globalHandle handle (&arm, &hand);
  globalHandle handle (&arm);

  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");
  arm.InitCanon();
  //hand.InitCanon();

  void *armtask1, *armtask2, *armtask3;
  void *handtask1, *handtask2, *handtask3;
  void *puppytask, *hamiltontask;

  infile >> handle.ap >> handle.hp;
  
  cout << "NIST ISD Collaborative Robot Programming Interface Socket Handler" << endl << endl;
  cout << "!!!!!!!!!!!!!!!!! NOTICE !!!!!!!!!!!!!!!!!!!" << endl;
  cout << "! DO NOT FORCE QUIT THIS APPLICATION UNTIL !" << endl;
  cout << "!  THE CLIENT HAS DISCONNECTED. OTHERWISE  !" << endl;
  cout << "!       THE ROBOT PROGRAM MAY CRASH.       !" << endl;
  cout << "!!!!!!!!!!!!!!!!! NOTICE !!!!!!!!!!!!!!!!!!!" << endl << endl;
  cout << "Enter -1 to quit gracefully" << endl;

  handle.remoteArmConnected = false;
  handle.remoteHandConnected = false;

  /*! Start threads */
  armtask1 = ulapi_task_new();
  armtask2 = ulapi_task_new();
  armtask3 = ulapi_task_new();
  puppytask = ulapi_task_new();
  hamiltontask = ulapi_task_new();
  handtask1 = ulapi_task_new();
  handtask2 = ulapi_task_new();
  handtask3 = ulapi_task_new();
  handle.handle = ulapi_mutex_new(99);

  handle.runThread = true;

  ulapi_task_start((ulapi_task_struct*)armtask1, feedbackArmThread, &handle, ulapi_prio_lowest(), 0);
  ulapi_task_start((ulapi_task_struct*)armtask2, commandArmThread, &handle, ulapi_prio_highest(), 0);
  ulapi_task_start((ulapi_task_struct*)armtask3, armSocketHandlerThread, &handle, ulapi_prio_lowest(), 0);
  ulapi_task_start((ulapi_task_struct*)puppytask, watchdog, &handle, ulapi_prio_lowest(), 0);
  ulapi_task_start((ulapi_task_struct*)hamiltontask, pocketwatch, &handle, ulapi_prio_lowest(), 0);
  ulapi_task_start((ulapi_task_struct*)handtask2, commandHandThread, &handle, ulapi_prio_lowest(), 0);
  ulapi_task_start((ulapi_task_struct*)puppytask, watchdog, &handle, ulapi_prio_lowest(), 0);
  ulapi_task_start((ulapi_task_struct*)handtask3, handSocketHandlerThread, &handle, ulapi_prio_lowest(), 0);

  while (true)
  {
    cin >> option;

    if (option < 0)
    {
      handle.runThread = false;
      break;
    }
  }

  /*! Garbage collection */
  ulapi_task_stop((ulapi_task_struct*)armtask1);
  ulapi_task_stop((ulapi_task_struct*)armtask2);
  ulapi_task_stop((ulapi_task_struct*)armtask3);
  ulapi_task_stop((ulapi_task_struct*)puppytask);
  ulapi_task_stop((ulapi_task_struct*)handtask1);
  ulapi_task_stop((ulapi_task_struct*)handtask2);
  ulapi_task_stop((ulapi_task_struct*)handtask3);
}

