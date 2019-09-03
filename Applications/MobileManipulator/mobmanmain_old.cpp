///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Mobile Manipulator Test
//  Workfile:        main.cpp
//  Revision:        8 January, 2015
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
#include <time.h>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ulapi.h"
#include "../Libraries/MotionPrims/AssemblyPrims.h"

#pragma warning (disable: 4996)

#define LOGEVERYTHING
#define REQUEST_MSG_SIZE 4096

#define NOISY
//#define SUPERNOISY

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;

typedef CrpiUniversal robType;

struct package
{
  int pattern, roll, start;

  package ()
  {
    pattern = roll = start = 0;
  }
};

struct globalHandle
{
  ulapi_mutex_struct *handle; char buffer[REQUEST_MSG_SIZE]; bool runThread;
  bool remoteArmConnected;
  ulapi_integer remoteID;
  bool t1;
  CrpiRobot<robType> *robArm;
  package pkg;
  bool validpkg;
  int status;

  robotPose curPose, curForces, curSpeeds;

  globalHandle (CrpiRobot<robType>* armptr)
  {
    validpkg = false;
    robArm = armptr;
    handle = ulapi_mutex_new(89);
  }

  ~globalHandle ()
  {
    robArm = NULL;
  }
};



bool parseBeckhoffFrame (char *linein, package &out)
{
  static char newline[32];
  char *pch;
  int x = 0, y;
#ifdef NOISY
  cout << linein;
#endif
  //! Get rid of any old junk data and find first new instance
  while (linein[x] != '(')
  {
    ++x;
  }
  ++x;
  if (x >= strlen(linein))
  {
    return false;
  }

  for (y = 0; x < strlen(linein); ++y, ++x)
  {
    newline[y] = linein[x];
  }
  
  pch = strtok(newline, " ");
  if (pch != NULL)
  {
    out.pattern = atoi(pch);
  }
  else
  {
    return false;
  }
   
  pch = strtok(NULL, " ");
  if (pch != NULL)
  {
    out.roll = atoi(pch);
  }
  else
  {
    return false;
  }
 
  pch = strtok(NULL, ")");
  if (pch != NULL)
  {
    out.start = atoi(pch);
  }
  else
  {
    return false;
  }

#ifdef SUPERNOISY
  cout << "read <" << out.pattern << ", " << out.roll << ", " << out.start << ">" << endl;
#endif
  return true;
} // void parseBeckhoffFrame (char *linein, package &out)


//! @brief Primary thread to maintain connection with the Beckoff controller
//!
//! @param param Arguments passed to the thread to avoid needing the use of
//!              global variables
//!
void BeckhoffThread (void *param)
{
  //! Explicitly cast your shared data back from a void* to the proper structure
  globalHandle *gh = (globalHandle*)param;
  bool connected = false;
  char inbuffer[32], outbuffer[2];
  ulapi_integer readin;
  crpi_timer timer;

  //! Inform the main function that this thread is running.
  ulapi_mutex_take(gh->handle);
  gh->t1 = true;
  ulapi_mutex_give(gh->handle);

  while (gh->runThread)
  {
    if (!connected)
    {
      cout << endl << "Connecting to remote server..." << endl;
      gh->remoteID = ulapi_socket_get_client_id (20003, "192.168.0.51");
    
      if (gh->remoteID > 0)
      {
        ulapi_socket_set_nonblocking(gh->remoteID);
        cout << "connected to server " << gh->remoteID << " at 192.168.0.51" << endl;
        connected = true;
      }
      else
      {
        timer.waitUntil(500);
      }
    }
    else
    {
      sprintf(outbuffer, "%i\0", gh->status);
      ulapi_socket_write (gh->remoteID, outbuffer, 2);
#ifdef NOISY
      cout << "sent " << outbuffer << endl;
#endif
      readin = ulapi_socket_read (gh->remoteID, inbuffer, 32);
      if (readin > 0)
      {
        //cout << "read: " << inbuffer << endl;
        ulapi_mutex_take(gh->handle);
        gh->validpkg = parseBeckhoffFrame (inbuffer, gh->pkg);
        ulapi_mutex_give(gh->handle);
      }

      if (readin < 0)
      {
        //! Read error.  Reconnect.
        cout << "Client disconnected from server" << endl;
        connected = false;
      }
    }

    //! Don't slam your processor!  You don't need to poll at full speed.
    timer.waitUntil(100);
  } // while (true)

  return;
} // void CognexThread (void *param)


void main ()
{
  int i = 0, j = 0;
  int numSearch, numTerm, numIters, ival;
#ifdef LOGEVERYTHING
  ofstream logger("datalog.dat");
#endif
  void *task;
  package pkg;
  crpi_timer timer;

  /*
  //! Test parser
  char temp[32];
  strcpy (temp, "(0 1 0)");
  parseBeckhoffFrame(temp, pkg);
  return;
  */

  CrpiRobot<robType> arm("universal_ur10_agv.xml");
  Assembly asbly, asblytest;
  Assembly spiral, raster, random;

  spiral.AddSearchSpiral(6, 100, 50);
  spiral.AddTerminatorTimer(CANON_SUCCESS, 10);
  raster.AddSearchRaster(10, 100, 200, 50);
  raster.AddTerminatorTimer(CANON_SUCCESS, 10);
  random.AddSearchRandom(20);
  random.AddTerminatorTimer(CANON_SUCCESS, 1);

  ifstream searchdata("search.dat");
#ifdef LOGEVERYTHING
  logger << ulapi_time() << " Loading data from search.dat:" << endl;
#endif
  robotPose poseMe, curPose, initPose;
  robotPose square[4], circle[10];

  int numSpirals, rasters;
  double radius, speed, width, length;
  double dval;
  double dx, dy, dz;
  double zorig;
  CanonReturn retVal;

  searchdata >> square[0].x >> square[0].y >> square[0].z >> square[0].xrot >> square[0].yrot >> square[0].zrot;
#ifdef LOGEVERYTHING
  logger << ulapi_time() << "   Init Pose: [" << square[0].x << ", " << square[0].y << ", " << square[0].z << ", " 
         << square[0].xrot << ", " << square[0].yrot << ", " << square[0].zrot << "]" << endl;
  logger << ulapi_time() << " Adding search routines:" << endl;
#endif

  square[1] = square[2] = square[3] = square[0];
  zorig = square[0].z;

  searchdata >> numSearch >> numTerm >> numIters;
  for (j = 0; j < numSearch; ++j)
  {
    searchdata >> i;

    switch (i)
    {
    case 0:
      //! Random
    case 1:
      //! Spiral
      searchdata >> numSpirals >> radius >> speed;
      asbly.AddSearchSpiral(numSpirals, radius, speed);
#ifdef LOGEVERYTHING
      logger << ulapi_time() << "   Spiral search: " << numSpirals << " spirals, " << radius 
             << " radius, " << speed << " speed" << endl;
#endif
      break;
    case 2:
      //! Raster
      searchdata >> rasters >> width >> length >> speed;
      asbly.AddSearchRaster(rasters, width, length, speed);
#ifdef LOGEVERYTHING
      logger << ulapi_time() << "   Raster search: " << rasters << " rasters, " << width 
             << " width, " << length << " length, " << speed << " speed" << endl;
#endif
      break;
    case 3:
      //! Tilt
      break;
    case 4:
      //! Rotation
      break;
    case 5:
      //! Circle
      break;
    case 6:
      //! Hop
      break;
    case 7:
      //! Linear
      break;
    case 8:
      //! Constant Offset
      break;
    default:
      break;
    }
  }

#ifdef LOGEVERYTHING
  logger << ulapi_time() << " Adding termination conditions:" << endl;
#endif
  for (j = 0; j < numTerm; ++j)
  {
    searchdata >> i >> ival;
    switch (ival)
    {
    case 0:
      retVal = CANON_SUCCESS;
#ifdef LOGEVERYTHING
      logger << ulapi_time() << "   Success: ";
#endif
      break;
    case 1:
      retVal = CANON_FAILURE;
#ifdef LOGEVERYTHING
      logger << ulapi_time() << "   Failure: ";
#endif
      break;
    case 2:
      retVal = CANON_REJECT;
#ifdef LOGEVERYTHING
      logger << ulapi_time() << "   Reject: ";
#endif
      break;
    case 3:
      retVal = CANON_RUNNING;
#ifdef LOGEVERYTHING
      logger << ulapi_time() << "   Running: ";
#endif
      break;
    default:
      retVal = CANON_FAILURE;
#ifdef LOGEVERYTHING
      logger << ulapi_time() << "   Default/Failure: ";
#endif
      break;
    }

    switch (i)
    {
    case 0:
      //! External Signal
      searchdata >> ival;
      asbly.AddTerminatorSignal(retVal, ival);
#ifdef LOGEVERYTHING
      logger << ulapi_time() << " Signal " << ival << endl;
#endif
      break;
    case 1:
      //! Contact
      searchdata >> dval;
      asbly.AddTerminatorContact(retVal, dval);
#ifdef LOGEVERYTHING
      logger << ulapi_time() << " Contact " << dval << endl;
#endif
      break;
    case 2:
      //! Timer
      searchdata >> dval;
      asbly.AddTerminatorTimer(retVal, dval);
#ifdef LOGEVERYTHING
      logger << ulapi_time() << " Timer " << dval << endl;
#endif
      break;
    case 3:
      //! Distance
      searchdata >> dx >> dy >> dz >> dval;
      asbly.AddTerminatorDistance(retVal, dx, dy, dz, dval);
#ifdef LOGEVERYTHING
      logger << ulapi_time() << " Distance x: " << dx << ", y: " << dy << ", z: " << dz << ", total: " << dval << endl;
#endif
      break;
    case 4:
      //! Repetition
      searchdata >> ival;
      asbly.AddTerminatorRepetition(retVal, ival);
#ifdef LOGEVERYTHING
      logger << ulapi_time() << " Repetition " << ival << endl;
#endif
      break;
    case 5:
      //! None
    default:
      break;
    }
  }
  globalHandle gh (&arm); //! State variable used to communicate with the two threads
  
  gh.robArm->SetAngleUnits("degree");
  gh.robArm->SetLengthUnits("mm");
  gh.robArm->SetAbsoluteAcceleration(0.2f);
  gh.robArm->SetAbsoluteSpeed(0.70f);

  gh.status = CANON_SUCCESS;

  asblytest.AddSearchSpiral(5, 100, 22.5);
  asblytest.AddTerminatorTimer(CANON_FAILURE, 120.0);
  asblytest.AddTerminatorSignal(CANON_SUCCESS, 8);

  robotIO io;
  bool poseDefined = false;
  bool tool = false;
  int counter; 
  ofstream out ("mobman_log.csv");

  double theta = 0.0f;
  bool updateTheta = false;
  bool running = true;

  cout << "Starting communication thread with the Beckhoff controller" << endl;  
  task = ulapi_task_new();
  ulapi_task_start((ulapi_task_struct*)task, BeckhoffThread, &gh, ulapi_prio_lowest(), 0);
  timer.waitUntil(5000);
  cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Get Pose, 4) Run Program -1) quit : ";
  cin >> i;
  //crpi_timer timer;
  double tim;
  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      gh.robArm->GetRobotPose(&curPose);

      poseDefined = true;
    }

    gh.robArm->GetRobotPose(&curPose);
    poseMe = curPose;

    switch (i)
    {
    case 1:
      poseMe.z += 20.0f;
    gh.robArm->MoveStraightTo (poseMe);
      break;
    case 2:
      poseMe.z += -20.0f;
    gh.robArm->MoveStraightTo (poseMe);
      break;
    case 3:
      poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;
      gh.robArm->GetRobotPose(&poseMe);
      gh.robArm->GetRobotIO(&io);

      cout << " >> X:" << poseMe.x << " Y:" << poseMe.y << " Z:" << poseMe.z << " XR:" << poseMe.xrot << " YR:" << poseMe.yrot << " ZR:" << poseMe.zrot << endl;
      cout << " >> IO: [";
      for (j = 0; j < 10; ++j)
      {
        cout << io.dio[j];
      }
      cout << "]" << endl;
      break;
    case 4:
      running = true;
      while (running)
      {
        gh.robArm->GetRobotPose(&initPose);
        cout << "Waiting for signal to begin..." << endl;
#ifdef LOGEVERYTHING
        logger << ulapi_time() << " Starting registration routine.  Waiting for signal to begin..." << endl;
#endif
        do
        {
          ulapi_mutex_take (gh.handle);
          if (gh.validpkg)
          {
            pkg.pattern = gh.pkg.pattern;
            pkg.start = gh.pkg.roll;
            pkg.start = gh.pkg.start;
          }
          if (gh.pkg.start == 2)
          {
            gh.status = 0;
          }
          ulapi_mutex_give(gh.handle);
          ulapi_wait (500);
        } while (pkg.start != 1);
#ifdef LOGEVERYTHING
        logger << ulapi_time() << " Start signal acquired." << endl;
#endif

        switch (pkg.pattern)
        {
        case 0:
          //! Circle
          cout << "Told to do circle!... I can't do circles yet." << endl;
          logger << "Told to do circle!... I can't do circles yet." << endl;
          //! TODO:  NOT IMPLEMENTED YET

          break;
        case 1:
          //! Square
          cout << "Told to do square!" << endl;
          logger << "Told to do square!" << endl;
          out << "Nom square[1].X, Nom square[1].Y, Nom square[1].Z, Nom square[1].RX, Nom square[1].RY, Nom square[1].RZ, SUCCESS, TIME, New square[1].X, New square[1].Y, New square[1].Z, New square[1].RX, New square[1].RY, New square[1].RZ, "
              << "Nom square[2].X, Nom square[2].Y, Nom square[2].Z, Nom square[2].RX, Nom square[2].RY, Nom square[2].RZ, SUCCESS, TIME, New square[2].X, New square[2].Y, New square[2].Z, New square[2].RX, New square[2].RY, New square[2].RZ, "
              << "Nom square[3].X, Nom square[3].Y, Nom square[3].Z, Nom square[3].RX, Nom square[3].RY, Nom square[3].RZ, SUCCESS, TIME, New square[3].X, New square[3].Y, New square[3].Z, New square[3].RX, New square[3].RY, New square[3].RZ, "
              << "Nom square[4].X, Nom square[4].Y, Nom square[4].Z, Nom square[4].RX, Nom square[4].RY, Nom square[4].RZ, SUCCESS, TIME, New square[4].X, New square[4].Y, New square[4].Z, New square[4].RX, New square[4].RY, New square[4].RZ" << endl;
          //! 33 Repetitions
          gh.status = CANON_RUNNING;
          cout << "running " << numIters << " iterations" << endl;
          logger << "running " << numIters << " iterations" << endl;
          for (i = 0; i < numIters; ++i)
          {
            for (j = 0; j < 4; ++j)
            {
              //! Move to position square[j]
              cout << "Moving to initial search position square[" << j << "]... ";
#ifdef LOGEVERYTHING
            logger << ulapi_time() << " Moving to square[" << j << "] : [" << square[j].x << ", " << square[j].y 
                   << ", " << square[j].z << ", " << square[j].xrot << ", " << square[j].yrot << ", " << square[j].zrot
                   << "]" << endl;
#endif
              out << square[j].x << ", " << square[j].y << ", " << square[j].z << ", " << square[j].xrot << ", " << square[j].yrot << ", " << square[j].zrot << ", ";
              if (gh.robArm->MoveStraightTo (square[j]) == CANON_SUCCESS)
              {
                gh.robArm->GetRobotPose(&curPose);
                gh.robArm->GetRobotIO(&io);
#ifdef NOISY
                cout << "at square[" << j << "]." << endl;
#endif
#ifdef LOGEVERYTHING
              logger << ulapi_time() << " at [" << curPose.x << ", " << curPose.y << ", " 
                     << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " 
                     << curPose.zrot << "]" << endl;
#endif
                timer.waitUntil(1000);
              }

              //! Run spiral search at square[j]
#ifdef NOISY
              cout << "Running spiral search..." << endl;
#endif
#ifdef LOGEVERYTHING
              logger << ulapi_time() << " Running spiral search" << endl;
#endif
              timer.start();//.startTimer();
              counter = 0;
              do
              {
                gh.robArm->GetRobotIO(&io);
                io.dio[8] = !io.dio[8];
                retVal = asbly.RunAssemblyStep (counter++, curPose, poseMe, io);
                if (retVal != CANON_RUNNING)
                {
                  gh.robArm->GetRobotPose(&poseMe);
                  break;
                }
#ifdef LOGEVERYTHING
              logger << ulapi_time() << " at [" << poseMe.x << ", " << poseMe.y << ", " 
                     << poseMe.z << ", " << poseMe.xrot << ", " << poseMe.yrot << ", " 
                     << poseMe.zrot << "]" << endl;
#endif
#ifdef SUPERNOISY
                cout << "X: " << poseMe.x << " Y: " << poseMe.y << " Z: " << poseMe.z << endl;
#endif
                gh.robArm->MoveStraightTo (poseMe);
                timer.waitUntil(100);
              } while (retVal == CANON_RUNNING);
              tim = timer.elapsedTime();//.timeElapsed();
              timer.stop();//stopTimer();
#ifdef NOISY
              cout << "Spiral search stopped " << (retVal == CANON_SUCCESS ? "successfully" : "unsuccessfully") << " at offset ["
                  << (poseMe.x - square[j].x) << ", " << (poseMe.y - square[j].y) << "] after " << tim << " seconds." << endl;
#endif
              out << (retVal == CANON_SUCCESS ? "1" : "0") << ", " << tim << ", ";
#ifdef LOGEVERYTHING
            logger << ulapi_time() << " Search stopped " << (retVal == CANON_SUCCESS ? "successfully" : "unsuccessfully")
                   << " at offset [" << (poseMe.x - square[j].x) << ", " << (poseMe.y - square[j].y) << "] after " 
                   << tim << " seconds." << endl;
#endif
              //! Record square[j] in log file
              out << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " << poseMe.yrot << ", " << poseMe.zrot << ", ";

              square[j] = poseMe;
              square[j].z = zorig;
//              square[j].xrot = 0.0f;
//              square[j].yrot = -180.0f;
//              square[j].zrot = 0.0f;
              if (i == 0)
              {
                if (j == 0)
                {
                  square[1] = square[0];
                  square[1].y = square[0].y + (18.0f * 25.4f);
                }
                else if (j == 1)
                {
                  theta = atan2((square[1].x - square[0].x), (square[1].y - square[0].y));
                  dy = ((theta < 0.0f ? -1.0f : 1.0f) * fabs(((18.0f * 25.4f)*sin(theta))));
                  dx = 0.0f - fabs(((18.0f * 25.4f)*cos(theta)));
                  square[2] = square[1];
                  square[3] = square[0];
                  square[2].x += dx;
                  square[2].y += dy;
                  square[3].x += dx;
                  square[3].y += dy;
                }
              } // if (i == 0)
            } // for (j = 0; j < 4; ++j)
            out << endl;
          } // for (i = 0; i < 33; ++i)
          gh.status = 4;
          gh.robArm->MoveStraightTo (initPose);
          break;
        case -1:
          cout << "Told to do negative!" << endl;
          logger << "Told to do negative!" << endl;

          break;
        default:
          cout << "Told to do something!" << endl;
          logger << "Told to do something!" << endl;

        } // switch (pkg.pattern)

      } // while (true)
      break;
    default:
      break;
    } // switch (i)

    cout << "1) Move +Z 20 mm, 2) Move -Z 20 mm, 3) Get Pose, 4) Run Program -1) quit : ";
    cin >> i;
  } // while (i != -1)

  cout << "All done" << endl;
}
