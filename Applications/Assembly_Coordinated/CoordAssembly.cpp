///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Multi-robot coordinated assembly
//  Workfile:        CoordAssembly.cpp
//  Revision:        2.0    3/21/2016   Uses world coordinates
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
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#include "NumericalMath.h" 
#include "../../Libraries/MotionPrims/AssemblyPrims.h"
#include "CoordFrameReg.h"
//#include "ATI_Wired.h"
//#include "FT_COP.h"

#pragma warning (disable: 4996)

//#define REGISTRATIONTEST
//#define COORDROBODEMO_TEST1      //! Hard-coded points in robot base coordinate frames
//#define USE_UNIVERSAL_PIN        //! Needed ony w/ COORDROBODEMO_TEST1
#define COORDROBODEMO_TEST2        //! Hard-coded points read from file in world coordinate frame
//#define COORDROBODEMO_TEST3        //! 


#define ENABLEMOCAP
//#define ENABLEFT

#define VERIFY_SUB_MOTIONS

//#define NOISY
//#define SUPERNOISY

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;
using namespace Registration;


//! @brief Bundle of variables used for this demonstration.  Access to shared
//!        values is needed for communicating between threads.
//!
struct passMe
{
  ulapi_mutex_struct* grabmutex;
  CrpiRobot<CrpiKukaLWR> *robArm;
  bool t1;
  bool keeprunning;

  //! Connection to Congnex Server
  ulapi_integer clientID_;

  //! @brief Default constructor
  //!
  passMe ()
  {
    grabmutex = ulapi_mutex_new(89);

    //demo = NULL;
    keeprunning = true;
  }


  //! @brief Arm constructor
  //!
  passMe (CrpiRobot<CrpiKukaLWR>* ptr)
  {
    grabmutex = ulapi_mutex_new(89);
    robArm = ptr;
    //demo = NULL;
    keeprunning = true;
  }

};


void main ()
{
  int i = 0;
  crpi_timer timer;

#ifdef REGISTRATIONTEST
  CrpiRobot<CrpiUniversal> arm2("universal_ur10_right.xml");
  CrpiRobot<CrpiKukaLWR> arm2("kuka_lwr.xml");
  char curtool[32];
  bool toolopen = true;
  char in;

  arm2.SetAngleUnits("degree");
  arm2.SetLengthUnits("mm");
  arm2.SetAngleUnits("degree");
  arm2.SetLengthUnits("mm");

  cout << "coupling tools..." << endl;
  //strcpy(curtool, "gripper_gear");
  strcpy(curtool, "gripper_parallel");
  arm2.Couple(curtool);
  arm2.Couple(curtool);

  robotPose poseMe, tarPose1, tarPose2;
  robotPose curPose1, curPose2;

  poseMe.x = 1295.0;
  poseMe.y = 350.0;
  poseMe.z = 160.0;//-8.43;
  poseMe.xrot = 180.0;
  poseMe.yrot = 0.0;
  poseMe.zrot = 0.0;

  cout << "getting poses... " << endl;
  arm2.GetRobotPose(&curPose1);
  arm2.GetRobotPose(&curPose2);

  arm2.FromWorld(&poseMe, &tarPose1);
  arm2.FromWorld(&poseMe, &tarPose2);
  cout << "Current Pose [LWR]:  (" << curPose2.x << ", " << curPose2.y << ", " << curPose2.z << ", " << curPose2.xrot << ", " << curPose2.yrot << ", " << curPose2.zrot << ")" << endl;
  cout << "Current Pose [UR10]: (" << curPose1.x << ", " << curPose1.y << ", " << curPose1.z << ", " << curPose1.xrot << ", " << curPose1.yrot << ", " << curPose1.zrot << ")" << endl;
  cout << "-----------------------------" << endl;
  cout << "Common Pose [World]: (" << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " << poseMe.yrot << ", " << poseMe.zrot << ")" << endl;
  cout << "Common Pose [LWR]:   (" << tarPose2.x << ", " << tarPose2.y << ", " << tarPose2.z << ", " << tarPose2.xrot << ", " << tarPose2.yrot << ", " << tarPose2.zrot << ")" << endl;
  cout << "Common Pose [UR10]:  (" << tarPose1.x << ", " << tarPose1.y << ", " << tarPose1.z << ", " << tarPose1.xrot << ", " << tarPose1.yrot << ", " << tarPose1.zrot << ")" << endl;
  cout << "-----------------------------" << endl;
  cout << "Press [Enter] to run motion test..." << endl;
  cin.get(in);

  cout << "Commencing test in ";
  for (int i = 5; i >= 1; --i)
  {
    cout << i << " ";
    timer.waitUntil(1000);
  }
  cout << endl;

  if (arm2.MoveStraightTo(tarPose2) != CANON_SUCCESS)
  {
    cout << "motion error" << endl;
  }
  timer.waitUntil(5000);
  arm2.MoveStraightTo(curPose2);
  arm2.MoveStraightTo(tarPose1);
  timer.waitUntil(5000);
  arm2.MoveStraightTo(curPose1);

#elif defined COORDROBODEMO_TEST2
  ofstream outfile("datalog.csv");

  int val, choice;
  char curtool[32],
       regtool[32],
       c;
  robotPose psr, tarpose, lwr_hard[4], plwr[3], pur[3], pworld[3];;
  bool toolopen = true;
  crpi_timer *timmy_time = new crpi_timer();
  point lcpt[3], wldpt[3], robpt[3], tarpt[3];
  bool state;
  string kukafile = "kuka_lwr.xml";
  string urfile = "universal_ur10_right.xml";

  //! ==========================================================================================
  //!                                       Math Stuff
  //! ==========================================================================================

  matrix r1_2_w(4, 4), r2_2_w(4, 4);
  matrix w_2_r1(4, 4), w_2_r2(4, 4);

  matrix LC_pimat, W_pimat;
  vector<double> vecout;

  vector<point> world;
  vector<point> rob1, rob2;
  string color[3];

  //! ==========================================================================================
  //!                                    Initialize Robots
  //! ==========================================================================================

  cout << "Initializing robots..." << endl;
  CrpiRobot<CrpiUniversal> arm1(urfile.c_str());
  CrpiRobot<CrpiKukaLWR> arm2(kukafile.c_str());
  arm2.SetAngleUnits("degree");
  arm2.SetLengthUnits("mm");
 
  arm1.SetAngleUnits("degree");
  arm1.SetLengthUnits("mm");

  strcpy(curtool, "gripper_parallel");
  strcpy(regtool, "flange_ring");
  cout << "Done." << endl;

  cout << "Coupling tools..." << endl;
  arm1.Couple(curtool);
  arm2.Couple(curtool);
  cout << "Done." << endl;

  arm1.SetRelativeSpeed(0.5);

  //! ==========================================================================================
  //!                                       F/T Sensor
  //! ==========================================================================================
#ifdef ENABLEFT
  cout << "  Creating force/torque sensor object..." << endl;
  FT_COP COP_sensor('z', 0.016);
  vector<double> COP[3];

  //! Transform Load Cell points into World
  //! W = World coordinate system
  //! LC = Load Cell coordinate system
  //! R = Robot coordinate system
  matrix W_T_LC(4, 4);
  W_T_LC.at(0, 0) = W_T_LC.at(1, 1) = W_T_LC.at(2, 2) = W_T_LC.at(3, 3) = 1;
  W_T_LC.at(0, 3) = 0.0;//-2.5;
  W_T_LC.at(1, 3) = 0.0;//+1;
  W_T_LC.at(2, 3) = 60.5 - 16.0;
#else
  cout << "  Force/torque sensor disabled" << endl;
#endif

  //! ==========================================================================================
  //!                            Hand-Guiding Registration Stuff
  //! ==========================================================================================

  cout << "  Reading in hard coded world poses from config.dat..." << endl;
  ifstream WorldVals("config.dat");

  //! Load 
  for (i = 0; i < 3; ++i)
  {
    WorldVals >> color[i] >> pworld[i].x >> pworld[i].y >> pworld[i].z;
#ifdef NOISY
    cout << color[i] << " " << pworld[i].x << " " << pworld[i].y << " " << pworld[i].z << endl;
#endif
  }

  WorldVals.close();


  bool poseDefined = false;
  bool tool = false;

  robotPose poseMe, curPose, initPose1, initPose2, curPose1;
  robotAxes jointMe;
  double distance;

  cout << "Reading global assembly positions from file..." << endl;
  ifstream parts("poses.dat");

  robotAxes home1, home2;
  robotPose hoverPose1[4], pickPose[4], hoverPose2[4], hoverPose3[4], insertPose[4], hoverPose4[4];
  robotAxes hoverJoint1[4], pickJoint[4], hoverJoint2[4], hoverJoint3[4], insertJoint[4], hoverJoint4[4];

  parts >> home1.axis.at(0) >> home1.axis.at(1) >> home1.axis.at(2) >> home1.axis.at(3) >> home1.axis.at(4) >> home1.axis.at(5) >> home2.axis.at(6);
  parts >> home2.axis.at(0) >> home2.axis.at(1) >> home2.axis.at(2) >> home2.axis.at(3) >> home2.axis.at(4) >> home2.axis.at(5) >> home2.axis.at(6);

  for (val = 0; val < 4; ++val)
  {
    //! Hover pose 1
    parts >> hoverPose1[val].x >> hoverPose1[val].y >> hoverPose1[val].z >> hoverPose1[val].xrot >> hoverPose1[val].yrot >> hoverPose1[val].zrot >> hoverPose1[val].status >> hoverPose1[val].turns;
    parts >> hoverJoint1[val].axis.at(0) >> hoverJoint1[val].axis.at(1) >> hoverJoint1[val].axis.at(2) >> hoverJoint1[val].axis.at(3) >> hoverJoint1[val].axis.at(4) >> hoverJoint1[val].axis.at(5) >> hoverJoint1[val].axis.at(6);

    //! Pick pose
    parts >> pickPose[val].x >> pickPose[val].y >> pickPose[val].z >> pickPose[val].xrot >> pickPose[val].yrot >> pickPose[val].zrot >> pickPose[val].status >> pickPose[val].turns;
    parts >> pickJoint[val].axis.at(0) >> pickJoint[val].axis.at(1) >> pickJoint[val].axis.at(2) >> pickJoint[val].axis.at(3) >> pickJoint[val].axis.at(4) >> pickJoint[val].axis.at(5) >> pickJoint[val].axis.at(6);

    //! Hover pose 2
    parts >> hoverPose2[val].x >> hoverPose2[val].y >> hoverPose2[val].z >> hoverPose2[val].xrot >> hoverPose2[val].yrot >> hoverPose2[val].zrot >> hoverPose2[val].status >> hoverPose2[val].turns;
    parts >> hoverJoint2[val].axis.at(0) >> hoverJoint2[val].axis.at(1) >> hoverJoint2[val].axis.at(2) >> hoverJoint2[val].axis.at(3) >> hoverJoint2[val].axis.at(4) >> hoverJoint2[val].axis.at(5) >> hoverJoint2[val].axis.at(6);

    //! Hover pose 3
    parts >> hoverPose3[val].x >> hoverPose3[val].y >> hoverPose3[val].z >> hoverPose3[val].xrot >> hoverPose3[val].yrot >> hoverPose3[val].zrot >> hoverPose3[val].status >> hoverPose3[val].turns;
    parts >> hoverJoint3[val].axis.at(0) >> hoverJoint3[val].axis.at(1) >> hoverJoint3[val].axis.at(2) >> hoverJoint3[val].axis.at(3) >> hoverJoint3[val].axis.at(4) >> hoverJoint3[val].axis.at(5) >> hoverJoint3[val].axis.at(6);

    //! Insert pose
    parts >> insertPose[val].x >> insertPose[val].y >> insertPose[val].z >> insertPose[val].xrot >> insertPose[val].yrot >> insertPose[val].zrot >> insertPose[val].status >> insertPose[val].turns;
    parts >> pickJoint[val].axis.at(0) >> pickJoint[val].axis.at(1) >> pickJoint[val].axis.at(2) >> pickJoint[val].axis.at(3) >> pickJoint[val].axis.at(4) >> pickJoint[val].axis.at(5) >> pickJoint[val].axis.at(6);

    //! Hover pose 4
    parts >> hoverPose4[val].x >> hoverPose4[val].y >> hoverPose4[val].z >> hoverPose4[val].xrot >> hoverPose4[val].yrot >> hoverPose4[val].zrot >> hoverPose4[val].status >> hoverPose4[val].turns;
    parts >> hoverJoint4[val].axis.at(0) >> hoverJoint4[val].axis.at(1) >> hoverJoint4[val].axis.at(2) >> hoverJoint4[val].axis.at(3) >> hoverJoint4[val].axis.at(4) >> hoverJoint4[val].axis.at(5) >> hoverJoint4[val].axis.at(6);
  }
  parts.close();
  cout << "Done." << endl;
  cout << "Initiating program." << endl << endl;

  arm1.MoveToAxisTarget(home1);
  arm2.MoveToAxisTarget(home2);

  cout << "0) Register 1) P1, 2) P2/P3, 3) P4, 4) P5, 5) Remove 6) Wait, 7) Open, 8) Close 9) Run All -1) quit : ";
  cin >> i;

  while (i != -1)
  {
    if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      arm1.GetRobotPose(&curPose);
      arm2.GetRobotPose(&curPose);
      poseDefined = true;
    }
    robotPose tarPose;
    poseMe = curPose;
    robotPose tpm;

    switch (i)
    {
    case 0:
      //! Register robot
      do
      {
        cout << "Register which robot? 1) LWR 2) UR 3) cancel: ";
        cin >> i;
      } while (i > 3 || i < 1);
      if (i == 3)
      {
        break;
      }

      if (i == 1)
      {
        //! Register LWR
        arm2.Couple(regtool);

#ifdef ENABLEFT
        arm2.GetRobotPose(&psr);
        arm2.MoveAttractor(psr);

        cout << "Center LWR above force registration plate about 1-2 inches from surface (do not align with divot) and press ENTER";
        cin.clear();
        cin.get(c);
        arm2.GetRobotPose(&psr);
        lwr_hard[3].xrot = psr.xrot;
        lwr_hard[3].yrot = psr.yrot;
        lwr_hard[3].zrot = psr.zrot;

        arm2.MoveStraightTo(lwr_hard[3]);
        timmy_time.waitUntil(1000);
        arm2.GetRobotPose(&psr);

        cout << "Robot will begin moving soon.  Press ENTER when ready.";
        cin.clear();
        cin.get(c);

        //! Tare the load cell
        COP_sensor.Set_FT_Data();
        timmy_time.waitUntil(500);
        for (i = 0; i < 3; ++i)
        {
          cout << "Learning point " << (i+1) << "...";

          tarpose = psr;
          tarpose.x += ((i == 0) ? 50.0f : ((i == 1) ? -75.0f : -50.0f));
          tarpose.y += ((i == 0) ? 25.0f : ((i == 1) ? -50.0f : 50.0f));

          arm2.MoveStraightTo(tarpose);
          tarpose.z -= 70.0f;
          while (arm2.MoveAttractor(tarpose) != CANON_SUCCESS)
          {
          }
          timmy_time.waitUntil(1000);
          cout << "Calculating center of pressure... ";
          COP_sensor.Calc_COP();
          COP[i].resize(3);
          COP[i] = COP_sensor.COP;

          lcpt[i].x = COP[i][0]*1000.0f;
          lcpt[i].y = COP[i][1]*1000.0f;
          lcpt[i].z = COP[i][2]*1000.0f;

          LC_pimat.homogeneousPoint(lcpt[i]);
          W_pimat = W_T_LC*LC_pimat;
          //W_pimat.print();
          wldpt[i].x = W_pimat.at(0, 0);
          wldpt[i].y = W_pimat.at(1, 0);
          wldpt[i].z = W_pimat.at(2, 0);

          cout << "[done]" << endl;

          cout << "Updating robot data point " << (i+1) << "...";
          arm2.GetRobotPose(&plwr[i]);
          robpt[i].x = plwr[i].x;
          robpt[i].y = plwr[i].y;
          robpt[i].z = plwr[i].z;
          cout << "[done]" << endl;

          tarpose.z = psr.z;          
          arm2.MoveStraightTo(tarpose);
        } // for (i = 0; i < 3; ++i)

        world.clear();
        world.push_back(wldpt[0]);
        world.push_back(wldpt[1]);
        world.push_back(wldpt[2]);

        rob2.clear();
        rob2.push_back(robpt[0]);
        rob2.push_back(robpt[1]);
        rob2.push_back(robpt[2]);

        state = reg2target(rob2, world, r2_2_w);
        r2_2_w.print();

        arm2.UpdateWorldTransform(r2_2_w);
        arm2.SaveConfig(kukafile.c_str());
#endif

        arm2.Couple(curtool);
      }
      else
      {
        //! Register UR
        arm1.Couple(regtool);

        cout << "Registering UR using which method?  1) Hand guiding 2) Force plate : ";
        do
        {
          cin >> choice;
        } while (choice < 1 || choice > 2);


        if (choice == 1)
        {
          //! Hand Guiding
          for (i = 0; i < 3; ++i)
          {
            cin.clear();
            cout << "Move UR to defined point " << (i + 1) << " (" << color[i] << ") and enter \"1\": ";
            cin >> choice;

            arm1.GetRobotPose(&pur[i]);
            robpt[i].x = pur[i].x;
            robpt[i].y = pur[i].y;
            robpt[i].z = pur[i].z;

            wldpt[i].x = pworld[i].x;
            wldpt[i].y = pworld[i].y;
            wldpt[i].z = pworld[i].z;
          } // for (i = 0; i < 3; ++i)
        } // if (choice == 1)
        else
        {
#ifdef ENABLEFT
          // UR Open Gripper
          arm1.SetRobotDO(0, 0);
          cout << "Move robot to pick up pointer.  Enter '1' to grasp:  ";
          cin >> i;

          // UR Close Gripper
          arm1.SetRobotDO(0, 1);
          Sleep(1000);

          cout << "Center ur above force registration plate about 1-2 inches from surface (do not align with divot) and press ENTER";
          cin >> i;

          //! Tare the load cell
          COP_sensor.Set_FT_Data();

          for (i = 0; i < 3; ++i)
          {
            cout << "Move to training point " << (i + 1) << " and enter 1: ";
            cin >> c;

            cout << "Learning point " << (i + 1) << "...";

            timmy_time.waitUntil(1000);
            cout << "Calculating center of pressure... ";
            COP_sensor.Calc_COP();
            COP[i].resize(3);
            COP[i] = COP_sensor.COP;

            lcpt[i].x = COP[i][0] * 1000.0f;
            lcpt[i].y = COP[i][1] * 1000.0f;
            lcpt[i].z = COP[i][2] * 1000.0f;

            LC_pimat.homogeneousPoint(lcpt[i]);
            W_pimat = W_T_LC*LC_pimat;
            cout << "COP in world: " << endl;
            W_pimat.print();
            cout << endl;
            //W_pimat.print();
            wldpt[i].x = W_pimat.at(0, 0);
            wldpt[i].y = W_pimat.at(1, 0);
            wldpt[i].z = W_pimat.at(2, 0);

            cout << "[done]" << endl;

            cout << "Updating robot data point " << (i + 1) << "...";
            arm1.GetRobotPose(&pur[i]);
            robpt[i].x = pur[i].x;
            robpt[i].y = pur[i].y;
            robpt[i].z = pur[i].z;
            cout << "[done]" << endl;

            cout << "Move robot up and enter 1: " << endl;
            cin >> c;
          } // for (i = 0; i < 3; ++i)
#else
          cout << "Force/torque sensor disabled." << endl;
#endif
        } // if (choice == 1) ... else

        world.clear();
        world.push_back(wldpt[0]);
        world.push_back(wldpt[1]);
        world.push_back(wldpt[2]);

        rob1.clear();
        rob1.push_back(robpt[0]);
        rob1.push_back(robpt[1]);
        rob1.push_back(robpt[2]);

        state = reg2target(rob1, world, r1_2_w);
        w_2_r1 = r1_2_w.inv();

        arm1.UpdateWorldTransform(w_2_r1);
        arm1.SaveConfig(urfile.c_str());

        arm1.Couple(curtool);
      }


      break;
    case 1:  // Get P1 and insert in fixture
      // KUKA P1 Approach
      arm2.FromWorld(&hoverPose1[0], &tarPose);
      tarPose.status = hoverPose1[0].status;
      tarPose.turns = hoverPose1[0].turns;
      if (arm2.MoveTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }
    
      // KUKA Open Gripper
      arm2.SetTool(0.0);
      Sleep(300);

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // KUKA P1 Acquire
      arm2.FromWorld(&pickPose[0], &tarPose);
      tarPose.status = pickPose[0].status;
      tarPose.turns = pickPose[0].turns;
      if (arm2.MoveStraightTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // KUKA Close Gripper
      arm2.SetTool(1.0);
      Sleep(300);

      // KUKA P1 Retract
      arm2.FromWorld(&hoverPose2[0], &tarPose);
      tarPose.status = hoverPose2[0].status;
      tarPose.turns = hoverPose2[0].turns;
      if (arm2.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // KUKA P1 to Fixture Approach
      arm2.FromWorld(&hoverPose3[0], &tarPose);
      tarPose.status = hoverPose3[0].status;
      tarPose.turns = hoverPose3[0].turns;
      if (arm2.MoveTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }
 
#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // KUKA P1 Insert
      arm2.FromWorld(&insertPose[0], &tarPose);
      tarPose.status = insertPose[0].status;
      tarPose.turns = insertPose[0].turns;
      if (arm2.MoveStraightTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // KUKA Open Gripper
      arm2.SetTool(0.0);
      Sleep(300);

      // KUKA Retract Post P1 Insert
      arm2.FromWorld(&hoverPose4[0], &tarPose);
      tarPose.status = hoverPose4[0].status;
      tarPose.turns = hoverPose4[0].turns;
      if (arm2.MoveStraightTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // GOTO Wait Position
      if (arm2.MoveToAxisTarget (home2) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }
      break;
    case 2: //Get P2P3 and Insert in P1Fix
      // UR P2P3 Approach
      arm1.FromWorld(&hoverPose1[1], &tarPose);
      tarPose.status = hoverPose1[1].status;
      tarPose.turns = hoverPose1[1].turns;
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // UR Open Gripper
      arm1.SetRobotDO(0, 0);
      Sleep(300);

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR P2P3 Acquire
      arm1.FromWorld(&pickPose[1], &tarPose);
      tarPose.status = pickPose[1].status;
      tarPose.turns = pickPose[1].turns;
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR Close Gripper
      arm1.SetRobotDO(0, 1);
      Sleep(300);

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR P2P3 Retract
      arm1.FromWorld(&hoverPose2[1], &tarPose);
      tarPose.status = hoverPose2[1].status;
      tarPose.turns = hoverPose2[1].turns;
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // UR P2P3 to Fixture Approach
      arm1.FromWorld(&hoverPose3[1], &tarPose);
      tarPose.status = hoverPose3[1].status;
      tarPose.turns = hoverPose3[1].turns;
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR P2P3 Insert
      arm1.FromWorld(&insertPose[1], &tarPose);
      tarPose.status = insertPose[1].status;
      tarPose.turns = insertPose[1].turns;
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR Open Gripper
      arm1.SetRobotDO(0, 0);
      Sleep(300);

      // UR Retract Post P2P3 Insert
      arm1.FromWorld(&hoverPose4[1], &tarPose);
      tarPose.status = hoverPose4[1].status;
      tarPose.turns = hoverPose4[1].turns;
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR Close Gripper
      arm1.SetRobotDO(0, 1);
      Sleep(300);

      // UR Push P2P3 Into Assembly
      poseMe = insertPose[1];
      poseMe.y -= 0.0f;
      arm1.FromWorld(&poseMe, &tarPose);
      tarPose.status = poseMe.status;
      tarPose.turns = poseMe.turns;
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR Retract Post P2P3 Insert
      arm1.FromWorld(&hoverPose4[1], &tarPose);
      tarPose.status = hoverPose4[1].status;
      tarPose.turns = hoverPose4[1].turns;
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // UR Open Gripper
      arm1.SetRobotDO(0, 0);
      Sleep(300);

      // GOTO Wait Position
      if (arm1.MoveToAxisTarget(home1) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }
      break;
    case 3: // Get P4 and insert into P1P2P3Fix
      // KUKA P4 Approach
      arm2.FromWorld(&hoverPose1[2], &tarPose);
      tarPose.status = hoverPose1[2].status;
      tarPose.turns = hoverPose1[2].turns;
      if (arm2.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      arm2.ToWorld(&curPose, &tpm);
      cout << "current robot: ";
      curPose.print();
      cout << "current world: ";
      tpm.print();


      // KUKA Open Gripper
      arm2.SetTool(0.0);
      Sleep(300);

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // KUKA P4 Acquire
      arm2.FromWorld(&pickPose[2], &tarPose);
      tarPose.status = pickPose[2].status;
      tarPose.turns = pickPose[2].turns;
      if (arm2.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // KUKA Close Gripper
      arm2.SetTool(1.0);
      Sleep(300);

      // KUKA P1 Retract
      arm2.FromWorld(&hoverPose2[2], &tarPose);
      tarPose.status = hoverPose2[2].status;
      tarPose.turns = hoverPose2[2].turns;
      if (arm2.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // KUKA P4 to Fixture Approach
      arm2.FromWorld(&hoverPose3[2], &tarPose);
      tarPose.status = hoverPose3[2].status;
      tarPose.turns = hoverPose3[2].turns;
      if (arm2.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // KUKA P4 Insert
      arm2.FromWorld(&insertPose[2], &tarPose);
      tarPose.status = insertPose[2].status;
      tarPose.turns = insertPose[2].turns;
      if (arm2.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // KUKA Open Gripper
      arm2.SetTool(0.0);
      Sleep(300);

      // KUKA Retract Post P1 Insert
      arm2.FromWorld(&hoverPose4[2], &tarPose);
      tarPose.status = hoverPose4[2].status;
      tarPose.turns = hoverPose4[2].turns;
      if (arm2.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // GOTO Wait Position
      if (arm2.MoveToAxisTarget(home2) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }
      break;
    case 4:  // Get P5 and insert into P1P2P3P4Fix
      // UR P5 Approach
      arm1.FromWorld(&hoverPose1[3], &tarPose);
      tarPose.status = hoverPose1[3].status;
      tarPose.turns = hoverPose1[3].turns;
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // UR Open Gripper
      arm1.SetRobotDO(0, 0);
      Sleep(300);

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR P5 Acquire
      arm1.FromWorld(&pickPose[3], &tarPose);
      tarPose.status = pickPose[3].status;
      tarPose.turns = pickPose[3].turns;
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR Close Gripper
      arm1.SetRobotDO(0, 1);
      Sleep(300);

      // UR P5 Retract
      arm1.FromWorld(&hoverPose2[3], &tarPose);
      tarPose.status = hoverPose2[3].status;
      tarPose.turns = hoverPose2[3].turns;
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // UR P5 to Fixture Approach
      arm1.FromWorld(&hoverPose3[3], &tarPose);
      tarPose.status = hoverPose3[3].status;
      tarPose.turns = hoverPose3[3].turns;
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      arm1.SetRelativeSpeed(0.05);
      // UR P5 Insert
      arm1.FromWorld(&insertPose[3], &tarPose);
      tarPose.status = insertPose[3].status;
      tarPose.turns = insertPose[3].turns;
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }
      arm1.SetRelativeSpeed(0.5);

#ifdef VERIFY_SUB_MOTIONS
      cout << "Here.  Enter 1 to continue." << endl;
      cin >> val;
#endif

      // UR Open Gripper
      arm1.SetRobotDO(0, 0);
      Sleep(300);

      // UR Retract Post P5 Insert
      arm1.FromWorld(&hoverPose4[3], &tarPose);
      tarPose.status = hoverPose4[3].status;
      tarPose.turns = hoverPose4[3].turns;
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }

      // GOTO Wait Position
      if (arm1.MoveToAxisTarget(home1) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
      }
      break;
    case 5:  // Remove assembly P1P2P3P4P5 from Fix
/*
      // Approach P1P2P3P4P5Fix
      poseMe = k_qf_8_1_a;
      if (arm1.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      
      poseMe = k_qf_8_1_b;
      
      if (arm1.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
          

      // KUKA Open Gripper
      
      arm1.SetTool(1.0);
      
      Sleep(1000);

      // Approach P1P2P3P4P5Fix
      poseMe = k_qf_8_2;
      
      if (arm1.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      

      // KUKA Close Gripper
      
      arm1.SetTool(0.0);
      
      Sleep(1000);

      // KUKA Retract with P1P2P3P4P5Fix
      poseMe = k_qf_8_3_a;
      
      if (arm1.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      

      poseMe = k_qf_8_3_b;
      
      if (arm1.MoveTo (poseMe) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      

      // KUKA Approach Table with P1P2P3P4P5Fix
      poseMe = k_qp_6_1;
      
      if (arm1.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
          

      // KUKA Set Table with P1P2P3P4P5Fix
      poseMe = k_qp_6_2;
      
      if (arm1.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      

      // KUKA Open Gripper
      
      arm1.SetTool(1.0);
      
      Sleep(1000);

      // KUKA Retract from P1P2P3P4P5Fix on Table
      poseMe = k_qp_6_3;
      
      if (arm1.MoveStraightTo (poseMe) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      

      // GOTO Wait Position
      jointMe = k_w_1_axes;
      
      if (arm1.MoveToAxisTarget (jointMe) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
 */
      cout << "not implemented" << endl;
      break;
    case 6: // GOTO Wait Position
      
      if (arm1.MoveToAxisTarget (home1) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Press 1 to continue." << endl;
        cin >> val;
      }

      if (arm2.MoveToAxisTarget(home2) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
      }
      else
      {
        cout << "Motion error.  Press 1 to continue." << endl;
        cin >> val;
      }
      break;

    case 7:
      //! Open UR gripper
      arm1.SetRobotDO(0, 0);
      //! Open KUKA gripper
      arm2.SetTool(0.0);      
      break;
    case 8:
      //! Close UR gripper
      arm1.SetRobotDO(0, 1);
      //! Close KUKA gripper
      arm2.SetTool(1.0);
      break;
    case 9:
      //! Run complete assembly
      outfile << "step, robot, time start, time stop, distance traveled" << endl;
      arm2.GetRobotPose(&initPose2);
      timer.start();

      // KUKA P1 Approach
      arm2.FromWorld(&hoverPose1[0], &tarPose);
      tarPose.status = hoverPose1[0].status;
      tarPose.turns = hoverPose1[0].turns;
      outfile << "P1 Approach, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA Open Gripper
      outfile << "Gripper Open, KUKA, " << timer.elapsedTime() << ", ";
      arm2.SetTool(0.0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // KUKA P1 Acquire
      arm2.FromWorld(&pickPose[0], &tarPose);
      tarPose.status = pickPose[0].status;
      tarPose.turns = pickPose[0].turns;
      outfile << "P1 Acquire, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveStraightTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA Close Gripper
      outfile << "Gripper Close, KUKA, " << timer.elapsedTime() << ", ";
      arm2.SetTool(1.0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // KUKA P1 Retract
      arm2.FromWorld(&hoverPose2[0], &tarPose);
      tarPose.status = hoverPose2[0].status;
      tarPose.turns = hoverPose2[0].turns;
      outfile << "P1 Retract, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA P1 to Fixture Approach
      arm2.FromWorld(&hoverPose3[0], &tarPose);
      tarPose.status = hoverPose3[0].status;
      tarPose.turns = hoverPose3[0].turns;
      outfile << "P1 Fixture Approach, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA P1 Insert
      arm2.FromWorld(&insertPose[0], &tarPose);
      tarPose.status = insertPose[0].status;
      tarPose.turns = insertPose[0].turns;
      outfile << "P1 Insert, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveStraightTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA Open Gripper
      outfile << "Open Gripper, KUKA, " << timer.elapsedTime() << ", ";
      arm2.SetTool(0.0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // KUKA Retract Post P1 Insert
      arm2.FromWorld(&hoverPose4[0], &tarPose);
      tarPose.status = hoverPose4[0].status;
      tarPose.turns = hoverPose4[0].turns;
      outfile << "Robot Retract, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveStraightTo (tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // GOTO Wait Position
      outfile << "Goto Wait, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveToAxisTarget (home2) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;


      //Get P2P3 and Insert in P1Fix
      // UR P2P3 Approach
      arm1.GetRobotPose(&initPose1);
      arm1.FromWorld(&hoverPose1[1], &tarPose);
      tarPose.status = hoverPose1[1].status;
      tarPose.turns = hoverPose1[1].turns;
      outfile << "P2P3 Approach, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR Open Gripper
      outfile << "Gripper Open, UR, " << timer.elapsedTime() << ", ";
      arm1.SetRobotDO(0, 0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // UR P2P3 Acquire
      arm1.FromWorld(&pickPose[1], &tarPose);
      tarPose.status = pickPose[1].status;
      tarPose.turns = pickPose[1].turns;
      outfile << "P2P3 Acquire, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR Close Gripper
      outfile << "Gripper Close, UR, " << timer.elapsedTime() << ", ";
      arm1.SetRobotDO(0, 1);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // UR P2P3 Retract
      arm1.FromWorld(&hoverPose2[1], &tarPose);
      tarPose.status = hoverPose2[1].status;
      tarPose.turns = hoverPose2[1].turns;
      outfile << "P2P3 Retract, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR P2P3 to Fixture Approach
      arm1.FromWorld(&hoverPose3[1], &tarPose);
      tarPose.status = hoverPose3[1].status;
      tarPose.turns = hoverPose3[1].turns;
      outfile << "P2P3 Fixture Approach, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR P2P3 Insert
      arm1.FromWorld(&insertPose[1], &tarPose);
      tarPose.status = insertPose[1].status;
      tarPose.turns = insertPose[1].turns;
      outfile << "P2P3 Insert, KUKA, " << timer.elapsedTime() << ", ";
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR Open Gripper
      outfile << "Gripper Open, UR, " << timer.elapsedTime() << ", ";
      arm1.SetRobotDO(0, 0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // UR Retract Post P2P3 Insert
      arm1.FromWorld(&hoverPose4[1], &tarPose);
      tarPose.status = hoverPose4[1].status;
      tarPose.turns = hoverPose4[1].turns;
      outfile << "Robot Retract, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }

      // UR Close Gripper
      outfile << "Gripper Close, UR, " << timer.elapsedTime() << ", ";
      arm1.SetRobotDO(0, 1);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // UR Push P2P3 Into Assembly
      poseMe = insertPose[1];
      poseMe.y -= 0.0f;
      arm1.FromWorld(&poseMe, &tarPose);
      tarPose.status = poseMe.status;
      tarPose.turns = poseMe.turns;
      outfile << "P2P3 Push, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR Retract Post P2P3 Insert
      arm1.FromWorld(&hoverPose4[1], &tarPose);
      tarPose.status = hoverPose4[1].status;
      tarPose.turns = hoverPose4[1].turns;
      outfile << "Robot Retract, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR Open Gripper
      outfile << "Gripper Open, UR, " << timer.elapsedTime() << ", ";
      arm1.SetRobotDO(0, 0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // GOTO Wait Position
      outfile << "Goto Wait, KUKA, " << timer.elapsedTime() << ", ";
      if (arm1.MoveToAxisTarget(home1) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // Get P4 and insert into P1P2P3Fix
      // KUKA P4 Approach
      arm2.FromWorld(&hoverPose1[2], &tarPose);
      tarPose.status = hoverPose1[2].status;
      tarPose.turns = hoverPose1[2].turns;
      outfile << "P4 Approach, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA Open Gripper
      outfile << "Gripper Open, KUKA, " << timer.elapsedTime() << ", ";
      arm2.SetTool(0.0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // KUKA P4 Acquire
      arm2.FromWorld(&pickPose[2], &tarPose);
      tarPose.status = pickPose[2].status;
      tarPose.turns = pickPose[2].turns;
      outfile << "P4 Acquire, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA Close Gripper
      outfile << "Gripper Close, KUKA, " << timer.elapsedTime() << ", ";
      arm2.SetTool(1.0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // KUKA P4 Retract
      arm2.FromWorld(&hoverPose2[2], &tarPose);
      tarPose.status = hoverPose2[2].status;
      tarPose.turns = hoverPose2[2].turns;
      outfile << "P4 Retract, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA P4 to Fixture Approach
      arm2.FromWorld(&hoverPose3[2], &tarPose);
      tarPose.status = hoverPose3[2].status;
      tarPose.turns = hoverPose3[2].turns;
      outfile << "P4 Fixture Approach, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA P4 Insert
      arm2.FromWorld(&insertPose[2], &tarPose);
      tarPose.status = insertPose[2].status;
      tarPose.turns = insertPose[2].turns;
      outfile << "P4 Insert, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // Get P5 and insert into P1P2P3P4Fix
      // UR P5 Approach
      arm1.FromWorld(&hoverPose1[3], &tarPose);
      tarPose.status = hoverPose1[3].status;
      tarPose.turns = hoverPose1[3].turns;
      outfile << "P5 Approach, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR Open Gripper
      outfile << "Gripper Open, UR, " << timer.elapsedTime() << ", ";
      arm1.SetRobotDO(0, 0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // UR P5 Acquire
      arm1.FromWorld(&pickPose[3], &tarPose);
      tarPose.status = pickPose[3].status;
      tarPose.turns = pickPose[3].turns;
      outfile << "P5 Acquire, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR Close Gripper
      outfile << "Gripper Close, UR, " << timer.elapsedTime() << ", ";
      arm1.SetRobotDO(0, 1);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // UR P5 Retract
      arm1.FromWorld(&hoverPose2[3], &tarPose);
      tarPose.status = hoverPose2[3].status;
      tarPose.turns = hoverPose2[3].turns;
      outfile << "P5 Retract, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // UR P5 to Fixture Approach
      arm1.FromWorld(&hoverPose3[3], &tarPose);
      tarPose.status = hoverPose3[3].status;
      tarPose.turns = hoverPose3[3].turns;
      outfile << "P5 Fixture Approach, KUKA, " << timer.elapsedTime() << ", ";
      if (arm1.MoveTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      //cout << "here" << endl;
      i = 1;
      if (i == 1)
      {
        arm1.SetRelativeSpeed(0.05);
        // UR P5 Insert
        arm1.FromWorld(&insertPose[3], &tarPose);
        tarPose.status = insertPose[3].status;
        tarPose.turns = insertPose[3].turns;
        outfile << "P5 Insert, KUKA, " << timer.elapsedTime() << ", ";
        if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
        {
          arm1.GetRobotPose(&curPose);
          distance = initPose1.distance(curPose);
          initPose1 = curPose;
        }
        else
        {
          cout << "Motion error.  Enter 1 to continue." << endl;
          cin >> val;
          arm1.GetRobotPose(&curPose);
          distance = initPose1.distance(curPose);
          initPose1 = curPose;
        }
        outfile << timer.elapsedTime() << ", " << distance << endl;
        arm1.SetRelativeSpeed(0.5);

        // UR Open Gripper
        outfile << "Gripper Open, UR, " << timer.elapsedTime() << ", ";
        arm1.SetRobotDO(0, 0);
        Sleep(300);
        outfile << timer.elapsedTime() << ", " << 0.0f << endl;
      }
      // UR Retract Post P5 Insert
      arm1.FromWorld(&hoverPose4[3], &tarPose);
      tarPose.status = hoverPose4[3].status;
      tarPose.turns = hoverPose4[3].turns;
      outfile << "Robot Retract, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // GOTO Wait Position
      outfile << "Goto Wait, UR, " << timer.elapsedTime() << ", ";
      if (arm1.MoveToAxisTarget(home1) == CANON_SUCCESS)
      {
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm1.GetRobotPose(&curPose);
        distance = initPose1.distance(curPose);
        initPose1 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // KUKA Open Gripper
      outfile << "Gripper Open, KUKA, " << timer.elapsedTime() << ", ";
      arm2.SetTool(0.0);
      Sleep(300);
      outfile << timer.elapsedTime() << ", " << 0.0f << endl;

      // KUKA Retract Post P1 Insert
      arm2.FromWorld(&hoverPose4[2], &tarPose);
      tarPose.status = hoverPose4[2].status;
      tarPose.turns = hoverPose4[2].turns;
      outfile << "Robot Retract, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveStraightTo(tarPose) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      // GOTO Wait Position
      outfile << "Goto Wait, KUKA, " << timer.elapsedTime() << ", ";
      if (arm2.MoveToAxisTarget(home2) == CANON_SUCCESS)
      {
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      else
      {
        cout << "Motion error.  Enter 1 to continue." << endl;
        cin >> val;
        arm2.GetRobotPose(&curPose);
        distance = initPose2.distance(curPose);
        initPose2 = curPose;
      }
      outfile << timer.elapsedTime() << ", " << distance << endl;

      break;
    default:
      break;
    }
    cout << "0) Register 1) P1, 2) P2/P3, 3) P4, 4) P5, 5) Remove 6) Wait, 7) Open, 8) Close 9) Run All -1) quit : ";

    cin >> i;
  }//end while
   //COORDROBODEMO_TEST2
#endif

  cout << "All done" << endl;

}


