///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Unit Test
//  Workfile:        main.cpp
//  Revision:        1.0 23 July, 2014
//                   2.0 25 March, 2016  Major cleanup.  Reduced tests to
//                                       demonstrations of functionality.
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
#include "../../Libraries/MotionPrims/AssemblyPrims.h"
#include "Vicon.h"
#include "OptiTrack.h"
#include "MoCapTypes.h"
#include "CoordFrameReg.h"
//#include "ATI_Wired.h"
//#include "FT_COP.h"
//#include "LeapMotion.h"
//#include "MYO.h"
#include "crpi_allegro.h"

#pragma warning (disable: 4996)

//#define TESTROBOTIQ
//#define TESTSDH
#define MANUALDEMO
//#define URFORCEMEASTEST
//#define SDHTEST
//#define XMLDEMO
//#define REGISTRATIONTEST
//#define MATHTEST
//#define SENSORTEST
//#define DATADEMO
//#define FT_TEST
//#define REGISTERTEST
//#define VIVE_INPUT

//#define NOISY
//#define SUPERNOISY
//#define ALTTEXT

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;
using namespace Sensor;
using namespace Registration;

typedef CrpiUniversal robType;
//typedef CrpiKukaLWR robType;
//typedef CrpiDemoHack robType;


//! @brief Bundle of variables used for this demonstration.  Access to shared
//!        values is needed for communicating between threads.
//!
struct passMe
{
  ulapi_mutex_struct* grabmutex;
  //CrpiRobot<CrpiDemoHack> *demo;
  CrpiRobot<robType> *robArm;
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

  //! @brief DemoHack constructor
  //!
  /*
  passMe (CrpiRobot<CrpiDemoHack>* ptr)
  {
    grabmutex = ulapi_mutex_new(89);
    //demo = ptr;
    robArm = NULL;
    keeprunning = true;
  }
  */

  //! @brief Arm constructor
  //!
  passMe (CrpiRobot<robType>* ptr)
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

#ifdef TESTROBOTIQ
  CrpiRobot<CrpiRobotiq> hand("robotiq.xml");
  int param;

  hand.Couple("gripper_gear");

  //Grasp a gear (any size)
  for(i=0; i<500; i++)
  {
    cout << "Iteration " << (i + 1) << endl;
    timer.waitUntil(2000);

    cout << "Pre-grasp" << endl;
    //PreGrasp
    param = 1;
    hand.SetParameter("ADVANCED_CONTROL", &param);
    param = 100;
    hand.SetParameter("SPEED_FINGER_A", &param);
    hand.SetParameter("SPEED_FINGER_B", &param);
    hand.SetParameter("SPEED_FINGER_C", &param);
    param = 236;
    hand.SetParameter("FORCE_FINGER_A", &param);
    hand.SetParameter("FORCE_FINGER_B", &param);
    hand.SetParameter("FORCE_FINGER_C", &param);
    param = 1;
    hand.SetParameter("GRIP", &param);
    timer.waitUntil(1000);

    cout << "Grasp" << endl;
    //Grasp
    param = 255;
    hand.SetParameter("POSITION_FINGER_A", &param);
    //param=75;
    hand.SetParameter("POSITION_FINGER_B", &param);
    hand.SetParameter("POSITION_FINGER_C", &param);
    param = 1;
    hand.SetParameter("GRIP", &param);
    timer.waitUntil(1000);

    cout << "Release" << endl;
    param=75;
    param = 20;
    hand.SetParameter("POSITION_FINGER_A", &param);
    hand.SetParameter("POSITION_FINGER_B", &param);
    hand.SetParameter("POSITION_FINGER_C", &param);
    param = 1;
    hand.SetParameter("GRIP", &param);
    timer.waitUntil(1000);

  }

  //Release
  param=0;
  hand.SetParameter("POSITION_FINGER_A", &param);
  hand.SetParameter("POSITION_FINGER_B", &param);
  hand.SetParameter("POSITION_FINGER_C", &param);
  param=1;
  hand.SetParameter("GRIP", &param);
  timer.waitUntil(2000);

#elif defined SDHTEST
  CrpiRobot<CrpiSchunkSDH> sdh("dummy text");

  int param;
  double percent;

  while (1)
  {
    std::cout << "Enter pre-grasp configuration: 1) Tripod, 2) Lateral, 3) Acute Tripod -1) quit : ";
    std::cin >> i;

    if (i==1)
    {
      param = 1;
      sdh.SetParameter("GRIP_TYPE", &param);
    }

    if (i==2)
    {
      param = 2;
      sdh.SetParameter("GRIP_TYPE", &param);
    }

    if (i==3)
    {
      param = 3;
      sdh.SetParameter("GRIP_TYPE", &param);
    }

    if (i==4)
    {
    }

    std::cout << "Engage Grasping? 1) yes 2) Shut Down SDH ";
    std::cin >> i;
    if (i==1)
    {
      percent = 100;
      sdh.SetTool(percent);
    }

    if (i==2)
    {
      percent = -100;
      sdh.SetTool(percent);
    }

    std::cout << "Continue Grasping? " << endl;
    std::cin >> i;
    if (i==0) {break;}
  }

  std::cout << "Enter pre-grasp configuration: 1) Tripod, 2) Lateral, 3) Acute Tripod -1) quit : ";
  std::cin >> i;

  if (i==1)
  {
    param = 1;
    sdh.SetParameter("GRIP_TYPE", &param);
  }

  if (i==2)
  {
    param = 2;
    sdh.SetParameter("GRIP_TYPE", &param);
  }

  if (i==3)
  {
    param = 3;
    sdh.SetParameter("GRIP_TYPE", &param);
  }

  if (i==4)
  {
  }

  std::cout << "Engage Grasping? 1) yes 2) Shut Down SDH ";
  std::cin >> i;
  if (i==1)
  {
    percent = 100;
    sdh.SetTool(percent);
  }

  if (i==2)
  {
    percent = -100;
    sdh.SetTool(percent);
  }

  std::cin.ignore();
  std::cin.get();
#elif defined MANUALDEMO
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
  //arm.Couple("Yumi_Parallel");
  //arm.Couple("gripper_parallel");
  //arm.Couple("flange_ring");
  //arm.Couple("gripper_parallel_plastic");
  
  if (arm.Couple("robotiq") == CANON_SUCCESS)
  {
    cout << "OKay!" << endl;
  }
  else
  {
    cout << "Oops!" << endl;
  }
  //arm.Couple("robotiq_laser");
  //arm.Couple("flange");

  robotPose poseMe, curPose;
  robotAxes curAxes, tarAxes; 
  bool poseDefined = false;
  bool tool = false;

  double theta = 0.0f;
  matrix pin(3,1), pout(3,1);

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

      pin.at(0,0) = poseMe.x;
      pin.at(1,0) = poseMe.y;
      pin.at(2,0) = poseMe.z;

      switch (i)
      {
      case 1:
        
        poseMe.z += 20.0;
        /*
        poseMe.xrot = 180.0f;
        poseMe.yrot = 0.0f;
        poseMe.zrot = 0.0f;
        */
        /*
        poseMe.x = 846.722;
        poseMe.y = 909.435;
        poseMe.z = -574.25;
        poseMe.xrot = -180.0f;
        poseMe.yrot = -90.0f;
        poseMe.zrot = 90.0f;
        */
        break;
      case 2:
        
        poseMe.z += -20.0;
        /*
        poseMe.x = 846.722;
        poseMe.y = 909.435;
        poseMe.z = -574.25;
        poseMe.xrot = 90.0f;
        poseMe.yrot = 90.0f;
        poseMe.zrot = 180.0f;
        */
        break;
      default:
        break;
      }

      pin.at(0,0) = poseMe.x;
      pin.at(1,0) = poseMe.y;
      pin.at(2,0) = poseMe.z;

      if (arm.MoveStraightTo(poseMe) == CANON_SUCCESS)
      {
      }
//        curPose = poseMe;
        arm.GetRobotPose(&curPose);
        cout << "curPose=" << curPose.z << endl;
 //     }
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
#elif defined URFORCEMEASTEST
CrpiRobot<CrpiAbb> arm("abb_irb14000_left.xml");
//CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");

cout << "initializing" << endl;
arm.SetAngleUnits("degree");
arm.SetLengthUnits("mm");

cout << "coupling tool" << endl;
arm.Couple("Yumi_Parallel");
//arm.Couple("flange");

  robotPose poseMe, curPose;
  robotAxes curAxes, tarAxes, origAxes;
  bool poseDefined = false;
  bool tool = false;

  double theta = 0.0f;

  ofstream outfile("urforcemeastest.csv");
  outfile << "x, y, z, xrot, yrot, zrot, j1, j2, j3, j4, j5, j6, xf, yf, zf, xt, yt, zt, t1, t2, t3, t4, t5, t6" << endl;
  int xmax = 5000;
  arm.GetRobotAxes(&origAxes);
  for (int x = 0; x < xmax; ++x)
  {
//    tarAxes = origAxes;
//    tarAxes.axis.at(0) += ((double)x / 100.0f);
//    arm.MoveToAxisTarget(tarAxes);
    cout << "Reading sample #" << (x + 1) << " of " << xmax << endl;
    arm.GetRobotPose(&curPose);
    arm.GetRobotForces(&poseMe);
    arm.GetRobotAxes(&curAxes);
    arm.GetRobotTorques(&tarAxes);
    outfile << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot
            << ", " << curPose.zrot << ", " << curAxes.axis.at(0) << ", " << curAxes.axis.at(1) << ", "
            << curAxes.axis.at(2) << ", " << curAxes.axis.at(3) << ", " << curAxes.axis.at(4) << ", " << curAxes.axis.at(5)
            << ", " << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " << poseMe.yrot 
            << ", " << poseMe.zrot << ", " << tarAxes.axis.at(0) << ", " << tarAxes.axis.at(1) << ", " << tarAxes.axis.at(2)
            << ", " << tarAxes.axis.at(3) << ", " << tarAxes.axis.at(4) << ", " << tarAxes.axis.at(5) << endl;
    Sleep(200);
  }

  outfile.close();

#elif defined XMLDEMO
  //CrpiRobot<robType> arm("universal_ur10_right.xml");
  CrpiRobot<robType> arm("kuka_lwr.xml");
  char curtool[32];
  char buffer[1024];
  bool toolopen = true;

  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->SetAngleUnits("degree");
  pm.robArm->SetLengthUnits("mm");

  //strcpy(curtool, "gripper_gear");
  //strcpy(curtool, "gripper_parallel");
  //pm.robArm->Couple(curtool);

  robotPose poseMe, curPose;
  robotIO io;
  bool poseDefined = false;
  bool tool = false;
  int param;
  int counter; 

  std::string str1, str2, str3, str4;

  str1 = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLCommandInstance xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"../xmlSchemas/CRCLCommandInstance.xsd\"><CRCLCommand xsi:type=\"MoveToType\"><CommandID>2</CommandID><MoveStraight>false</MoveStraight><Point><X>2.5</X> <Y>1</Y> <Z>1</Z></Point><XAxis><I>1</I><J>0</J><K>0</K></XAxis><ZAxis><I>0</I><J>0</J><K>-1</K></ZAxis></CRCLCommand></CRCLCommandInstance>";
  str2 = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLCommandInstance xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"><CRCLCommand xsi:type\"SetEndEffectorType\"><CommandID>123</CommandID><NumPositions>0.5</NumPositions></CRCLCommand></CRCLCommandInstance>"; 
  str3 = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLStatus xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"../xmlSchemas/CRCLStatus.xsd\"><CommandStatus><CommandID>1</CommandID><StatusID>1</StatusID><CommandState>Working</CommandState></CommandStatus><Pose><Point><X>1.5</X> <Y>1</Y> <Z>1</Z></Point><XAxis><I>1</I> <J>0</J> <K>0</K></XAxis><ZAxis><I>0</I> <J>0</J> <K>-1</K></ZAxis></Pose></CRCLStatus>";

  timer.waitUntil(2000);

  curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
  ulapi_mutex_take(pm.grabmutex);
  pm.robArm->GetRobotPose(&curPose);
  ulapi_mutex_give(pm.grabmutex);

  cout << "(" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;

  matrix m(3, 3);

//  curPose.xrot = 90.0f;
//  curPose.yrot = 0.0f;
//  curPose.zrot = 0.0f;

  poseMe = curPose;

  //poseMe.xrot *= (3.141592654f / 180.0f);
  //poseMe.yrot *= (3.141592654f / 180.0f);
  //poseMe.zrot *= (3.141592654f / 180.0f);
  pm.robArm->RPYMatrixConvert(poseMe, m);

  cout << "Test orientation conversion: " << endl;
  cout << "Robot representation (deg): (" << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;
  cout << "Robot representation (rad): (" << poseMe.xrot << ", " << poseMe.yrot << ", " << poseMe.zrot << ")" << endl;
  cout << "CRCL representation (rad):  |" << m.at(0, 0) << " " << m.at(0, 1) << " " << m.at(0,2) << "|" << endl;
  cout << "                            |" << m.at(1, 0) << " " << m.at(1, 1) << " " << m.at(1,2) << "|" << endl;
  cout << "                            |" << m.at(2, 0) << " " << m.at(2, 1) << " " << m.at(2,2) << "|" << endl;

  pm.robArm->matrixRPYconvert(m, poseMe);

  poseMe.xrot *= (180.0f / 3.141592654f);
  poseMe.yrot *= (180.0f / 3.141592654f);
  poseMe.zrot *= (180.0f / 3.141592654f);

  cout << "Back to robot representation (deg): (original, reverted)" << endl;
  cout << "xrot(" << curPose.xrot << ", " << poseMe.xrot << "), yrot(" << curPose.yrot << ", " << poseMe.yrot << "), zrot(" << curPose.zrot << ", " << poseMe.zrot << ")" << endl;

  poseMe.z += 20.0f;

  sprintf (buffer, "<?xml version=\"1.0\" encoding=\"UTF-8\"?><CRCLCommandInstance xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"../xmlSchemas/CRCLCommandInstance.xsd\"><CRCLCommand xsi:type=\"MoveToType\"><CommandID>2</CommandID><MoveStraight>false</MoveStraight><Point><X>%f</X><Y>%f</Y><Z>%f</Z></Point><XAxis><I>%f</I><J>%f</J><K>%f</K></XAxis><ZAxis><I>%f</I><J>%f</J><K>%f</K></ZAxis></CRCLCommand></CRCLCommandInstance>",
           poseMe.x,
           poseMe.y,
           poseMe.z,
           m.at(0, 0),
           m.at(1, 0),
           m.at(2, 0),
           m.at(0, 2),
           m.at(1, 2),
           m.at(2, 2));

  cout << buffer << endl;

  str4 = buffer;

  pm.robArm->XmlHandler(str4);

  timer.waitUntil(2000);
  pm.robArm->MoveTo(curPose);

  pm.robArm->XmlResponse(buffer);
  cout << buffer << endl;
#elif defined REGISTRATIONTEST
  CrpiRobot<CrpiUniversal> arm1("universal_ur10_right.xml");
  CrpiRobot<CrpiKukaLWR> arm2("kuka_lwr.xml");
  char curtool[32];
  bool toolopen = true;
  char in;

  arm1.SetAngleUnits("degree");
  arm1.SetLengthUnits("mm");
  arm2.SetAngleUnits("degree");
  arm2.SetLengthUnits("mm");

  cout << "coupling tools..." << endl;
  //strcpy(curtool, "gripper_gear");
  strcpy(curtool, "gripper_parallel");
  arm1.Couple(curtool);
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
  arm1.GetRobotPose(&curPose1);
  arm2.GetRobotPose(&curPose2);

  arm1.FromWorld(&poseMe, &tarPose1);
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
  arm1.MoveStraightTo(tarPose1);
  timer.waitUntil(5000);
  arm1.MoveStraightTo(curPose1);
#elif defined MATHTEST
  matrix m1(3, 3), m2(3, 3), mout(3, 3);
  m1.at(0, 0) = 1;
  m1.at(1, 1) = 1;
  m1.at(2, 2) = 1;
  m2.at(0, 1) = 2;
  m2.at(1, 2) = 2;
  m2.at(2, 0) = 2;

  mout = m1 + m2;

  cout << endl;
  mout.print();


  cout << endl << endl;
  vector<double> euler;
  vector<double> axang;
  euler.push_back(-135.0 * (3.141592654f / 180.0f));
  euler.push_back(0.0f);
  euler.push_back(0.0f);
  m1.rotEulerMatrixConvert(euler);
  m1.rotMatrixAxisAngleConvert(axang);
  cout << "euler:      " << euler.at(0) << ", " << euler.at(1) << ", " << euler.at(2) << endl;
  cout << "matrix:     " << endl;
  m1.print();
  cout << "axis angle: " << axang.at(0) << ", " << axang.at(1) << ", " << axang.at(2) << endl;


#elif defined SENSORTEST

/*
  //! Myo Test

  MyoObj cb;
  vector<MyoSubject> subjects;
  int index;

  while (true)
  {
    cb.getData(subjects);
    index = subjects.size();
    for (int i = 0; i < index; ++i)
    {
      subjects.at(i).print();
    }
    Sleep(200);
  }
*/  

  //! MoCap Test
  //OptiTrack ottest("169.254.152.10");
  Vicon vtest ("129.6.35.213");
  vector<MoCapSubject> vec;
  vector<MoCapSubject>::iterator iter;

  ofstream outwrite("log.csv");  
  vector<point> mvec;
  vector<point>::iterator miter;

  double curtime;

  outwrite << "timestamp,name,X,Y,Z,XRot,YRot,Zrot" << endl;
  timer.start();
  while (true)
  {
    curtime = timer.elapsedTime();
    vtest.GetCurrentSubjects(vec);
    //ottest.GetCurrentSubjects(vec);
    vtest.GetUnlabeledMarkers(mvec);
    //ottest.GetUnlabeledMarkers(mvec);

    i = 0;
    if (vec.size() > 0)
    {
      for (iter = vec.begin(); iter != vec.end(); ++iter, ++i)
      {
        if (iter->valid)
        {
          cout << i << ": " << iter->name << " (" << iter->pose.x << ", " << iter->pose.y << ", " << iter->pose.z << ", " << iter->pose.xr << ", " << iter->pose.yr << ", " << iter->pose.zr << ")" << endl;
          outwrite << curtime << ", " << iter->name.c_str() << ", " << iter->pose.x << ", " << iter->pose.y << ", " 
                   << iter->pose.z << ", " << iter->pose.xr << ", " << iter->pose.yr << ", " << iter->pose.zr << endl;
        }
      }
    }

    /*
    i = 0;
    if (mvec.size() > 0)
    {
      outwrite << curtime;
      for (miter = mvec.begin(); miter != mvec.end(); ++miter, ++i)
      {
        cout << i << ": (" << miter->x << ", " << miter->y << ", " << miter->z << ")" << endl;
        outwrite << ", Name, Marker_" << i << ", X, " << miter->x << ", Y, " << miter->y << ", Z, " << miter->z;
      }
      outwrite << endl;
    }
    */
    // Sleep(200);
  }

#elif defined FT_TEST  
  //ATI_Wired ft_sensor;
  FT _COP COP_sensor('z',0.008);
  vector<double> COP1(3);
  vector<double> COP2(3);
  vector<double> COP3(3);

  /*
  for(int i=0;i<200;++i)
  {
    //ft_sensor.Get_FT();
    COP_sensor.Calc_COP();
    std::cout << COP_sensor.COP[0] << " " << COP_sensor.COP[1] << " " << COP_sensor.COP[2] << std::endl;
    cin.get();
  }
  */
  
  
  COP_sensor.Calc_COP();
  COP1 = COP_sensor.COP;
  std::cout << "Move to another location and press any key to sample another COP" << std::endl;
  cin.get();
  COP_sensor.Calc_COP();
  COP2 = COP_sensor.COP;
  std::cout << "Move to another location and press any key to sample another COP" << std::endl;
  cin.get();
  COP_sensor.Calc_COP();
  COP3 = COP_sensor.COP;

  std::cout << "COP1: " << COP1[0] << " " << COP1[1] << " " << COP1[2] << std::endl;
  std::cout << "COP2: " << COP2[0] << " " << COP2[1] << " " << COP2[2] << std::endl;
  std::cout << "COP3: " << COP3[0] << " " << COP3[1] << " " << COP3[2] << std::endl;
  
  std::cout << "press any key to quit" << std::endl;
  
  cin.get();
#elif defined DATADEMO

  string str = "gravcomp on";

  cout << str.c_str() << " == ";
  if (str == "gravcomp")
  {
    cout << "gravcomp";
  }
  else if (str == "gravcomp on")
  {
    cout << "gravcomp on";
  }
  else
  {
    cout << "nothing";
  }
  cout << endl;
  return;

  //! Open COM port
  serialStruct serialData;
  int numlines = 5;
  int num = (5 + (6 * 11)) * numlines;
  char *inbuffer = new char[num];
  char *pch;
//  ofstream datalogger("robotlog.csv");
  clock_t t1;

  bool test;

  test = serialData.setBaud(57600);
  test &= serialData.setChannel(7);
  Network::serial *serial_ = new Network::serial();
  if (serial_->attach(serialData))
  {
    //! Connected
    cout << "connected to robot" << endl;
  }
  else
  {
    //! Failed to connect
    cout << "could not connect to robot" << endl;
  }

  while (true)
  {
    //! Read data
    if (serial_->getData(inbuffer, serialData, num))
    {
      cout << inbuffer << endl;
     } // if (serial_->getData (inbuffer, serialData, 285))
    else
    {
  #ifdef NOISY
      cout << "no data..." << endl;
  #endif
    }
  }
#elif defined  REGISTERTEST
  CrpiRobot<CrpiKukaLWR> arm("kuka_lwr.xml");
  robotPose ps_in, ps_out;

  vector<point> world;
  vector<point> rob1;
  vector<point> rob2;

  vector<point>::const_iterator iter;

  int k = 1;

  matrix w2r1_1(4, 4), w2r2_1(4, 4);
  matrix rot(3, 3);
  matrix posin(4, 4), posout(4, 4);

  vector<double> outs;

  point temp;
  //! Set world coordinates
  //! World Origin:  0, 0, 0
  point w1(575.0f, 0.0f, 0.0f),
        w2(575.0f, 10.0f, 0.0f),
        w3(585.0f, 0.0f, 0.0f),
        w4(585.0f, 10.0f, 0.0f);

  point r1(-315.0f, -890.0f, 0.0f),
        r2(-315.0f, -880.0f, 0.0f),
        r3(-305.0f, -890.0f, 0.0f),
        r4(-305.0f, -880.0f, 0.0f);

  point s1(-30.0f, 790.0f, -10.0f),
        s2(-20.0f, 790.0f, -10.0f),
        s3(-30.0f, 780.0f, -10.0f),
        s4(-20.0f, 780.0f, -10.0f);

  world.push_back(w1);
  world.push_back(w2);
  world.push_back(w3);
  world.push_back(w4);
  
  rob1.push_back(r1);
  rob1.push_back(r2);
  rob1.push_back(r3);
  rob1.push_back(r4);
  
  rob2.push_back(s1);
  rob2.push_back(s2);
  rob2.push_back(s3);
  rob2.push_back(s4);
  
  bool state = reg2target(rob1, world, w2r1_1);

  posin.at(0, 0) = posin.at(1, 1) = posin.at(2, 2) = posin.at(3, 3) = 1.0f;
  posin.at(0, 3) = world.at(k).x;
  posin.at(1, 3) = world.at(k).y;
  posin.at(2, 3) = world.at(k).z;

  matrix posin_inv(4, 4);
  posin_inv = w2r1_1.inv();

  ps_in.x = world.at(k).x;
  ps_in.y = world.at(k).y;
  ps_in.z = world.at(k).z;
  ps_in.xrot = ps_in.yrot = ps_in.zrot = 0.0f;
  arm.FromWorld(&ps_in, &ps_out);

  posout = posin_inv * posin;
  cout << "looking at target " << k << " for robot 1" << endl;
  cout << "origin:  [[" << world.at(k).x << ", " << world.at(k).y  << ", " << world.at(k).z << "]]" << endl;
  cout << "output:  [[" << posout.at(0, 3) << ", " << posout.at(1, 3) << ", " << posout.at(2, 3) << "]]" << endl;
  cout << "target:  [[" << rob1.at(k).x << ", " << rob1.at(k).y << ", " << rob1.at(k).z << "]]" << endl;
  cout << "target2: [[" << ps_out.x << ", " << ps_out.y << ", " << ps_out.z << "]]" << endl;

  cout << endl << endl << endl;

  state = reg2target(rob2, world, w2r2_1);
  posin.at(0, 0) = posin.at(1, 1) = posin.at(2, 2) = posin.at(3, 3) = 1.0f;
  posin.at(0, 3) = world.at(k).x;
  posin.at(1, 3) = world.at(k).y;
  posin.at(2, 3) = world.at(k).z;

  posin_inv = w2r2_1.inv();
  posout = posin_inv * posin;
  cout << "looking at target " << k << " for robot 2" << endl;
  cout << "origin:  [[" << world.at(k).x << ", " << world.at(k).y  << ", " << world.at(k).z << "]]" << endl;
  cout << "output:  [[" << posout.at(0, 3) << ", " << posout.at(1, 3) << ", " << posout.at(2, 3) << "]]" << endl;
  cout << "target:  [[" << rob2.at(k).x << ", " << rob2.at(k).y << ", " << rob2.at(k).z << "]]" << endl;

#elif defined VIVE_INPUT
string hmdpath = "C:\\Users\\mlz\\Documents\\openvr-master\\samples\\hellovr_opengl\\rightc.txt";
string leftcpath = "C:\\Users\\mlz\\Documents\\openvr-master\\samples\\hellovr_opengl\\leftc.txt";

std::ifstream indata, indatal;
indatal.open(leftcpath);
indata.open(hmdpath);
//getting last line of data from hmd
if (indata.is_open() && indatal.is_open())
{
	std::cout << "accessedfile" << endl;
	//Getting the last line of data based on the last endline

	//messy and should be made into a function at a later point in time
	indata.seekg(-1, ios_base::end);
	bool looper = true;
	while (looper)
	{
		char ch;
		indata.get(ch);
		//case for only one line in the file
		if ((int)indata.tellg() <= 1)
		{
			indata.seekg(0);
			looper = false;
		}
		//newline
		else if (ch == '\n')
		{
			looper = false;
		}
		// Move to the front of that data, then to the front of the data before it
		else
		{
			indata.seekg(-2, ios_base::cur);
		}
	}
	string lastLine = "";
	std::getline(indata, lastLine);                      // Read the current line

	//parsing the input
	vector<string> hmd;
	vector<float> hmdf;
	string delimiter = ",";
	float floken;
	size_t pos = 0;
	string token;
	while ((pos = lastLine.find(delimiter)) != string::npos) {
		token = lastLine.substr(0, pos);
		remove(token.begin(), token.end(), ' ');
		floken = stof(token);
		hmdf.push_back(floken);
		lastLine.erase(0, pos + delimiter.length());
	}
	remove(lastLine.begin(), lastLine.end(), ' ');
	floken = stof(lastLine);
	hmdf.push_back(floken);
	double goalrx, goalry, goalrz;

	if (hmdf.size() >= 3)
	{
		/*
		* Also:
		* Open VR Convention (same as OpenGL)
		* right-handed system
		* +y is up -> 
		* +x is to the right
		* -z is going away from you
		*/

		std::cout << "right controller: -y: " << hmdf[0] << ",z: " << hmdf[1] << ",-x: " << hmdf[2] << endl;
		goalrx = (hmdf[2] * 1800)-60;
		goalry = hmdf[0] * -200;
		goalrz = hmdf[1] - 1;
		goalrz = (goalrz * 100)*7.235;
		goalrz = goalrz - 340;
		std::cout << "right controller reorder transform:  x:" << goalrx << ",y: " << goalry << ",z: " << goalrz << endl;

	}
	indata.close();
	indatal.seekg(-1, ios_base::end);
	looper = true;
	while (looper)
	{
		char ch;
		indatal.get(ch);
		//case for only one line in the file
		if ((int)indatal.tellg() <= 1)
		{
			indatal.seekg(0);
			looper = false;
		}
		//newline
		else if (ch == '\n')
		{
			cout << "reached" << endl;
			looper = false;
		}
		// Move to the front of that data, then to the front of the data before it
		else
		{
			indatal.seekg(-2, ios_base::cur);
		}
	}
	string lastLine1 = "";
	std::getline(indatal, lastLine1);                      // Read the current line
	cout << lastLine1 << endl;
														   //parsing the input
	vector<string> left;
	vector<float> leftfl;
	//string delimiter = ",";
	floken=0;
	//pos = 0;
	while ((pos = lastLine1.find(',')) != string::npos) {
		cout << "here" << endl;
		token = lastLine1.substr(0, pos);
		remove(token.begin(), token.end(), ' ');
		floken = stof(token);
		leftfl.push_back(floken);
		lastLine1.erase(0, pos + delimiter.length());
	}
	remove(lastLine1.begin(), lastLine1.end(), ' ');
	floken = stof(lastLine1);
	leftfl.push_back(floken);
	double goalx, goaly, goalz;
	if (leftfl.size() >= 3)
	{
		/*
		* Also:
		* Open VR Convention (same as OpenGL)
		* right-handed system
		* +y is up ->
		* +x is to the right
		* -z is going away from you
		*/
		std::cout << "left controller: : " << leftfl[0] << ",z: " << leftfl[1] << ",-x: " << leftfl[2] << endl;
		//reordering the coordinates
		goalx = leftfl[2] * 1800;
		goaly = leftfl[0]*-200;
		goalz = leftfl[1]-1;
		goalz = (goalz * 100)*7.235;
		goalz = goalz - 340;
		std::cout << "left controller reorder transform:  x:" << goalx << ",y: " << goaly << ",z: " << goalz << endl;

	}
	indatal.close();

	std::cout << "here" << endl;
	//initialize arm variables
	CrpiRobot<CrpiAbb> armL("abb_irb14000_left.xml");
	std::cout << "left arm connected" << endl;

	std::cout << "initializing" << endl;
	armL.SetAngleUnits("degree");
	armL.SetLengthUnits("mm");

	CrpiRobot<CrpiAbb> armR("abb_irb14000_right.xml");
	std::cout << "right arm connected" << endl;

	std::cout << "initializing" << endl;
	armR.SetAngleUnits("degree");
	armR.SetLengthUnits("mm");

	std::cout << "coupling tools" << endl;
	armL.Couple("Yumi_Parallel");
	armR.Couple("Yumi_Parallel");
	std::cout << "coupling tools done" << endl;
	//cin.get();

	robotPose poseMe, curPose, curPose_r, poseR;
	robotAxes curAxes, curAxesR, tarAxes, tarAxesR;
	bool poseDefined = false;
	bool tool = false;
	int param;

	double theta = 0.0f;
	matrix pin(3, 1), pout(3, 1);



	//initializing pose variables
	curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
	poseMe.x = poseMe.y = poseMe.z = poseMe.xrot = poseMe.yrot = poseMe.zrot = 0.0f;

	curPose_r.x = curPose_r.y = curPose_r.z = curPose_r.xrot = curPose_r.yrot = curPose_r.zrot = 0.0f;
	poseR.x = poseR.y = poseR.z = poseR.xrot = poseR.yrot = poseR.zrot = 0.0f;

	//matrixes for quaternion conversion
	Math::matrix rightRot(3, 3);
	Math::matrix leftRot(3, 3);
	vector<double> rightq;
	vector<double> leftq;

	//Moving the left arm
	if (armL.GetRobotPose(&curPose) == CANON_SUCCESS)
	{
		std::cout << "(" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;
		armL.GetRobotPose(&poseMe);

		armL.GetRobotAxes(&curAxes);
		
		//- goes inwards, + goes outwards
		//poseMe.x = curPose.x  - 39.098;
		//- goes right, + goes left
		//poseMe.y = curPose.y + 10.0;
		//- goes down, + goes up
		//poseMe.z = curPose.z + 10.0;
		
		poseMe.x = goalx;
		poseMe.y = goaly;
		poseMe.z = goalz;

		if (armL.MoveStraightTo(poseMe) != CANON_SUCCESS)
		{
			std::cout << "motion error" << endl;
		}
	//if (Math::rotQuaternionMatrixConvert()) {
			std::cout << "New (" << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " << poseMe.yrot << ", " << poseMe.zrot << ")" << endl;
	//}
		
	}
	//moving the right arm
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		std::cout << "(" << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << ")" << endl;
		armR.GetRobotPose(&poseR);

		armR.GetRobotAxes(&curAxesR);
		//- goes inwards, + goes outwards
		//poseR.x = curPose_r.x + 15.0;
		//- goes right, + goes left
		//poseR.y = curPose_r.y - 20.0;
		//- goes down, + goes up
		//poseR.z = curPose_r.z - 50.0;

		//poseR.x = goalrx;
		//poseR.y = goalry;
		//poseR.z = goalrz;
		if (armR.MoveStraightTo(poseR) != CANON_SUCCESS)
		{
			std::cout << "motion error" << endl;
		}
		timer.waitUntil(500);
		//armR.MoveStraightTo(curPose_r);
		
		timer.waitUntil(500);

		//armL.MoveStraightTo(curPose);
		Math::matrix rightRot(3,3);
		Math::matrix leftRot(3, 3);
		//if (Math::rotQuaternionMatrixConvert(poseR)) 
		//{
			std::cout << "New1 (" << poseR.x << ", " << poseR.y << ", " << poseR.z << ", " << poseR.xrot << ", " << poseR.yrot << ", " << poseR.zrot << ")" << endl;
		//}
	}


}
else
{
	std::cout << "failure" << endl;
}

#endif

  std::cout << "All done" << endl;

}

