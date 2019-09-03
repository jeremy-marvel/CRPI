///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Multi-Robot Motion Demonstration
//  Workfile:        multi_robot.cpp
//  Revision:        23 September, 2016
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
#include "crpi_abb.h"
#include "ulapi.h"
#include "NumericalMath.h" 

#pragma warning (disable: 4996)

//#define test1
//#define test2
#define test3

using namespace crpi_robot;
using namespace std;

void main ()
{
  int i = 0;
  crpi_timer timer;

  CrpiRobot<CrpiUniversal> ur5("universal_ur5_table.xml");
  CrpiRobot<CrpiAbb> abb_l("abb_irb14000_left.xml");
  CrpiRobot<CrpiAbb> abb_r("abb_irb14000_right.xml");


  abb_l.SetAngleUnits("degree");
  abb_l.SetLengthUnits("mm");
  abb_l.Couple("Yumi_Parallel");
  abb_l.SetRelativeSpeed(0.5);

  abb_r.SetLengthUnits("mm");
  abb_r.SetAngleUnits("degree");
  abb_r.Couple("Yumi_Parallel");
  abb_r.SetRelativeSpeed(0.5);

  ur5.SetAngleUnits("degree");
  ur5.SetLengthUnits("mm");
  ur5.Couple("gripper_parallel");




  robotPose ur5_hover1, ur5_acquire, ur5_hand, ur5_hover2, ur5_grasp;
  robotAxes ur5_stow, ur5_mid;

  robotPose abb_l_hover0, abb_l_acquire, abb_l_hover1, abb_l_grasp1, abb_l_hand;
  robotAxes abb_l_stow;

  robotPose abb_r_hover1, abb_r_grasp, abb_r_hand;
  robotAxes abb_r_stow;

  //! UR5
  ur5_hover1.x = 640.0f;
  ur5_hover1.y = 460.0f;
  ur5_hover1.z = 147.0f;
  ur5_hover1.xrot = -180.0f;
  ur5_hover1.yrot = 0.0f;
  ur5_hover1.zrot = -135.0f;

  ur5_acquire.x = 640.0f;
  ur5_acquire.y = 460.0f;
  ur5_acquire.z = 47.0f;
  ur5_acquire.xrot = -180.0f;
  ur5_acquire.yrot = 0.0f;
  ur5_acquire.zrot = -135.0f;

  ur5_hand.x = 270.0f;
  ur5_hand.y = 700.0f;
  ur5_hand.z = 360.0f;
  ur5_hand.xrot = -135.0f;
  ur5_hand.yrot = -90.0f;
  ur5_hand.zrot = 90.0f;

  ur5_hover2.x = -71.0f;
  ur5_hover2.y = 454.0f;
  ur5_hover2.z = 590.0f;
  ur5_hover2.xrot = -180.0f;
  ur5_hover2.yrot = 0.0f;
  ur5_hover2.zrot = -135.0f;

  ur5_grasp.x = -71.0f;
  ur5_grasp.y = 454.0f;
  ur5_grasp.z = 490.0f;
  ur5_grasp.xrot = -180.0f;
  ur5_grasp.yrot = 0.0f;
  ur5_grasp.zrot = -135.0f;

  ur5_stow.axis.at(0) = 162.0f;
  ur5_stow.axis.at(1) = -71.0f;
  ur5_stow.axis.at(2) = 58.0f;
  ur5_stow.axis.at(3) = -76.0f;
  ur5_stow.axis.at(4) = -90.0f;
  ur5_stow.axis.at(5) = 113.5f;
  ur5_stow.axis.at(6) = 0.0f;

  ur5_mid.axis.at(0) = 213.0f;
  ur5_mid.axis.at(1) = -116.0f;
  ur5_mid.axis.at(2) = 82.0f;
  ur5_mid.axis.at(3) = -54.0f;
  ur5_mid.axis.at(4) = -90.0f;
  ur5_mid.axis.at(5) = 86.0f;
  ur5_mid.axis.at(6) = 0.0f;

  //! ABB_L
  abb_l_hover0.x = 358.72;
  abb_l_hover0.y = 178.1;
  abb_l_hover0.z = 185.0;
  abb_l_hover0.xrot = 180.0f;
  abb_l_hover0.yrot = 0.0f;
  abb_l_hover0.zrot = 83.0f;

  abb_l_acquire.x = 358.72f;
  abb_l_acquire.y = 178.1f;
  abb_l_acquire.z = 42.0f;
  abb_l_acquire.xrot = 180.0f;
  abb_l_acquire.yrot = 0.0f;
  abb_l_acquire.zrot = 83.0f;

  abb_l_hover1.x = 557.0f;
  abb_l_hover1.y = 193.0f;
  abb_l_hover1.z = 369.0f;
  abb_l_hover1.xrot = -90.0f;
  abb_l_hover1.yrot = 0.0f;
  abb_l_hover1.zrot = -90.0f;

  abb_l_grasp1.x = 657.0f;
  abb_l_grasp1.y = 193.0f;
  abb_l_grasp1.z = 369.0f;
  abb_l_grasp1.xrot = -90.0f;
  abb_l_grasp1.yrot = 0.0f;
  abb_l_grasp1.zrot = -90.0f;

  abb_l_hand.x = 388.0f;
  abb_l_hand.y = 121.0f;
  abb_l_hand.z = 173.0f;
  abb_l_hand.xrot = -90.0f;
  abb_l_hand.yrot = 0.0f;
  abb_l_hand.zrot = 180.0f;

  abb_l_stow.axis.at(0) = -35.0f;
  abb_l_stow.axis.at(1) = -126.0f;
  abb_l_stow.axis.at(2) = 88.0f;
  abb_l_stow.axis.at(3) = 32.0f;
  abb_l_stow.axis.at(4) = 154.0f;
  abb_l_stow.axis.at(5) = -54.0f;
  abb_l_stow.axis.at(6) = 97.0f;

  //! ABB_R
  abb_r_hover1.x = 388.0f;
  abb_r_hover1.y = 0.0f;
  abb_r_hover1.z = 173.0f;
  abb_r_hover1.xrot = 90.0f;
  abb_r_hover1.yrot = 0.0f;
  abb_r_hover1.zrot = 180.0f;

  abb_r_grasp.x = 388.0f;
  abb_r_grasp.y = 100.0f;
  abb_r_grasp.z = 173.0f;
  abb_r_grasp.xrot = 90.0f;
  abb_r_grasp.yrot = 0.0f;
  abb_r_grasp.zrot = 180.0f;

  abb_r_hand.x = 612.0f;
  abb_r_hand.y = -230.0f;
  abb_r_hand.z = 467.0f;
  abb_r_hand.xrot = 0.0f;
  abb_r_hand.yrot = 0.0f;
  abb_r_hand.zrot = 180.0f;

  abb_r_stow.axis.at(0) = 30.0f;
  abb_r_stow.axis.at(1) = -109.0f;
  abb_r_stow.axis.at(2) = -102.0f;
  abb_r_stow.axis.at(3) = 38.0f;
  abb_r_stow.axis.at(4) = 8.0f;
  abb_r_stow.axis.at(5) = 74.0f;
  abb_r_stow.axis.at(6) = -90.0f;

  /*
  ur5.SetRelativeSpeed(0.5f);
  abb_r.SetRelativeSpeed(0.5f);
  abb_l.SetRelativeSpeed(0.5f);
  */

  //! Move to stow positions
#ifdef test1
  ur5.MoveToAxisTarget(ur5_stow);
  ur5.SetRobotDO(0, false);
#endif
  abb_l.MoveToAxisTarget(abb_l_stow);
  abb_l.SetTool(0.0f);
  abb_r.MoveToAxisTarget(abb_r_stow);
  abb_r.SetTool(0.0f);
  cout << "At stow.  Enter 1:  ";
  cin >> i;

#ifdef test3
  abb_l.SetTool(1.0f);
  cout << "Grasped object.  Enter 1:  ";
  cin >> i;
#endif

  //! Pick up the box
#ifdef test1
  cout << "Pick up box";
  ur5.MoveTo(ur5_hover1);
  ur5.MoveStraightTo(ur5_acquire);
  ur5.SetRobotDO(0, true);
  Sleep(100);
  ur5.MoveStraightTo(ur5_hover1);
#elif defined test2
  cout << "Pick up box";
  abb_l.MoveTo(abb_l_hover0);
  Sleep(1000);
  abb_l.MoveStraightTo(abb_l_acquire);
  Sleep(1000);
  abb_l.SetTool(1.0f);
  Sleep(500);
  abb_l.MoveStraightTo(abb_l_hover0);
#endif

#ifndef test3
  for (i = 0; i < 8; ++i)
#else
  while(true)
#endif
  {
#ifdef test1
    ur5.MoveTo(ur5_hand);
    //! Transfer from UR5 to YuMi
#endif
    abb_l.MoveTo(abb_l_hover1);
    Sleep(1000);
    abb_l.MoveStraightTo(abb_l_grasp1);
    Sleep(750);
#ifdef test1
    abb_l.SetTool(1.0f);
#endif
    Sleep(500);
#ifdef test1
    ur5.SetRobotDO(0, false);
#endif

    //! Move out of the way
    abb_l.MoveStraightTo(abb_l_hover1);
    Sleep(1000);

    //! Switch hands of the YuMi
    abb_l.MoveTo(abb_l_hand);
#ifdef test1
    //! Move UR to mid before moving to next hover
    ur5.MoveToAxisTarget(ur5_mid);
#endif
    abb_r.MoveTo(abb_r_hover1);
    Sleep(1000);
    abb_r.MoveStraightTo(abb_r_grasp);
    Sleep(1000);
    abb_r.SetTool(1.0f);
    Sleep(500);
    abb_l.SetTool(0.0f);
    Sleep(1000);
    abb_r.MoveStraightTo(abb_r_hover1);
    //abb_l.MoveToAxisTarget(abb_l_stow);

    //! Transfer back to the UR5
    abb_r.MoveTo(abb_r_hand);
#ifndef test1
    Sleep(1000);
#endif
#ifdef test1
    ur5.MoveTo(ur5_hover2);
    ur5.MoveStraightTo(ur5_grasp);
    ur5.SetRobotDO(0, true);
    Sleep(500);
    abb_r.SetTool(0.0f);
#endif

    //! Move away
#ifdef test1
    ur5.MoveStraightTo(ur5_hover2);
#endif
    //abb_r.MoveToAxisTarget(abb_r_stow);

#ifdef test3
    abb_l.MoveTo(abb_l_hand);
    Sleep(1000);
    abb_r.MoveTo(abb_r_hover1);
    Sleep(1000);
    abb_r.MoveStraightTo(abb_r_grasp);
    Sleep(1000);
    abb_l.SetTool(1.0f);
    Sleep(500);
    abb_r.SetTool(0.0f);
    Sleep(1000);
    abb_r.MoveStraightTo(abb_r_hover1);
    Sleep(1000);
    //abb_r.MoveToAxisTarget(abb_r_stow);
    //abb_l.MoveToAxisTarget(abb_l_stow);
    Sleep(1000);
#endif

  }

  //! complete cycle
#ifdef test1
  ur5.MoveTo(ur5_hover1);
  ur5.MoveStraightTo(ur5_acquire);
  ur5.SetRobotDO(0, false);
  Sleep(100);
  ur5.MoveStraightTo(ur5_hover1);
  ur5.MoveToAxisTarget(ur5_stow);
#elif defined test2
  abb_l.MoveTo(abb_l_hover0);
  Sleep(1000);
  abb_l.MoveStraightTo(abb_l_acquire);
  Sleep(1000);
  abb_l.SetTool(1.0f);
  Sleep(500);
  abb_l.MoveStraightTo(abb_l_hover0);
  Sleep(500);
  abb_l.MoveToAxisTarget(abb_l_stow);
#endif

}


