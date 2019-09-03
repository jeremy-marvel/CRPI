////////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       System Registration
//  Workfile:        Registration.cpp
//  Revision:        9 October, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "crpi_abb.h"
#include "crpi_kuka_lwr.h"
#include "CoordFrameReg.h"
#include "ulapi.h"
#include "Vicon.h"
#include "ATI_Wired.h"
#include "FT_COP.h"

#pragma warning (disable: 4996)

//#define NOISY
//#define SUPERNOISY
#define ENABLEROBOTS
//#define ENABLELWR
#define ENABLEUR

//#define ENABLEVICON
//#define ENABLEFT

using namespace crpi_robot;
using namespace std;
using namespace Registration;
using namespace Sensor;


void newReg2World()
{
  int x;
  vector<point> abb_left_poses,
                abb_right_poses,
                ur5_poses,
                ur10_left_poses,
                ur10_right_poses,
                kuka_poses,
                world_poses;
  point pointIn;
  robotPose poseIn;

  cout << "Read registration data..." << endl;
  cout << "World coordinates..." << endl;
  ifstream in;
  in.open("world.dat");
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z;
    world_poses.push_back(pointIn);
  }
  in.close();

  cout << "ABB Left coordinates..." << endl;
  in.open("abb_left_reg.dat");
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    abb_left_poses.push_back(pointIn);
  }
  in.close();

  cout << "ABB Right coordinates..." << endl;
  in.open("abb_left_reg.dat");
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    abb_right_poses.push_back(pointIn);
  }
  in.close();

  cout << "UR5 coordinates..." << endl;
  in.open("ur5_reg.dat");
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    ur5_poses.push_back(pointIn);
  }
  in.close();

  cout << "Done." << endl;
  cout << "Registering to world..." << endl;

  vector<matrix> abbl_2_world,
                 abbr_2_world,
                 ur5_2_world,
                 ur10l_2_world,
                 ur10r_2_world,
                 kuka_2_world;
  matrix world_2_robot(4, 4);
  matrix robot_2_world(4, 4);

  unsigned int abbl2worldchoice, abbr2worldchoice, ur52worldchoice, ur10l2worldchoice, ur10r2worldchoice, lwr2worldchoice;
  bool state;
  vector<point> kernels;

  cout << "ABB IRB 14000 Left to World..." << endl;
  state = reg2targetML(abb_left_poses, world_poses, 2, kernels, abbl_2_world);
  cout << (state ? "okay" : "nope") << endl;

  cout << "Option 0: " << endl;
  abbl_2_world.at(0).print();
  cout << "Option 1: " << endl;
  abbl_2_world.at(1).print();
  cout << "Use which option? : ";
  cin >> abbl2worldchoice;

  cout << "ABB IRB 14000 Right to World..." << endl;
  state = reg2targetML(abb_right_poses, world_poses, 2, kernels, abbr_2_world);
  cout << (state ? "okay" : "nope") << endl;

  cout << "Option 0: " << endl;
  abbr_2_world.at(0).print();
  cout << "Option 1: " << endl;
  abbr_2_world.at(1).print();
  cout << "Use which option? : ";
  cin >> abbr2worldchoice;

  cout << "Universal UR5 to World..." << endl;
  state = reg2targetML(ur5_poses, world_poses, 2, kernels, ur5_2_world);
  cout << (state ? "okay" : "nope") << endl;

  cout << "Option 0: " << endl;
  ur5_2_world.at(0).print();
  cout << "Option 1: " << endl;
  ur5_2_world.at(1).print();
  cout << "Use which option? : ";
  cin >> ur52worldchoice;

  cout << "Updating world transformations..." << endl;

  CrpiRobot<CrpiAbb> *abb_r, *abb_l;
  CrpiRobot<CrpiUniversal> *ur5, *ur10l, *ur10r;
  CrpiRobot<CrpiKukaLWR> *kuka;

  cout << "ABB Left" << endl;
  robot_2_world = abbl_2_world.at(abbl2worldchoice);
  abb_l = new CrpiRobot<CrpiAbb>("abb_irb14000_left.xml", true);
  cout << "   Update world..." << endl;
  abb_l->UpdateWorldTransform(robot_2_world);
  cout << "   Save configuration..." << endl;
  abb_l->SaveConfig("abb_irb14000_left.xml");
  Sleep(1000);
  delete abb_l;

  cout << "ABB Right" << endl;
  robot_2_world = abbr_2_world.at(abbr2worldchoice);
  abb_r = new CrpiRobot<CrpiAbb>("abb_irb14000_right.xml", true);
  cout << "   Update world..." << endl;
  abb_r->UpdateWorldTransform(robot_2_world);
  cout << "   Save configuration..." << endl;
  abb_r->SaveConfig("abb_irb14000_right.xml");
  Sleep(1000);
  delete abb_r;

  cout << "UR5" << endl;
  robot_2_world = ur5_2_world.at(ur52worldchoice);
  ur5 = new CrpiRobot<CrpiUniversal>("universal_ur5_table.xml", true);
  cout << "      Update world..." << endl;
  ur5->UpdateWorldTransform(robot_2_world);
  cout << "      Save configuration..." << endl;
  ur5->SaveConfig("universal_ur5_table.xml");
  Sleep(1000);
  delete ur5;
}


void main ()
{
  newReg2World();
  return;


  int i = 0, choice;
  crpi_timer timmy_time;
  char c;
  bool state;
  char buffer[64];

  robotAxes axs, axs2;
  robotPose psr, tarpose;
  vector<robotPose> regposes;
  vector<robotPose>::iterator regiter;
  point lcpt[3], wldpt[3], robpt[3], tarpt[3];
  string color[3];


  cout << "Initializing..." << endl;

  //! ==========================================================================================
  //!                                        ROBOTS
  //! ==========================================================================================

  cout << "  Connecting to robots..." << endl;
  string kukafile = "kuka_lwr.xml";
  string urfile = "universal_ur5_table.xml";

#ifdef ENABLEROBOTS
#ifdef ENABLELWR
  cout << "LWR Enabled" << endl;
  CrpiRobot<CrpiKukaLWR> lwr(kukafile.c_str());
#endif
#ifdef ENABLEUR
  cout << "UR Enabled" << endl;
  CrpiRobot<CrpiUniversal> ur(urfile.c_str());
#endif
  cout << "  Configuring robots..." << endl;

#ifdef ENABLELWR
  cout << "  Configure LWR..." << endl;
  lwr.SetAngleUnits("degree");
  lwr.SetLengthUnits("mm");
  lwr.Couple("flange_ring");
#endif

#ifdef ENABLEUR
  cout << "  Configure UR..." << endl;
  ur.SetAngleUnits("degree");
  ur.SetLengthUnits("mm");
  ur.Couple("flange_ring");
#endif
  cout << "done" << endl;
#else
  cout << "  Robots disabled." << endl;
#endif
  robotPose plwr[3], pur[3], pworld[3];

#ifdef ENABLELWR
  robotPose lwr_hard[4];
  lwr_hard[0].x = -12.5 * 25.0f;
  lwr_hard[0].y = -12.5 * 25.0f;
  lwr_hard[0].z = 40.0f;
  lwr_hard[1].x = -18.5 * 25.0f;
  lwr_hard[1].y = -12.5 * 25.0f;
  lwr_hard[1].z = 40.0f;
  lwr_hard[2].x = -12.5f * 25.0f;
  lwr_hard[2].y = -18.5 * 25.0f;
  lwr_hard[2].z = 40;
  lwr_hard[3].x = -610.0f;
  lwr_hard[3].y = -5.0f;
  lwr_hard[3].z = 100.0f;
#endif

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
  matrix W_T_LC(4,4);
  W_T_LC.at(0,0) = W_T_LC.at(1, 1) = W_T_LC.at(2, 2) = W_T_LC.at(3, 3) = 1;
  W_T_LC.at(0,3) = 0.0;//-2.5;
  W_T_LC.at(1,3) = 0.0;//+1;
  W_T_LC.at(2,3) = 60.5-16.0;
#else
  cout << "  Force/torque sensor disabled" << endl;
#endif
    //! ==========================================================================================
    //!                                      MoCap Sensor
    //! ==========================================================================================
#ifdef ENABLEVICON
  cout << "  Connecting to Vicon motion capture system..." << endl;
  Vicon vicon("129.6.35.25");
  vector<MoCapSubject> subjects, subout;
  vector<MoCapSubject>::iterator viconiter;
#else
  cout << "  Vicon motion capture system disabled." << endl;
#endif

  vector<robotAxes> axsout;
  vector<robotPose> psrout;
  vector<robotPose>::iterator psriter;
  vector<robotAxes>::iterator axsiter;

  vector<point> mvec;
  vector<point>::iterator miter;

  vector<robotPose> validhard;
  vector<robotPose>::iterator validiter;
  ifstream validin("valid.dat");
  validin >> i;
  for (int j = 0; j < i; ++j)
  {
    validin >> psr.x >> psr.y >> psr.z >> psr.xrot >> psr.yrot >> psr.zrot;
    psr.print();
    validhard.push_back(psr);
  }
  validin.close();


  //! ==========================================================================================
  //!                                       Math Stuff
  //! ==========================================================================================

  matrix r1_2_w(4, 4), r2_2_w(4, 4);
  matrix w_2_r1(4, 4), w_2_r2(4, 4);

  matrix LC_pimat, W_pimat;
  vector<double> vecout;

  vector<point> world; 
  vector<point> rob1, rob2;

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

  cout << "  Reading in registration test points from points.dat... ";
  int samples;
  ifstream regpoints("points.dat");
  ofstream regpointsout;
  regpoints >> samples;
#ifdef ENABLELWR
  lwr.GetRobotAxes(&axs);
#endif

  axsout.clear();
  for (i = 0; i < samples; ++i)
  {
    regpoints >> psr.x >> psr.y >> psr.z >> psr.xrot >> psr.yrot >> psr.zrot;
    regposes.push_back(psr);
    for (int j = 0; j < axs.axes; ++j)
    {
      regpoints >> axs.axis.at(j);
    }
    axsout.push_back(axs);
  }
  regpoints.close();
  cout << "[" << regposes.size() << " points read]" << endl;

  ofstream resout;

  cout << "Ready." << endl << endl;

  cout << "Select action:  1) Register ur to World 2) Register LWR to World 3)" << endl
       << "Test UR Reg 4) Test LWR Reg 5) Test Vicon 6) Create test set" << endl
       << "7) Compare robot poses to Vicon" << endl
       << "8) Validate LWR 9) Validat UR" << endl
       << "-1) Quit: ";
  do
  {
    cin >> choice;
  } while (choice < 1 || choice > 10);

  int choice2;
  while (choice != -1)
  {
    cout << "switcheroo..." << endl;
    switch (choice)
    {
    case 1:
#ifdef ENABLEROBOTS
#ifdef ENABLEUR
      cout << "Registering ur using which method?  1) Hand guiding 2) Force plate : ";
      do
      {
        cin >> choice2;
      } while (choice2 < 1 || choice > 2);

      if (choice2 == 1)
      {
        for (i = 0; i < 3; ++i)
        {
          cin.clear();
          cout << "Move ur to defined point " << (i+1) << " (" << color[i] << ") and press ENTER";
          cin >> choice;

          ur.GetRobotPose(&pur[i]);
          robpt[i].x = pur[i].x;
          robpt[i].y = pur[i].y;
          robpt[i].z = pur[i].z;

          wldpt[i].x = pworld[i].x;
          wldpt[i].y = pworld[i].y;
          wldpt[i].z = pworld[i].z;

          robpt[i].print();
          wldpt[i].print();
        } // for (i = 0; i < 3; ++i)
      } // if (choice2 == 1)
      else
      {
#ifdef ENABLEFT
        cout << "Center ur above force registration plate about 1-2 inches from surface (do not align with divot) and press ENTER";
        cin >> i;

        //! Tare the load cell
        COP_sensor.Set_FT_Data();

        for (i = 0; i < 3; ++i)
        {
          cout << "Move to training point " << (i+1) << " and enter 1: ";
          cin >> c;

          cout << "Learning point " << (i+1) << "...";
          
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
          cout << "COP in world: " << endl;
          W_pimat.print();
          cout << endl;
          //W_pimat.print();
          wldpt[i].x = W_pimat.at(0, 0);
          wldpt[i].y = W_pimat.at(1, 0);
          wldpt[i].z = W_pimat.at(2, 0);

          cout << "[done]" << endl;

          cout << "Updating robot data point " << (i+1) << "...";
          ur.GetRobotPose(&pur[i]);
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
      } // if (choice2 == 1) ... else

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

      ur.UpdateWorldTransform(w_2_r1);
      ur.SaveConfig(urfile.c_str());
#else
      cout << "UR is disabled." << endl;
#endif

#else
      cout << "ur is disabled." << endl;
#endif
      break;
    case 2:
#ifdef ENABLEROBOTS
#ifdef ENABLELWR
      cout << "Registering LWR using which method?  1) Hand guiding 2) Force plate : ";
      do
      {
        cin >> choice2;
      } while (choice2 < 1 || choice > 2);

      if (choice2 == 1)
      {
          lwr.GetRobotPose(&psr);
          lwr.MoveAttractor(psr);

        for (i = 0; i < 3; ++i)
        {
          /*
          lwr_hard[i].xrot = psr.xrot;
          lwr_hard[i].yrot = psr.yrot;
          lwr_hard[i].zrot = psr.zrot;
          cout << "moving to point " << (i+1) << "...";
          lwr.MoveStraightTo(lwr_hard[i]);
          lwr.GetRobotPose(&psr);
          psr.z = -10.0f;
          lwr_hard[i].xrot = psr.xrot;
          lwr_hard[i].yrot = psr.yrot;
          lwr_hard[i].zrot = psr.zrot;
          lwr.MoveAttractor(psr);
          timmy_time.waitUntil(1000);
          */
          
          cout << "Move LWR to defined point " << (i+1) << " (" << color[i] << ") and press ENTER";
          cin >> c;
//          cin.clear();
//          cin.get(c);
          
          lwr.GetRobotPose(&plwr[i]);
          robpt[i].x = plwr[i].x;
          robpt[i].y = plwr[i].y;
          robpt[i].z = plwr[i].z;

          wldpt[i].x = pworld[i].x;
          wldpt[i].y = pworld[i].y;
          wldpt[i].z = pworld[i].z;

//          lwr.MoveAttractor(lwr_hard[i]);
          timmy_time.waitUntil(1000);
        } // for (i = 0; i < 3; ++i)

        cout << "Press ENTER when ready to exit" << endl;
        cin >> c;
        lwr.GetRobotAxes(&axs);
        lwr.MoveToAxisTarget(axs);
      } // if (choice2 == 1)
      else
      {
        cout << "Center LWR above force registration plate about 1-2 inches from surface (do not align with divot) and press ENTER";
        cin.clear();
        cin.get(c);
        lwr.GetRobotPose(&psr);
        lwr_hard[3].xrot = psr.xrot;
        lwr_hard[3].yrot = psr.yrot;
        lwr_hard[3].zrot = psr.zrot;

        lwr.MoveStraightTo(lwr_hard[3]);
        timmy_time.waitUntil(1000);
        lwr.GetRobotPose(&psr);

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

          lwr.MoveStraightTo(tarpose);
          tarpose.z -= 70.0f;
          while (lwr.MoveAttractor(tarpose) != CANON_SUCCESS)
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
          lwr.GetRobotPose(&plwr[i]);
          robpt[i].x = plwr[i].x;
          robpt[i].y = plwr[i].y;
          robpt[i].z = plwr[i].z;
          cout << "[done]" << endl;

          tarpose.z = psr.z;          
          lwr.MoveStraightTo(tarpose);
        } // for (i = 0; i < 3; ++i)
      } // if (choice2 == 1) ... else

      world.clear();
      world.push_back(wldpt[0]);
      world.push_back(wldpt[1]);
      world.push_back(wldpt[2]);

      rob2.clear();
      rob2.push_back(robpt[0]);
      rob2.push_back(robpt[1]);
      rob2.push_back(robpt[2]);

#ifdef DOITRIGHTTHISTIME
      state = reg2target(rob2, world, r2_2_w);
      r2_2_w.print();

      lwr.UpdateWorldTransform(r2_2_w);
#else
      state = reg2target(rob2, world, r2_2_w);
      w_2_r2 = r2_2_w.inv();
      w_2_r2.print();

      lwr.UpdateWorldTransform(w_2_r2);
#endif
      lwr.SaveConfig(kukafile.c_str());
#else
      cout << "LWR is disabled" << endl;
#endif
#else
      cout << "LWR is disabled." << endl;
#endif
      break;
    case 3:
#ifdef ENABLEROBOTS
#ifdef ENABLEUR
      i = 0;
      ur.SetRelativeSpeed(0.50);
      ur.GetRobotAxes(&axs);
      axs.print();
      for (i = 0; i < axs.axes; ++i)
      {
        axs.axis.at(i) = 0.0f;//(3.141592654f / 180.0f);
      }
      axs.axis.at(1) = axs.axis.at(3) = -90.0f * (3.141592654f / 180.0f);
      cout << endl;
      axs.print();
      timmy_time.waitUntil(5000);
      i = 0;
      sprintf (buffer, "%f.csv", getCurrentTime());
      resout.open(buffer);
      resout << "regpoint, target.x, target.y, target.z, target.xrot, target.yrot, target.zrot, rob.x, rob.y, rob.z, rob.xrot, rob.yrot, rob.zrot, worldproj.x, worldproj.y, worldproj.z, worldproj.xrot, worldproj.yrot, worldproj.zrot, measured.x, measured.y, measured.z, measured.xrot, measured.yrot, measured.zrot" << endl;
      for (regiter = regposes.begin(); regiter != regposes.end(); ++regiter, ++i)
      {
        cout << "Testing point " << (i+1) << " of " << regposes.size() << endl;
        psr = *regiter;
        ur.FromWorld(&psr, &tarpose);

        resout << i << ", " << psr.x << ", " << psr.y << ", " << psr.z << ", " << psr.xrot << ", " << psr.yrot << ", " 
               << psr.zrot << ", " << tarpose.x << ", " << tarpose.y << ", " << tarpose.z << ", " << tarpose.xrot 
               << ", " << tarpose.yrot << ", " << tarpose.zrot;
        resout.flush();
        ur.MoveTo(tarpose);
        timmy_time.waitUntil(1000);
        ur.GetRobotPose(&tarpose);
        ur.ToWorld(&tarpose, &psr);
        resout << ", " << psr.x << ", " << psr.y << ", " << psr.z << ", " << psr.xrot << ", " << psr.yrot << ", " 
               << psr.zrot;

        resout.flush();
#ifdef ENABLEVICON
        vicon.GetCurrentSubjects(subjects);

        state = false;
        for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter)
        {
          if (viconiter->name == "UR_flange_Ring" && !state)
          {
            state = true;
            resout << ", " << viconiter->pose.x << ", " << viconiter->pose.y << ", " << viconiter->pose.z << ", " << viconiter->pose.xrot
                   << ", " << viconiter->pose.yrot << ", " << viconiter->pose.zrot << endl;
          }
        } // for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)
        if (!state)
        {
          resout << ", -1, -1 , -1, 0.0, 0.0, 0.0" << endl;
        }
#endif

        cout << "moving back to home" << endl;
        axs.print();
        ur.MoveToAxisTarget(axs);
      } //for (regiter = regposes.begin(); regiter != regposes.end(); ++regiter)
      resout.close();
#else
      cout << "UR is disabled." << endl;
#endif
#else
      cout << "UR is disabled." << endl;
#endif
      break;
    case 4:
#ifdef ENABLEROBOTS
#ifdef ENABLELWR
      i = 0;
      sprintf (buffer, "%f.csv", getCurrentTime());
      resout.open(buffer);
      resout << "regpoint, target.x, target.y, target.z, target.xrot, target.yrot, target.zrot, rob.x, rob.y, rob.z, rob.xrot, rob.yrot, rob.zrot, worldproj.x, worldproj.y, worldproj.z, worldproj.xrot, worldproj.yrot, worldproj.zrot, measured.x, measured.y, measured.z, measured.xrot, measured.yrot, measured.zrot" << endl;
      axsiter = axsout.begin();
      for (regiter = regposes.begin(); regiter != regposes.end(); ++regiter, ++i, ++axsiter)
      {
        cout << "Testing point " << (i+1) << " of " << regposes.size() << endl;
        psr = *regiter;
        lwr.FromWorld(&psr, &tarpose);
#ifdef USECARTESIAN
        resout << i << ", " << psr.x << ", " << psr.y << ", " << psr.z << ", " << psr.xrot << ", " << psr.yrot << ", " 
               << psr.zrot << ", " << tarpose.x << ", " << tarpose.y << ", " << tarpose.z << ", " << tarpose.xrot 
               << ", " << tarpose.yrot << ", " << tarpose.zrot;
        resout.flush();
        lwr.MoveTo(tarpose);
        timmy_time.waitUntil(1000);
        lwr.GetRobotPose(&tarpose);
        lwr.ToWorld(&tarpose, &psr);
        resout << ", " << psr.x << ", " << psr.y << ", " << psr.z << ", " << psr.xrot << ", " << psr.yrot << ", " 
               << psr.zrot;
#else
        lwr.MoveToAxisTarget(*axsiter);
        timmy_time.waitUntil(1000);
        lwr.GetRobotPose(&tarpose);
        resout << i << ", " << psr.x << ", " << psr.y << ", " << psr.z << ", " << psr.xrot << ", " << psr.yrot << ", " 
               << psr.zrot << ", " << tarpose.x << ", " << tarpose.y << ", " << tarpose.z << ", " << tarpose.xrot 
               << ", " << tarpose.yrot << ", " << tarpose.zrot;
        lwr.ToWorld(&tarpose, &psr);
        resout << ", " << psr.x << ", " << psr.y << ", " << psr.z << ", " << psr.xrot << ", " << psr.yrot << ", " 
               << psr.zrot;
#endif
        resout.flush();
//        cin >> choice;
        vicon.GetCurrentSubjects(subjects);

        state = false;
        for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter)
        {
          if (viconiter->name == "KUKA_flange_Ring" && !state)
          {
            state = true;
            resout << ", " << viconiter->pose.x << ", " << viconiter->pose.y << ", " << viconiter->pose.z << ", " << viconiter->pose.xrot
                   << ", " << viconiter->pose.yrot << ", " << viconiter->pose.zrot << endl;
          }
        } // for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)
        if (!state)
        {
          resout << ", -1, -1 , -1, 0.0, 0.0, 0.0" << endl;
        }

      } //for (regiter = regposes.begin(); regiter != regposes.end(); ++regiter)
      resout.close();
#else
      cout << "LWR is disabled." << endl;
#endif
#else
      cout << "LWR is disabled." << endl;
#endif
      break;
    case 5:
#ifdef ENABLEVICON
      vicon.GetCurrentSubjects(subjects);
    
      i = 0;
      for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)
      {
        cout << i << ": " << viconiter->name << " (" << viconiter->pose.x << ", " << viconiter->pose.y << ", " << viconiter->pose.z << ", " << viconiter->pose.xrot << ", " << viconiter->pose.yrot << ", " << viconiter->pose.zrot << ")" << endl;
      }
      break;
#else
      cout << "Vicon motion capture system disabled." << endl;
#endif
    case 6:
#ifdef ENABLEROBOTS
#ifdef ENABLELWR
      lwr.GetRobotPose(&psr);
      lwr.MoveAttractor(psr);

      do
      {
        cout << "Option 1) Recapture all points 2) replay previous points to recapture ground truth : ";
        cin >> choice;
      } while (choice != 1 && choice != 2);

      if (choice == 1)
      {
        axsout.clear();
        subout.clear();
        cout << "Option  1) Capture new data point 2) Quit and save -1) Quit (no save) : ";
        cin >> i;
        while (i != 2 && i != -1)
        {
          if (i == 1)
          {
            lwr.GetRobotAxes(&axs);
            lwr.MoveToAxisTarget(axs);
            lwr.GetRobotPose(&psr);
            timmy_time.waitUntil(3000);
            vicon.GetCurrentSubjects(subjects);

            for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter)
            {
              if (viconiter->name == "KUKA_flange_Ring")
              {
                axsout.push_back(axs);
                subout.push_back(*viconiter);
                cout << "Captured point #" << subout.size() << endl;
                cout << "Vicon:  (" << viconiter->pose.x << ", " << viconiter->pose.y << ", " << viconiter->pose.z
                     << ", " << viconiter->pose.xrot << ", " << viconiter->pose.yrot << ", " << viconiter->pose.zrot << ")" << endl;
                lwr.ToWorld(&psr, &tarpose);
                cout << "LWR:  (" << psr.x << ", " << psr.y << ", " << psr.z
                     << ", " << psr.xrot << ", " << psr.yrot << ", " << psr.zrot << ")" << endl;
                cout << "LWR2World:  (" << tarpose.x << ", " << tarpose.y << ", " << tarpose.z
                     << ", " << tarpose.xrot << ", " << tarpose.yrot << ", " << tarpose.zrot << ")" << endl;
              }
            } // for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)
            lwr.MoveAttractor(psr);
          } //if (i == 1)
          else
          {
            cout << "Invalid option" << endl;
          } //if (i == 1) ... else
          cout << "Option  1) Capture new data point 2) Quit and save -1) Quit (no save) : ";
          cin >> i;
        } //while (i != 2 && i != -1)

        if (i == 2)
        {
          cout << "Saving " << subout.size() << " data points to disk" << endl;
          regpointsout.open("points.dat");
          regpointsout << subout.size() << endl;
          axsiter = axsout.begin();
          for (viconiter = subout.begin(); viconiter != subout.end(); ++viconiter, ++axsiter)
          {
            regpointsout << viconiter->pose.x << " " << viconiter->pose.y << " " << viconiter->pose.z 
                         << " " << viconiter->pose.xrot << " " 
                         << viconiter->pose.yrot << " " 
                         << viconiter->pose.zrot;
            for (i = 0; i < axsiter->axes; ++i)
            {
              regpointsout << " " << axsiter->axis.at(i);
            }
            regpointsout << endl;
          } // for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)
          regpointsout.close();
          cout << "Done" << endl;
        }
        lwr.GetRobotAxes(&axs);
        lwr.MoveToAxisTarget(axs);
      } // if (choice == 1)
      else
      {
        psrout.clear();
        regpointsout.open("points.dat");
        cout << "Recapturing " << axsout.size() << " data points..." << endl;
        regpointsout << axsout.size() << endl;

        choice = 0;
        for (axsiter = axsout.begin(); axsiter != axsout.end(); ++axsiter, ++choice)
        {
          cout << "  Recapturing point " << (choice + 1) << endl;
          lwr.MoveToAxisTarget(*axsiter);
          timmy_time.waitUntil(3000);

          tarpose.x = tarpose.y = tarpose.z = tarpose.xrot = tarpose.yrot = tarpose.zrot = 0.0f;

          for (int j = 0; j < 10; ++j)
          {
            vicon.GetCurrentSubjects(subjects);
            for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter)
            {
              if (viconiter->name == "KUKA_flange_Ring")
              {
                tarpose = tarpose + viconiter->pose;
              }
            } // for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)
          } // for (int j = 0; j < 10; ++j)
          tarpose = tarpose / 10.0f;

          regpointsout << tarpose.x << " " << tarpose.y << " " << tarpose.z << " " << tarpose.xrot 
                       << " " << tarpose.yrot << " " << tarpose.zrot;
          for (i = 0; i < axsiter->axes; ++i)
          {
            regpointsout << " " << axsiter->axis.at(i);
          }
          regpointsout << endl;
        } // for (axsiter = axsout.begin(); axsiter != axsout.end(); ++axsiter
        cout << "Done" << endl;
      } // if (choice == 1) ... else
#else
      cout << "LWR must be enabled." << endl;
#endif
#else
      cout << "Robots are disabled." << endl;
#endif
      break;
    case 7:
#ifdef ENABLEROBOTS
#ifdef ENABLELWR
      lwr.GetRobotPose(&psr);
      cout << "LWR:  (" << psr.x << ", " << psr.y << ", " << psr.z
            << ", " << psr.xrot << ", " << psr.yrot << ", " << psr.zrot << ")" << endl;
      vicon.GetCurrentSubjects(subjects);
      for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter)
      {
        if (viconiter->name == "KUKA_flange_Ring")
        {
          cout << "Vicon:  (" << viconiter->pose.x << ", " << viconiter->pose.y << ", " << viconiter->pose.z
                << ", " << viconiter->pose.xrot << ", " << viconiter->pose.yrot << ", " << viconiter->pose.zrot << ")" << endl;
        }
      } // for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)
#else
      cout << "LWR is disabled." << endl;
#endif

#ifdef ENABLEUR
      ur.GetRobotPose(&psr);
      cout << "UR:  (" << psr.x << ", " << psr.y << ", " << psr.z
            << ", " << psr.xrot << ", " << psr.yrot << ", " << psr.zrot << ")" << endl;
#ifdef ENABLEVICON
      vicon.GetCurrentSubjects(subjects);
      for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter)
      {
        if (viconiter->name == "UR_flange_Ring")
        {
          cout << "Vicon:  (" << viconiter->pose.x << ", " << viconiter->pose.y << ", " << viconiter->pose.z
                << ", " << viconiter->pose.xrot << ", " << viconiter->pose.yrot << ", " << viconiter->pose.zrot << ")" << endl;
        }
      } // for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)
#else
      cout << "Vicon motion capture system disabled." << endl;
#endif
#else
      cout << "UR is disabled." << endl;
#endif

#else
      cout << "Robots are disabled." << endl;
#endif
      break;
    case 8:
#ifdef ENABLEROBOTS
#ifdef ENABLELWR
      resout.open("lwrvalidate.csv");
      resout << "point, frame, X, Y, Z, XROT, YROT, ZROT" << endl;
      lwr.GetRobotAxes(&axs2);
      i = 0;
      for (validiter = validhard.begin(); validiter != validhard.end(); ++validiter, ++i)
      {
        psr = *validiter;
        cout << "WLD: ";
        psr.print();
        resout << i << ", world target, " << psr.x << ", " << psr.y << ", " << psr.z << ", "
               << psr.xrot << ", " << psr.yrot << ", " << psr.zrot << endl;
        lwr.FromWorld(&psr, &tarpose);
        cout << "RBT: ";
        tarpose.print();
        resout << i << ", robot target, " << tarpose.x << ", " << tarpose.y << ", " << tarpose.z << ", "
               << tarpose.xrot << ", " << tarpose.yrot << ", " << tarpose.zrot << endl;
        tarpose.z += 55.0f;
        lwr.MoveTo(tarpose);

        timmy_time.waitUntil(5000);
        cout << "desired world: (X=" << psr.x << ", Y=" << psr.y << ", Z=" << psr.z
             << ", XROT=" << psr.xrot << ", YROT=" << psr.yrot << ", ZROT=" << psr.zrot << ")" << endl;
        psr.z = -15.0f;
        lwr.FromWorld(&psr, &tarpose);
        lwr.MoveAttractor(tarpose);

        timmy_time.waitUntil(5000);

        vicon.GetCurrentSubjects(subjects);
        for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter)
        {
          if (viconiter->name == "KUKA_flange_Ring")
          {
            resout << i << ", vicon, " << viconiter->pose.x << ", " << viconiter->pose.y 
                   << ", " << viconiter->pose.z << ", " << viconiter->pose.xrot << ", " 
                   << viconiter->pose.yrot << ", " << viconiter->pose.zrot << endl;
            cout << "Vicon:  (" << viconiter->pose.x << ", " << viconiter->pose.y << ", " << viconiter->pose.z
                  << ", " << viconiter->pose.xrot << ", " << viconiter->pose.yrot << ", " << viconiter->pose.zrot << ")" << endl;
          }
        } // for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)

        lwr.GetRobotPose(&tarpose);
        lwr.ToWorld(&tarpose, &psr);
        resout << i << ", actual world, " << psr.x << ", " << psr.y << ", " << psr.z << ", "
               << psr.xrot << ", " << psr.yrot << ", " << psr.zrot << endl;
        cout << "actual world: (X=" << psr.x << ", Y=" << psr.y << ", Z=" << psr.z
             << ", XROT=" << psr.xrot << ", YROT=" << psr.yrot << ", ZROT=" << psr.zrot << ")" << endl;
        tarpose.z += 100.0f;
        lwr.MoveAttractor(tarpose);
        cout << "ready.  Press ENTER. " << endl;
        cin >> choice; 
        lwr.GetRobotAxes(&axs);
        lwr.MoveToAxisTarget(axs);
      } // for (validiter = validhard.begin(); validiter != validhard.end(); ++validiter)
      lwr.MoveToAxisTarget(axs2);
#else
      cout << "LWR is disabled." << endl;
#endif
#else
      cout << "LWR is disabled." << endl;
#endif
      break;
    case 9:
#ifdef ENABLEROBOTS
#ifdef ENABLEUR
      for (validiter = validhard.begin(); validiter != validhard.end(); ++validiter)
      {
        psr = *validiter;
        ur.FromWorld(&psr, &tarpose);
        tarpose.z += 55.0f;
        ur.MoveTo(tarpose);

        timmy_time.waitUntil(5000);
        cout << "desired world: (X=" << psr.x << ", Y=" << psr.y << ", Z=" << psr.z
             << ", XROT=" << psr.xrot << ", YROT=" << psr.yrot << ", ZROT=" << psr.zrot << ")" << endl;
        psr.z = -15.0f;
        ur.FromWorld(&psr, &tarpose);

        cout << "Move robot down and enter 1: ";
        cin >> i;

        timmy_time.waitUntil(5000);
#ifdef ENABLEVICON
        vicon.GetCurrentSubjects(subjects);
        for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter)
        {
          if (viconiter->name == "UR_flange_Ring")
          {
            cout << "Vicon:  (" << viconiter->pose.x << ", " << viconiter->pose.y << ", " << viconiter->pose.z
                  << ", " << viconiter->pose.xrot << ", " << viconiter->pose.yrot << ", " << viconiter->pose.zrot << ")" << endl;
          }
        } // for (viconiter = subjects.begin(); viconiter != subjects.end(); ++viconiter, ++i)
#endif

        ur.GetRobotPose(&tarpose);
        ur.ToWorld(&tarpose, &psr);
        cout << "actual world: (X=" << psr.x << ", Y=" << psr.y << ", Z=" << psr.z
             << ", XROT=" << psr.xrot << ", YROT=" << psr.yrot << ", ZROT=" << psr.zrot << ")" << endl;
        tarpose.z += 100.0f;
        cout << "Move robot up and enter 1: ";
        cin >> i;
      } // for (validiter = validhard.begin(); validiter != validhard.end(); ++validiter)
#else
      cout << "UR is disabled." << endl;
#endif
#else
      cout << "UR is disabled." << endl;
#endif

      break;
    default:
      cout << "Invalid entry" << endl;
      break;
    }
    cout << "Select action:  1) Register ur to World 2) Register LWR to World" << endl
         << "3) Test UR Reg 4) Test LWR Reg 5) Test Vicon 6) Create test set" << endl
         << "7) Compare robot poses to Vicon" << endl
         << "8) Validate LWR 9) Validat UR" << endl
         << "-1) Quit: ";
    cin >> choice;
  } // while (choice != -1)

}