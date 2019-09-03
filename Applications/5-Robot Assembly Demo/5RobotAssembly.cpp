///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       5-Robot, coordinated assembly process
//  Workfile:        5RobotAssembly.cpp
//  Revision:        1.0    2/5/2017
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Demo in which 5 robots are coordinated to complete a simple cylinder
//  artifact assembly.
//  Robot #1 (dual arm):   assemble kit tray from different feeders
//  Robot #2 (single arm): organize kit trays (empty and prepared)
//                         remove finished parts from trays
//  Robot #3 (single arm): move kit trays into and out of assembly cell
//  Robot #4 (single arm): place kit trays in assembly arena
//                         act as dynamic fixture for main assembly body
//  Robot #5 (single arm): insert components into assembly body
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_abb.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#include "NumericalMath.h" 
#include "../../Libraries/MotionPrims/AssemblyPrims.h"
#include "CoordFrameReg.h"

#pragma warning (disable: 4996)

//#define useworld
#define verifysteps
//#define bypasskitting
#define bypasstransfer
#define bypassassemble

#define enableABB
#define enableUR5
//#define enableUR10L
//#define enableUR10R
//#define enableLWR

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
  bool keeprunning;
  bool TransfererOkay;
  bool AssemblerOkay;

  vector<string> kit_tray_assemble_j_labels, kit_tray_assemble_c_labels,
                 kit_tray_transfer_j_labels, kit_tray_transfer_c_labels,
                 assemble_j_labels, assemble_c_labels;

  //! @brief Default constructor
  //!
  passMe ()
  {
    string str;
    grabmutex = ulapi_mutex_new(89);
    TransfererOkay = AssemblerOkay = false;

    str = "abb_r_store";
    kit_tray_assemble_j_labels.push_back(str);
    str = "abb_l_store";
    kit_tray_assemble_j_labels.push_back(str);
    //! Cartesian poses
    str = "cylinder_get_hover";
    kit_tray_assemble_c_labels.push_back(str);
    str = "cylinder_get";
    kit_tray_assemble_c_labels.push_back(str);
    str = "cylinder_remove";
    kit_tray_assemble_c_labels.push_back(str);
    str = "cylinder_put_hover";
    kit_tray_assemble_c_labels.push_back(str);
    str = "cylinder_put";
    kit_tray_assemble_c_labels.push_back(str);
    str = "rod_get_intermed";
    kit_tray_assemble_j_labels.push_back(str);
    str = "rod_get_hover";
    kit_tray_assemble_c_labels.push_back(str);
    str = "rod_get";
    kit_tray_assemble_c_labels.push_back(str);
    str = "rod_remove";
    kit_tray_assemble_c_labels.push_back(str);
    str = "rod_hand_hover";
    kit_tray_assemble_c_labels.push_back(str);
    str = "rod_hand";
    kit_tray_assemble_c_labels.push_back(str);
    str = "rod_take_hover";
    kit_tray_assemble_c_labels.push_back(str);
    str = "rod_take";
    kit_tray_assemble_c_labels.push_back(str);
    str = "rod_put_hover";
    kit_tray_assemble_c_labels.push_back(str);
    str = "rod_put";
    kit_tray_assemble_c_labels.push_back(str);    
    str = "matrix_1";
    kit_tray_assemble_c_labels.push_back(str);
    str = "matrix_2";
    kit_tray_assemble_c_labels.push_back(str);
    str = "matrix_3";
    kit_tray_assemble_c_labels.push_back(str);
    str = "cap_put_hover_1";
    kit_tray_assemble_c_labels.push_back(str);
    str = "cap_put_1";
    kit_tray_assemble_c_labels.push_back(str);
    str = "cap_put_hover_2";
    kit_tray_assemble_c_labels.push_back(str);
    str = "cap_put_2";
    kit_tray_assemble_c_labels.push_back(str);
    str = "button_hover";
    kit_tray_assemble_c_labels.push_back(str);
    str = "button_push";
    kit_tray_assemble_c_labels.push_back(str);

    //! Kit tray transfer
    //! Joint positions
    str = "ur5_store";
    kit_tray_transfer_j_labels.push_back(str);
    str = "ur10_left_store";
    kit_tray_transfer_j_labels.push_back(str);
    //! Cartesian positions
    str = "tray_get_hover_filled_1";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_get_filled_1";
    kit_tray_transfer_c_labels.push_back(str); 
    str = "tray_put_hover_filled_1";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_put_filled_1";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_get_hover_filled_2";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_get_filled_2";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_hand_hover";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_hand";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_put_hover_complete";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_put_complete";
    kit_tray_transfer_c_labels.push_back(str);
    str = "part_get_hover";
    kit_tray_transfer_c_labels.push_back(str);
    str = "part_get";
    kit_tray_transfer_c_labels.push_back(str);
    str = "part_put_hover";
    kit_tray_transfer_c_labels.push_back(str);
    str = "part_put";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_get_hover_empty";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_get_empty";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_put_hover_empty";
    kit_tray_transfer_c_labels.push_back(str);
    str = "tray_put_empty";
    kit_tray_transfer_c_labels.push_back(str);

    str = "tray_hand_grasp_2";
    kit_tray_transfer_c_labels.push_back(str);

    //! Assemble
    //! Joint position
    str = "ur10_right_store";
    assemble_j_labels.push_back(str);
    str = "kuka_store";
    assemble_j_labels.push_back(str);
    //! Cartesian position
    str = "tray_hand_hover";
    assemble_c_labels.push_back(str);
    str = "tray_hand_grasp";
    assemble_c_labels.push_back(str);
    str = "tray_hand_reorient";
    assemble_c_labels.push_back(str);
    str = "tray_place_hover";
    assemble_c_labels.push_back(str);
    str = "tray_place";
    assemble_c_labels.push_back(str);
    str = "cylinder_get_hover";
    assemble_c_labels.push_back(str);
    str = "cylinder_get";
    assemble_c_labels.push_back(str);
    str = "cylinder_hold_back";
    assemble_c_labels.push_back(str);
    str = "cylinder_hold_backj";
    assemble_j_labels.push_back(str);
    str = "cylinder_fixture_1";
    assemble_c_labels.push_back(str);
    str = "cylinder_fixture_2";
    assemble_c_labels.push_back(str);
    str = "cylinder_put_hover";
    assemble_c_labels.push_back(str);
    str = "cylinder_put";
    assemble_c_labels.push_back(str);
    str = "rod_get_hover";
    assemble_c_labels.push_back(str);
    str = "rod_get";
    assemble_c_labels.push_back(str);
    str = "rod_put_hover";
    assemble_c_labels.push_back(str);
    str = "rod_put";
    assemble_c_labels.push_back(str);
    str = "cap_get_hover_1";
    assemble_c_labels.push_back(str);
    str = "cap_get_1";
    assemble_c_labels.push_back(str);
    str = "cap_put_hover";
    assemble_c_labels.push_back(str);
    str = "cap_put";
    assemble_c_labels.push_back(str);
    str = "cap_get_hover_2";
    assemble_c_labels.push_back(str);
    str = "cap_get_2";
    assemble_c_labels.push_back(str);

    //demo = NULL;
    keeprunning = true;
  }

};


int getLabelIndex(vector<string> &labels, const char *label)
{
  for (int x = 0; x < labels.size(); ++x)
  {
    //cout << label << " " << labels.at(x) << endl;
    if (labels.at(x) == label)
    {
      return x;
    }
  }
  return -1;
}


void saveandconvert()
{
  double speed = 0.5f;
  
  //CrpiRobot<CrpiAbb> rob("abb_irb14000_right.xml", true);
  //CrpiRobot<CrpiAbb> rob("abb_irb14000_left.xml", true);
  //rob.Couple("Yumi_Parallel");
  CrpiRobot<CrpiUniversal> rob("universal_ur5.xml", true);
  //CrpiRobot<CrpiKukaLWR> rob("kuka_lwr.xml", true);
  rob.Couple("gripper_parallel");
  //CrpiRobot<CrpiUniversal> rob("universal_ur10_left.xml", true);
  //CrpiRobot<CrpiUniversal> rob("universal_ur10_right.xml", true);
  //rob.Couple("gripper_parallel_plastic");

  rob.SetAngleUnits("degree");
  rob.SetLengthUnits("mm");
  rob.SetRelativeSpeed(speed);

  int option;
  string instr, outstr, name;
  char chr;
  double val;

  cout << "1) Save or 2) Convert saved to World: ";
  cin >> option;
  robotPose inpose, outpose;
  vector<robotPose> inposes;

  if (option == 1)
  {
    cout << "Enter file name: ";
    cin >> instr;

    do
    {
      cout << "Move to target location.  Enter 1 to record pose: " << endl;
      cin >> option;
      rob.GetRobotPose(&inpose);
      inposes.push_back(inpose);

      cout << "-1) Done 1) Add another pose";
      cin >> option;
    } while (option > 0);


    ofstream outfile(instr.c_str());
    outfile << inposes.size() << endl;
    for (int x = 0; x < inposes.size(); ++x)
    {
      outfile << inposes.at(x).x << " " << inposes.at(x).y << " " << inposes.at(x).z << " " <<
                 inposes.at(x).xrot << " " << inposes.at(x).yrot << " " << inposes.at(x).zrot << " " <<
                 inposes.at(x).status << " " << inposes.at(x).turns << endl;
    }

  }
  else if (option == 2)
  {
    cout << "Enter input file name:  ";
    cin >> instr;
    cout << "Enter output file name:  ";
    cin >> outstr;

    ifstream infile(instr.c_str());
    ofstream outfile(outstr.c_str());

    while (infile >> name)
    {
      infile >> chr;

      if (chr == 'j')
      {
        outfile << name.c_str() << " " << chr << " ";
        for (int j = 0; j < 7; ++j)
        {
          infile >> val;
          outfile << val << " ";
        }
        outfile << endl;
      }
      else if (chr == 'c')
      {
        infile >> inpose.x >> inpose.y >> inpose.z >> inpose.xrot >> inpose.yrot >> inpose.zrot >> inpose.status >> inpose.turns;
        rob.ToWorld(&inpose, &outpose);
        outfile << name.c_str() << " " << chr << " " << outpose.x << " " << outpose.y << " " << outpose.z << " " << outpose.xrot << " " << outpose.yrot << " " << outpose.zrot << " " << outpose.status << " " << outpose.turns << endl;       
      }
    }


    cout << "Done." << endl;
  }
}

void KitTrayAssembly(CrpiRobot<CrpiAbb> &abb_r, CrpiRobot<CrpiAbb> &abb_l)
{
}




void SerialAssembly(passMe *pM)
{
  int index;
  string str;
  char type;
  bool okay = true;
  double speed = 0.5;

  cout << "Initializing robots..." << endl;

#ifdef enableABB
  cout << "Connecting to ABB... ";
  CrpiRobot<CrpiAbb> abb_r("abb_irb14000_right.xml");
  CrpiRobot<CrpiAbb> abb_l("abb_irb14000_left.xml");
  abb_r.Couple("Yumi_Parallel");
  abb_l.Couple("Yumi_Parallel");
  abb_r.SetRelativeSpeed(speed);
  abb_l.SetRelativeSpeed(speed);
  abb_r.SetAngleUnits("degree");
  abb_r.SetLengthUnits("mm");
  abb_l.SetAngleUnits("degree");
  abb_l.SetLengthUnits("mm");
  cout << "done." << endl;
#endif
#ifdef enableUR5
  cout << "Connecting to UR5... ";
  CrpiRobot<CrpiUniversal> ur5("universal_ur5.xml");
  ur5.Couple("gripper_parallel");
  ur5.SetRelativeSpeed(speed);
  ur5.SetAngleUnits("degree");
  ur5.SetLengthUnits("mm");
  cout << "done." << endl;
#endif
#ifdef enableUR10L
  cout << "Connecting to UR10 Left... ";
  CrpiRobot<CrpiUniversal> ur10_left("universal_ur10_left.xml");
  ur10_left.Couple("gripper_parallel_plastic");
  ur10_left.SetRelativeSpeed(speed);
  ur10_left.SetAngleUnits("degree");
  ur10_left.SetLengthUnits("mm");
  cout << "done." << endl;
#endif
#ifdef enableUR10R
  cout << "Connecting to UR10 Right... ";
  CrpiRobot<CrpiUniversal> ur10_right("universal_ur10_right.xml");
  ur10_right.Couple("gripper_parallel_plastic");
  ur10_right.SetRelativeSpeed(speed);
  ur10_right.SetAngleUnits("degree");
  ur10_right.SetLengthUnits("mm");
  cout << "done." << endl;
#endif
#ifdef enableLWR
  cout << "Connecting to KUKA... ";
  CrpiRobot<CrpiKukaLWR> kuka("kuka_lwr.xml");
  kuka.Couple("gripper_parallel");
  kuka.SetRelativeSpeed(speed);
  kuka.SetAngleUnits("degree");
  kuka.SetLengthUnits("mm");
  cout << "done."
#endif
  cout << "Populating poses..." << endl;
  robotAxes jtemp, curjoints;
  robotPose ctemp, ctar, curpose;

  vector<robotAxes> kit_tray_assemble_joints, kit_tray_transfer_joints, assemble_joints;
  kit_tray_assemble_joints.resize(pM->kit_tray_assemble_j_labels.size());
  kit_tray_transfer_joints.resize(pM->kit_tray_transfer_j_labels.size());
  assemble_joints.resize(pM->assemble_j_labels.size());

  vector<robotPose> kit_tray_assemble_poses, kit_tray_transfer_poses, assemble_poses;
  kit_tray_assemble_poses.resize(pM->kit_tray_assemble_c_labels.size());
  kit_tray_transfer_poses.resize(pM->kit_tray_transfer_c_labels.size());
  assemble_poses.resize(pM->assemble_c_labels.size());

  ifstream in;
  in.open("poses_kit_tray_assemble.dat");
  //! Read poses from disk
  while (in >> str)
  {
    in >> type;
    if (type == 'c')
    {
      //! Cartesian pose
      in >> ctemp.x >> ctemp.y >> ctemp.z >> ctemp.xrot >> ctemp.yrot >> ctemp.zrot >> ctemp.status >> ctemp.turns;
      index = getLabelIndex(pM->kit_tray_assemble_c_labels, str.c_str());
      if (index >= 0)
      {
        kit_tray_assemble_poses.at(index) = ctemp;
      }
      else
      {
        cout << str.c_str() << " not found in kit_tray_assemble_c_labels." << endl;

        //! Error:  Bad value
      }
    }
    else if (type == 'j')
    {
      in >> jtemp.axis.at(0) >> jtemp.axis.at(1) >> jtemp.axis.at(2) >> jtemp.axis.at(3) >> jtemp.axis.at(4) >> jtemp.axis.at(5) >> jtemp.axis.at(6);
      index = getLabelIndex(pM->kit_tray_assemble_j_labels, str.c_str());
      if (index >= 0)
      {
        kit_tray_assemble_joints.at(index) = jtemp;
      }
      else
      {
        cout << str.c_str() << " not found in kit_tray_assemble_j_labels." << endl;

        //! Error:  Bad value
      }
    }
    else
    {
      //! Error:  Bad type, likely corrupted file.
      okay = false;
    }
  }
  in.close();

  //! Calculate the matrix of end-cap positions
  int i1 = getLabelIndex(pM->kit_tray_assemble_c_labels, "matrix_1");
  if (i1 < 0)
  {
    cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
    return;
  }
  int i2 = getLabelIndex(pM->kit_tray_assemble_c_labels, "matrix_2");
  if (i2 < 0)
  {
    cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
    return;
  }
  int i3 = getLabelIndex(pM->kit_tray_assemble_c_labels, "matrix_3");
  if (i3 < 0)
  {
    cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
    return;
  }

  vector<robotPose> cap_matrix;
  robotPose p1, p2, p3, d31, d21, pt;
  p1 = kit_tray_assemble_poses.at(i1);
  p2 = kit_tray_assemble_poses.at(i2);
  p3 = kit_tray_assemble_poses.at(i3);
  d31 = (p3 - p1) / 4.0;
  d21 = (p2 - p1) / 3.0;
  unsigned int matrixcounter = 0;

  cap_matrix.clear();
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 5; ++j)
    {
      pt = p1 + (d31 * j) + (d21 * i);
      cap_matrix.push_back(pt);
//      pt.print();
    }
  }

  in.open("poses_kit_tray_transfer.dat");
  while (in >> str)
  {
    in >> type;
    if (type == 'c')
    {
      //! Cartesian pose
      in >> ctemp.x >> ctemp.y >> ctemp.z >> ctemp.xrot >> ctemp.yrot >> ctemp.zrot >> ctemp.status >> ctemp.turns;
      index = getLabelIndex(pM->kit_tray_transfer_c_labels, str.c_str());
      if (index >= 0)
      {
        kit_tray_transfer_poses.at(index) = ctemp;
      }
      else
      {
        cout << str.c_str() << " not found." << endl;

        //! Error:  Bad value
      }
    }
    else if (type == 'j')
    {
      in >> jtemp.axis.at(0) >> jtemp.axis.at(1) >> jtemp.axis.at(2) >> jtemp.axis.at(3) >> jtemp.axis.at(4) >> jtemp.axis.at(5) >> jtemp.axis.at(6);
      index = getLabelIndex(pM->kit_tray_transfer_j_labels, str.c_str());
      if (index >= 0)
      {
        kit_tray_transfer_joints.at(index) = jtemp;
      }
      else
      {
        //! Error:  Bad value
      }
    }
    else
    {
      cout << str.c_str() << " not found." << endl;

      //! Error:  Bad type, likely corrupted file.
      okay = false;
    }
  }
  in.close();

  in.open("poses_assemble.dat");
  //! Read poses from disk
  while (in >> str)
  {
    in >> type;
    if (type == 'c')
    {
      //! Cartesian pose
      in >> ctemp.x >> ctemp.y >> ctemp.z >> ctemp.xrot >> ctemp.yrot >> ctemp.zrot >> ctemp.status >> ctemp.turns;
      index = getLabelIndex(pM->assemble_c_labels, str.c_str());
      if (index >= 0)
      {
        assemble_poses.at(index) = ctemp;
      }
      else
      {
        cout << str.c_str() << " not found." << endl;
        //! Error:  Bad value
      }
    }
    else if (type == 'j')
    {
      in >> jtemp.axis.at(0) >> jtemp.axis.at(1) >> jtemp.axis.at(2) >> jtemp.axis.at(3) >> jtemp.axis.at(4) >> jtemp.axis.at(5) >> jtemp.axis.at(6);
      index = getLabelIndex(pM->assemble_j_labels, str.c_str());
      if (index >= 0)
      {
        assemble_joints.at(index) = jtemp;
      }
      else
      {
        cout << str.c_str() << " not found." << endl;
        //! Error:  Bad value
      }
    }
    else
    {
      //! Error:  Bad type, likely corrupted file.
      okay = false;
    }
  }
  in.close();

#ifdef verifysteps
  int option = 0;
#endif

  do
  {
    cout << "Moving robots to stow positions." << endl;
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    //! -----------------------------------------------------------------------------
    //! STEP0:  Reset All
    //! -----------------------------------------------------------------------------

    //!  Move to stow positions...
#ifdef enableABB
    cout << "Stow ABB" << endl;
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_r_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    if (abb_r.MoveToAxisTarget(kit_tray_assemble_joints.at(index)) == CANON_SUCCESS)
    {
      abb_r.GetRobotAxes(&curjoints);
    }
    abb_r.SetTool(0.0f);
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_l_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    if (abb_l.MoveToAxisTarget(kit_tray_assemble_joints.at(index)) == CANON_SUCCESS)
    {
      abb_l.GetRobotAxes(&curjoints);
    }
    abb_l.SetTool(0.0f);
#endif
#ifdef enableUR5
    //!  Move to stow positions...
    cout << "Stow UR5" << endl;
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur5_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur5.MoveToAxisTarget(kit_tray_transfer_joints.at(index)) == CANON_SUCCESS)
    {
      ur5.GetRobotAxes(&curjoints);
    }
    ur5.SetRobotDO(0, 0);
#endif
#ifdef enableUR10L
    cout << "Stow Left UR10" << endl;
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur10_left_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur10_left.MoveToAxisTarget(kit_tray_transfer_joints.at(index)) == CANON_SUCCESS)
    {
      ur10_left.GetRobotAxes(&curjoints);
    }
    ur10_left.SetRobotDO(0, 0);
#endif
#ifdef enableUR10R
    cout << "Stow Right UR10" << endl;
    index = getLabelIndex(pM->assemble_j_labels, "ur10_right_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    if (ur10_right.MoveToAxisTarget(assemble_joints.at(index)) == CANON_SUCCESS)
    {
      ur10_right.GetRobotAxes(&curjoints);
    }
    ur10_right.SetRobotDO(0, 0);
#endif
#ifdef enableLWR
    cout << "Stow KUKA" << endl;
    index = getLabelIndex(pM->assemble_j_labels, "kuka_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    if (kuka.MoveToAxisTarget(assemble_joints.at(index)) == CANON_SUCCESS)
    {
      kuka.GetRobotAxes(&curjoints);
    }
    kuka.SetTool(0.0);
#endif
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    cout << "Starting kit tray assembly..." << endl;
#ifndef bypasskitting
    //!   Get cylinder
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_get");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_l.SetTool(1.0f);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_remove");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Place part #1
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    Sleep(1000);
    abb_l.SetTool(0.0f);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Move ABB_L out of way
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_l_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    if (abb_l.MoveToAxisTarget(kit_tray_assemble_joints.at(index)) == CANON_SUCCESS)
    {
      abb_l.GetRobotAxes(&curjoints);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Get first cap
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &pt);
#else
    pt = kit_tray_assemble_poses.at(index);
#endif
    pt = cap_matrix.at(matrixcounter);
    ctar = pt;
    ctar.z += 100.0f;
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    if (abb_r.MoveStraightTo(pt) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    abb_r.SetTool(1.0f);
    Sleep(500);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    matrixcounter++;
    if (matrixcounter >= 20)
    {
      matrixcounter = 0;
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Place cap
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(1000);
    abb_r.SetTool(0.0f);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_r_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    if (abb_r.MoveToAxisTarget(kit_tray_assemble_joints.at(index)) == CANON_SUCCESS)
    {
      abb_r.GetRobotAxes(&curjoints);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Move toward rod
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "rod_get_intermed");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    if (abb_l.MoveToAxisTarget(kit_tray_assemble_joints.at(index)) == CANON_SUCCESS)
    {
      abb_l.GetRobotAxes(&curjoints);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Get rod
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_get");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_l.SetTool(1.0f);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_remove");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Hand over rod
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_hand");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_take_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(750);
    abb_r.SetTool(0.0f);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_take");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    abb_r.SetTool(1.0f);
    Sleep(500);
    abb_l.SetTool(0.0f);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_l.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Move ABB_L out of way
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_l_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    if (abb_l.MoveToAxisTarget(kit_tray_assemble_joints.at(index)) == CANON_SUCCESS)
    {
      abb_l.GetRobotAxes(&curjoints);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Place rod
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(1000);
    abb_r.SetTool(0.0f);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif



    //!   Get second cap
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &pt);
#else
    pt = kit_tray_assemble_poses.at(index);
#endif
    pt = cap_matrix.at(matrixcounter);

    ctar = pt;
    ctar.z += 100.0f;
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    if (abb_r.MoveStraightTo(pt) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(1000);
    abb_r.SetTool(1.0f);
    Sleep(500);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    matrixcounter++;
    if (matrixcounter >= 20)
    {
      matrixcounter = 0;
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Place cap #2
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    abb_r.SetTool(0.0f);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
#ifdef useworld
    abb_r.FromWorld(&kit_tray_assemble_poses.at(index), &ctar);
#else
    ctar = kit_tray_assemble_poses.at(index);
#endif
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Done assembly kit tray
    //! Move to store
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_r_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    if (abb_r.MoveToAxisTarget(kit_tray_assemble_joints.at(index)) == CANON_SUCCESS)
    {
      abb_r.GetRobotAxes(&curjoints);
    }
    abb_r.SetTool(0.0f);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
#endif //bypasskitting

    //! -----------------------------------------------------------------------------
    //! STEP2:  Kit Tray Transfer
    //! -----------------------------------------------------------------------------
#ifndef bypasstransfer
    //!   Get filled part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 1);;
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Place filled part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_hover_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 0);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_hover_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Go to store
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur5_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur5.MoveToAxisTarget(kit_tray_transfer_joints.at(index)) == CANON_SUCCESS)
    {
      ur5.GetRobotAxes(&curjoints);
    }
    ur5.SetRobotDO(0, 0);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Robot #3:
    //!   Get filled part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_filled_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_filled_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    ur10_left.SetRobotDO(0, 1);;
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_filled_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    //qqqq


    //!   Move to hand off position
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_hand");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! -----------------------------------------------------------------------------
    //! STEP3:  Assembly
    //! -----------------------------------------------------------------------------

    //!   Move to tray hand-off
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move to hand off position
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_grasp");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Grasp part tray
    Sleep(500);
    ur10_right.SetRobotDO(0, 1);;
    Sleep(500);

    //!   Release part tray
    Sleep(500);
    ur10_left.SetRobotDO(0, 0);;
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    //!   Move to hover
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Go to store
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur10_left_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur10_left.MoveToAxisTarget(kit_tray_transfer_joints.at(index)) == CANON_SUCCESS)
    {
      ur10_left.GetRobotAxes(&curjoints);
    }
    ur10_left.SetRobotDO(0, 0);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Remove Tray
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Remove Tray
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_reorient");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif



    //!   Place filled tray
    index = getLabelIndex(pM->assemble_c_labels, "tray_place_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "tray_place");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_right.SetRobotDO(0, 0);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "tray_place_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif


    //!   Move to stow
    index = getLabelIndex(pM->assemble_j_labels, "ur10_right_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    if (ur10_right.MoveToAxisTarget(assemble_joints.at(index)) == CANON_SUCCESS)
    {
      ur10_right.GetRobotAxes(&curjoints);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
#endif //bypasstransfer

#ifndef bypassassemble
    //! Robot #4
    //!   Take part #1
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_get");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_right.SetRobotDO(0, 1);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move out of the way
    index = getLabelIndex(pM->assemble_j_labels, "cylinder_hold_backj");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    jtemp = assemble_joints.at(index);
    if (ur10_right.MoveToAxisTarget(jtemp) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    /*
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    */
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Robot #5
    //!   Take part #3_1
    index = getLabelIndex(pM->assemble_c_labels, "cap_get_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "cap_get_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    Sleep(500);
    kuka.SetTool(1.0f);;
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "cap_get_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    index = getLabelIndex(pM->assemble_c_labels, "cap_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move to assembly pose 1
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_fixture_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Insert part
    index = getLabelIndex(pM->assemble_c_labels, "cap_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    kuka.SetTool(0.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "cap_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //TODO
    /*
    //!   Tuck part
    kuka.SetTool(1.0f);
    index = getLabelIndex(pM->assemble_c_labels, "cap_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "part3_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    kuka.SetTool(0.0f);;
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    */

    //!   Move out of the way
    //!   Move out of the way
    index = getLabelIndex(pM->assemble_j_labels, "cylinder_hold_backj");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    jtemp = assemble_joints.at(index);
    if (ur10_right.MoveToAxisTarget(jtemp) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    /*
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_hold_back");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    */
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif


    //! Robot #5
    //!   Take part #2
    index = getLabelIndex(pM->assemble_c_labels, "rod_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "rod_get");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    kuka.SetTool(1.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "rod_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    index = getLabelIndex(pM->assemble_c_labels, "rod_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Robot #4
    //!   Move to assembly pose 2
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_fixture_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Insert part
    index = getLabelIndex(pM->assemble_c_labels, "rod_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    kuka.SetTool(0.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "rod_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move out of the way
    //!   Move out of the way
    index = getLabelIndex(pM->assemble_j_labels, "cylinder_hold_backj");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    jtemp = assemble_joints.at(index);
    if (ur10_right.MoveToAxisTarget(jtemp) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    /*
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_hold_back");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    */
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Take part #3_2
    index = getLabelIndex(pM->assemble_c_labels, "cap_get_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "cap_get_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    kuka.SetTool(1.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "cap_get_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    index = getLabelIndex(pM->assemble_c_labels, "cap_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move into fixture position
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_fixture_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Insert part
    index = getLabelIndex(pM->assemble_c_labels, "cap_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    kuka.SetTool(0.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "cap_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! TODO
    //!   Tuck part
    /*
    kuka.SetTool(1.0f);
    index = getLabelIndex(pM->assemble_c_labels, "part3_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "part3_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    kuka.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    kuka.SetTool(0.0f);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    */

    //!   Move out of the way
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_hold_back");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move to store pose
    index = getLabelIndex(pM->assemble_j_labels, "kuka_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    if (kuka.MoveToAxisTarget(assemble_joints.at(index)) == CANON_SUCCESS)
    {
      kuka.GetRobotAxes(&curjoints);
    }
    kuka.SetTool(0.0);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move into fixture position
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_fixture_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Robot #4
    //!   Put completed part in tray
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_right.SetRobotDO(0, 0);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "cylinder_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
#endif //bypassassemble

#ifndef bypasstransfer
    //!   Take part tray
    index = getLabelIndex(pM->assemble_c_labels, "tray_place_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->assemble_c_labels, "tray_place");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    ur10_right.SetRobotDO(0, 1);
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "tray_place_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_reorient");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move to hand off position
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Set down tray
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_grasp");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move to hand off position
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_hand");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Grab part tray
    Sleep(500);
    ur10_left.SetRobotDO(0, 1);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    //!   Release part tray
    ur10_right.SetRobotDO(0, 0);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    //!   Move to hover
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
#ifdef useworld
    ur10_right.FromWorld(&assemble_poses.at(index), &ctar);
#else
    ctar = assemble_poses.at(index);
#endif
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Move to stow
    index = getLabelIndex(pM->assemble_j_labels, "ur10_right_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    if (ur10_right.MoveToAxisTarget(assemble_joints.at(index)) == CANON_SUCCESS)
    {
      ur10_right.GetRobotAxes(&curjoints);
    }
    ur10_right.SetRobotDO(0, 0);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //! Remove tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Place completed part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_hover_complete");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_complete");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_left.SetRobotDO(0, 0);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_hover_complete");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur10_left.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Go to store
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur10_left_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur10_left.MoveToAxisTarget(kit_tray_transfer_joints.at(index)) == CANON_SUCCESS)
    {
      ur10_left.GetRobotAxes(&curjoints);
    }
    ur10_left.SetRobotDO(0, 0);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Remove completed part
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "part_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "part_get");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 1);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "part_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Place completed part
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "part_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "part_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    ur5.SetRobotDO(0, 0);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "part_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Take empty part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_empty");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_empty");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    Sleep(500);
    ur5.SetRobotDO(0, 1);;
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_empty");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Place empty part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_hover_empty");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_empty");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 0);
    Sleep(500);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_hover_empty");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
#ifdef useworld
    ur5.FromWorld(&kit_tray_transfer_poses.at(index), &ctar);
#else
    ctar = kit_tray_transfer_poses.at(index);
#endif
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif

    //!   Go to store
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur5_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur5.MoveToAxisTarget(kit_tray_transfer_joints.at(index)) == CANON_SUCCESS)
    {
      ur5.GetRobotAxes(&curjoints);
    }
    ur5.SetRobotDO(0, 0);
#ifdef verifysteps
    cout << "Enter 1 to continue: ";
    cin >> option;
#endif
#endif //bypasstransfer
  } while (pM->keeprunning);

}



void KitTrayAssembleThread(void *param)
{
  cout << "Starting Kit Tray Assembly Thread." << endl;

  passMe *pM = (passMe*)param;
  CrpiRobot<CrpiAbb> abb_r("abb_irb14000_right.xml");
  CrpiRobot<CrpiAbb> abb_l("abb_irb14000_left.xml");
  crpi_timer timer;
  string str;
  char type;
  bool okay = true;

  vector<robotAxes> joints;
  robotAxes jtemp, curjoints;
  joints.resize(pM->kit_tray_assemble_j_labels.size());

  vector<robotPose> poses;
  robotPose ctemp, ctar, curpose;
  poses.resize(pM->kit_tray_assemble_c_labels.size());
  
  int index;

  ifstream in("poses_kit_tray_assemble.dat");
  //! Read poses from disk
  while (in >> str)
  {
    in >> type;
    if (type == 'c')
    {
      //! Cartesian pose
      in >> ctemp.x >> ctemp.y >> ctemp.z >> ctemp.xrot >> ctemp.yrot >> ctemp.zrot >> ctemp.status >> ctemp.turns;
      index = getLabelIndex(pM->kit_tray_assemble_c_labels, str.c_str());
      if (index > 0)
      {
        poses.at(index) = ctemp;
      }
      else
      {
        //! Error:  Bad value
      }
    }
    else if (type == 'j')
    {
      in >> jtemp.axis.at(0) >> jtemp.axis.at(1) >> jtemp.axis.at(2) >> jtemp.axis.at(3) >> jtemp.axis.at(4) >> jtemp.axis.at(5) >> jtemp.axis.at(6);
      index = getLabelIndex(pM->kit_tray_assemble_j_labels, str.c_str());
      if (index > 0)
      {
        joints.at(index) = jtemp;
      }
      else
      {
        //! Error:  Bad value
      }
    }
    else
    {
      //! Error:  Bad type, likely corrupted file.
      okay = false;
    }
  }
  in.close();

  int i1 = getLabelIndex(pM->kit_tray_assemble_c_labels, "matrix_1");
  if (i1 < 0)
  {
    cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
    return;
  }
  int i2 = getLabelIndex(pM->kit_tray_assemble_c_labels, "matrix_2");
  if (i2 < 0)
  {
    cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
    return;
  }
  int i3 = getLabelIndex(pM->kit_tray_assemble_c_labels, "matrix_3");
  if (i3 < 0)
  {
    cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
    return;
  }
  

  vector<robotPose> cap_matrix;
  robotPose p1, p2, p3, d31, d21, pt;
  p1 = poses.at(i1);
  p2 = poses.at(i2);
  p3 = poses.at(i3);
  d31 = (p3 - p1) / 4.0;
  d21 = (p2 - p1) / 3.0;
  unsigned int matrixcounter = 0;

  cap_matrix.clear();
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 5; ++j)
    {
      pt = p1 + (d31 * j) + (d21 * i);
      cout << endl;
      cap_matrix.push_back(pt);
    }
  }

  //! Connect to central server
  while (pM->keeprunning && okay)
  {
    //! Loop:
    //!  Move to stow positions...
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_r_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    if (abb_r.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      abb_r.GetRobotAxes(&curjoints);
    }
    abb_r.SetTool(0.0f);
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_l_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.MoveToAxisTarget(joints.at(index));
    if (abb_l.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      abb_l.GetRobotAxes(&curjoints);
    }
    abb_l.SetTool(0.0f);

    //!   Check for empty part tray
    //! TODO
    Sleep(1000);

    //!   Get cylinder
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_get");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_l.SetTool(1.0f);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_remove");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }

    //!   Place part #1
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_l.SetTool(0.0f);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cylinder_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }

    //!   Get first cap
    abb_r.FromWorld(&cap_matrix.at(matrixcounter), &pt);
    ctar = pt;
    ctar.z += 30.0f;
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    if (abb_r.MoveStraightTo(pt) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_r.SetTool(1.0f);
    Sleep(500);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    matrixcounter++;
    if (matrixcounter >= 20)
    {
      matrixcounter = 0;
    }

    //!   Place cap
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_r.SetTool(0.0f);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }

    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_r_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.MoveToAxisTarget(joints.at(index));
    if (abb_r.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      abb_r.GetRobotAxes(&curjoints);
    }

    //!   Get rod
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_get");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_l.SetTool(1.0f);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_remove");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }

    //! Hand over rod
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_hand");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }

    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_take_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_r.SetTool(0.0f);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_take");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_r.SetTool(1.0f);
    Sleep(500);
    abb_l.SetTool(0.0f);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }

    //! TODO:  MOVE ABB_L to STOW
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "abb_l_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.MoveToAxisTarget(joints.at(index));
    if (abb_l.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      abb_l.GetRobotAxes(&curjoints);
    }

    //!   Place rod
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_r.SetTool(0.0f);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "rod_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }

    //!   Get second cap
    abb_r.FromWorld(&cap_matrix.at(matrixcounter), &pt);
    ctar = pt;
    ctar.z += 30.0f;
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    if (abb_r.MoveStraightTo(pt) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_r.SetTool(1.0f);
    Sleep(500);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    matrixcounter++;
    if (matrixcounter >= 20)
    {
      matrixcounter = 0;
    }


    //!   Place cap #2
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }
    Sleep(500);
    abb_r.SetTool(0.0f);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "cap_put_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_r.FromWorld(&poses.at(index), &ctar);
    if (abb_r.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_r.GetRobotPose(&curpose);
    }

    //!   Push button #1
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "button_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    abb_l.SetTool(1.0f);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "button_push");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }

    //!   Wait for tray removal
    //!   TODO

    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "button_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Assembly." << endl;
      return;
    }
    abb_l.FromWorld(&poses.at(index), &ctar);
    if (abb_l.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      abb_l.GetRobotPose(&curpose);
    }
    
    //! End loop
  }

  return;
}


void KitTrayTransferThread(void *param)
{
  cout << "Starting Kit Tray Transfer Thread." << endl;

  passMe *pM = (passMe*)param;
  CrpiRobot<CrpiUniversal> ur5("universal_ur5.xml");
  CrpiRobot<CrpiUniversal> ur10_left("universal_ur10_left.xml");

  crpi_timer timer;
  string str;
  int index;
  char type;
  bool okay = true;
  vector<robotAxes> joints;
  robotAxes jtemp, curjoints;
  joints.resize(pM->kit_tray_transfer_j_labels.size());

  vector<robotPose> poses;
  robotPose ctemp, ctar, curpose;
  poses.resize(pM->kit_tray_transfer_c_labels.size());

  ifstream in("poses_kit_tray_transfer.dat");
  //! Read poses from disk
  while (in >> str)
  {
    in >> type;
    if (type == 'c')
    {
      //! Cartesian pose
      in >> ctemp.x >> ctemp.y >> ctemp.z >> ctemp.xrot >> ctemp.yrot >> ctemp.zrot >> ctemp.status >> ctemp.turns;
      index = getLabelIndex(pM->kit_tray_transfer_c_labels, str.c_str());
      if (index > 0)
      {
        poses.at(index) = ctemp;
      }
      else
      {
        //! Error:  Bad value
      }
    }
    else if (type == 'j')
    {
      in >> jtemp.axis.at(0) >> jtemp.axis.at(1) >> jtemp.axis.at(2) >> jtemp.axis.at(3) >> jtemp.axis.at(4) >> jtemp.axis.at(5) >> jtemp.axis.at(6);
      index = getLabelIndex(pM->kit_tray_transfer_j_labels, str.c_str());
      if (index > 0)
      {
        joints.at(index) = jtemp;
      }
      else
      {
        //! Error:  Bad value
      }
    }
    else
    {
      //! Error:  Bad type, likely corrupted file.
      okay = false;
    }
  }
  in.close();

  while (pM->keeprunning && okay)
  {
    //! Loop:
    //!  Move to stow positions...
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur5_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur5.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      ur5.GetRobotAxes(&curjoints);
    }
    ur5.SetRobotDO(0, 0);
    index = getLabelIndex(pM->kit_tray_assemble_j_labels, "ur10_left_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.MoveToAxisTarget(joints.at(index));
    if (ur10_left.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      ur10_left.GetRobotAxes(&curjoints);
    }
    ur10_left.SetRobotDO(0, 0);

    //! Robot #2:
    //!   Wait for button #1 pushed
    //!   TODO

    //!   Get filled part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_get_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 1);;
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_get_hover_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }

    //!   Stack filled part tray
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_hover_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 0);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_hover_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }

    //TODO:  Assembly differs at this point



    //!   Take empty part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_empty_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_get_empty_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 1);;
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_get_hover_empty_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }

    //!   Place empty part tray
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_hover_empty_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_empty_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 0);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_empty_filled_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }

    //!   Go to store
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur5_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur5.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      ur5.GetRobotAxes(&curjoints);
    }
    ur5.SetRobotDO(0, 0);

    //! Robot #3:
    //!   Get filled part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_filled_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_get_filled_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_left.SetRobotDO(0, 1);;
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_get_hover_filled_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }

    //!   Move to hand off position
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_hand_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_hand_grasp_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }


    //!   Send signal
    //! TODO
    //!   Wait for signal
    //! TODO


    //!   Release part tray
    Sleep(500);
    ur10_left.SetRobotDO(0, 0);;
    Sleep(500);

    //!   Move to hover
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_hand_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }

    //!   Send signal
    //!   TODO
    //!   Wait for signal
    //!   TODO

    //!   Move to hand off position
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_hand_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_hand_grasp_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }

    //!   Wait for signal
    //!   TODO

    //!   Grab part tray
    Sleep(500);
    ur10_left.SetRobotDO(0, 1);
    Sleep(500);

    //!   Send signal
    //!   TODO

    //!   Place completed part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_hover_complete_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_complete_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_left.SetRobotDO(0, 0);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_hover_complete_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur10_left.FromWorld(&poses.at(index), &ctar);
    if (ur10_left.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_left.GetRobotPose(&curpose);
    }

    //!   Go to store
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur10_left_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur10_left.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      ur10_left.GetRobotAxes(&curjoints);
    }
    ur10_left.SetRobotDO(0, 0);

    //! Robot #2:
    //!   Get completed part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_get_hover_complete_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_get_complete_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 1);;
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_get_hover_complete_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }

    //!   Place completed part tray
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "tray_put_hover_complete_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_complete_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 0);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "tray_put_hover_complete_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }

    //!   Remove completed part
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "part_get_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "part_get_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 1);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "part_get_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }

    //!   Place completed part
    index = getLabelIndex(pM->kit_tray_transfer_c_labels, "part_put_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "part_put_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur5.SetRobotDO(0, 0);
    Sleep(500);
    index = getLabelIndex(pM->kit_tray_assemble_c_labels, "part_put_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    ur5.FromWorld(&poses.at(index), &ctar);
    if (ur5.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur5.GetRobotPose(&curpose);
    }

    //!   Go to store
    index = getLabelIndex(pM->kit_tray_transfer_j_labels, "ur5_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Kit Tray Transfer." << endl;
      return;
    }
    if (ur5.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      ur5.GetRobotAxes(&curjoints);
    }
    ur5.SetRobotDO(0, 0);
  }
  return;
}


void AssembleThread(void *param)
{
  cout << "Starting Kit Tray Transfer Thread." << endl;

  passMe *pM = (passMe*)param;
  CrpiRobot<CrpiUniversal> ur10_right("universal_ur10_right.xml");
  CrpiRobot<CrpiKukaLWR> kuka("kuka_lwr.xml");

  crpi_timer timer;
  string str;
  int index;
  char type;
  bool okay = true;
  vector<robotAxes> joints;
  robotAxes jtemp, curjoints;
  joints.resize(pM->kit_tray_transfer_j_labels.size());

  vector<robotPose> poses;
  robotPose ctemp, ctar, curpose;
  poses.resize(pM->kit_tray_transfer_c_labels.size());

  ifstream in("poses_kit_tray_transfer.dat");
  //! Read poses from disk
  while (in >> str)
  {
    in >> type;
    if (type == 'c')
    {
      //! Cartesian pose
      in >> ctemp.x >> ctemp.y >> ctemp.z >> ctemp.xrot >> ctemp.yrot >> ctemp.zrot >> ctemp.status >> ctemp.turns;
      index = getLabelIndex(pM->assemble_c_labels, str.c_str());
      if (index > 0)
      {
        poses.at(index) = ctemp;
      }
      else
      {
        //! Error:  Bad value
      }
    }
    else if (type == 'j')
    {
      in >> jtemp.axis.at(0) >> jtemp.axis.at(1) >> jtemp.axis.at(2) >> jtemp.axis.at(3) >> jtemp.axis.at(4) >> jtemp.axis.at(5) >> jtemp.axis.at(6);
      index = getLabelIndex(pM->assemble_j_labels, str.c_str());
      if (index > 0)
      {
        joints.at(index) = jtemp;
      }
      else
      {
        //! Error:  Bad value
      }
    }
    else
    {
      //! Error:  Bad type, likely corrupted file.
      okay = false;
    }
  }
  in.close();

  while (pM->keeprunning && okay)
  {
    //! Loop:
    //!  Move to stow positions...
    index = getLabelIndex(pM->assemble_j_labels, "ur10_right_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    if (ur10_right.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      ur10_right.GetRobotAxes(&curjoints);
    }
    ur10_right.SetRobotDO(0, 0);
    index = getLabelIndex(pM->assemble_j_labels, "kuka_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.MoveToAxisTarget(joints.at(index));
    if (kuka.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      kuka.GetRobotAxes(&curjoints);
    }
    kuka.SetTool(0.0);

    //! Robot #4:
    //!   Move to hover #2
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //!   Wait for signal
    //!   TODO

    //!   Move to hand off position
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_grasp");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //!   Grasp part tray
    Sleep(500);
    ur10_right.SetRobotDO(0, 1);;
    Sleep(500);

    //!   Send Signal
    //!   TODO

    //!   Remove Tray
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //!   Place filled tray
    index = getLabelIndex(pM->assemble_c_labels, "tray_place_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "tray_place");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_right.SetRobotDO(0, 0);
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "tray_place_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    /*
    //!   Move to stow
    index = getLabelIndex(pM->assemble_j_labels, "ur10_right_store");
    if (index < 0)
    {
    cout << "Bad ID.  Exiting Assembly." << endl;
    return;
    }
    if (ur10_right.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      ur10_right.GetRobotAxes(&curjoints);
    }
    */

    //! Robot #4
    //!   Take part #1
    index = getLabelIndex(pM->assemble_c_labels, "part1_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part1_get");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_right.SetRobotDO(0, 1);
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "part1_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //!   Move to assembly pose 1
    index = getLabelIndex(pM->assemble_c_labels, "part1_fixture_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //! Robot #5
    //!   Take part #3_1
    index = getLabelIndex(pM->assemble_c_labels, "part3_get_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part3_get_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    Sleep(500);
    kuka.SetTool(1.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "part3_get_hover_1");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }

    //!   Insert part
    index = getLabelIndex(pM->assemble_c_labels, "part3_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part3_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    Sleep(500);
    kuka.SetTool(0.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "part3_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }

    //!   Tuck part
    kuka.SetTool(1.0f);
    index = getLabelIndex(pM->assemble_c_labels, "part3_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part3_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    kuka.SetTool(0.0f);;

    //! Robot #4
    //!   Move to assembly pose 2
    index = getLabelIndex(pM->assemble_c_labels, "part1_fixture_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //! Robot #5
    //!   Take part #2
    index = getLabelIndex(pM->assemble_c_labels, "part2_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part2_get");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    Sleep(500);
    kuka.SetTool(1.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "part2_get_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }

    //!   Insert part
    index = getLabelIndex(pM->assemble_c_labels, "part2_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part2_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    Sleep(500);
    kuka.SetTool(0.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "part2_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }

    //!   Take part #3_2
    index = getLabelIndex(pM->assemble_c_labels, "part3_get_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part3_get_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    Sleep(500);
    kuka.SetTool(1.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "part3_get_hover_2");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }

    //!   Insert part
    index = getLabelIndex(pM->assemble_c_labels, "part3_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part3_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    Sleep(500);
    kuka.SetTool(0.0f);;
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "part3_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }

    //!   Tuck part
    kuka.SetTool(1.0f);
    index = getLabelIndex(pM->assemble_c_labels, "part3_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part3_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.FromWorld(&poses.at(index), &ctar);
    if (kuka.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      kuka.GetRobotPose(&curpose);
    }
    kuka.SetTool(0.0f);

    //!   Move to store pose
    index = getLabelIndex(pM->assemble_j_labels, "kuka_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    kuka.MoveToAxisTarget(joints.at(index));
    if (kuka.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      kuka.GetRobotAxes(&curjoints);
    }
    kuka.SetTool(0.0);

    //! Robot #4
    //!   Put completed part in tray
    index = getLabelIndex(pM->assemble_c_labels, "part1_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "part1_put");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_right.SetRobotDO(0, 0);
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "part1_put_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //!   Take part tray
    index = getLabelIndex(pM->assemble_c_labels, "tray_place_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    index = getLabelIndex(pM->assemble_c_labels, "tray_place");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }
    Sleep(500);
    ur10_right.SetRobotDO(0, 1);
    Sleep(500);
    index = getLabelIndex(pM->assemble_c_labels, "tray_place_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //!   Move to hand off position
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_grasp");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //!   Send signal
    //!   TODO

    //!   Wait for signal
    //!   WAITI

    //!   Release part tray
    ur10_right.SetRobotDO(0, 0);

    //!   Move to hover
    index = getLabelIndex(pM->assemble_c_labels, "tray_hand_hover");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    ur10_right.FromWorld(&poses.at(index), &ctar);
    if (ur10_right.MoveStraightTo(ctar) == CANON_SUCCESS)
    {
      ur10_right.GetRobotPose(&curpose);
    }

    //!   Send signal
    //!   TODO

    //!   Move to stow
    index = getLabelIndex(pM->assemble_j_labels, "ur10_right_store");
    if (index < 0)
    {
      cout << "Bad ID.  Exiting Assembly." << endl;
      return;
    }
    if (ur10_right.MoveToAxisTarget(joints.at(index)) == CANON_SUCCESS)
    {
      ur10_right.GetRobotAxes(&curjoints);
    }
    ur10_right.SetRobotDO(0, 0);
  }
  return;
}



int readInt(char *buffer, int &index, bool little)
{
  static char ucval[sizeof(int)];
  static int x;
  static int returnMe;

  for (x = 0; x < sizeof(int); ++x)
  {
    ucval[x] = (little ? buffer[index + (sizeof(int) - (x + 1))] : buffer[index + x]);
  }
  index += sizeof(int);
  memcpy(&returnMe, ucval, sizeof(int));
  return returnMe;
}


double readDouble(char *buffer, int &index, bool little)
{
  static char ucval[sizeof(double)];
  static int x;
  static double returnMe;

  for (x = 0; x < sizeof(double); ++x)
  {
    ucval[x] = (little ? buffer[index + (sizeof(double) - (x + 1))] : buffer[index + x]);
  }
  index += sizeof(double);
  memcpy(&returnMe, ucval, sizeof(double));
  return returnMe;
}


long readLong(char *buffer, int &index, bool little)
{
  static char ucval[sizeof(double)];
  static int x;
  static long returnMe;

  for (x = 0; x < sizeof(long); ++x)
  {
    ucval[x] = (little ? buffer[index + (sizeof(long) - (x + 1))] : buffer[index + x]);
  }
  index += sizeof(long);
  memcpy(&returnMe, ucval, sizeof(long));
  return returnMe;
}

bool parseFeedback(int bytes, char *buffer, robotPose &pose, robotAxes &axes, robotIO &io, robotPose &forces, robotPose &speeds)
{
  double dval;
  int ival;
  bool little = false;

  //! JAM:  Note that the UR sends feedback in big endian format.  Test for endianess.
  int i = 0x01234567;
  int j;
  char* c = (char*)&i;
  little = (c[0] == 'g');
  //for (j = 0; j < sizeof(int); j++)
  //{
  //  printf(" %.2x", c[j]);
  //}
  //printf("\n");

  //! If big endian:  01 23 45 67
  //! If little endian: 67 45 23 01

  //cout << bytes << endl;
  if (bytes != 1044 && bytes != 812)
  {
    //! unknown byte length
    return false;
  }

  int index = 0;
  //! Number of bytes sent
  ival = readInt(buffer, index, little);

  //! Time
  dval = readDouble(buffer, index, little);

  //! Target joint positions
  for (j = 0; j < 6; ++j)
  {
    //dval = readDouble(buffer, index, little);
    axes.axis.at(j) = readDouble(buffer, index, little);
  }

  //! Target joint velocities
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Target joint accelerations
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Target joint current
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Target joint moments
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Actual joint positions
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Actual joint velocities
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Actual joint currents
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Joint control currents
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Actual Cartesian coordinates of TCP
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Actual speed of TCP in Cartesian space
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Forces at TCP
  /*
  for (j = 0; j < 6; ++j)
  {
  dval = readDouble(buffer, index, little);
  }
  */
  forces.x = readDouble(buffer, index, little);
  forces.y = readDouble(buffer, index, little);
  forces.z = readDouble(buffer, index, little);
  forces.xrot = readDouble(buffer, index, little);
  forces.yrot = readDouble(buffer, index, little);
  forces.zrot = readDouble(buffer, index, little);

  //! TCP target coordinates
  /*
  for (j = 0; j < 6; ++j)
  {
  dval = readDouble(buffer, index, little);
  }
  */
  pose.x = readDouble(buffer, index, little);
  pose.y = readDouble(buffer, index, little);
  pose.z = readDouble(buffer, index, little);
  pose.xrot = readDouble(buffer, index, little);
  pose.yrot = readDouble(buffer, index, little);
  pose.zrot = readDouble(buffer, index, little);
  //cout << "raw: " << pose.x << " " << pose.y << " " << pose.z << " " << pose.xrot << " " << pose.yrot << " " << pose.zrot << endl;

  //! TCP target speed
  /*
  for (j = 0; j < 6; ++j)
  {
  dval = readDouble(buffer, index, little);
  }
  */
  speeds.x = readDouble(buffer, index, little);
  speeds.y = readDouble(buffer, index, little);
  speeds.z = readDouble(buffer, index, little);
  speeds.xrot = readDouble(buffer, index, little);
  speeds.yrot = readDouble(buffer, index, little);
  speeds.zrot = readDouble(buffer, index, little);

  //! Digital input states
  ival = (int)(readDouble(buffer, index, little));
  for (j = (CRPI_IO_MAX - 1); j >= 0; j--)
  {
    io.dio[j] = (ival >= (1 << j));
    if (io.dio[j])
    {
      ival -= (int)(1 << j);
    }
  }

  //! Motor temperatures
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! Controller timer
  dval = readDouble(buffer, index, little);

  //! Test value
  dval = readDouble(buffer, index, little);

  //! Robot mode
  dval = readDouble(buffer, index, little);

  //! Joint modes
  for (j = 0; j < 6; ++j)
  {
    dval = readDouble(buffer, index, little);
  }

  //! 1044 bytes beyond this point
  if (ival > 812)
  {
    //! Safety mode
    dval = readDouble(buffer, index, little);

    //! Reserved
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Tool accelerometer
    for (j = 0; j < 3; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Reserved
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }

    //! Speed scaling
    dval = readDouble(buffer, index, little);

    //! Linear momentum norm
    dval = readDouble(buffer, index, little);

    //! Reserved
    dval = readDouble(buffer, index, little);

    //! Reserved
    dval = readDouble(buffer, index, little);

    //! Main voltage
    dval = readDouble(buffer, index, little);

    //! Robot voltage
    dval = readDouble(buffer, index, little);

    //! Robot current
    dval = readDouble(buffer, index, little);

    //! Joint voltages
    for (j = 0; j < 6; ++j)
    {
      dval = readDouble(buffer, index, little);
    }
  } // if (ival > 812)

  return true;
}



void FeedbackThread(void *param)
{
  cout << "Robot feedback thread." << endl;

  passMe *pM = (passMe*)param;

  ifstream settings("feedback.dat");
  string robt, ipt;
  int portt;
  vector<string> robot, ip;
  vector<int> port;

  while (settings >> robt)
  {
    settings >> ipt >> portt;
    robot.push_back(robt);
    ip.push_back(ipt);
    port.push_back(portt);

    /*
      ulapi_integer client = ulapi_socket_get_client_id (30003, uH->params.tcp_ip_addr);
      ulapi_socket_set_nonblocking(client);

      if (client > 0)
      {
        //! Read feedback from robot
        get = ulapi_socket_read(client, buffer, 1044);
        ulapi_socket_close(client);

        //! Parse feedback from robot
        if (parseFeedback(get, buffer, pose, axes, io, force, speed))
        {
        }
      }
    */
  }


  while (pM->keeprunning)
  {
    //! If not connected, connect to robot
  }

  robot.clear();
  ip.clear();
  port.clear();
  return;
}


void RegisterRobots()
{
  int val, x, offset;
  matrix trans(4, 4);
  CrpiRobot<CrpiAbb> *abb_r, *abb_l;
  CrpiRobot<CrpiUniversal> *univ, *univ1, *univ2;
  CrpiRobot<CrpiKukaLWR> *kuka;
  robotPose poseIn;
  vector<robotPose> poses;
  robotPose R1, R2, R3, R4, R5;
  ofstream out;
  ifstream in;
  string abb_left_reg = "abb_left_reg.dat",
         abb_right_reg = "abb_right_reg.dat",
         ur5_reg = "ur5_reg.dat",
         ur10_left_reg = "ur10_left_reg.dat",
         ur10_right_reg = "ur10_right_reg.dat",
         kuka_reg = "kuka_reg.dat",
         world_reg = "world.dat";

  do
  {
    cout << "Register which robot?" << endl;
    cout << "  1) R1 (ABB IRB 14000)" << endl;
    cout << "  2) R2 (UR UR5)" << endl;
    cout << "  3) R3 (UR UR10 Left)" << endl;
    cout << "  4) R4 (UR UR10 Right)" << endl;
    cout << "  5) R5 (KUKA LWR 4+)" << endl;
    cout << "  0) Finished" << endl;
    cin >> val;
    if (val < 1 || val > 5)
    {
      continue;
    }

    switch (val)
    {
    case 1:
      //! ABB IRB 14000
      
      abb_r = new CrpiRobot<CrpiAbb>("abb_irb14000_right.xml", true);
      abb_l = new CrpiRobot<CrpiAbb>("abb_irb14000_left.xml", true);
      abb_r->SetAngleUnits("degree");
      abb_r->SetLengthUnits("mm");
      abb_l->SetAngleUnits("degree");
      abb_l->SetLengthUnits("mm");
      abb_r->Couple("Yumi_Parallel");
      abb_r->SetRelativeSpeed(0.5);
      abb_l->Couple("Yumi_Parallel");
      abb_l->SetRelativeSpeed(0.5);

      cout << "Registering IRB 14000 to Universal UR5" << endl;
      cout << "ABB IRB 14000 Left Arm:" << endl;
      out.open(abb_left_reg.c_str());
      for (x = 0; x < 5; ++x)
      {
        cout << "Move to position " << (x + 1) << " and enter '1':";
        cin >> val;
        abb_l->GetRobotPose(&poseIn);
        out << poseIn.x << " " << poseIn.y << " " << poseIn.z << " "
            << poseIn.xrot << " " << poseIn.yrot << " " << poseIn.zrot << " "
            << poseIn.status << " " << poseIn.turns << endl;
      }
      out.close();

      cout << "ABB IRB 14000 Right Arm:" << endl;
      out.open(abb_right_reg.c_str());
      for (x = 0; x < 5; ++x)
      {
        cout << "Move to position " << (x + 1) << " and enter '1':";
        cin >> val;
        abb_r->GetRobotPose(&poseIn);
        out << poseIn.x << " " << poseIn.y << " " << poseIn.z << " "
            << poseIn.xrot << " " << poseIn.yrot << " " << poseIn.zrot << " "
            << poseIn.status << " " << poseIn.turns << endl;
      }
      out.close();

      //! Garbage collection
      delete abb_r;
      delete abb_l;
      cout << "Finished capturing data for ABB IRB 14000." << endl;
      break;
    case 2:
      //! Universal Robots UR5
      
      in.open(ur5_reg.c_str());
      poses.clear();
      for (x = 0; x < 10; ++x)
      {
        in >> poseIn.x >> poseIn.y >> poseIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
        poses.push_back(poseIn);
      }
      in.close();

      cout << "Universal Robots UR5:" << endl;
      do
      {
        cout << "Register to which frame?" << endl;
        cout << "  1) ABB IRB 14000" << endl;
        cout << "  2) Table 1" << endl;
        cout << "Selection: ";
        cin >> val;
      } while (val < 0 || val > 2);

      offset = (val - 1) * 5;

      univ = new CrpiRobot<CrpiUniversal>("universal_ur5.xml", true);
      univ->SetAngleUnits("degree");
      univ->SetLengthUnits("mm");
      univ->Couple("flange_ring");
      univ->SetRelativeSpeed(0.5);

      out.open(ur5_reg.c_str());
      for (x = 0; x < 5; ++x)
      {
        cout << "Move to position " << (x + 1) << " and enter '1':";
        cin >> val;
        univ->GetRobotPose(&poseIn);
        poses.at(x + val) = poseIn;
      }
      for (x = 0; x < 10; ++x)
      {
        out << poses.at(x).x << " " << poses.at(x).y << " " << poses.at(x).z << " "
            << poses.at(x).xrot << " " << poses.at(x).yrot << " " << poses.at(x).zrot << " "
            << poses.at(x).status << " " << poses.at(x).turns << endl;
      }
      out.close();
      //! Garbage collection
      delete univ;
      cout << "Finished capturing data for Universal Robots UR5." << endl;
      break;
    case 3:
      //! Universal Robots UR10 Left

      in.open(ur10_left_reg.c_str());
      poses.clear();
      for (x = 0; x < 10; ++x)
      {
        in >> poseIn.x >> poseIn.y >> poseIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
        poses.push_back(poseIn);
      }
      in.close();

      cout << "Universal Robots UR10 Left:" << endl;
      do
      {
        cout << "Register to which frame?" << endl;
        cout << "  1) Table 1" << endl;
        cout << "  2) Universal Robots UR10 Right" << endl;
        cout << "Selection: ";
        cin >> val;
      } while (val < 0 || val > 2);

      offset = (val - 1) * 5;

      univ = new CrpiRobot<CrpiUniversal>("universal_ur10_left.xml", true);
      univ->SetAngleUnits("degree");
      univ->SetLengthUnits("mm");
      univ->Couple("flange_ring");
      univ->SetRelativeSpeed(0.5);

      out.open(ur10_left_reg.c_str());
      for (x = 0; x < 5; ++x)
      {
        cout << "Move to position " << (x + 1) << " and enter '1':";
        cin >> val;
        univ->GetRobotPose(&poseIn);
        poses.at(val + x) = poseIn;
      }
      for (x = 0; x < 10; ++x)
      {
        out << poses.at(x).x << " " << poses.at(x).y << " " << poses.at(x).z << " "
            << poses.at(x).xrot << " " << poses.at(x).yrot << " " << poses.at(x).zrot << " "
            << poses.at(x).status << " " << poses.at(x).turns << endl;
      }
      out.close();
      //! Garbage collection
      delete univ;
      cout << "Finished capturing data for Universal Robots UR10 Left." << endl;
      break;
    case 4:
      //! Universal Robots UR10 Right
      in.open(ur10_right_reg.c_str());
      poses.clear();
      for (x = 0; x < 10; ++x)
      {
        in >> poseIn.x >> poseIn.y >> poseIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
        poses.push_back(poseIn);
      }
      in.close();

      cout << "Universal Robots UR10 Right:" << endl;
      do
      {
        cout << "Register to which frame?" << endl;
        cout << "  1) Universal Robots UR10 Left" << endl;
        cout << "  2) Table 2" << endl;
        cout << "Selection: ";
        cin >> val;
      } while (val < 0 || val > 2);

      offset = (val - 1) * 5;

      univ = new CrpiRobot<CrpiUniversal>("universal_ur10_right.xml", true);
      univ->SetAngleUnits("degree");
      univ->SetLengthUnits("mm");
      univ->Couple("flange_ring");
      univ->SetRelativeSpeed(0.5);

      out.open(ur10_right_reg.c_str());
      for (x = 0; x < 5; ++x)
      {
        cout << "Move to position " << (x + 1) << " and enter '1':";
        cin >> val;
        univ->GetRobotPose(&poseIn);
        poses.at(val + x) = poseIn;
      }
      for (x = 0; x < 10; ++x)
      {
        out << poses.at(x).x << " " << poses.at(x).y << " " << poses.at(x).z << " "
            << poses.at(x).xrot << " " << poses.at(x).yrot << " " << poses.at(x).zrot << " "
            << poses.at(x).status << " " << poses.at(x).turns << endl;
      }
      out.close();
      //! Garbage collection
      delete univ;
      cout << "Finished capturing data for Universal Robots UR10 Right." << endl;
      break;
    case 5:
      //! KUKA LWR 4+

      kuka = new CrpiRobot<CrpiKukaLWR>("kuka_lwr.xml", true);
      kuka->SetAngleUnits("degree");
      kuka->SetLengthUnits("mm");
      kuka->Couple("flange_ring");
      kuka->SetRelativeSpeed(0.5);

      cout << "Registering KUKA LWR 4+ to Table 2" << endl;
      cout << "KUKA LWR 4+:" << endl;
      out.open(kuka_reg.c_str());
      for (x = 0; x < 5; ++x)
      {
        kuka->GetRobotPose(&poseIn);
        kuka->MoveAttractor(poseIn);
        cout << "Move to position " << (x + 1) << " and enter '1':";
        cin >> val;
        kuka->GetRobotPose(&poseIn);
        out << poseIn.x << " " << poseIn.y << " " << poseIn.z << " "
            << poseIn.xrot << " " << poseIn.yrot << " " << poseIn.zrot << " "
            << poseIn.status << " " << poseIn.turns << endl;
      }
      cout << "Move robot to safe stow position and enter '1':";
      cin >> val;
      kuka->GetRobotPose(&poseIn);
      kuka->MoveTo(poseIn);

      out.close();
      //! Garbage collection
      delete kuka;
      cout << "Finished capturing data for KUKA LWR 4+." << endl;
      break;
    default:
      //! Shouldn't be here.
      break;
    }
  } while (val > 0);

  //! Update transformations
  cout << "Updating transformations..." << endl;
  vector<point> abb_left_poses,
                abb_right_poses,
                ur5_poses_1, ur5_poses_2,
                ur10_left_poses_1, ur10_left_poses_2,
                ur10_right_poses_1, ur10_right_poses_2,
                kuka_poses,
                world_poses;
  point pointIn;

  cout << "  Reading all registration data..." << endl;
  cout << "    ABB IRB 14000 Left..." << endl;
  in.open(abb_left_reg.c_str());
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    abb_left_poses.push_back(pointIn);
  }
  in.close();
  cout << "    ABB IRB 14000 Right..." << endl;
  in.open(abb_right_reg.c_str());
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    abb_right_poses.push_back(pointIn);
  }
  in.close();
  cout << "    Universal UR5..." << endl;
  in.open(ur5_reg.c_str());
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    ur5_poses_1.push_back(pointIn);
  }
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    ur5_poses_2.push_back(pointIn);
  }
  in.close();
  cout << "    Universal UR10 Left..." << endl;
  in.open(ur10_left_reg.c_str());
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    ur10_left_poses_1.push_back(pointIn);
  }
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    ur10_left_poses_2.push_back(pointIn);
  }
  in.close();
  cout << "    Universal UR10 Right..." << endl;
  in.open(ur10_right_reg.c_str());
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    ur10_right_poses_1.push_back(pointIn);
  }
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    ur10_right_poses_2.push_back(pointIn);
  }
  in.close();
  cout << "    KUKA LWR 4+..." << endl;
  in.open(kuka_reg.c_str());
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z >> poseIn.xrot >> poseIn.yrot >> poseIn.zrot >> poseIn.status >> poseIn.turns;
    kuka_poses.push_back(pointIn);
  }
  in.close();
  cout << "    World..." << endl;
  in.open(world_reg.c_str());
  for (x = 0; x < 5; ++x)
  {
    in >> pointIn.x >> pointIn.y >> pointIn.z;
    world_poses.push_back(pointIn);
  }
  in.close();
  cout << "  Done." << endl;

  vector<matrix> abbl_2_abbr,
                 ur5_2_abbl,
                 ur10l_2_ur5,
                 ur10r_2_ur10l,
                 world_2_ur10r,
                 world_2_kuka;
  matrix world_2_robot(4, 4);
  matrix robot_2_world(4, 4);
  
  unsigned int abbl2abbrchoice, ur52abblchoice, ur10l2ur5choice, ur10r2ur10lchoice, ur10rworldchoice, lwrworldchoice;
  bool state;
  vector<point> kernels;
  cout << "  Generating registrations..." << endl;
  cout << "    ABB IRB 14000 Left to ABB IRB 14000 Right..." << endl;
  state = reg2targetML(abb_left_poses, abb_right_poses, 2, kernels, abbl_2_abbr);
  cout << (state ? "okay" : "nope") << endl;
  cout << "Option 0: " << endl;
  abbl_2_abbr.at(0).print();
  cout << "Option 1: " << endl;
  abbl_2_abbr.at(1).print();
  cout << "Use which option? : ";
  cin >> abbl2abbrchoice;
  cout << "    Universal UR5 to ABB IRB 14000 Left..." << endl;
  state &= reg2targetML(ur5_poses_1, abb_left_poses, 2, kernels, ur5_2_abbl);
  cout << "Option 0: " << endl;
  ur5_2_abbl.at(0).print();
  cout << "Option 1: " << endl;
  ur5_2_abbl.at(1).print();
  cout << "Use which option? : ";
  cin >> ur52abblchoice;
  cout << "    Universal UR10 Left to Universal UR5..." << endl;
  state &= reg2targetML(ur10_left_poses_1, ur5_poses_2, 2, kernels, ur10l_2_ur5);
  cout << "Option 0: " << endl;
  ur10l_2_ur5.at(0).print();
  cout << "Option 1: " << endl;
  ur10l_2_ur5.at(1).print();
  cout << "Use which option? : ";
  cin >> ur10l2ur5choice;
  cout << "    Universal UR10 Right to Universal UR10 Left..." << endl;
  state &= reg2targetML(ur10_right_poses_1, ur10_left_poses_2, 2, kernels, ur10r_2_ur10l);
  cout << "Option 0: " << endl;
  ur10r_2_ur10l.at(0).print();
  cout << "Option 1: " << endl;
  ur10r_2_ur10l.at(1).print();
  cout << "Use which option? : ";
  cin >> ur10r2ur10lchoice;
  cout << "    World to Universal UR10 Right..." << endl;
  state &= reg2targetML(world_poses, ur10_right_poses_2, 2, kernels, world_2_ur10r);
  cout << "Option 0: " << endl;
  world_2_ur10r.at(0).print();
  cout << "Option 1: " << endl;
  world_2_ur10r.at(1).print();
  cout << "Use which option? : ";
  cin >> ur10rworldchoice;
  cout << "    World to KUKA LWR 4+..." << endl;
  state &= reg2targetML(world_poses, kuka_poses, 2, kernels, world_2_kuka);
  cout << "Option 0: " << endl;
  world_2_kuka.at(0).print();
  cout << "Option 1: " << endl;
  world_2_kuka.at(1).print();
  cout << "Use which option? : ";
  cin >> lwrworldchoice;
  cout << "  Done." << endl;

  cout << "  Generating transformation from world to..." << endl;
  cout << "    KUKA LWR 4+..." << endl;
  //! World to KUKA LWR:  Already computed
  robot_2_world = world_2_kuka.at(lwrworldchoice).inv();
  cout << "      Create robot..." << endl;
  kuka = new CrpiRobot<CrpiKukaLWR>("kuka_lwr.xml", true);
  cout << "      Update world..." << endl;
  kuka->UpdateWorldTransform(robot_2_world);
  cout << "      Save configuration..." << endl;
  kuka->SaveConfig("kuka_lwr.xml");
  Sleep(1000);

  cout << "    Universal UR10 Right..." << endl;
  //! World to UR10 Right:  Already computed
  cout << "      Compute transformation..." << endl;
  robot_2_world = world_2_ur10r.at(ur10rworldchoice).inv();
  cout << "      Create robot..." << endl;
  univ = new CrpiRobot<CrpiUniversal>("universal_ur10_right.xml", true);
  cout << "      Update world..." << endl;
  univ->UpdateWorldTransform(robot_2_world);
  cout << "      Save configuration..." << endl;
  univ->SaveConfig("universal_ur10_right.xml");
  Sleep(1000);

  cout << "    Universal UR10 Left..." << endl;
  //! World to UR10 Left:  world_2_ur10r * ur10r_2_ur10l
  cout << "      Compute transformation..." << endl;
  //robot_2_world = world_2_ur10r.at(ur10rworldchoice).inv() * ur10r_2_ur10l.at(ur10r2ur10lchoice).inv();
  robot_2_world = ur10r_2_ur10l.at(ur10r2ur10lchoice).inv() * world_2_ur10r.at(ur10rworldchoice).inv();
  cout << "      Create robot..." << endl;
  univ1 = new CrpiRobot<CrpiUniversal>("universal_ur10_left.xml", true);
  cout << "      Update world..." << endl;
  univ1->UpdateWorldTransform(robot_2_world);
  cout << "      Save configuration..." << endl;
  univ1->SaveConfig("universal_ur10_left.xml");
  Sleep(1000);

  cout << "    Universal UR5..." << endl;
  //! World to UR5:  world_2_ur10r * ur10r_2_ur10l * ur10l_2_ur5
  cout << "      Compute transformation..." << endl;
  //robot_2_world = world_2_ur10r.at(ur10rworldchoice).inv() * ur10r_2_ur10l.at(ur10r2ur10lchoice).inv() * ur10l_2_ur5.at(ur10l2ur5choice).inv();
  robot_2_world = ur10l_2_ur5.at(ur10l2ur5choice).inv() * ur10r_2_ur10l.at(ur10r2ur10lchoice).inv() * world_2_ur10r.at(ur10rworldchoice).inv();
  cout << "      Create robot..." << endl;
  univ2 = new CrpiRobot<CrpiUniversal>("universal_ur5.xml", true);
  cout << "      Update world..." << endl;
  univ2->UpdateWorldTransform(robot_2_world);
  univ2->UpdateSystemTransform("toYuMiL", ur5_2_abbl.at(0));
  cout << "      Save configuration..." << endl;
  univ2->SaveConfig("universal_ur5.xml");
  Sleep(1000);

  cout << "    ABB IRB 14000 Left..." << endl;
  //! World to ABB Left:  world_2_ur10r * ur10r_2_ur10l * ur10l_2_ur5 * ur5_2_abbl
  cout << "      Compute transformation..." << endl;
  //robot_2_world = world_2_ur10r.at(ur10rworldchoice).inv() * ur10r_2_ur10l.at(ur10r2ur10lchoice).inv() * ur10l_2_ur5.at(ur10l2ur5choice).inv() * ur5_2_abbl.at(ur52abblchoice).inv();
  robot_2_world = ur5_2_abbl.at(ur52abblchoice).inv() * ur10l_2_ur5.at(ur10l2ur5choice).inv() * ur10r_2_ur10l.at(ur10r2ur10lchoice).inv() * world_2_ur10r.at(ur10rworldchoice).inv();
  cout << "      Create robot..." << endl;
  abb_l = new CrpiRobot<CrpiAbb>("abb_irb14000_left.xml", true);
  cout << "      Update world..." << endl;
  abb_l->UpdateWorldTransform(robot_2_world);
  abb_l->UpdateSystemTransform("toUR5", ur5_2_abbl.at(0).inv());
  cout << "      Save configuration..." << endl;
  abb_l->SaveConfig("abb_irb14000_left.xml");
  Sleep(1000);

  cout << "    ABB IRB 14000 Right..." << endl;
  //! World to ABB Right:  world_2_ur10r * ur10r_2_ur10l * ur10l_2_ur5 * ur5_2_abbl * abbl_2_abbr
  cout << "      Compute transformation..." << endl;
  //robot_2_world = world_2_ur10r.at(ur10rworldchoice).inv() * ur10r_2_ur10l.at(ur10r2ur10lchoice).inv() * ur10l_2_ur5.at(ur10l2ur5choice).inv() * ur5_2_abbl.at(ur52abblchoice).inv() * abbl_2_abbr.at(abbl2abbrchoice).inv();
  robot_2_world = abbl_2_abbr.at(abbl2abbrchoice).inv() * ur5_2_abbl.at(ur52abblchoice).inv() * ur10l_2_ur5.at(ur10l2ur5choice).inv() * ur10r_2_ur10l.at(ur10r2ur10lchoice).inv() * world_2_ur10r.at(ur10rworldchoice).inv();
  cout << "      Create robot..." << endl;
  abb_r = new CrpiRobot<CrpiAbb>("abb_irb14000_right.xml", true);
  cout << "      Update world..." << endl;
  abb_r->UpdateWorldTransform(robot_2_world);
  cout << "      Save configuration..." << endl;
  abb_r->SaveConfig("abb_irb14000_right.xml");
  Sleep(1000);

  cout << "  Done." << endl;

  cout << "      Destroy robot..." << endl;
  delete kuka;
  cout << "      Destroy robot..." << endl;
  delete univ;
  cout << "      Destroy robot..." << endl;
  delete univ1;
  cout << "      Destroy robot..." << endl;
  delete univ2;
  cout << "      Destroy robot..." << endl;
  delete abb_l;
  cout << "      Destroy robot..." << endl;
  delete abb_r;
  cout << "Completed registration." << endl;
}



void main ()
{
  int i = 0;
  crpi_timer timer;
  passMe handle;
  void *threadtask;
  vector<passMe> handles;
  vector<void*> threadTasks;
  handle.keeprunning = true;

  int option;

  saveandconvert();
  
  //SerialAssembly(&handle);
  return;

  do
  {
    cout << "Select option:  1) Run demonstration 2) Register robots 3) Train robots:  ";
    cin >> option;

    if (option == 1)
    {
      //! Get out of this loop and go assemble something!
      break;
    }

    if (option == 2)
    {
      //! Register robots
      RegisterRobots();
    }
    else if (option == 3)
    {
      //! TODO:  Train robot processes

    }
  } while (true);



#ifdef ASSEMBLE_KIT_TRAY
  //! Start thread

  threadtask = ulapi_task_new();
  //! Start new ABB thread
  ulapi_task_start((ulapi_task_struct*)threadtask, KitTrayAssembleThread, &handle, ulapi_prio_lowest(), 0);
  handles.push_back(handle);
  threadTasks.push_back(threadtask);
#endif

#ifdef TRANSFER_TRAYS
  threadtask = ulapi_task_new();
  //! Start new ABB thread
  ulapi_task_start((ulapi_task_struct*)threadtask, KitTrayTransferThread, &handle, ulapi_prio_lowest(), 0);
  handles.push_back(handle);
  threadTasks.push_back(threadtask);
#endif

#ifdef ASSEMBLE_PARTS
  threadtask = ulapi_task_new();
  //! Start new ABB thread
  ulapi_task_start((ulapi_task_struct*)threadtask, AssembleThread, &handle, ulapi_prio_lowest(), 0);
  handles.push_back(handle);
  threadTasks.push_back(threadtask);
#endif


  cout << "All done" << endl;

}


