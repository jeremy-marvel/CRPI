//Grasp planning with Shuo and Stefano's planner

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_schunk_sdh.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include <math.h>
#include <algorithm>
#include <map>
#include <process.h>

//#define GRASP_PLANNER
#define LARGE_OBJECTS
//#define SMALL_OBJECTS
//#define USE_GRASP_PLANNER

#include "src/ulapi.h"

#pragma warning (disable: 4996)

//#define NOISY
//#define SUPERNOISY
#define ALTTEXT
#define PI 3.14159265
#define REQUEST_MSG_SIZE 8192

using namespace crpi_robot;
using namespace std;

typedef CrpiKukaLWR robType;
//typedef CrpiUniversal robType;

//globals
CrpiRobot<CrpiSchunkSDH> sdh("SDH.xml");
robotAxes hand_joint_pose, hand_joint_approach;
bool grasp_switch = false;

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

void poseHand(CrpiRobot<CrpiSchunkSDH> &sdh,robotAxes & hand_joint_pose,bool grasp)
{
  robotIO dummy;
  //Pregrasp shape
  /*
  vector<double> pregrasp3(7);
  pregrasp3[0] = 60; pregrasp3[1] = pregrasp3[3] = pregrasp3[5] = 0;
  pregrasp3[2] = pregrasp3[4] = pregrasp3[6] = 10;

  vector<double> pregrasp2(7);
  pregrasp2[0] = 90; pregrasp2[3] = -90; pregrasp2[4] = 90;
  pregrasp2[1] = pregrasp2[5] = 0;
  pregrasp2[2] = pregrasp2[6] = 0;

  vector<double> pregrasp_home(7);
  pregrasp_home[0] = 60; pregrasp_home[1] = pregrasp_home[3] = pregrasp_home[5] = -35;
  pregrasp_home[2] = pregrasp_home[4] = pregrasp_home[6] = 90;
  */

  //Other initializations
  sdh.MoveToAxisTarget(hand_joint_pose);

  /*
  int param;
  if (hand_pose == "ready") {param = 0;}
  else if (hand_pose == "tripod") {param = 3;}
  else if (hand_pose == "pinch") {param = 2;}

  //Pregrasp shape
  if (param == 3)
  {
    for (int i=0; i < 7; ++i) {axes.axis[i] = pregrasp3[i];} //setting SDH joint configuration
    sdh.MoveToAxisTarget(axes);
  }

  else if (param == 2)
  {
    for (int i=0; i < 7; ++i) {axes.axis[i] = pregrasp2[i];} //setting SDH joint configuration
    sdh.MoveToAxisTarget(axes);
  }

  else if (param == 0)
  {
    for (int i=0; i < 7; ++i) {axes.axis[i] = pregrasp_home[i];} //setting SDH joint configuration
    sdh.MoveToAxisTarget(axes);
  }
  */
  while (sdh.GetRobotIO(&dummy) != CANON_SUCCESS); //action completed

  //Indicate intent 1) grasp next, 0) get ready for another hand pose command
  if (grasp == true) {sdh.SetTool(1);} //run this command to engage closing
  else {sdh.SetTool(0);} //wait for another pose

  while (sdh.GetRobotIO(&dummy) != CANON_SUCCESS); //action completed

  //Get completion status
  
  
};

static unsigned int __stdcall poseHandThread(void* inst)
{
  robotIO dummy;
  //hand_struct *sdh_struct = (hand_struct*)inst;

  //Other initializations
  sdh.MoveToAxisTarget(hand_joint_pose);

  while (sdh.GetRobotIO(&dummy) != CANON_SUCCESS); //action completed

  //Indicate intent 1) grasp next, 0) get ready for another hand pose command
  if (grasp_switch == true) {sdh.SetTool(1);} //run this command to engage closing
  else {sdh.SetTool(0);} //wait for another pose

  while (sdh.GetRobotIO(&dummy) != CANON_SUCCESS); //action completed

  return 0;
};

void killHand(CrpiRobot<CrpiSchunkSDH> &sdh)
{
  robotIO dummy;
  vector<double> pregrasp_home(7);
  pregrasp_home[0] = 60; pregrasp_home[1] = pregrasp_home[3] = pregrasp_home[5] = -35;
  pregrasp_home[2] = pregrasp_home[4] = pregrasp_home[6] = 90;

  //Other initializations
  robotAxes axes;

  for (int i=0; i < 7; ++i) {axes.axis.at(i) = pregrasp_home[i];} //setting SDH joint configuration
  sdh.MoveToAxisTarget(axes);

  while (sdh.GetRobotIO(&dummy) != CANON_SUCCESS); //action completed

  sdh.SetTool(-1);

  while (sdh.GetRobotIO(&dummy) != CANON_SUCCESS); //action completed
};

void grasp(CrpiRobot<CrpiSchunkSDH> &sdh,int num_fingers )
{
  robotIO dummy;
  //Set number of closing fingers
  sdh.SetParameter("NUM_FINGERS",&num_fingers);
  
  //Confirmation when grasp acquired
  while (sdh.GetRobotIO(&dummy) != CANON_SUCCESS);
};

struct ID_desired_angles
{
  int ID;
  robotAxes joint_angles; //joint angles for hand or arm
};

struct part_data
{
  int ID;
  double world_pose[6];
  double robot_pose[6];
  matrix W_T_part; //4x4 transformation matrix of part in world
  matrix W_R_part; //3x3 rotation matrix of part in world
  matrix R_T_part; //4x4 transformation matrix of part in robot
  matrix R_R_part; //3x3 rotation matrix of part in robot
  vector<double> World_Euler_Angles; //part orientation in world
  vector<double> Robot_Euler_Angles; //part orientation in robot

  part_data(){};

  part_data(int ID_in,double * pose_in,matrix &R_T_W)
  {
    ID = ID_in;
    for(int i=0; i<6; ++i)
    {
      world_pose[i] = pose_in[i];
    }

    for(int i=3; i<6; ++i)
    {
      World_Euler_Angles.push_back(world_pose[i]);
    }

    //Get rotation matrix of part in world based on Euler Angles
    W_R_part.resize(3,3);

    W_R_part.rotEulerMatrixConvert(World_Euler_Angles);

    W_T_part.resize(4,4);
    W_T_part.at(3,3) = 1; //homogeneous scaling

    //Inserting translation and rotational elements
    for (int i=0; i<3; ++i)
    {
      W_T_part.at(i,3) = pose_in[i]; //translation

      for (int j=0; j<3; ++j)
      {
        W_T_part.at(i,j) = W_R_part.at(i,j);
      }
    }

    //Conversion of part pose into robot coordinate system
    R_T_part = R_T_W * W_T_part;

    //Populate part pose vector expressed in robot coordinate system
    R_R_part.resize(3,3);
    
    for (int i=0; i<3; ++i)
    {
      for (int j=0; j<3; ++j)
      {
        R_R_part.at(i,j) = R_T_part.at(i,j);
      }
    }

    R_R_part.rotMatrixEulerConvert(Robot_Euler_Angles);

    //populate translation components
    for (int i=0; i<3; ++i)
    {
      robot_pose[i] = R_T_part.at(i,3);
    }

    //populate rotation components
    for (int i=0; i<3; ++i)
    {
      robot_pose[i+3] = Robot_Euler_Angles[i];
    }

  };
};

void send_pose_get_plan(ulapi_integer &server, string part_name, map<string, struct part_data> &Parts_Database,robotAxes &curArmPose,vector<struct ID_desired_angles> &vector_angle_data)
{
// Units: mm, deg
//  Our message to planner is 14 space-separated numbers: 
//    1) the first number is integer that serves as a unique identifier of the part. 
//    2) The following 6 numbers will be the pose (x,y,z,rx,ry,rz) as doubles where we will adopt Euler ZYX convention.
//    3) The last 7 numbers are the current joint position of the KUKA robot arm

//  Message from planner is a stream of 7 space-separated doubles:
//  First number is leading integer: 0 = joint angles for hand, 1 = way point joint angles for arm, 2 = terminating joint angles for arm (all in degrees)

  //Clear existing data
  vector_angle_data.clear();

  //First step is to send grasp planner unique_id of object, its pose in robot coordinate system, and the current arm joint angles for motion planning
  int get, send;
  
  char incoming_data[REQUEST_MSG_SIZE], outgoing_data[REQUEST_MSG_SIZE];
  
  strcpy(incoming_data,"");
  strcpy(outgoing_data,"");
  
  //Send data
  std::ostringstream sstream;
  std::string DataAsString;

  
  //Unique Identifier
  sstream << Parts_Database[part_name].ID;
  DataAsString = sstream.str();
  strcat(outgoing_data, DataAsString.c_str());
  strcat(outgoing_data, " ");
  sstream.str(std::string());
  

  /*
  //HACK
  Parts_Database[part_name].robot_pose[0] = -0.2375;
  Parts_Database[part_name].robot_pose[1] = -0.4625;
  Parts_Database[part_name].robot_pose[2] = 1.04;
  Parts_Database[part_name].robot_pose[3] = 0;
  Parts_Database[part_name].robot_pose[4] = 0;
  Parts_Database[part_name].robot_pose[5] = 0;
  */
  
  //Object pose in robot coordinate system
  for (unsigned int i=0;i<6;++i)
  {
    sstream << Parts_Database[part_name].robot_pose[i]/1000.; //conversion to meters
    DataAsString = sstream.str();
    strcat(outgoing_data, DataAsString.c_str());
    strcat(outgoing_data, " ");
    sstream.str(std::string());
  }

  //KUKA arm current joint angles
  for (unsigned int i=0;i<7;++i)
  {
    if (i == 1) {curArmPose.axis.at(i) = -(curArmPose.axis.at(i)-90);}
    else if (i == 5) {curArmPose.axis.at(i) = -curArmPose.axis.at(i);}
    else if (i == 6) {curArmPose.axis.at(i) = curArmPose.axis.at(i) + 90;}

    sstream << curArmPose.axis.at(i)*(PI/180); //conversion to radians
    DataAsString = sstream.str();
    strcat(outgoing_data, DataAsString.c_str());
    strcat(outgoing_data, " ");
    sstream.str(std::string());
  }
  
  strcat(outgoing_data, " \n");

  //Parts_Database[part_name].ID
  //strcpy(outgoing_data,"");
  //for (int x = 0; x < REQUEST_MSG_SIZE; ++x) {outgoing_data[x] = '\0'; outgoing_data[x] = '\0';}
  //sprintf(outgoing_data, "%d -.237 -.465 -.007 0 0 0 -1.57 0 0 1.57 0 1.57 0\n",1);
  /*
  sprintf(outgoing_data, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d \n", Parts_Database[part_name].ID, Parts_Database[part_name].robot_pose[0]/1000.,
  Parts_Database[part_name].robot_pose[1]/1000., Parts_Database[part_name].robot_pose[2]/1000., Parts_Database[part_name].robot_pose[3]/1000.,
  Parts_Database[part_name].robot_pose[4]/1000., Parts_Database[part_name].robot_pose[5]/1000.,
  curArmPose.axis[0], -(curArmPose.axis[1]-90), curArmPose.axis[2], curArmPose.axis[3], curArmPose.axis[4], -curArmPose.axis[5], curArmPose.axis[6]+90);
  */
  //string test123 = outgoing_data;
  //std::cout << std::endl;
  //std::cout << test123 << std::endl;

  int msg_len = strlen(outgoing_data);

  std::cout << "attempting to write socket" << std::endl;
  send = ulapi_socket_write(server, outgoing_data, msg_len); //sizeof(outgoing_data)
  //std::cout << "successful" << std::endl;

  ID_desired_angles angle_data;

  std::cout << "Getting Data" << std::endl;

  //Get Grasp Planner Solution
  while(Parts_Database[part_name].ID != -1)
  {
    std::cout << "new data" << std::endl;
    strcpy(incoming_data,"");
    get = ulapi_socket_read(server, incoming_data, sizeof(incoming_data));

    //std::cout << get << std::endl;
    //std::cout << "successful" << std::endl;

    //string testy = incoming_data;
    //std::cout << testy << std::endl;
    //cin.get();

    stringstream ss(incoming_data);
    ss >> angle_data.ID;
    
    for(int j=0; j<7; j++) {ss >> angle_data.joint_angles.axis.at(j);}

    vector_angle_data.push_back(angle_data);

    if (angle_data.ID == 2) {break;} //received terminating set of joint angles
  }

}

/*
struct hand_struct
{
  CrpiRobot<CrpiSchunkSDH> * sdh;
  robotAxes hand_joint_pose;
  bool grasp;
  
  hand_struct(){};

  hand_struct(CrpiRobot<CrpiSchunkSDH> & sdh_in, robotAxes & hand_joint_pose_in,bool grasp_in)
  {
    sdh = &sdh_in;
    for (int i=0;i<7;++i) {hand_joint_pose.axis[i] = hand_joint_pose_in.axis[i];}
    grasp = grasp_in;
    
  };
};

*/
int main()
{
  /*
  //Connect to Grasp Planner------------------------------------------------
  cout << "Connecting to Grasp Planner" << endl;
  ulapi_integer server, client;
  int sent,get;
  
  server=ulapi_socket_get_client_id (1990, "169.254.253.21");
  ulapi_socket_set_blocking(server);
  //cout << "Client connected!" << endl;

  if (server < 0) {cout << "failed connection to grasp planner" << endl; return 0;}
  else {cout << "Successful! Robot is now self-aware." << endl;}
  */
  //char graspData[REQUEST_MSG_SIZE], outbuffer[REQUEST_MSG_SIZE];
  
  //Robot Initializations------------------------------
  cout << "Initializing Robot" << endl;
  CrpiRobot<robType> arm("kuka_lwr.xml");
  //CrpiRobot<robType> arm("universal_ur5.xml");
  
  char curtool[32];
  bool toolopen = true;
  //robotPose poseMe, curPose, camPose, offsetPose;
 
  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->SetAngleUnits("degree");
  pm.robArm->SetLengthUnits("mm");

  strcpy(curtool, "schunk_hand");
  //strcpy(curtool, "gripper_parallel");
  
  bool poseDefined = false;
  pm.robArm->Couple(curtool);

  robotAxes tarPose, curArmPose;
  
  //Initializing current, command robot position
  if (!poseDefined)
    {
      //ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotAxes(&curArmPose);
      //ulapi_mutex_give(pm.grabmutex);
      poseDefined = true;
    }  

  tarPose.axis.at(0) = -90;
  tarPose.axis.at(1) = 90;
  tarPose.axis.at(2) = 0;
  tarPose.axis.at(3) = 90;
  tarPose.axis.at(4) = 0;
  tarPose.axis.at(5) = -90;
  tarPose.axis.at(6) = -90;
  

  //Initialization for SDH
  cout << "Initializing SDH" << endl;
  //CrpiRobot<CrpiSchunkSDH> sdh("SDH.xml");
  robotAxes hand_ready_pose;//,hand_joint_pose;

  hand_ready_pose.axis.at(0) = 60;
  hand_ready_pose.axis.at(1) = hand_ready_pose.axis.at(3) = hand_ready_pose.axis.at(5) = -35;
  hand_ready_pose.axis.at(2) = hand_ready_pose.axis.at(4) = hand_ready_pose.axis.at(6) = 90;

  //Initialization of other variables---------------------------------------
  
  //For multi-threading
  bool graspThreadRun = false;
  uintptr_t graspThread = 0;

  //Bring in registration matrix
  matrix R_T_W(4,4); //transformation from Robot to World
  pm.robArm->ToWorldMatrix(R_T_W);

  //Declare 6 DOF pose for each object in WORLD coordinate system, and inject in structs along with UNIQUE IDENTIFIER
  double pose[6] = {0,0,0,0,0,0};

  pose[0] = 35.0*25; pose[1] = -18.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Large_Cylinder(5,pose,R_T_W);

  pose[0] = 35.0*25; pose[1] = -21.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Cylinder(10,pose,R_T_W);

  pose[0] = 20.0*25; pose[1] = -18.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Large_Cone(2,pose,R_T_W);

  pose[0] = 20.0*25; pose[1] = -21.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Cone(7,pose,R_T_W);

  pose[0] = 25.0*25; pose[1] = -18.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Large_Plate(3,pose,R_T_W);

  pose[0] = 25.0*25; pose[1] = -21.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Plate(8,pose,R_T_W);

  pose[0] = 30.0*25; pose[1] = -18.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 45*(PI/180);
  part_data Large_Sphere(4,pose,R_T_W);

  pose[0] = 30.0*25; pose[1] = -21.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Sphere(9,pose,R_T_W);

  pose[0] = 15.0*25; pose[1] = -18.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Large_Cube(1,pose,R_T_W);

  pose[0] = 15.0*25; pose[1] = -21.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Cube(6,pose,R_T_W);

  pose[0] = 0; pose[1] = 0; pose[2] = 0; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Quit(-1,pose,R_T_W);

  //Create database with part information for easy retrieval
  map<string, struct part_data> Parts_Database;
  
  Parts_Database.insert(make_pair("Large_Cylinder",Large_Cylinder));
  Parts_Database.insert(make_pair("Small_Cylinder",Small_Cylinder));
  Parts_Database.insert(make_pair("Large_Cone",Large_Cone));
  Parts_Database.insert(make_pair("Small_Cone",Small_Cone));
  Parts_Database.insert(make_pair("Large_Plate",Large_Plate));
  Parts_Database.insert(make_pair("Small_Plate",Small_Plate));
  Parts_Database.insert(make_pair("Large_Sphere",Large_Sphere));
  Parts_Database.insert(make_pair("Small_Sphere",Small_Sphere));
  Parts_Database.insert(make_pair("Large_Cube",Large_Cube));
  Parts_Database.insert(make_pair("Small_Cube",Small_Cube));
  Parts_Database.insert(make_pair("Quit",Quit));

  //Declare 6 DOF pose for each object's COLLAR in WORLD coordinate system, and inject in structs along with UNIQUE IDENTIFIER
  pose[0] = 35.0*25; pose[1] = -24.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Large_Cylinder_Collar(5,pose,R_T_W);

  pose[0] = 35.0*25; pose[1] = -27.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Cylinder_Collar(10,pose,R_T_W);

  pose[0] = 20.0*25; pose[1] = -24.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Large_Cone_Collar(2,pose,R_T_W);

  pose[0] = 20.0*25; pose[1] = -27.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Cone_Collar(7,pose,R_T_W);

  pose[0] = 25.0*25; pose[1] = -24.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Large_Plate_Collar(3,pose,R_T_W);

  pose[0] = 25.0*25; pose[1] = -27.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Plate_Collar(8,pose,R_T_W);

  pose[0] = 30.0*25; pose[1] = -24.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 45*(PI/180);
  part_data Large_Sphere_Collar(4,pose,R_T_W);

  pose[0] = 30.0*25; pose[1] = -27.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Sphere_Collar(9,pose,R_T_W);

  pose[0] = 15.0*25; pose[1] = -24.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Large_Cube_Collar(1,pose,R_T_W);

  pose[0] = 15.0*25; pose[1] = -27.0*25; pose[2] = 5; pose[3] = 0; pose[4] = 0; pose[5] = 0;
  part_data Small_Cube_Collar(5,pose,R_T_W);

  //Create database with part information for easy retrieval
  map<string, struct part_data> Part_Collar_Database;

  Part_Collar_Database.insert(make_pair("Large_Cylinder_Collar",Large_Cylinder_Collar));
  Part_Collar_Database.insert(make_pair("Small_Cylinder_Collar",Small_Cylinder_Collar));
  Part_Collar_Database.insert(make_pair("Large_Cone_Collar",Large_Cone_Collar));
  Part_Collar_Database.insert(make_pair("Small_Cone_Collar",Small_Cone_Collar));
  Part_Collar_Database.insert(make_pair("Large_Plate_Collar",Large_Plate_Collar));
  Part_Collar_Database.insert(make_pair("Small_Plate_Collar",Small_Plate_Collar));
  Part_Collar_Database.insert(make_pair("Large_Sphere_Collar",Large_Sphere_Collar));
  Part_Collar_Database.insert(make_pair("Small_Sphere_Collar",Small_Sphere_Collar));
  Part_Collar_Database.insert(make_pair("Large_Cube_Collar",Large_Cube_Collar));
  Part_Collar_Database.insert(make_pair("Small_Cube_Collar",Small_Cube_Collar));
  
  //Ready position for arm and hand-----------------------------------------
  cout << "Moving to ready pose" << endl;
  pm.robArm->MoveToAxisTarget(tarPose);
  poseHand(sdh,hand_ready_pose,false);

#ifdef USE_GRASP_PLANNER

  vector<struct ID_desired_angles> vector_angle_data;
  
  robotPose upwards;

  //for (int x = 0; x < REQUEST_MSG_SIZE; ++x) {graspData[x] = '\0'; outbuffer[x] = '\0';}

  //strcpy(outbuffer,"1 -237 -465 -7 0 0 0 0 0 0 0 0 0 0");
  //strcpy(graspData,"");

    //sprintf(outbuffer, "%d -.237 -.465 .043 0 0 0 -1.57 0 0 1.57 0 1.57 0 \n",1);  // Dummy values for object location - "i" is part ID
    //sent = ulapi_socket_write(server, outbuffer, strlen(outbuffer)); //Send request to grasp planner
    //get = ulapi_socket_read(server, graspData, REQUEST_MSG_SIZE); // Solution from grasp planner

  //string whaaat = graspData;

  //std::cout << whaaat << std::endl;
  //cin.get();
  
  //Get plans
  pm.robArm->GetRobotAxes(&curArmPose); //get current arm pose
  send_pose_get_plan(server, "Large_Cylinder", Parts_Database, curArmPose, vector_angle_data);

  for (int ii=0; ii < vector_angle_data.size(); ++ii)
  {
    //std::cout << "First set of data" << std::endl;
    std::cout << vector_angle_data[ii].ID << std::endl;
    vector_angle_data[ii].joint_angles.print();
    //cin.get();
    if (vector_angle_data[ii].ID == 0) {poseHand(sdh,vector_angle_data[ii].joint_angles,false);}
    else if (vector_angle_data[ii].ID == 1) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else if (vector_angle_data[ii].ID == 2) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else {std::cout << "ERROR WITH JOINT ANGLES ID" << std::endl; cin.get();}
  }
  
  poseHand(sdh,vector_angle_data[0].joint_angles,true); //first received message is always the hand joint angles

  grasp(sdh,3);

  //Move up
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z + 30;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  //Move over
  pm.robArm->GetRobotPose(&upwards);
  upwards.y = upwards.y-6*25;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  //Move down
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z-30;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  //Release hand
  poseHand(sdh,vector_angle_data[0].joint_angles,false); //first received message is always the hand joint angles

  //Move up
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z + 70;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  //Move to ready pose
  while(pm.robArm->MoveToAxisTarget(tarPose)!=CANON_SUCCESS) {}
    
  //-----------------------------------------------------
  /*
  pm.robArm->GetRobotAxes(&curArmPose); //get current arm pose
  send_pose_get_plan(server, "Large_Cube", Parts_Database, curArmPose, vector_angle_data);

  for (int ii=0; ii < vector_angle_data.size(); ++ii)
  {
    //std::cout << "First set of data" << std::endl;
    std::cout << vector_angle_data[ii].ID << std::endl;
    vector_angle_data[ii].joint_angles.print();
    //cin.get();
    if (vector_angle_data[ii].ID == 0) {poseHand(sdh,vector_angle_data[ii].joint_angles,false);}
    else if (vector_angle_data[ii].ID == 1) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else if (vector_angle_data[ii].ID == 2) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else {std::cout << "ERROR WITH JOINT ANGLES ID" << std::endl; cin.get();}
  }

  
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z + 70;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  while(pm.robArm->MoveToAxisTarget(tarPose)!=CANON_SUCCESS) {}

  pm.robArm->GetRobotAxes(&curArmPose); //get current arm pose
  */
  //-----------------------------------------------------
  /*
  send_pose_get_plan(server, "Large_Cone", Parts_Database, curArmPose, vector_angle_data);

  for (int ii=0; ii < vector_angle_data.size(); ++ii)
  {
    //std::cout << "First set of data" << std::endl;
    std::cout << vector_angle_data[ii].ID << std::endl;
    vector_angle_data[ii].joint_angles.print();
    //cin.get();
    if (vector_angle_data[ii].ID == 0) {poseHand(sdh,vector_angle_data[ii].joint_angles,false);}
    else if (vector_angle_data[ii].ID == 1) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else if (vector_angle_data[ii].ID == 2) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else {std::cout << "ERROR WITH JOINT ANGLES ID" << std::endl; cin.get();}
  }

  
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z + 70;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  while(pm.robArm->MoveToAxisTarget(tarPose)!=CANON_SUCCESS) {}

  
  */
  //------------------------------------------------------------
  /*
  pm.robArm->GetRobotAxes(&curArmPose); //get current arm pose
  send_pose_get_plan(server, "Large_Plate", Parts_Database, curArmPose, vector_angle_data);

  for (int ii=0; ii < vector_angle_data.size(); ++ii)
  {
    //std::cout << "First set of data" << std::endl;
    std::cout << vector_angle_data[ii].ID << std::endl;
    vector_angle_data[ii].joint_angles.print();
    //cin.get();
    if (vector_angle_data[ii].ID == 0) {poseHand(sdh,vector_angle_data[ii].joint_angles,false);}
    else if (vector_angle_data[ii].ID == 1) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else if (vector_angle_data[ii].ID == 2) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else {std::cout << "ERROR WITH JOINT ANGLES ID" << std::endl; cin.get();}
  }

  
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z + 70;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  while(pm.robArm->MoveToAxisTarget(tarPose)!=CANON_SUCCESS) {}
  */
  //------------------------------------------------------------

  /*
  pm.robArm->GetRobotAxes(&curArmPose); //get current arm pose
  send_pose_get_plan(server, "Large_Sphere", Parts_Database, curArmPose, vector_angle_data);

  for (int ii=0; ii < vector_angle_data.size(); ++ii)
  {
    //std::cout << "First set of data" << std::endl;
    std::cout << vector_angle_data[ii].ID << std::endl;
    vector_angle_data[ii].joint_angles.print();
    //cin.get();
    if (vector_angle_data[ii].ID == 0) {poseHand(sdh,vector_angle_data[ii].joint_angles,false);}
    else if (vector_angle_data[ii].ID == 1) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else if (vector_angle_data[ii].ID == 2) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else {std::cout << "ERROR WITH JOINT ANGLES ID" << std::endl; cin.get();}
  }

  
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z + 70;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  while(pm.robArm->MoveToAxisTarget(tarPose)!=CANON_SUCCESS) {}

  */
  
  //------------------------------------------------------
  pm.robArm->GetRobotAxes(&curArmPose); //get current arm pose
  send_pose_get_plan(server, "Large_Sphere", Parts_Database, curArmPose, vector_angle_data);

  for (int ii=0; ii < vector_angle_data.size(); ++ii)
  {
    //std::cout << "First set of data" << std::endl;
    std::cout << vector_angle_data[ii].ID << std::endl;
    vector_angle_data[ii].joint_angles.print();
    //cin.get();
    if (vector_angle_data[ii].ID == 0) {poseHand(sdh,vector_angle_data[ii].joint_angles,false);}
    else if (vector_angle_data[ii].ID == 1) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else if (vector_angle_data[ii].ID == 2) {pm.robArm->MoveToAxisTarget(vector_angle_data[ii].joint_angles);}
    else {std::cout << "ERROR WITH JOINT ANGLES ID" << std::endl; cin.get();}
  }

  poseHand(sdh,vector_angle_data[0].joint_angles,true); //first received message is always the hand joint angles

  grasp(sdh,3);
  
  //Move up
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z + 30;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  //Move over
  pm.robArm->GetRobotPose(&upwards);
  upwards.y = upwards.y-6*25;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  //Move down
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z-30;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  //Release hand
  poseHand(sdh,vector_angle_data[0].joint_angles,false); //first received message is always the hand joint angles

  //Move up
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z + 70;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  //Move to ready pose
  while(pm.robArm->MoveToAxisTarget(tarPose)!=CANON_SUCCESS) {}

  //Kill planner
  send_pose_get_plan(server,"Quit",Parts_Database,curArmPose,vector_angle_data);


#elif defined GRASP_PLANNER
  
  robotAxes arm_joint_pose;
  double F1Base1, F2Base, F3Base;

  char approachData[REQUEST_MSG_SIZE], graspData[REQUEST_MSG_SIZE], outbuffer[REQUEST_MSG_SIZE];
  ulapi_integer server, client;
  int sent, get, i;
  
  //Connect to Grasp Planner
  cout << "Connecting to grasp planner..." << endl;
  server=ulapi_socket_get_client_id (1990, "169.254.253.21");
  ulapi_socket_set_blocking(server);
  cout << "Client connected!" << endl;

  
  pm.robArm->GetRobotAxes(&arm_joint_pose);

  // Request grasp plan for part number

  cout << "1) L-Cube, 2) L-Cone, 3) L-Cuboid, 4) L-Sphere, 5) L-Cylinder, 6) S-Cube, 7) S-Cone 8) S-Cuboid 9) S-Sphere 10) S-Cylinder, -1) quit : ";
  cin >> i;

   while (i != -1)
   {
    for (int x = 0; x < REQUEST_MSG_SIZE; ++x)
      {
        approachData[x] = '\0';
        graspData[x] = '\0';
        outbuffer[x] = '\0';
      }

    sprintf(outbuffer, "%d 1.1 2.2 3.3 4.4 5.5 6.6 \n", i);  // Dummy values for object location - "i" is part ID
    sent = ulapi_socket_write(server, outbuffer, strlen(outbuffer)); //Send request to grasp planner
    get = ulapi_socket_read(server, graspData, REQUEST_MSG_SIZE); // Solution from grasp planner

    //Extract joint values from socket message
    stringstream ss(graspData);
    for(int j=0; j<7; j++) {ss >> hand_joint_approach.axis.at(j);}
    for(int j=0; j<7; j++) {ss >> arm_joint_pose.axis.at(j);}
    for(int j=0; j<7; j++) {ss >> hand_joint_pose.axis.at(j);}  // This is the actual grasp pose.  Need to use approach grasp then call "grasp(sdh,3)"
    // ... but, need an approach robot pose in the tool +z axis.  Right now, CRPI cannot handle this.  Maybe can be implemented with the
    // right hooks into the KUKA controller.  (set coordinate frame to tool and move relative in +Z)  Discuss w/ Jeremy.

    // Display incomming values
    cout << "approach" << endl;
    for(int j=0; j<7; j++) {cout << hand_joint_approach.axis.at(j) << endl;}
    
    cout << "grasp" << endl;
    for(int j=0; j<7; j++) {cout << arm_joint_pose.axis.at(j) << endl;}
   
    for(int j=0; j<7; j++) {cout << hand_joint_pose.axis.at(j) << endl;}

    poseHand(sdh,hand_joint_approach,false);   


    pm.robArm->MoveToAxisTarget(arm_joint_pose);

    cout << "1) L-Cube, 2) L-Cone, 3) L-Cuboid, 4) L-Sphere, 5) L-Cylinder, 6) S-Cube, 7) S-Cone 8) S-Cuboid 9) S-Sphere 10) S-Cylinder, -1) quit : ";
    cin >> i;
   } // while (i != -1)
   killHand(sdh);

   // All in all, grasp planner runs slow and sometimes does not find a solution.  Shuo has lots of ideas to help w/ its efficiency.
   // Also, lack of solutions may be due to the fact that the parts are too close.  Try separating a bit (25-50 mm).

   //To Run the planner in Vrep: 1. in terminal start Vrep with sudo, i.e., sudo ./vrep.sh 2. Load the scene (LWR+SDHNistSetupwTableFingerCloseVerifyNoPreArm.ttt) 3. Run the Scene and wait for the socket signal

#elif defined LARGE_OBJECTS
  //OPERATIONS ON Large Cube--------------------------------------------

  //Pick
  robotPose CartPose;
  
  CartPose.x = Parts_Database["Large_Cube"].robot_pose[0];
  CartPose.y = Parts_Database["Large_Cube"].robot_pose[1];
  CartPose.z = Parts_Database["Large_Cube"].robot_pose[2]+300;
  CartPose.xrot = Parts_Database["Large_Cube"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Large_Cube"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Large_Cube"].robot_pose[5]*(180/PI)+180;

  pm.robArm->MoveStraightTo(CartPose);

  //Ready Hand
  hand_joint_pose.axis.at(0) = 0;
  hand_joint_pose.axis.at(1) = hand_joint_pose.axis.at(3) = hand_joint_pose.axis.at(5) = -45;
  hand_joint_pose.axis.at(2) = hand_joint_pose.axis.at(4) = hand_joint_pose.axis.at(6) = 45;
  grasp_switch = true;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);
  //poseHand(sdh,hand_joint_pose,true);

  while(pm.robArm->MoveStraightTo(CartPose)!=CANON_SUCCESS) {}
  
  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;

  CartPose.z = Parts_Database["Large_Cube"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,3);

  CartPose.z = Parts_Database["Large_Cube"].robot_pose[2]+210;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Large_Cube_Collar"].robot_pose[0];
  CartPose.y = Part_Collar_Database["Large_Cube_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Large_Cube_Collar"].robot_pose[2]+210;
  CartPose.xrot = Part_Collar_Database["Large_Cube_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Large_Cube_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Large_Cube_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Large_Cube_Collar"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Large_Cube_Collar"].robot_pose[2]+300;
  pm.robArm->MoveStraightTo(CartPose);

  //pm.robArm->MoveToAxisTarget(tarPose);

  //OPERATIONS ON Large Cone--------------------------------------------

  //Pick
  CartPose.x = Parts_Database["Large_Cone"].robot_pose[0];
  CartPose.y = Parts_Database["Large_Cone"].robot_pose[1];
  CartPose.z = Parts_Database["Large_Cone"].robot_pose[2]+300;
  CartPose.xrot = Parts_Database["Large_Cone"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Large_Cone"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Large_Cone"].robot_pose[5]*(180/PI)+180;

  hand_joint_pose.axis[0] = 60;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -20;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 20;
  grasp_switch = true;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);
  //poseHand(sdh,hand_joint_pose,true);

  while(pm.robArm->MoveStraightTo(CartPose)!=CANON_SUCCESS) {}
  
  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;
  

  CartPose.z = Parts_Database["Large_Cone"].robot_pose[2]+170;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,3);

  CartPose.z = Parts_Database["Large_Cone"].robot_pose[2]+180;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Large_Cone_Collar"].robot_pose[0];
  CartPose.y = Part_Collar_Database["Large_Cone_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Large_Cone_Collar"].robot_pose[2]+180;
  CartPose.xrot = Part_Collar_Database["Large_Cone_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Large_Cone_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Large_Cone_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Large_Cone_Collar"].robot_pose[2]+170;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Large_Cone_Collar"].robot_pose[2]+300;
  pm.robArm->MoveStraightTo(CartPose);

  //OPERATIONS ON Large Plate--------------------------------------------

  //Pick

  CartPose.x = Parts_Database["Large_Plate"].robot_pose[0];
  CartPose.y = Parts_Database["Large_Plate"].robot_pose[1];
  CartPose.z = Parts_Database["Large_Plate"].robot_pose[2]+300;
  CartPose.xrot = Parts_Database["Large_Plate"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Large_Plate"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Large_Plate"].robot_pose[5]*(180/PI)+180;

  //pm.robArm->MoveStraightTo(CartPose);

  //Ready Hand
  hand_joint_pose.axis[0] = 0;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -10;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 10;
  grasp_switch = true;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);

  while(pm.robArm->MoveStraightTo(CartPose)!=CANON_SUCCESS) {}
  
  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;
  

  //poseHand(sdh,hand_joint_pose,true);

  CartPose.z = Parts_Database["Large_Plate"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,3);

  CartPose.z = Parts_Database["Large_Plate"].robot_pose[2]+210;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Large_Plate_Collar"].robot_pose[0];
  CartPose.y = Part_Collar_Database["Large_Plate_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Large_Plate_Collar"].robot_pose[2]+210;
  CartPose.xrot = Part_Collar_Database["Large_Plate_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Large_Plate_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Large_Plate_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Large_Plate_Collar"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Large_Plate_Collar"].robot_pose[2]+300;
  pm.robArm->MoveStraightTo(CartPose);

  //OPERATIONS ON Large Sphere--------------------------------------------

  //Pick

  CartPose.x = Parts_Database["Large_Sphere"].robot_pose[0];
  CartPose.y = Parts_Database["Large_Sphere"].robot_pose[1];
  CartPose.z = Parts_Database["Large_Sphere"].robot_pose[2]+300;
  CartPose.xrot = Parts_Database["Large_Sphere"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Large_Sphere"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Large_Sphere"].robot_pose[5]*(180/PI)+180;

  //pm.robArm->MoveStraightTo(CartPose);

  //Ready Hand
  hand_joint_pose.axis[0] = 60;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -30;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 30;
  grasp_switch = true;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);

  while(pm.robArm->MoveStraightTo(CartPose)!=CANON_SUCCESS) {}
  
  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;
  

  //poseHand(sdh,hand_joint_pose,true);

  CartPose.z = Parts_Database["Large_Sphere"].robot_pose[2]+190;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,3);

  CartPose.z = Parts_Database["Large_Sphere"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Large_Sphere_Collar"].robot_pose[0];
  CartPose.y = Part_Collar_Database["Large_Sphere_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Large_Sphere_Collar"].robot_pose[2]+200;
  CartPose.xrot = Part_Collar_Database["Large_Sphere_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Large_Sphere_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Large_Sphere_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Large_Sphere_Collar"].robot_pose[2]+190;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Large_Sphere_Collar"].robot_pose[2]+300;
  pm.robArm->MoveStraightTo(CartPose);

  //OPERATIONS ON Large Cylinder--------------------------------------------

  //Pick

  CartPose.x = Parts_Database["Large_Cylinder"].robot_pose[0];
  CartPose.y = Parts_Database["Large_Cylinder"].robot_pose[1];
  CartPose.z = Parts_Database["Large_Cylinder"].robot_pose[2]+300;
  CartPose.xrot = Parts_Database["Large_Cylinder"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Large_Cylinder"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Large_Cylinder"].robot_pose[5]*(180/PI)+180;

  //pm.robArm->MoveStraightTo(CartPose);

  //Ready Hand
  hand_joint_pose.axis[0] = 60;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -30;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 30;
  grasp_switch = true;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);

  while(pm.robArm->MoveStraightTo(CartPose)!=CANON_SUCCESS) {}
  
  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;
  

  //poseHand(sdh,hand_joint_pose,true);

  CartPose.z = Parts_Database["Large_Cylinder"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,3);

  CartPose.z = Parts_Database["Large_Cylinder"].robot_pose[2]+210;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Large_Cylinder_Collar"].robot_pose[0];
  CartPose.y = Part_Collar_Database["Large_Cylinder_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Large_Cylinder_Collar"].robot_pose[2]+210;
  CartPose.xrot = Part_Collar_Database["Large_Cylinder_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Large_Cylinder_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Large_Cylinder_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Large_Cylinder_Collar"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Large_Cylinder_Collar"].robot_pose[2]+300;
  pm.robArm->MoveStraightTo(CartPose);

#elif defined SMALL_OBJECTS

  //OPERATIONS ON Small Cube--------------------------------------------

  //Pick
  robotPose CartPose;
  
  CartPose.x = Parts_Database["Small_Cube"].robot_pose[0]+25;
  CartPose.y = Parts_Database["Small_Cube"].robot_pose[1];
  CartPose.z = Parts_Database["Small_Cube"].robot_pose[2]+200;
  CartPose.xrot = Parts_Database["Small_Cube"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Small_Cube"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Small_Cube"].robot_pose[5]*(180/PI)+180;

  pm.robArm->MoveStraightTo(CartPose);

  //Ready Hand
  hand_joint_pose.axis[0] = 90;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -25;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 25;

  poseHand(sdh,hand_joint_pose,true);

  CartPose.z = Parts_Database["Small_Cube"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,2);

  CartPose.z = Parts_Database["Small_Cube"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Small_Cube_Collar"].robot_pose[0]+25;
  CartPose.y = Part_Collar_Database["Small_Cube_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Small_Cube_Collar"].robot_pose[2]+200;
  CartPose.xrot = Part_Collar_Database["Small_Cube_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Small_Cube_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Small_Cube_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Small_Cube_Collar"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Small_Cube_Collar"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //pm.robArm->MoveToAxisTarget(tarPose);

  //OPERATIONS ON Small Cone--------------------------------------------

  //Pick
  CartPose.x = Parts_Database["Small_Cone"].robot_pose[0]+25;
  CartPose.y = Parts_Database["Small_Cone"].robot_pose[1];
  CartPose.z = Parts_Database["Small_Cone"].robot_pose[2]+200;
  CartPose.xrot = Parts_Database["Small_Cone"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Small_Cone"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Small_Cone"].robot_pose[5]*(180/PI)+180;

  hand_joint_pose.axis[0] = 90;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -25;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 25;
  grasp_switch = true;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);

  while(pm.robArm->MoveStraightTo(CartPose)!=CANON_SUCCESS) {}
  
  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;
  

  CartPose.z = Parts_Database["Small_Cone"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,2);

  CartPose.z = Parts_Database["Small_Cone"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Small_Cone_Collar"].robot_pose[0]+25;
  CartPose.y = Part_Collar_Database["Small_Cone_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Small_Cone_Collar"].robot_pose[2]+200;
  CartPose.xrot = Part_Collar_Database["Small_Cone_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Small_Cone_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Small_Cone_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Small_Cone_Collar"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Small_Cone_Collar"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //OPERATIONS ON Small Plate--------------------------------------------

  //Pick

  CartPose.x = Parts_Database["Small_Plate"].robot_pose[0]+25;
  CartPose.y = Parts_Database["Small_Plate"].robot_pose[1];
  CartPose.z = Parts_Database["Small_Plate"].robot_pose[2]+200;
  CartPose.xrot = Parts_Database["Small_Plate"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Small_Plate"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Small_Plate"].robot_pose[5]*(180/PI)+180;

  //pm.robArm->MoveStraightTo(CartPose);

  //Ready Hand
  hand_joint_pose.axis[0] = 90;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -25;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 25;
  grasp_switch = true;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);

  while(pm.robArm->MoveStraightTo(CartPose)!=CANON_SUCCESS) {}
  
  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;
  

  //poseHand(sdh,hand_joint_pose,true);

  CartPose.z = Parts_Database["Small_Plate"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,2);

  CartPose.z = Parts_Database["Small_Plate"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Small_Plate_Collar"].robot_pose[0]+25;
  CartPose.y = Part_Collar_Database["Small_Plate_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Small_Plate_Collar"].robot_pose[2]+200;
  CartPose.xrot = Part_Collar_Database["Small_Plate_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Small_Plate_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Small_Plate_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Small_Plate_Collar"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Small_Plate_Collar"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //OPERATIONS ON Small Sphere--------------------------------------------

  //Pick

  CartPose.x = Parts_Database["Small_Sphere"].robot_pose[0]+25;
  CartPose.y = Parts_Database["Small_Sphere"].robot_pose[1];
  CartPose.z = Parts_Database["Small_Sphere"].robot_pose[2]+200;
  CartPose.xrot = Parts_Database["Small_Sphere"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Small_Sphere"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Small_Sphere"].robot_pose[5]*(180/PI)+180;

  //pm.robArm->MoveStraightTo(CartPose);

  //Ready Hand
  hand_joint_pose.axis[0] = 90;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -25;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 25;
  grasp_switch = true;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);

  while(pm.robArm->MoveStraightTo(CartPose)!=CANON_SUCCESS) {}
  
  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;
  

  //poseHand(sdh,hand_joint_pose,true);

  CartPose.z = Parts_Database["Small_Sphere"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,2);

  CartPose.z = Parts_Database["Small_Sphere"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Small_Sphere_Collar"].robot_pose[0]+25;
  CartPose.y = Part_Collar_Database["Small_Sphere_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Small_Sphere_Collar"].robot_pose[2]+200;
  CartPose.xrot = Part_Collar_Database["Small_Sphere_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Small_Sphere_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Small_Sphere_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Small_Sphere_Collar"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Small_Sphere_Collar"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //OPERATIONS ON Small Cylinder--------------------------------------------

  //Pick

  CartPose.x = Parts_Database["Small_Cylinder"].robot_pose[0]+25;
  CartPose.y = Parts_Database["Small_Cylinder"].robot_pose[1];
  CartPose.z = Parts_Database["Small_Cylinder"].robot_pose[2]+200;
  CartPose.xrot = Parts_Database["Small_Cylinder"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Parts_Database["Small_Cylinder"].robot_pose[4]*(180/PI);
  CartPose.zrot = Parts_Database["Small_Cylinder"].robot_pose[5]*(180/PI)+180;

  //pm.robArm->MoveStraightTo(CartPose);

  //Ready Hand
  hand_joint_pose.axis[0] = 90;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -25;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 25;
  grasp_switch = true;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);

  while(pm.robArm->MoveStraightTo(CartPose)!=CANON_SUCCESS) {}
  
  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;
  

  //poseHand(sdh,hand_joint_pose,true);

  CartPose.z = Parts_Database["Small_Cylinder"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  grasp(sdh,2);

  CartPose.z = Parts_Database["Small_Cylinder"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

  //Place

  CartPose.x = Part_Collar_Database["Small_Cylinder_Collar"].robot_pose[0]+25;
  CartPose.y = Part_Collar_Database["Small_Cylinder_Collar"].robot_pose[1];
  CartPose.z = Part_Collar_Database["Small_Cylinder_Collar"].robot_pose[2]+200;
  CartPose.xrot = Part_Collar_Database["Small_Cylinder_Collar"].robot_pose[3]*(180/PI)+180;
  CartPose.yrot = Part_Collar_Database["Small_Cylinder_Collar"].robot_pose[4]*(180/PI);
  CartPose.zrot = Part_Collar_Database["Small_Cylinder_Collar"].robot_pose[5]*(180/PI)+180;
  
  pm.robArm->MoveStraightTo(CartPose);

  CartPose.z = Part_Collar_Database["Small_Cylinder_Collar"].robot_pose[2]+155;
  pm.robArm->MoveStraightTo(CartPose);

  poseHand(sdh,hand_joint_pose,false);

  CartPose.z = Part_Collar_Database["Small_Cylinder_Collar"].robot_pose[2]+200;
  pm.robArm->MoveStraightTo(CartPose);

#endif
  //Return to ready pose and shutdown----------------------------------------------------------------------------
  
  /*
  //Move arm Up first
  pm.robArm->GetRobotPose(&upwards);
  upwards.z = upwards.z + 75;
  while(pm.robArm->MoveStraightTo(upwards)!=CANON_SUCCESS) {}

  while(pm.robArm->MoveToAxisTarget(tarPose)!=CANON_SUCCESS) {}
  */

  hand_joint_pose.axis[0] = 60;
  hand_joint_pose.axis[1] = hand_joint_pose.axis[3] = hand_joint_pose.axis[5] = -35;
  hand_joint_pose.axis[2] = hand_joint_pose.axis[4] = hand_joint_pose.axis[6] = 90;
  grasp_switch = false;

  graspThreadRun = true;
  graspThread = _beginthreadex(NULL, 0, poseHandThread, NULL, 0, NULL);
  
  while(pm.robArm->MoveToAxisTarget(tarPose)!=CANON_SUCCESS) {}

  WaitForSingleObject((HANDLE)graspThread, INFINITE);
  CloseHandle((HANDLE)graspThread);
  graspThreadRun = false;  

  killHand(sdh);
  
  return 0;

}