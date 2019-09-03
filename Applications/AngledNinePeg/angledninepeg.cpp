//9-peg angled SDH+KUKA test method for grasping, vision, and their combination
//CrpiRobot<CrpiSchunkSDH> sdh("dummy text");

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_schunk_sdh.h"
#include "crpi_kuka_lwr.h"
#include <math.h>
#include <algorithm>
#include "crpi_robotiq.h"

#define USE_SDH
//#define USE_ROBOTIQ

#include "src/ulapi.h"
//#include "../Libraries/Math/MatrixMath.h" 
//#include "../Libraries/MotionPrims/AssemblyPrims.h"

/*
//Bring in Boost for multi-threading
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/random.hpp>
#include <boost/bind.hpp>
#include <conio.h>
#include <armadillo>
*/

#pragma warning (disable: 4996)


//#define NOISY
//#define SUPERNOISY
#define ALTTEXT
#define PI 3.14159265

using namespace crpi_robot;
using namespace std;

typedef CrpiKukaLWR robType;


struct passMe
{
  ulapi_mutex_struct* grabmutex;
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

//Defining function that will generate new pose based on hole selected and a scaling factor
robotPose newPose(int holeNumber, robotPose holePose, double scale) //scale should be positive
{
  robotPose hole;
  if (holeNumber == 1 || holeNumber == 3 || holeNumber == 7 || holeNumber == 9)
  {
    hole.x = holePose.x;
    hole.y = holePose.y-sin(holePose.yrot*(PI/180))*scale;
    hole.z = holePose.z+cos(holePose.yrot*(PI/180))*scale;
    hole.xrot = holePose.xrot;
    hole.yrot = holePose.yrot;
    hole.zrot = holePose.zrot;
  }

  else if (holeNumber == 2)
  {
    hole.x = holePose.x;
    hole.y = holePose.y-sin(-40*(PI/180))*scale;
    hole.z = holePose.z+cos(-40*(PI/180))*scale;
    hole.xrot = holePose.xrot;
    hole.yrot = holePose.yrot;
    hole.zrot = holePose.zrot;
  }

  else if (holeNumber == 4)
  {
    hole.x = holePose.x+sin(-45*(PI/180))*scale;
    hole.y = holePose.y;
    hole.z = holePose.z+cos(-45*(PI/180))*scale;
    hole.xrot = holePose.xrot;
    hole.yrot = holePose.yrot;
    hole.zrot = holePose.zrot;
  }

  else if (holeNumber == 5)
  {
    hole.x = holePose.x;
    hole.y = holePose.y;
    hole.z = holePose.z+scale;
    hole.xrot = holePose.xrot;
    hole.yrot = holePose.yrot;
    hole.zrot = holePose.zrot;
  }

  else if (holeNumber == 6)
  {
    hole.x = holePose.x+sin(15*(PI/180))*scale;
    hole.y = holePose.y;
    hole.z = holePose.z+cos(15*(PI/180))*scale;
    hole.xrot = holePose.xrot;
    hole.yrot = holePose.yrot;
    hole.zrot = holePose.zrot;
  }

  else if (holeNumber == 8)
  {
    hole.x = holePose.x;
    hole.y = holePose.y-sin(20*(PI/180))*scale;
    hole.z = holePose.z+cos(20*(PI/180))*scale;
    hole.xrot = holePose.xrot;
    hole.yrot = holePose.yrot;
    hole.zrot = holePose.zrot;
  }
  
  else {cout << "Incorrect hole number selected!!!"; cin.get();}

  return hole;
};

void poseHand(CrpiRobot<CrpiSchunkSDH> &sdh,char hand_pose[],bool grasp)
{
  robotIO dummy;
  //Pregrasp shape
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

  //Other initializations
  robotAxes axes;
  int param;
  

  if (hand_pose == "ready") {param = 0;}
  else if (hand_pose == "tripod") {param = 3;}
  else if (hand_pose == "pinch") {param = 2;}

  //Pregrasp shape
  if (param == 3)
  {
    for (int i=0; i < 7; ++i) {axes.axis.at(i) = pregrasp3[i];} //setting SDH joint configuration
    sdh.MoveToAxisTarget(axes);
  }

  else if (param == 2)
  {
    for (int i=0; i < 7; ++i) {axes.axis.at(i) = pregrasp2[i];} //setting SDH joint configuration
    sdh.MoveToAxisTarget(axes);
  }

  else if (param == 0)
  {
    for (int i=0; i < 7; ++i) {axes.axis.at(i) = pregrasp_home[i];} //setting SDH joint configuration
    sdh.MoveToAxisTarget(axes);
  }

  //Indicate intent 1) grasp next, 0) get ready for another hand pose command
  if (grasp == true) {sdh.SetTool(1);} //run this command to engage closing
  else {sdh.SetTool(0);} //wait for another pose
  sdh.GetRobotIO(&dummy); //action completed
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
  sdh.SetTool(-1);
  sdh.GetRobotIO(&dummy); //action completed
};

void grasp(CrpiRobot<CrpiSchunkSDH> &sdh,int num_fingers )
{
  robotIO dummy;
  //Set number of closing fingers
  sdh.SetParameter("NUM_FINGERS",&num_fingers);
  
  //Confirmation when grasp acquired
  sdh.GetRobotIO(&dummy); //action completed
};

void getCognexFrame(ulapi_integer &server_cam,robotPose &peg_1,robotPose &peg_2,robotPose &peg_3,robotPose &peg_4,robotPose &peg_5,robotPose &peg_6,robotPose &peg_7,robotPose &peg_8,robotPose &peg_9)
{
  int get_cam;
  
  char inbuffer_cam[MSG_SIZE];
  
  for (int x = 0; x < MSG_SIZE; ++x)  {inbuffer_cam[x] = '\0';}
  
  while(1)
  {
    get_cam = ulapi_socket_read(server_cam, inbuffer_cam, MSG_SIZE);
    if (get_cam > 0) {break;}
  }

  //Copy over into string and replace commas with spaces
  string temp(inbuffer_cam);
  int data_size = temp.size();
  replace(temp.begin(),temp.end(),',',' ');

  //Copy back over into existing char array
  for (int ii=0;ii<data_size; ++ii)
  {
    inbuffer_cam[ii] = temp[ii];
  }

  //Pull out XYZ numbers assuming they all exist and are in order, and are in a char array with spaces separating values
  vector<double> cog_data(36);
  char* pEnd;
  cog_data[0] = strtod (inbuffer_cam, &pEnd); //get first angle
  for (unsigned short int jj=1;jj<=34;++jj) //gets other angles
  {
    cog_data[jj] = strtod (pEnd, &pEnd);
  }
  cog_data[35] = strtod (pEnd, NULL); //get last angle
  
  //Replace nominal peg locations with Cognex data
  peg_1.x = cog_data[1];
  peg_1.y = cog_data[2];
  //peg_1.z = cog_data[3];

  peg_2.x = cog_data[5];
  peg_2.y = cog_data[6];
  //peg_2.z = cog_data[7];

  peg_3.x = cog_data[9];
  peg_3.y = cog_data[10];
  //peg_3.z = cog_data[11];

  peg_4.x = cog_data[13];
  peg_4.y = cog_data[14];
  //peg_4.z = cog_data[15];

  peg_5.x = cog_data[17];
  peg_5.y = cog_data[18];
  //peg_5.z = cog_data[19];

  peg_6.x = cog_data[21];
  peg_6.y = cog_data[22];
  //peg_6.z = cog_data[23];

  peg_7.x = cog_data[25];
  peg_7.y = cog_data[26];
  //peg_7.z = cog_data[27];

  peg_8.x = cog_data[29];
  peg_8.y = cog_data[30];
  //peg_8.z = cog_data[31];

  peg_9.x = cog_data[33];
  peg_9.y = cog_data[34];
  //peg_9.z = cog_data[35];

  cout << peg_1.x << " " << peg_1.y << " " << peg_1.z << endl;
}

void robotiq_pose(CrpiRobot<CrpiRobotiq>& riq, int A, int B, int C)
{
  int param=1;
  riq.SetParameter("POSITION_FINGER_A", &A);
  riq.SetParameter("POSITION_FINGER_B", &B);
  riq.SetParameter("POSITION_FINGER_C", &C);
  riq.SetParameter("GRIP", &param);
}

int main()
{
  //Setting camera and peg hole locations

  robotPose peg_1, peg_2, peg_3, peg_4, peg_5, peg_6, peg_7, peg_8, peg_9; //these will be the nominal poses of hand to deposit pegs
  robotPose hole_1, hole_2, hole_3, hole_4, hole_5, hole_6, hole_7, hole_8, hole_9; //these will be the nominal poses of hand to deposit pegs
  robotPose mod_1, mod_2, mod_3, mod_4, mod_5, mod_6, mod_7, mod_8, mod_9; //custom modifiers for the hand per peg
  robotPose hole; //set this and command to this
  robotPose poseMe, curPose, camPose, offsetPose;

  camPose.x = -238.63;
  camPose.y = 307.92;
  camPose.z = 308.91;
  camPose.xrot = -180;
  camPose.yrot = 0;
  camPose.zrot = -90;

  //Nominal CAD peg locations and custom modifiers
  peg_1.x = -612.5+175;
  peg_1.y = 60.7513+300;
  peg_1.z = 61.3205;
  peg_1.xrot = camPose.xrot;
  peg_1.yrot = -30+camPose.yrot;
  peg_1.zrot = 0+camPose.zrot;

  mod_1.x = 0;
  mod_1.y = 8;
  mod_1.z = 0;
  mod_1.xrot = 0;
  mod_1.yrot = 0;
  mod_1.zrot = 0;

  peg_2.x = -562.5+175;
  peg_2.y = 60.2782+300;
  peg_2.z = 46.4612;
  peg_2.xrot = 40+camPose.xrot;
  peg_2.yrot = camPose.yrot;
  peg_2.zrot = -90+camPose.zrot;

  mod_2.x = 18;
  mod_2.y = 15;
  mod_2.z = 0;
  mod_2.xrot = 0;
  mod_2.yrot = 0;
  mod_2.zrot = 0;

  peg_3.x = -512.5+175;
  peg_3.y = 58.9821+300;
  peg_3.z = 64.5368;
  peg_3.xrot = camPose.xrot;
  peg_3.yrot = -25+camPose.yrot;
  peg_3.zrot = 0+camPose.zrot;

  mod_3.x = -3;
  mod_3.y = +5;
  mod_3.z = 0;
  mod_3.xrot = 0;
  mod_3.yrot = 0;
  mod_3.zrot = 0;

  peg_4.x = -631.136+175;
  peg_4.y = 0.032+300;
  peg_4.z = 41.7193;
  peg_4.xrot = 45+camPose.xrot;
  peg_4.yrot = camPose.yrot;
  peg_4.zrot = 0+camPose.zrot;

  mod_4.x = -8;
  mod_4.y = 15;
  mod_4.z = 0;
  mod_4.xrot = 0;
  mod_4.yrot = 0;
  mod_4.zrot = 0;

  peg_5.x = -562.5+175;
  peg_5.y = 0+300;
  peg_5.z = 70;
  peg_5.xrot = 0+camPose.xrot;
  peg_5.yrot = 0+camPose.yrot;
  peg_5.zrot = 0+camPose.zrot;

  mod_5.x = 0;
  mod_5.y = 0;
  mod_5.z = 0;
  mod_5.xrot = 0;
  mod_5.yrot = 0;
  mod_5.zrot = 0;

  peg_6.x = -507.921+175;
  peg_6.y = 0+300;
  peg_6.z = 63.1772;
  peg_6.xrot = -15+camPose.xrot;
  peg_6.yrot = camPose.yrot;
  peg_6.zrot = 0+camPose.zrot;

  mod_6.x = 10;
  mod_6.y = 20;
  mod_6.z = 0;
  mod_6.xrot = 0;
  mod_6.yrot = 0;
  mod_6.zrot = 0;

  peg_7.x = -612.5+175;
  peg_7.y = -51.8382+300;
  peg_7.z = 71.1028;
  peg_7.xrot = camPose.xrot;
  peg_7.yrot = 5+camPose.yrot;
  peg_7.zrot = 0+camPose.zrot;

  mod_7.x = 0;
  mod_7.y = 0;
  mod_7.z = 0;
  mod_7.xrot = 0;
  mod_7.yrot = 0;
  mod_7.zrot = 0;

  peg_8.x = -562.5+175;
  peg_8.y = -56.4687+300;
  peg_8.z = 59.8784;
  peg_8.xrot = -20+camPose.xrot;
  peg_8.yrot = 0+camPose.yrot;
  peg_8.zrot = -90+camPose.zrot;

  mod_8.x = 15;
  mod_8.y = -7;
  mod_8.z = 0;
  mod_8.xrot = 0;
  mod_8.yrot = 0;
  mod_8.zrot = 0;

  peg_9.x = -512.5+175;
  peg_9.y = -53.8528+300;
  peg_9.z = 73.0374;
  peg_9.xrot = camPose.xrot;
  peg_9.yrot = 10+camPose.yrot;
  peg_9.zrot = 0+camPose.zrot;

  mod_9.x = 0;
  mod_9.y = 20;
  mod_9.z = 0;
  mod_9.xrot = 0;
  mod_9.yrot = 0;
  mod_9.zrot = 0;

  //Dropoff hole locations
  hole_1.x = -562.5-25*2-1; //extra -1
  hole_1.y = 2.5*25-5; // extra +1
  hole_1.z = 30;
  hole_1.xrot = 0+camPose.xrot;
  hole_1.yrot = 0+camPose.yrot;
  hole_1.zrot = 0+camPose.zrot;

  hole_2.x = -562.5-1; //extra -1
  hole_2.y = 2.5*25+18; // extra +1
  hole_2.z = 30;
  hole_2.xrot = 0+camPose.xrot;
  hole_2.yrot = 0+camPose.yrot;
  hole_2.zrot = 0+camPose.zrot;

  hole_3.x = -562.5+25*2-3; //extra -1
  hole_3.y = 2.5*25-6+1; // extra +1
  hole_3.z = 30;
  hole_3.xrot = 0+camPose.xrot;
  hole_3.yrot = 0+camPose.yrot;
  hole_3.zrot = 0+camPose.zrot;

  hole_4.x = -562.5-44; //extra -1
  hole_4.y = .5*25+13; // extra +1
  hole_4.z = 30;
  hole_4.xrot = 0+camPose.xrot;
  hole_4.yrot = 0+camPose.yrot;
  hole_4.zrot = 0+camPose.zrot;

  hole_5.x = -562.5-1; //extra -1
  hole_5.y = .5*25-2; // extra +1
  hole_5.z = 30;
  hole_5.xrot = 0+camPose.xrot;
  hole_5.yrot = 0+camPose.yrot;
  hole_5.zrot = 0+camPose.zrot;

  hole_6.x = -562.5+54; //extra -1
  hole_6.y = .5*25+16; // extra +1
  hole_6.z = 30;
  hole_6.xrot = 0+camPose.xrot;
  hole_6.yrot = 0+camPose.yrot;
  hole_6.zrot = 0+camPose.zrot;

  hole_7.x = -562.5-50; //extra -1
  hole_7.y = .5*25-50; // extra +1
  hole_7.z = 30;
  hole_7.xrot = 0+camPose.xrot;
  hole_7.yrot = 0+camPose.yrot;
  hole_7.zrot = 0+camPose.zrot;

  hole_8.x = -562.5-4; //extra -1
  hole_8.y = .5*25-50+16; // extra +1
  hole_8.z = 30;
  hole_8.xrot = 0+camPose.xrot;
  hole_8.yrot = 0+camPose.yrot;
  hole_8.zrot = 0+camPose.zrot;

  hole_9.x = -562.5+50; //extra -1
  hole_9.y = .5*25-50+21; // extra +1
  hole_9.z = 30;
  hole_9.xrot = 0+camPose.xrot;
  hole_9.yrot = 0+camPose.yrot;
  hole_9.zrot = 0+camPose.zrot;

  cout << "Initializing Robot" << endl;
  CrpiRobot<robType> arm("kuka_lwr.xml");
  char curtool[32];
  bool toolopen = true;
  //robotPose poseMe, curPose, camPose, offsetPose;
 
  passMe pm (&arm); //! State variable used to communicate with the two threads
  pm.robArm->SetAngleUnits("degree");
  pm.robArm->SetLengthUnits("mm");
  //-----------------------------------------------------------------------------------------------
  #ifdef USE_SDH
  strcpy(curtool, "schunk_hand");
  bool poseDefined = false;
  pm.robArm->Couple(curtool);

  //Initializing current, command robot position
  if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      //ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      //ulapi_mutex_give(pm.grabmutex);
      poseDefined = true;
    }  
  poseMe = curPose;

  //Initialization for SDH
  cout << "Initializing SDH" << endl;
  CrpiRobot<CrpiSchunkSDH> sdh("SDH.xml");

  //STARTING POSITIONS!!!!!!!!!!!!!!!!!!!!!
  //Open Hand to 'ready' pose
  poseHand(sdh,"ready",false); //contains all code for hand control
  
  //GO TO CAM POSITION
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = camPose;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //CODE FOR USING COGNEX DATA
  int use_camera=0;
  cout << "Use camera data? 1) yes, 2) no" << endl;
  cin >> use_camera;
  
  if (use_camera == 1)
  {
    cout << "connecting to camera" << endl;
    //Initialization for Cognex and Camera
    ulapi_integer server_cam;
    server_cam = ulapi_socket_get_client_id (1456, "129.6.35.46");
    cout << "DONE!" << endl;
    getCognexFrame(server_cam,peg_1,peg_2,peg_3,peg_4,peg_5,peg_6,peg_7,peg_8,peg_9);
  }
  
  //Add custom mods to peg locations
  peg_1 = peg_1+mod_1;
  peg_2 = peg_2+mod_2;
  peg_3 = peg_3+mod_3;
  peg_4 = peg_4+mod_4;
  peg_5 = peg_5+mod_5;
  peg_6 = peg_6+mod_6;
  peg_7 = peg_7+mod_7;
  peg_8 = peg_8+mod_8;
  peg_9 = peg_9+mod_9;

  //GO TO READY POSITION----------------------------
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 5
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,125);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"tripod",true);
  grasp(sdh,3);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_5;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_5;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_5;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 1
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(1,peg_1,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(1,peg_1,122);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"tripod",true);
  grasp(sdh,3);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(1,peg_1,185);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_1;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_1;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_1;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 2
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(2,peg_2,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(2,peg_2,122);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(2,peg_2,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_2;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_2;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_2;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 3
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(3,peg_3,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(3,peg_3,125);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"tripod",true);
  grasp(sdh,3);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(3,peg_3,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_3;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_3;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_3;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  /*
  //GET AND MOVE PEG 4
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(4,peg_4,130);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(4,peg_4,122);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(4,peg_4,145);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);
  poseMe.x = poseMe.x-50; //modifier to avoid collision
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_4;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_4;
  hole.z = hole.z+145;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_4;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);
  */

  //GET AND MOVE PEG 6
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(6,peg_6,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(6,peg_6,122);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(6,peg_6,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_6;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_6;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_6;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 7
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(7,peg_7,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(7,peg_7,125);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"tripod",true);
  grasp(sdh,3);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(7,peg_7,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_7;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_7;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_7;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 8
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(8,peg_8,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(8,peg_8,125);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(8,peg_8,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_8;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_8;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_8;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 9
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(9,peg_9,135);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(9,peg_9,130);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(9,peg_9,165);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_9;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_9;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_9;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  killHand(sdh);

  #elif defined USE_ROBOTIQ
  strcpy(curtool, "schunk_hand");
  bool poseDefined = false;
  pm.robArm->Couple(curtool);

  //Initializing current, command robot position
  if (!poseDefined)
    {
      curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
      //ulapi_mutex_take(pm.grabmutex);
      pm.robArm->GetRobotPose(&curPose);
      //ulapi_mutex_give(pm.grabmutex);
      poseDefined = true;
    }  
  poseMe = curPose;

  //Initialization for SDH
  cout << "Initializing Robotiq" << endl;
  double percent;
  int param;
  CrpiRobot<CrpiRobotiq> riq("robotiq.xml");
  param = 1;
  riq.SetParameter("ADVANCED_CONTROL", &param);
  param=100;
  riq.SetParameter("SPEED_FINGER_A", &param);
  riq.SetParameter("SPEED_FINGER_B", &param);
  riq.SetParameter("SPEED_FINGER_C", &param);
    param=0;
  riq.SetParameter("FORCE_FINGER_A", &param);
  riq.SetParameter("FORCE_FINGER_B", &param);
  riq.SetParameter("FORCE_FINGER_C", &param);
  param=1;
  riq.SetParameter("GRIP", &param);

  //STARTING POSITIONS!!!!!!!!!!!!!!!!!!!!!
  //Open Hand to 'ready' pose
  robotiq_pose(riq,0,0,0);  
  
  //GO TO CAM POSITION
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = camPose;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //CODE FOR USING COGNEX DATA
  int use_camera=0;
  cout << "Use camera data? 1) yes, 2) no" << endl;
  cin >> use_camera;
  
  if (use_camera == 1)
  {
    cout << "connecting to camera" << endl;
    //Initialization for Cognex and Camera
    ulapi_integer server_cam;
    server_cam = ulapi_socket_get_client_id (1456, "129.6.35.46");
    cout << "DONE!" << endl;
    getCognexFrame(server_cam,peg_1,peg_2,peg_3,peg_4,peg_5,peg_6,peg_7,peg_8,peg_9);
  }
  
  //Add custom mods to peg locations
  peg_1 = peg_1+mod_1;
  peg_2 = peg_2+mod_2;
  peg_3 = peg_3+mod_3;
  peg_4 = peg_4+mod_4;
  peg_5 = peg_5+mod_5;
  peg_6 = peg_6+mod_6;
  peg_7 = peg_7+mod_7;
  peg_8 = peg_8+mod_8;
  peg_9 = peg_9+mod_9;

  //GO TO READY POSITION----------------------------
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 5
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,125);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"tripod",true);
  grasp(sdh,3);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_5;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_5;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_5;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 1
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(1,peg_1,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(1,peg_1,122);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"tripod",true);
  grasp(sdh,3);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(1,peg_1,185);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_1;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_1;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_1;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 2
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(2,peg_2,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(2,peg_2,122);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(2,peg_2,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_2;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_2;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_2;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 3
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(3,peg_3,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(3,peg_3,125);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"tripod",true);
  grasp(sdh,3);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(3,peg_3,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_3;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_3;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_3;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  /*
  //GET AND MOVE PEG 4
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(4,peg_4,130);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(4,peg_4,122);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(4,peg_4,145);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);
  poseMe.x = poseMe.x-50; //modifier to avoid collision
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_4;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_4;
  hole.z = hole.z+145;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_4;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);
  */

  //GET AND MOVE PEG 6
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(6,peg_6,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(6,peg_6,122);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(6,peg_6,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_6;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_6;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_6;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 7
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(7,peg_7,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(7,peg_7,125);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"tripod",true);
  grasp(sdh,3);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(7,peg_7,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_7;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_7;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_7;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 8
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(8,peg_8,175);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(8,peg_8,125);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(8,peg_8,200);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_8;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_8;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_8;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //GET AND MOVE PEG 9
  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(9,peg_9,135);//newPose(9,peg_9,135);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(9,peg_9,130);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"pinch",true);
  grasp(sdh,2);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(9,peg_9,165);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  poseMe = newPose(5,peg_5,175);
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_9;
  hole.z = hole.z+250;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_9;
  hole.z = hole.z+155;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  poseHand(sdh,"ready",false);

  //Go to robot pose
  ulapi_mutex_take(pm.grabmutex);
  hole = hole_9;
  hole.z = hole.z+220;
  poseMe = hole;
  //cout << hole.x << " " << hole.y << " " << hole.z << " " << endl;
    while (pm.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS){}; curPose = poseMe;
    ulapi_mutex_give(pm.grabmutex);

  killHand(sdh);

  #endif

  return 0;

}