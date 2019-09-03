///////////////////////////////////////////////////////////////////////////////
//
//  Original System: ISD CRPI
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
#include <sstream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#include "../Libraries/MotionPrims/AssemblyPrims.h"

#include "posemath.h"
#include "config.hh"
#include "agv_status.hh"
#include "scanUtils.h"
#include "AR_Interface.h"
#include "agv_comm.h"
#include <list>

#pragma warning (disable: 4996)

#define LOGEVERYTHING

#define NOISY
//#define SUPERNOISY

//#define CLASSIC_LASER
#define RUN_ASSEMBLY
//#define TEST_ACQUIRE
//#define TEST_ASSEMBLY
//#define BYPASS_AGV
//#define CONFIRM_MOVE
#define REFINE_POSE_FIRST

#define AGV_TARGET 1 //! 1 is square, 2 is circle
#define AGV_X_ 7185.0
#define AGV_Y_ 15215.0
#define AGV_THETA_ 270.0
//      agvX = 8050.0;//6095.0;//5704.0;// 7135.0;
//      agvY = 14090.0;//14915.0;//13870;//11810.0;
//      agvAngle = 270.0;//315.0;//0.0;//90.0;

#define REQUEST_MSG_SIZE 8192

using namespace crpi_robot;
using namespace MotionPrims;
using namespace std;

typedef CrpiUniversal robType;
#ifdef RUN_ASSEMBLY
typedef CrpiRobotiq handType;
#endif

//////////////////////////////////////////////////////////////////////
// Make output streams global so they can log data from subroutines.
ofstream logger;
ofstream out;

/////////////////////////////////////////////////////////////////////
struct TargetOffset
{
  int target;      // target type
  double agvX;     // pose of AGV
  double agvY;
  double agvAngle;
  double point1X;  // world coordinates of reference points
  double point1Y;
  double point2X;
  double point2Y;
};
typedef vector<TargetOffset> TargetOffsetList;

/////////////////////////////////////////////////////////////////////////
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
  ulapi_mutex_struct *handle; 
  char buffer[REQUEST_MSG_SIZE]; 
  bool runThread;
  bool remoteArmConnected;
  ulapi_integer remoteID;
  bool t1;
  CrpiRobot<robType> *robArm;
#ifdef RUN_ASSEMBLY
  CrpiRobot<handType> *robHand;
#endif
  package pkg;
  bool validpkg;
  int status;

  robotPose curPose, curSpeeds;

  globalHandle (CrpiRobot<robType>* armptr)
  {
    validpkg = false;
    robArm = armptr;
    handle = ulapi_mutex_new(89);
  }

#ifdef RUN_ASSEMBLY
  globalHandle(CrpiRobot<robType>* armptr, CrpiRobot<handType>* handptr)
  {
    validpkg = false;
    robArm = armptr;
    robHand = handptr;
    handle = ulapi_mutex_new(89);
  }
#endif

  ~globalHandle ()
  {
    robArm = NULL;
  }
};

/////////////////////////////////
// Convenience function for copying cartesian point into robotPose
void setPosition( robotPose &pose,
                  const PM_CARTESIAN point,
                  const double rot[] )
{
  pose.x = point.x;
  pose.y = point.y;
  pose.z = point.z;
  pose.xrot = rot[0];
  pose.yrot = rot[1];
  pose.zrot = rot[2];
}

// convenience function for converting from PM_CARTESIAN to robotPose
// @param position is used to set the x and y coordinates
// @param orientation is used to set the z value and the rotation 
robotPose cart2rob( PM_CARTESIAN position,
                    robotPose orientation )
{
  robotPose pose = orientation;
  pose.x = position.x;
  pose.y = position.y;

  return pose;
}

// convert from a PM_POSE to a robotPose
robotPose pm2robotConvert( PM_POSE pm )
{
  robotPose pose;
  pose.x = pm.tran.x;
  pose.y = pm.tran.y;
  pose.z = pm.tran.z;
  PM_RPY rpy = pm.rot;
  pose.xrot = rpy.r*180/PM_PI;
  pose.yrot = rpy.p*180/PM_PI;
  pose.zrot = rpy.y*180/PM_PI;

  return pose;
}

// convert from a robotPose to a PM_POSE
PM_POSE robot2pmConvert( robotPose robot )
{
  PM_POSE pose;
  pose.tran.x = robot.x;
  pose.tran.y = robot.y;
  pose.tran.z = robot.z;
  PM_RPY rpy( robot.xrot*PM_PI/180, robot.yrot*PM_PI/180, robot.zrot*PM_PI/180 );
  pose.rot = rpy;

  return pose;
}

// convenience function for converting from robotPose to PM_CARTESIAN
// @param pose is used to set the x and y values, z is set to 0
PM_CARTESIAN rob2cart( robotPose pose )
{
  return PM_CARTESIAN( pose.x, pose.y, 0 );
}

bool readTargetOffsets( 
  const char *filename, 
  TargetOffsetList & offsetList )
{
  ifstream targetOffsetsStream( filename );
  if( !targetOffsetsStream )
    return false;

  string line = "";
  while( getline( targetOffsetsStream, line ) )
  {
    // separate comma seperated list of numbers (doubles)
    vector<double> value;
    stringstream strstr(line);
        string word = "";
        while (getline(strstr, word, ',')) 
      value.push_back( stod( word ) );

    // make sure we have the right number of values
    if( value.size() != 8 )
      return false;

    // assign values to TargetOffset structure and add to list
    TargetOffset offset;
    offset.target = (int)value[0];
    offset.agvX = value[1];
    offset.agvY = value[2];
    offset.agvAngle = value[3];
    offset.point1X = value[4];
    offset.point1Y = value[5];
    offset.point2X = value[6];
    offset.point2Y = value[7];
    offsetList.push_back( offset );
  }

  return true;
}

////////////////////////////////////////////////////////////////
void writeTargetOffset( 
  ofstream &file,
  TargetOffset offset )
{
  file << offset.target 
    << "," << offset.agvX 
    << "," << offset.agvY
    << "," << offset.agvAngle
    << "," << offset.point1X
    << "," << offset.point1Y
    << "," << offset.point2X
    << "," << offset.point2Y
    << "," << endl;
}

/////////////////////////////////////////////////////////////////
// Locate a target offset with a matching AGV position and target id.
bool findTargetOffset(
  TargetOffset &offset,
  TargetOffsetList offsetList )
{
  for( unsigned int i=0; i<offsetList.size(); ++i )
  {
    double distance = sqrt( sq( offset.agvX - offsetList[i].agvX )
      + sq( offset.agvY - offsetList[i].agvY ) );
    double angle = fabs( offset.agvAngle - offsetList[i].agvAngle );
    angle = angle < 360.0 ? angle : angle - 360.0;

    if( distance < targetDistanceThreshold
      && angle < targetAngleThreshold 
      && offsetList[i].target == offset.target )
    {
      offset = offsetList[i];
      return true;
    }
  }

  // couldn't find a match
  return false;
}

double time_since( int seconds )
{
  SYSTEMTIME sysTime;
  GetSystemTime( &sysTime );

  double timestamp =
    sysTime.wHour * 60 * 60 +
    sysTime.wMinute * 60 +
    sysTime.wSecond +
    sysTime.wMilliseconds / 1000.0
    - seconds;
  return timestamp;
}

int get_seconds()
{
  SYSTEMTIME sysTime;
  GetSystemTime( &sysTime );

  return 
    sysTime.wHour * 60 * 60 +
    sysTime.wMinute * 60 +
    sysTime.wSecond;
}

//////////////////////////////////////////////////////////////////
void main ()
{
  CanonReturn retVal;
  int mutex_key = 80;

  PM_CARTESIAN squarePoint1World, squarePoint2World;
  PM_CARTESIAN circlePoint1World, circlePoint2World;
  PM_CARTESIAN point2Offset;

  // declare AGV status variables
  double agvX;
  double agvY;
  double agvAngle;
  int agvTarget;

  // test pose conversions
  PM_POSE pmPose;
  robotPose robot;
  robot.xrot = 180;
  robot.zrot = 90;
  pmPose = robot2pmConvert( robot );
  robot = pm2robotConvert( pmPose );

  // define base pose to use when converting between PM_CARTESIAN and robot position
  robotPose basePose;
  basePose.x = basePose.y = 0;
  basePose.z = sensorHeight;
  basePose.xrot = sensorRot[0];
  basePose.yrot = sensorRot[1];
  basePose.zrot = sensorRot[2];

#ifndef TEST_ACQUIRE
#ifndef BYPASS_AGV
  // initialize ACI link with AGV
  AgvComm agv( "192.168.0.62", 30009 );
  agv.setVerbose( true );

  // initialize AGV position status link
  // status include position and orientation data(nav)
  // xy theta 16 hz
  ulapi_integer priority = ulapi_prio_lowest();
  AGV_Status agv_status( priority );
  std::cout << "Waiting for status connection...  ";
  while( !agv_status.is_connected() ) 
  {
    ulapi_sleep(.1);
  }
  std::cout << "connected to AGV" << endl;

  // initialize interface to AR toolkit
  // process priority for secondary communication thread
  priority = ulapi_prio_next_higher( priority );
  AR_Interface *ar_interface = AR_Interface::Get_Instance();
  if( useARToolkit )
  {
    ar_interface->Init( priority, ++mutex_key );
  }
#else
  cout << "Bypassing AGV." << endl;
#endif

#endif

  package pkg;
  crpi_timer timer;
  crpi_timer pause;
  cout << "Connecting to arm." << endl;
  CrpiRobot<robType> arm("universal_ur10_agv.xml");
  cout << "Set up searches..." << endl;
  Assembly square_spiral;

  cout << "Coupling arm with ";
  pause.waitUntil(robotSettleTime);
#ifdef CLASSIC_LASER
  cout << "classic laser" << endl;
  arm.Couple("laser");
#else
  cout << "Robotiq laser" << endl;
  arm.Couple("robotiq_laser");
#endif
  pause.waitUntil(robotSettleTime);
  arm.SetRelativeSpeed(0.5f);

#ifdef RUN_ASSEMBLY
  cout << "Connecting to robotiq." << endl;
  CrpiRobot<handType> hand("robotiq.xml");
#endif


  // set up search parameters
  square_spiral.AddSearchSqSpiral( searchStep, searchRadius );
  square_spiral.AddTerminatorSignal( CANON_SUCCESS, 9 );
  square_spiral.AddTerminatorTimer( CANON_FAILURE, searchTimeout );

  cout << "Running development version of MobileManipulator" << endl;

  robotPose poseMe, tarPose, curPose, initPose;
  robotAxes curAxes;
  robotPose position[10];
  robotPose startPose;

  // initialize target offsets
  TargetOffsetList  targetOffsetList;
  ofstream targetOffsetsStream;
  if( recordTargetOffsets && useTargetOffsets )
  {
    cout << "ERROR *** recordTargetOffsets and useTargetOffsets cannot both be set true." << endl;
    ulapi_sleep(60);
    exit(1);
  }
  if( recordTargetOffsets )
  {
    targetOffsetsStream.open( targetOffsetsFile );
    cout << "Recording target offsets" << endl;
  }
  if( useTargetOffsets )
  {
    cout << "Using target offsets" << endl;

    if( !readTargetOffsets(  targetOffsetsFile,  targetOffsetList ) )
    {
      cout << "ERROR *** Error reading target offsets file, " << targetOffsetsFile << endl;
      ulapi_sleep( 60 );
      exit(1);
    }
  }

#ifndef RUN_ASSEMBLY
  globalHandle gh (&arm); //! State variable used to communicate with the two threads
#else
  globalHandle gh(&arm, &hand); //! State variable used to communicate with the two threads
#endif
  gh.robArm->SetAngleUnits("degree");
  gh.robArm->SetLengthUnits("mm");
  gh.robArm->SetAbsoluteAcceleration(0.2f);
  gh.robArm->SetAbsoluteSpeed(0.40f);
#ifdef RUN_ASSEMBLY
  int pegIndex = 0;
  vector<robotPose> pegMatrix;
  robotPose binPose;
  robotPose testHolePose;
  pause.waitUntil(robotSettleTime);
  gh.robHand->Couple("gripper_peg");
  robotPose poseTemp;

  ifstream infile;
  infile.open("mobman_poses.dat");
  pegMatrix.clear();
  pegMatrix.resize(0);

  //! Read peg matrix
  for (int x = 0; x < 10; ++x)
  {
    infile >> poseTemp.x >> poseTemp.y >> poseTemp.z >> poseTemp.xrot >> poseTemp.yrot >> poseTemp.zrot;
    pegMatrix.push_back(poseTemp);
  }
  //! Read waste bin location
  infile >> binPose.x >> binPose.y >> binPose.z >> binPose.xrot >> binPose.yrot >> binPose.zrot;
  infile >> testHolePose.x >> testHolePose.y >> testHolePose.z >> testHolePose.xrot >> testHolePose.yrot >> testHolePose.zrot;
#endif

#ifdef TEST_ACQUIRE
  gh.robArm->Couple("robotiq");
  gh.robArm->SetRelativeSpeed(0.5);

  int x;
  cout << "Ready to start.  Enter 1." << endl;
  cin >> x;

  for (pegIndex = 0; pegIndex < 9; pegIndex += 2)
  {
    //! Get next peg
    double pegDepth = 140.0f;
    int param;
    //! Move to hover position over next peg
    while (gh.robArm->MoveStraightTo(pegMatrix.at(pegIndex)) != CANON_SUCCESS)
    {}
      pause.waitUntil(robotSettleTime);
    

#ifdef CONFIRM_MOVE
    cout << "At point.  Enter 1." << endl;
    cin >> x;
#endif

    //! Open gripper
    //PreGrasp
    param = 1;
    hand.SetParameter("ADVANCED_CONTROL", &param);
    param = 100;
    hand.SetParameter("SPEED_FINGER_A", &param);
    hand.SetParameter("SPEED_FINGER_B", &param);
    hand.SetParameter("SPEED_FINGER_C", &param);
    param = 0;
    hand.SetParameter("FORCE_FINGER_A", &param);
    hand.SetParameter("FORCE_FINGER_B", &param);
    hand.SetParameter("FORCE_FINGER_C", &param);
    param = 60;
    gh.robHand->SetParameter("POSITION_FINGER_A", &param);
    gh.robHand->SetParameter("POSITION_FINGER_B", &param);
    gh.robHand->SetParameter("POSITION_FINGER_C", &param);
    param = 1;
    gh.robHand->SetParameter("GRIP", &param);

    //! Move to acquire depth
    poseMe = pegMatrix.at(pegIndex);
    poseMe.z -= pegDepth;
    while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
    {}
      pause.waitUntil(robotSettleTime);
    

#ifdef CONFIRM_MOVE
    cout << "At point.  Enter 1." << endl;
    cin >> x;
#endif
    //! Close gripper
    param = 255;
    gh.robHand->SetParameter("POSITION_FINGER_A", &param);
    gh.robHand->SetParameter("POSITION_FINGER_B", &param);
    gh.robHand->SetParameter("POSITION_FINGER_C", &param);
    param = 1;
    gh.robHand->SetParameter("GRIP", &param);

    //! Retract, holding peg
    while (gh.robArm->MoveStraightTo(pegMatrix.at(pegIndex)) != CANON_SUCCESS)
    {}
      pause.waitUntil(robotSettleTime);
    

#ifdef CONFIRM_MOVE
    cout << "At point.  Enter 1." << endl;
    cin >> x;
#endif

#ifdef TEST_ASSEMBLY

    //! Move over post
    while (gh.robArm->MoveStraightTo(testHolePose) != CANON_SUCCESS)
    {}
      pause.waitUntil(robotSettleTime);
    

#ifdef CONFIRM_MOVE
    cout << "At point.  Enter 1." << endl;
    cin >> x;
#endif
    //! We are now nominally aligned with the column

    double zDelta = 0.5f; //! 0.5 mm
    bool goodForce = true;
    bool goodMove = true;
    robotPose origPose;
    robotPose origForces, curForces, diffForces;
    double forceThresh = 30.0f;
    double hover = 20.0f,
           insert = 25.0f;

    //! Tare forces and pose
    gh.robArm->GetRobotForces(&origForces);
//    gh.robArm->GetRobotPose(&origPose);

    origPose = testHolePose;
    //! Define hover and insertion poses (relative to current position)
    //! Hover = current - d_hover
    //! Insertion = hover - d_insert
    poseMe = origPose;
    poseMe.z -= hover;
    tarPose = poseMe;
    tarPose.z -= insert;

    //! Move to pre-assemble hover pose
    while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
    {}
      pause.waitUntil(robotSettleTime);
    

#ifdef CONFIRM_MOVE
    cout << "At point.  Enter 1." << endl;
    cin >> x;
#endif

    do
    {
      //! Move down one incremental unit
      poseMe.z -= zDelta;

      while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
      {}
        pause.waitUntil(robotSettleTime);
      

      gh.robArm->GetRobotPose(&curPose);
      gh.robArm->GetRobotForces(&curForces);
      diffForces = curForces - origForces;

      if (fabs(diffForces.z) > forceThresh)
      {
        goodForce = false;
        break;
      }
    } while (curPose.z >= tarPose.z);

    if (goodForce)
    {
      cout << "YAY!" << endl;
      //! Insertion succeeded
      //! Release peg
      param = 20;
      gh.robHand->SetParameter("POSITION_FINGER_A", &param);
      gh.robHand->SetParameter("POSITION_FINGER_B", &param);
      gh.robHand->SetParameter("POSITION_FINGER_C", &param);
      param = 1;
      gh.robHand->SetParameter("GRIP", &param);

      //! Retract
      poseMe = origPose;
      while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
      {}
        pause.waitUntil(robotSettleTime);
      

      retVal = CANON_SUCCESS;
      //! Ready to acquire next peg
    }
    else
    {
      //! Insertion failed
      cout << "boooooo!" << endl;
      //! Retract
      poseMe = origPose;
      while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
      {}
        pause.waitUntil(robotSettleTime);
      

      //! Put failed peg in bin (likely not a good grip anymore)
      //! Move over bin
      while (gh.robArm->MoveStraightTo(binPose) != CANON_SUCCESS)
      {}
        pause.waitUntil(robotSettleTime);
      

      //! Release peg
      param = 20;
      gh.robHand->SetParameter("POSITION_FINGER_A", &param);
      gh.robHand->SetParameter("POSITION_FINGER_B", &param);
      gh.robHand->SetParameter("POSITION_FINGER_C", &param);
      param = 1;
      gh.robHand->SetParameter("GRIP", &param);

      retVal = CANON_FAILURE;

      //! Ready to acquire next peg
    }
#else
    //! Drop part in waste bin

    //! Move over bin
    while (gh.robArm->MoveStraightTo(binPose) != CANON_SUCCESS)
    {}
      pause.waitUntil(robotSettleTime);
    

    //! Release peg
    param = 40;
    gh.robHand->SetParameter("POSITION_FINGER_A", &param);
    gh.robHand->SetParameter("POSITION_FINGER_B", &param);
    gh.robHand->SetParameter("POSITION_FINGER_C", &param);
    param = 1;
    gh.robHand->SetParameter("GRIP", &param);
#endif
  }
  return;

#else

  pause.waitUntil(robotSettleTime);
#ifdef CLASSIC_LASER
  gh.robArm->Couple("laser");
#else
  gh.robArm->Couple("robotiq_laser");
#endif
  gh.status = CANON_SUCCESS;

  // To get scanner sensor data from robot
  robotIO io;
  bool poseDefined = false;
  bool tool = false;
  bool overbin = false;
  int counter; 
  //ofstream out (("mobman_log_" + timestamp + ".csv").c_str());

  double theta = 0.0f;
  bool updateTheta = false;

  // initialize stow poses
  robotPose stagePose1, stagePose2, stowPose;
  setPosition( stagePose1, stagePoint1, sensorRot );
  setPosition( stagePose2, stagePoint2, sensorRot );
  setPosition( stowPose, stowPoint, sensorRot );

  // compute AGV coordiantes of start points based on configuration values
  squarePoint1World = squareAgvPose * agvToRobot * squarePoint1Robot;
  squarePoint2World = squareAgvPose * agvToRobot * squarePoint2Robot; 
  circlePoint1World = circleAgvPose * agvToRobot * circlePoint1Robot;
  circlePoint2World = circleAgvPose * agvToRobot * circlePoint2Robot;

  PM_CARTESIAN largeSquarePoint1World, largeSquarePoint2World;
  PM_CARTESIAN largeCirclePoint1World, largeCirclePoint2World;
  largeSquarePoint1World = largeSquareAgvPose * agvToRobot * largeSquarePoint1Robot;
  largeSquarePoint2World = largeSquareAgvPose * agvToRobot * largeSquarePoint2Robot; 
  largeCirclePoint1World = largeCircleAgvPose * agvToRobot * largeCirclePoint1Robot;
  largeCirclePoint2World = largeCircleAgvPose * agvToRobot * largeCirclePoint2Robot;

  // wait for robot arm to connect
  pause.waitUntil( robotSettleTime );
  gh.robArm->GetRobotPose(&curPose);
  while( curPose.x==0 && curPose.y==0 && curPose.z==0 )
  {
    cout << "Waiting for connection to robot arm controller...." << endl;
    pause.waitUntil( 2000 );
    gh.robArm->GetRobotPose(&curPose);
  }

  // give background threads time to initialize
  ulapi_sleep( 5 );

  // Sense-decide-act loop
  bool running = true;
  while (running)
  {
    if( useManualContinue )
    {
      int start = 0;
      do
      {
        cout << endl << "Enter target id (1=square, 2=circle):  ";
        cin >> start;
      } while( start == 0 ); // while not 1 or 2?

      /*  agv.getStatus( agvX, agvY, agvAngle, agvTarget );*/

      /*  agv.setTarget( 0 );
      agv.getStatus( agvX, agvY, agvAngle, agvTarget );*/

      agvTarget = start;
    }
    else
    {
      cout << "Waiting for signal from AGV to begin..." << endl;
#ifdef BYPASS_AGV
      cout << "Bypassing AGV.  Setting target to " << AGV_TARGET << "." << endl;
      agvTarget = AGV_TARGET;
      agvX = AGV_X_;//8050.0;//6095.0;//5704.0;// 7135.0;
      agvY = AGV_Y_;//14090.0;//14915.0;//13870;//11810.0;
      agvAngle = AGV_THETA_;//270.0;//315.0;//0.0;//90.0;
#else
      agvTarget = 0;
      while( agvTarget == 0 )
      {
        cout << "Getting status..." << endl;
        agv.getStatus( agvX, agvY, agvAngle, agvTarget );
        ulapi_sleep(0.5);  // pause so we don't lock up the cpu
      }
#endif
    }
    cout << "Ready to begin... " << endl;

    // generate timestamped log files
    char stamp[100];
    time_t mytime;
    struct tm *mytm;
    mytime = time(NULL);
    int mytime_seconds = get_seconds();
    mytm = localtime( &mytime );
    strftime( stamp, sizeof(stamp), "%Y-%m-%d_%H.%M.%S", mytm );
    string timestamp( stamp );

    cout << "Log file timestamp is " << timestamp << endl;

    switch( agvTarget )
    {
    case 1:
      logger.open(("logs/log_" + timestamp + "_square.dat").c_str());
      out.open(("logs/data_" + timestamp + "_square.csv").c_str());
      break;
    case 2:
      logger.open(("logs/log_" + timestamp + "_circle.dat").c_str());
      out.open(("logs/data_" + timestamp + "_circle.csv").c_str());
      break;
    default:
      cout << "Invalid target ID = " << agvTarget << endl;
      continue;
      break;
    }

    // write out header line to CSV file
    out << "time, iter, marker, init_x, init_y, init_z, init_xrot, init_yrot, "
        << "init_zrot, ok, time, steps, x, y, z, xrot, yrot, zrot" << endl;

    // read position from status link
    ulapi_sleep( 2.0 );  // wait for position to settle
    AGV_State agv_state;
    double sumX = 0;
    double sumY = 0;
    double sumAngle = 0;
    double offsetAngle;
    double minX, maxX, minY, maxY, minAngle, maxAngle;

#ifndef BYPASS_AGV
    cout << "Averaging " << agvPositionSamples << " positions" << endl;
    logger << "Averaging " << agvPositionSamples << " positions" << endl;
    for( int i=0; i<agvPositionSamples; ++i )
    {
      agv_state = agv_status.get_state();
      ulapi_sleep( 0.060 );  // FIXME - watch for timestamp change
      sumX += agv_state.pos_x * 1000;
      sumY += agv_state.pos_y * 1000;
      if( i == 0 )
      {
        offsetAngle = agv_state.pos_th * TO_DEG;

        minX = maxX = agv_state.pos_x;
        minY = maxY = agv_state.pos_y;
        minAngle = maxAngle = 0;
      }
      double angle = agv_state.pos_th * TO_DEG - offsetAngle;
      if( angle > 180 )
        angle = angle - 360;
      if( angle < -180 )
        angle = angle + 360;
      sumAngle += angle;

      // collect sample statistics
      minX = min( minX, agv_state.pos_x );
      maxX = max( maxX, agv_state.pos_x );
      minY = min( minY, agv_state.pos_y );
      maxY = max( maxY, agv_state.pos_y );
      minAngle = min( minAngle, angle );
      maxAngle = max( maxAngle, angle );
    }
    if( agvPositionSamples > 1 )
    {
      cout << "Sample spread: " << (maxX - minX) * 1000 << "mm " 
           << (maxY - minY) * 1000 << "mm "
           << maxAngle - minAngle << "deg" << endl;
      logger << "Sample spread: " << (maxX - minX) * 1000 << "mm " 
             << (maxY - minY) * 1000 << "mm "
             << maxAngle - minAngle << "deg" << endl;
    }
    agvX = sumX / agvPositionSamples;  // convert position to millimeters
    agvY = sumY / agvPositionSamples;
    agvAngle = sumAngle / agvPositionSamples + offsetAngle;  // convert angle to degrees

    cout << "Vehicle position = " << agvX << " " << agvY << " " << agvAngle << endl;
    logger << "Vehicle position = " << agvX << " " << agvY << " " << agvAngle << endl;
#else
    agvTarget = AGV_TARGET;
#endif

    switch( agvTarget )
    {
    case 1:
      cout << "Starting square" << endl;
      logger << "Starting square" << endl;
      break;
    case 2:
      cout << "Starting circle" << endl;
      logger << "Starting circle" << endl;
      break;
    default:
      cout << "Unrecognized target number " << agvTarget << endl;
      logger << "Unrecognized target number " << agvTarget << endl;

#ifndef BYPASS_AGV
      // signal AGV that robot is done and wait for new target
      agv.setTarget( 0 );
#endif
      continue;
      break;
    }

    // move to safe approach point
    if( stowArm )
    {
      // move robot out of stow position to stagePoint2 which should provide a clean
      //   straight line motion to the first search location (and back from final
      //   search location)

      // assume arm starts at or near the stowPoint
      // move to stagePoint2 by way of stagePoint1, to go around robot base
      while (gh.robArm->MoveStraightTo( stagePose1 ) != CANON_SUCCESS)
      {}
      while (gh.robArm->MoveStraightTo( stagePose2 ) != CANON_SUCCESS)
      {}
    }
    else
    {
      // record initial location and return to it later
      gh.robArm->GetRobotPose(&initPose);
      cout << "Recording home position: " << initPose.x << " " << initPose.y << " " << initPose.z << endl;
      logger << "Recording home position: " << initPose.x << " " << initPose.y << " " << initPose.z << endl;
    }

    bool aligned = true;
    PM_CARTESIAN point1, point2;
    robotPose refPoint1, refPoint2;
#ifdef BYPASS_AGV
    agvX = AGV_X_;//8050.0;//5704.0;//7135.0;
    agvY = AGV_Y_;//14090.0;//14915.0;//13870.0;//11810.0;
    agvAngle = AGV_THETA_;//270.0;//315.0;//0.0;//90.0;
#endif

    PM_POSE agvPose = PM_POSE( PM_CARTESIAN(agvX, agvY, 0), PM_RPY(0, 0, agvAngle * TO_RAD));
    if( useLargeTargets ) // bisection search
    {
      TargetOffset targetOffset;
      bool targetFound = false;
      if( useTargetOffsets || recordTargetOffsets )
      {
        // check to see if we have a matching location in the list
        targetOffset.agvX = agvX;
        targetOffset.agvY = agvY;
        targetOffset.agvAngle = agvAngle;
        targetOffset.target = agvTarget;
        targetFound = findTargetOffset( targetOffset,  targetOffsetList );

        cout << "Search key: " << agvTarget << " " << agvX << " " << agvY 
             << " " << agvAngle << endl;
        logger << "Search key: " << agvTarget << " " << agvX << " " << agvY 
               << " " << agvAngle << endl;

        if( targetFound )
        {
        cout << "Target found: " << targetOffset.target << " " 
             << targetOffset.agvX << " " << targetOffset.agvY << " " 
             << targetOffset.agvAngle << endl;
        cout << "   (" << targetOffset.point1X << " " << targetOffset.point1Y
             << ") (" << targetOffset.point2X << " " << targetOffset.point2Y 
             << ")" << endl;

        logger << "Target found: " << targetOffset.target << " " 
               << targetOffset.agvX << " " << targetOffset.agvY << " " 
               << targetOffset.agvAngle << endl;
        logger << "   (" << targetOffset.point1X << " " << targetOffset.point1Y
               << ") (" << targetOffset.point2X << " " << targetOffset.point2Y 
               << ")" << endl;
        }
        else
        {
          cout << "Target not found in database" << endl;
          logger << "Target not found in database" << endl;
        }
      }

      if( !targetFound )
      {
        if( useTargetOffsets )
        {
          cout << "Did not find docking location in database" << endl;
          logger << "Did not find docking location in database" << endl;
        }

        // compute initial locations of large markers
        switch( agvTarget )
        {
        case 1: // square
          point1 = inv(agvToRobot) * inv(agvPose) * largeSquarePoint1World;
          point2 = inv(agvToRobot) * inv(agvPose) * largeSquarePoint2World;
          break;

        case 2: // circle
          point1 = inv(agvToRobot) * inv(agvPose) * largeCirclePoint1World;
          point2 = inv(agvToRobot) * inv(agvPose) * largeCirclePoint2World;
          break;

        default:
          cout << "Unknown target id value (" << agvTarget << ")"  << endl;  
          break;
        }
        robotPose startPoint1, startPoint2;
        double searchTime1, searchTime2;
        int stepCount1, stepCount2;
        int success1 = 1;
        int success2 = 1;
        startPoint1 = refPoint1 = cart2rob( point1, basePose );
        startPoint2 = refPoint2 = cart2rob( point2, basePose );

#ifdef CLASSIC_LASER
        gh.robArm->Couple("laser");
#else
        gh.robArm->Couple("robotiq_laser");
#endif
        pause.waitUntil(robotSettleTime);

        // move to markers and refine position
        while (gh.robArm->MoveStraightTo( refPoint1 ) != CANON_SUCCESS)
        {}
        double startPoint1_time = time_since( mytime_seconds );
        overbin = false;

        cout << "running bisect" << endl;
        refPoint1.print();
        refPoint1.z = 110.0f;
        refPoint1.print();
        if( !bisect( logger, gh.robArm, refPoint1, largeTargetStepSize, &stepCount1, &searchTime1 ) )
        {
          cout << "Error finding center of reflector 1" << endl;
          logger << "Error finding center of reflector 1" << endl;
          aligned = false;
          success1 = 0;
        }
        if( scanTargets )
        {
          string filename( "logs/refPoint1-" + timestamp );
          cout << "Doing a raster scan of first reflector.  Saving in " << filename << endl;
          logger << "Doing a raster scan of first reflector.  Saving in " << filename << endl;
          runTargetScan( gh.robArm, filename, scanTargetStep, refPoint1 );
        }

        while (gh.robArm->MoveStraightTo( refPoint2 ) != CANON_SUCCESS)
        {}
        double startPoint2_time = time_since( mytime_seconds );

        refPoint2.z = 160.0f;
        if( !bisect( logger, gh.robArm, refPoint2, largeTargetStepSize, &stepCount2, &searchTime2 ) )
        {
          cout << "Error finding center of reflector 2" << endl;
          logger << "Error finding center of reflector 2" << endl;
          aligned = false;
          success2 = 0;
        }
        if( scanTargets )
        {
          string filename( "logs/refPoint2-" + timestamp );
          cout << "Doing a raster scan of second reflector.  Saving in " << filename << endl;
          logger << "Doing a raster scan of second reflector.  Saving in " << filename << endl;
          runTargetScan( gh.robArm, filename, scanTargetStep, refPoint2 );
        }

        // write large reflector stats to CSV file with target id -1 and -2
        out << startPoint1_time << ", ";
        out << "0, -1, " << startPoint1.x << ", " << startPoint1.y << ", " << startPoint1.z << ", "
            << startPoint1.xrot << ", " << startPoint1.yrot << ", " << startPoint1.zrot;
        out << ", " << success1 << ", " << searchTime1 << ", " << stepCount1;
        out << ", " << refPoint1.x << ", " << refPoint1.y << ", " << refPoint1.z << ", "
            << refPoint1.xrot << ", " << refPoint1.yrot << ", " << refPoint1.zrot << endl;

        out << startPoint2_time << ", ";
        out << "0, -2, " << startPoint2.x << ", " << startPoint2.y << ", " << startPoint2.z << ", "
            << startPoint2.xrot << ", " << startPoint2.yrot << ", " << startPoint2.zrot;
        out << ", " << success2 << ", " << searchTime2 << ", " << stepCount2;
        out << ", " << refPoint2.x << ", " << refPoint2.y << ", " << refPoint2.z << ", "
            << refPoint2.xrot << ", " << refPoint2.yrot << ", " << refPoint2.zrot << endl;

        if( recordTargetOffsets || useTargetOffsets )
        {
          // transform points into world coordinates
          PM_CARTESIAN point1World, point2World;
          point1World = agvPose * agvToRobot * rob2cart( refPoint1 );
          point2World = agvPose * agvToRobot * rob2cart( refPoint2 );

          // save search results in target offset list
          targetOffset.point1X = point1World.x;
          targetOffset.point1Y = point1World.y;
          targetOffset.point2X = point2World.x;
          targetOffset.point2Y = point2World.y;
          targetOffsetList.push_back( targetOffset );

          if( recordTargetOffsets )
          {
            // write search results to target offsets file
            writeTargetOffset( targetOffsetsStream, targetOffset );
          }
        }
      }
      else
      {
        if( recordTargetOffsets )
        {
          cout << "Current location already recorded in database" << endl;
          logger << "Current location already recorded in database" << endl;
        }

        // use points from the recorded offset to compute reference points
        refPoint1 = cart2rob( inv(agvToRobot) * inv(agvPose) 
          * PM_CARTESIAN( targetOffset.point1X, targetOffset.point1Y, sensorHeight ), basePose );
        refPoint2 = cart2rob( inv(agvToRobot) * inv(agvPose) 
          * PM_CARTESIAN( targetOffset.point2X, targetOffset.point2Y, sensorHeight ), basePose );
      }

      // compute index locations
      point2Offset = rob2cart(refPoint2 - refPoint1);
      point2Offset.z = 0;
      PM_CARTESIAN offset90 = unit( PM_QUATERNION( PM_Z, PM_PI/2 ) * point2Offset );
      switch( agvTarget )
      {
      case 1: // square
      {
        PM_CARTESIAN cornerOffset = (-8 * 25.4) * offset90;
        position[0] = cart2rob(rob2cart(refPoint1) + cornerOffset, basePose);
        break;
      }
      case 2: // circle
      {
        PM_CARTESIAN firstPoint = (rob2cart(refPoint1) + rob2cart(refPoint2)) / 2; // center
        firstPoint = firstPoint + offset90 * (6 * 25.4);
        position[0] = cart2rob(firstPoint, basePose);

        // recompute offset
        point2Offset = offset90 * (-12 * 25.4);
        break;
      }
      default:
        cout << "Unknown target id value (" << agvTarget << ")"  << endl;  
        break;
      }
    }
    else
    {
      // compute start point using AGV position
      switch( agvTarget )
      {
      case 1: // square
        point1 = inv(agvToRobot) * inv(agvPose) * squarePoint1World;
        point2 = inv(agvToRobot) * inv(agvPose) * squarePoint2World;
        break;

      case 2: // circle
        point1 = inv(agvToRobot) * inv(agvPose) * circlePoint1World;
        point2 = inv(agvToRobot) * inv(agvPose) * circlePoint2World;
        break;

      default:
        cout << "Unknown target id value (" << agvTarget << ")"  << endl;  
        break;
      }
      point2Offset = point2 - point1;  
      position[0] = cart2rob( point1, basePose );
    }

    // iterate through the search
    gh.status = CANON_RUNNING;
    cout << "running " << numIters << " iterations" << endl;
    logger << "running " << numIters << " iterations" << endl;

    if( aligned )
    {
      int i = 0;
      //for (int i = 0; i < numIters; ++i)
      {
        int numPoints = 0;
        switch( agvTarget )
        {
        case 1:
          numPoints = 4;
          break;
        case 2:
          numPoints = 6;
          break;
        }

        cout << "Starting iteration " << i << endl;
        logger << "Starting iteration " << i << endl;

        int x;
        pegIndex = 1;
        //! Begin search/assembly
        //for (int j = 0; j < numPoints; ++j)
        for (int j = 0; j < 2; ++j)
        {
          robotPose oldPose;

#ifdef RUN_ASSEMBLY

#ifdef REFINE_POSE_FIRST
          if (j <= 1)
          {
            cout << "Verifying point " << (j+1) << endl;
            pause.waitUntil(robotSettleTime);
#ifdef CLASSIC_LASER
            gh.robArm->Couple("laser");
#else
            gh.robArm->Couple("robotiq_laser");
#endif

            //! Move to position j
            cout << "Moving to initial search position [" << j << "]... ";
#ifdef LOGEVERYTHING
            logger << ulapi_time() << " Moving to position [" << j << "] : [" << position[j].x << ", " << position[j].y
                   << ", " << position[j].z << ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot
                   << "]" << endl;
#endif
            double ts = time_since(mytime_seconds);
            out << ts << ", ";
            out << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
                << ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";

            position[j].z = 148.0f;
            cout << "Moving to:" << endl;
            position[j].print();

            if (overbin)
            {
              //! If we're over the bin, get back over to the table
              while (gh.robArm->MoveStraightTo( stagePose1 ) != CANON_SUCCESS)
              {
              }

              while (gh.robArm->MoveStraightTo( stagePose2 ) != CANON_SUCCESS)
              {
              }
            }
            overbin = false;

            while (gh.robArm->MoveStraightTo(position[j]) != CANON_SUCCESS)
            {
            }
              pause.waitUntil(robotSettleTime);

              gh.robArm->GetRobotPose(&curPose);
              gh.robArm->GetRobotIO(&io);
#ifdef NOISY
              cout << "at position[" << j << "]." << endl;
              logger << "at position[" << j << "]." << endl;
#endif
#ifdef LOGEVERYTHING
              logger << ulapi_time() << " at [" << curPose.x << ", " << curPose.y << ", "
                     << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", "
                     << curPose.zrot << "]" << endl;
#endif
//            }
//            else
//            {
//              cout << "*** Move command failed! ***" << endl;
//            }

            timer.start();//.startTimer();
            counter = 0;
            do
            {
              gh.robArm->GetRobotIO(&io);
              //cout << "IO[9] : " << io.dio[9] << endl;
              io.dio[9] = !io.dio[9];
              retVal = square_spiral.RunAssemblyStep (counter++, curPose, poseMe, io);
              if (retVal != CANON_RUNNING)
              {
                if (retVal == CANON_SUCCESS)
                {
                  cout << "found it!" << endl;
                }
                else
                {
                  cout << "error!" << endl;
                }
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
              logger << "X: " << poseMe.x << " Y: " << poseMe.y << " Z: " << poseMe.z << endl;
#endif
              while (gh.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS)
              {
              }
              pause.waitUntil( robotSettleTime );

              ++counter;
            } while (retVal == CANON_RUNNING);
            double tim = timer.elapsedTime() / 1000.0;  // convert from milliseconds to seconds
            timer.stop();
#ifdef NOISY
            logger << "Spiral search stopped " << (retVal == CANON_SUCCESS ? "successfully" : "unsuccessfully") << " at offset ["
                   << (poseMe.x - position[j].x) << ", " << (poseMe.y - position[j].y) << "] after " << tim << " seconds and " 
                   << counter << " steps." << endl;
#endif
#ifdef LOGEVERYTHING
            logger << ulapi_time() << " Search stopped " << (retVal == CANON_SUCCESS ? "successfully" : "unsuccessfully")
                   << " at offset [" << (poseMe.x - position[j].x) << ", " << (poseMe.y - position[j].y) << "] after " 
                   << tim << " seconds and " << counter << " steps." << endl;
#endif
            //! Record position[j] in log file
            ts = time_since( mytime_seconds );
            out << ts << ", ";
            out << (retVal == CANON_SUCCESS ? "1" : "0") << ", " << tim << ", ";
            out << counter << ", ";
            out << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " 
                << poseMe.yrot << ", " << poseMe.zrot << endl;
            if (retVal == CANON_SUCCESS)
            {
              oldPose = position[j];
              cout << "Updating search pose..." << endl;
              cout << "Original: ";
              oldPose.print();
              cout << endl << "New: ";
              poseMe.print();
              cout << endl;
              position[j] = poseMe;
            }
          } // if (j <= 1)
#endif
          pause.waitUntil(robotSettleTime);
          gh.robArm->Couple("robotiq");
          pause.waitUntil(robotSettleTime);
          gh.robArm->SetRelativeSpeed(0.5);
          pause.waitUntil(robotSettleTime);

          cout << "we are over the bin: " << (overbin ? "true" : "false") << endl;
//          cin >> x;
          if (!overbin)
          {
            //! At this point we should be over the artifact.  Move back to stow to acquire the next peg.
            //! Otherwise we're over the bin, and thus don't have to worry about going through these motions.
            while (gh.robArm->MoveStraightTo( stagePose2 ) != CANON_SUCCESS)
            {
            }
            while (gh.robArm->MoveStraightTo( stagePose1 ) != CANON_SUCCESS)
            {
            }
          }
          overbin = false;

          //! Get next peg
          double pegDepth = 140.0f;
          int param;
          //! Move to hover position over next peg
          while (gh.robArm->MoveStraightTo(pegMatrix.at(pegIndex)) != CANON_SUCCESS)
          {
          }
          pause.waitUntil(robotSettleTime);

#ifdef CONFIRM_MOVE
          cout << "At point.  Enter 1." << endl;
          cin >> x;
#endif

          //! Open gripper
          //PreGrasp
          param = 1;
          hand.SetParameter("ADVANCED_CONTROL", &param);
          param = 80;
          hand.SetParameter("SPEED_FINGER_A", &param);
          hand.SetParameter("SPEED_FINGER_B", &param);
          hand.SetParameter("SPEED_FINGER_C", &param);
          param = 0;
          hand.SetParameter("FORCE_FINGER_A", &param);
          hand.SetParameter("FORCE_FINGER_B", &param);
          hand.SetParameter("FORCE_FINGER_C", &param);
          param = 60;
          gh.robHand->SetParameter("POSITION_FINGER_A", &param);
          gh.robHand->SetParameter("POSITION_FINGER_B", &param);
          gh.robHand->SetParameter("POSITION_FINGER_C", &param);
          param = 1;
          hand.SetParameter("GRIP", &param);

          //! Move to acquire depth
          poseMe = pegMatrix.at(pegIndex);
          poseMe.z -= pegDepth;
          while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
          {}
            pause.waitUntil(robotSettleTime);
          

#ifdef CONFIRM_MOVE
          cout << "At point.  Enter 1." << endl;
          cin >> x;
#endif

          //! Close gripper
          param = 255;
          gh.robHand->SetParameter("POSITION_FINGER_A", &param);
          gh.robHand->SetParameter("POSITION_FINGER_B", &param);
          gh.robHand->SetParameter("POSITION_FINGER_C", &param);
          param = 1;
          gh.robHand->SetParameter("GRIP", &param);

#ifdef CONFIRM_MOVE
          cout << "At point.  Enter 1." << endl;
          cin >> x;
#endif          

          //! Retract, holding peg
          while (gh.robArm->MoveStraightTo(pegMatrix.at(pegIndex)) != CANON_SUCCESS)
          {
          }
            pause.waitUntil(robotSettleTime);
         

#ifdef CONFIRM_MOVE
          cout << "At point.  Enter 1." << endl;
          cin >> x;
#endif

          //! Advance peg index
          //++pegIndex;
          pegIndex += 2;
          if (pegIndex > 9)
          {
            pegIndex = 1;
          }

          //! Move through stow poses to make sure we don't try to go through the robot itself
          while (gh.robArm->MoveStraightTo( stagePose1 ) != CANON_SUCCESS)
          {}
          while (gh.robArm->MoveStraightTo( stagePose2 ) != CANON_SUCCESS)
          {}
          //! JAM: At this point we should have the peg, and be nominally close to the table.
#endif

          //! Move to position j
          cout << "Moving to initial search position [" << j << "]... ";
#ifdef LOGEVERYTHING
          logger << ulapi_time() << " Moving to position [" << j << "] : [" << position[j].x << ", " << position[j].y
                 << ", " << position[j].z << ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot
                 << "]" << endl;
#endif
          double ts = time_since(mytime_seconds);
          out << ts << ", ";
          out << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z
              << ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";

          pause.waitUntil(robotSettleTime);
#ifndef RUN_ASSEMBLY
          //! Not running the assembly.  Using the laser to align with the pegs.  Otherwise continue to use Robotiq tool.
#ifdef CLASSIC_LASER
          gh.robArm->Couple("laser");
#else
          gh.robArm->Couple("robotiq_laser");
#endif
#endif
          position[j].z = 148.0f;
          //position[j].x -= 2.0f;
          position[j].y += 2.0f;
          position[j].zrot = 45.0f;
          cout << "going to: " << endl;
          position[j].print();
          cout << endl;
          while (gh.robArm->MoveStraightTo(position[j]) != CANON_SUCCESS)
          {
          }
            pause.waitUntil(robotSettleTime);

            gh.robArm->GetRobotPose(&curPose);
            gh.robArm->GetRobotIO(&io);
#ifdef NOISY
            cout << "at position[" << j << "]." << endl;
            logger << "at position[" << j << "]." << endl;
#endif
#ifdef LOGEVERYTHING
            logger << ulapi_time() << " at [" << curPose.x << ", " << curPose.y << ", "
              << curPose.z << ", " << curPose.xrot << ", " << curPose.yrot << ", "
              << curPose.zrot << "]" << endl;
#endif
//          }
//          else
//          {
//            cout << "*** Move command failed! ***" << endl;
//          }

#ifdef CONFIRM_MOVE
          cout << "At point.  Enter 1." << endl;
          cin >> x;
#endif

          //! We are now nominally aligned with the column

#ifdef RUN_ASSEMBLY
          //! JAM:  Using onboard gripper to run physical assembly (assuming UR10, but could be anything)
          double zDelta = 0.5f; //! 0.5 mm
          bool goodForce = true;
          bool goodMove = true;
          robotPose origPose;
          robotPose origForces, curForces, diffForces;
          double forceThresh = 30.0f;
          double hover = 20.0f,
                 insert = 25.0f;

          //! Tare forces and pose
          gh.robArm->GetRobotForces(&origForces);
          //gh.robArm->GetRobotPose(&origPose);
          origPose = position[j];

#ifdef CONFIRM_MOVE
          cout << "At point.  Enter 1." << endl;
          cin >> x;
#endif

          //! Define hover and insertion poses (relative to current position)
          //! Hover = current - d_hover
          //! Insertion = hover - d_insert
          poseMe = origPose;
          poseMe.z -= hover;
          tarPose = poseMe;
          tarPose.z -= insert;

          //! Move to pre-assemble hover pose
          while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
          {
          }
          pause.waitUntil(robotSettleTime);
          goodMove = true;

#ifdef CONFIRM_MOVE
          cout << "At point.  Enter 1." << endl;
          cin >> x;
#endif

          if (goodMove)
          {
            do
            {
              //! Move down one incremental unit
              poseMe.z -= zDelta;

              while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
              {
              }
              pause.waitUntil(robotSettleTime);

              gh.robArm->GetRobotPose(&curPose);
              gh.robArm->GetRobotForces(&curForces);
              diffForces = curForces - origForces;

              if (fabs(diffForces.z) > forceThresh)
              {
                goodForce = false;
                break;
              }
            } while (curPose.z >= tarPose.z);
          } // if (goodMove)

          if (!goodMove)
          {
            /*
            cout << "Could not move to insertion pose.  Insertion failed.  Aborting." << endl;

            //! Drop part in waste bin
            //! Retract
            poseMe = origPose;
            while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
            {
            }
              pause.waitUntil(robotSettleTime);
            

            //! Move through stow poses to make sure we don't try to go through the robot itself
            while (gh.robArm->MoveStraightTo( stagePose2 ) != CANON_SUCCESS)
            {}
            while (gh.robArm->MoveStraightTo( stagePose1 ) != CANON_SUCCESS)
            {}

            //! Put failed peg in bin (likely not a good grip anymore)
            //! Move over bin
            while (gh.robArm->MoveStraightTo(binPose) != CANON_SUCCESS)
            {
            }
              pause.waitUntil(robotSettleTime);
            //}

            //! Release peg
            param = 40;
            gh.robHand->SetParameter("POSITION_FINGER_A", &param);
            gh.robHand->SetParameter("POSITION_FINGER_B", &param);
            gh.robHand->SetParameter("POSITION_FINGER_C", &param);
            param = 1;
            gh.robHand->SetParameter("GRIP", &param);

            retVal = CANON_FAILURE;
#ifdef LOGEVERYTHING
            logger << ulapi_time() << "insertion failed" << endl;
#endif
            overbin = true;
            */
          } // if (!goodMove)
          else
          {
            if (goodForce)
            {
              cout << "Insert succeeded!" << endl;
              //! Insertion succeeded
              //! Release peg
              param = 20;
              gh.robHand->SetParameter("POSITION_FINGER_A", &param);
              gh.robHand->SetParameter("POSITION_FINGER_B", &param);
              gh.robHand->SetParameter("POSITION_FINGER_C", &param);
              param = 1;
              gh.robHand->SetParameter("GRIP", &param);

              //! Retract
              poseMe = origPose;
              while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
              {}
                pause.waitUntil(robotSettleTime);
              

              retVal = CANON_SUCCESS;
              //! Ready to acquire next peg
            }
            else
            {
              //! Insertion failed
              cout << "Insert failed!" << endl;
              //! Retract
              poseMe = origPose;
              while (gh.robArm->MoveStraightTo(poseMe) != CANON_SUCCESS)
              {
              }
                pause.waitUntil(robotSettleTime);
              

              //! Move through stow poses to make sure we don't try to go through the robot itself
              while (gh.robArm->MoveStraightTo( stagePose2 ) != CANON_SUCCESS)
              {}
              while (gh.robArm->MoveStraightTo( stagePose1 ) != CANON_SUCCESS)
              {}

              //! Put failed peg in bin (likely not a good grip anymore)
              //! Move over bin
              while (gh.robArm->MoveStraightTo(binPose) != CANON_SUCCESS)
              {}
                pause.waitUntil(robotSettleTime);
              

              //! Release peg
              param = 20;
              gh.robHand->SetParameter("POSITION_FINGER_A", &param);
              gh.robHand->SetParameter("POSITION_FINGER_B", &param);
              gh.robHand->SetParameter("POSITION_FINGER_C", &param);
              param = 1;
              gh.robHand->SetParameter("GRIP", &param);

              retVal = CANON_FAILURE;

              //! Ready to acquire next peg
              overbin = true;
            }
            //! Record results
#ifdef LOGEVERYTHING
            logger << ulapi_time() << "insertion " << ((retVal == CANON_SUCCESS) ? "succeeded" : "failed") << " at ["
                   << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " << curPose.xrot << ", "
                   << curPose.yrot << ", " << curPose.zrot << "]" << endl;
#endif
#ifdef SUPERNOISY
            cout << "X: " << curPose.x << " Y: " << curPose.y << " Z: " << curPose.z << endl;
            logger << "X: " << curPose.x << " Y: " << curPose.y << " Z: " << curPose.z << endl;
#endif
          } // if (!goodMove) ... else

#else // ifdef RUN_ASSEMBLY
          //! JAM:  Using laser as virtual assembly (tigher tolerances)
          //! Run spiral search at position[j]
#ifdef NOISY
          cout << "Running spiral search..." << endl;
          logger << "Running spiral search..." << endl;
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
            retVal = square_spiral.RunAssemblyStep (counter++, curPose, poseMe, io);
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
            logger << "X: " << poseMe.x << " Y: " << poseMe.y << " Z: " << poseMe.z << endl;
#endif
            while (gh.robArm->MoveStraightTo (poseMe) != CANON_SUCCESS)
            {}
            pause.waitUntil( robotSettleTime );

            ++counter;
          } while (retVal == CANON_RUNNING);
          double tim = timer.elapsedTime() / 1000.0;  // convert from milliseconds to seconds
          timer.stop();
#ifdef NOISY
          logger << "Spiral search stopped " << (retVal == CANON_SUCCESS ? "successfully" : "unsuccessfully") << " at offset ["
                 << (poseMe.x - position[j].x) << ", " << (poseMe.y - position[j].y) << "] after " << tim << " seconds and " 
                 << counter << " steps." << endl;
#endif
#ifdef LOGEVERYTHING
          logger << ulapi_time() << " Search stopped " << (retVal == CANON_SUCCESS ? "successfully" : "unsuccessfully")
                 << " at offset [" << (poseMe.x - position[j].x) << ", " << (poseMe.y - position[j].y) << "] after " 
                 << tim << " seconds and " << counter << " steps." << endl;
#endif
          //! Record position[j] in log file
          ts = time_since( mytime_seconds );
          out << ts << ", ";
          out << (retVal == CANON_SUCCESS ? "1" : "0") << ", " << tim << ", ";
          out << counter << ", ";
          out << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " 
              << poseMe.yrot << ", " << poseMe.zrot << endl;
          if (retVal == CANON_SUCCESS)
            position[j] = poseMe;
#endif // ifdef RUN_ASSEMBLY else

          pause.waitUntil(robotSettleTime);
#ifdef CLASSIC_LASER
          gh.robArm->Couple("laser");
#else
          gh.robArm->Couple("robotiq_laser");
#endif

#ifdef RUN_ASSEMBLY
          retVal = CANON_SUCCESS;
#endif

          //! Check on validity of the nominal assembly poses, correct if necessary
          //! Note:  Correction based on spiral search not possible with physical assembly, simply run-bisection
          if (i == 0)
          {
            //! First search iteration
            if (j == 0)
            {
              //! First point of the first search iteration
              if( retVal != CANON_SUCCESS )
              {
                cout << "Failed to find first point" << endl;
                logger << "Failed to find first point" << endl;
                i = numIters;  // make sure we dont do verification loops
                break;
              }

              // refine the target position
              if( useBisection )
              {
                int stepCount = 0;
                double searchTime = 0;
                bool ok = bisect( logger, gh.robArm, position[0], .1, &stepCount, &searchTime );
                if( ok )
                {
                  cout << "Bisection results: " << position[0].x << " " << position[0].y << endl;
                  logger << "Bisection results: " << position[0].x << " " << position[0].y << endl;
                }
                else 
                {
                  cout << "Bisection failed" << endl;
                  logger << "Bisection failed" << endl;
                }
              }
              /*
              position[1] = oldPose;
              position[1].x = oldPose.x + point2Offset.x;
              position[1].y = oldPose.y + point2Offset.y;
              position[1].z = oldPose.z;
              */
              
              position[1] = position[0];
              position[1].x = position[0].x + point2Offset.x;
              position[1].y = position[0].y + point2Offset.y;
              position[1].z = position[0].z;
            }
            else if (j == 1)
            {
              if( retVal != CANON_SUCCESS )
              {
                cout <<  "Failed to find second point" << endl;
                logger <<  "Failed to find second point" << endl;
                i = numIters;  // make sure we dont do verification loops
                break;
              }

              // refine the target position
              if( useBisection )
              {
                int stepCount = 0;
                double searchTime = 0;
                bool ok = bisect( logger, gh.robArm, position[1], .1, &stepCount, &searchTime );
                if( ok )
                {
                  cout << "Bisection results: " << position[1].x << " " << position[1].y << endl;
                  logger << "Bisection results: " << position[1].x << " " << position[1].y << endl;
                }
                else 
                {
                  cout << "Bisection failed" << endl;
                  logger << "Bisection failed" << endl;
                }
              }

              switch( agvTarget )
              {
              case 1: // square
                {
                  theta = atan2((position[1].y - position[0].y), (position[1].x - position[0].x));
                  double dx = (18.0 * 25.4) * cos( theta + PM_PI/2 );
                  double dy = (18.0 * 25.4) * sin( theta + PM_PI/2 );
                  position[2] = position[1];
                  position[3] = position[0];
                  position[2].x += dx;
                  position[2].y += dy;
                  position[3].x += dx;
                  position[3].y += dy;
                }
                break;
              case 2: // circle
                {
                  // find the center point of the circle and the angle for position[0]
                  robotPose center = position[0];
                  center.x = (position[0].x + position[1].x) / 2;
                  center.y = (position[0].y + position[1].y) / 2;
                  double t0 = atan2( position[0].y - position[1].y, position[0].x - position[1].x );

                  // recompute positions based on center, radius and initial angle
                  double radius = 6 * 25.4; // 6 inch radius
                  for( int a=0; a<6; ++a )
                  {
                    double angle = a * 60 * PM_PI / 180;
                    position[a] = center;  // to copy height and orientation from initial point
                    position[a].x = center.x + radius * cos(angle + t0);
                    position[a].y = center.y + radius * sin(angle + t0);
                  }

                  // skip directly to second loop where the robot will progress counter clockwise around the circle
                  j = numPoints;
                }
                break;
              }
            } // if (i == 0)
          } // for (j = 0; j < 4; ++j)

          cout << "Completed iteration " << i << endl;
          logger << "Completed iteration " << i << endl;
        } // for (int j = 0; j < numPoints; ++j)

      } // for (i = 0; i < numIter; ++i)
    }

    cout << "Finished pattern.  Stowing." << endl;
    // move robot back to stow location
    if( stowArm )
    {
      cout << "Stowing arm." << endl;
      // move stow point by way of stage point 2 and 1
#ifdef RUN_ASSEMBLY
      if (!overbin)
      {
        cout << "Go to interim 1" << endl;
        while (gh.robArm->MoveStraightTo( stagePose2 ) != CANON_SUCCESS)
        {
        }
        cout << "Go to interim 1" << endl;
        while (gh.robArm->MoveStraightTo( stagePose1 ) != CANON_SUCCESS)
        {
        }
        overbin = false;
      }
      else
      {
        cout << "Over bin" << endl;
      }
#endif
      while (gh.robArm->MoveStraightTo( stowPose ) != CANON_SUCCESS)
      {}
    } 
    else
    {
      gh.status = 4;
      cout << "Moving back to initial position: " << initPose.x << " " << initPose.y << " " << initPose.z << endl;
      logger << "Moving back to initial position: " << initPose.x << " " << initPose.y << " " << initPose.z << endl;

      while (gh.robArm->MoveStraightTo (initPose) != CANON_SUCCESS)
      {}
    }

    // signal AGV that robot is done
#ifndef BYPASS_AGV
    agv.setTarget( 0 );
#endif

    logger.close();
    out.close();
  } // while (i != -1)
#endif
  cout << "All done" << endl;
}
