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
#include "ulapi.h"
#include "../../Libraries/MotionPrims/AssemblyPrims.h"

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

using namespace crpi_robot;
using namespace MotionPrims;
using namespace std;

typedef CrpiUniversal robType;

// scale from inches to mm
const double IN_TO_MM = 25.4;

/////////////////////////////////////////////////////////////////////
// AR offset values. Target specific values are indexed by agvTarget-1
const int num_stations = 2;
PM_POSE tag_offset[num_stations]; // location of AR tag relative to pattern frame
PM_POSE point1_offset[num_stations];  // location of point1 relative to pattern frame
PM_POSE point2_offset[num_stations];  // location of point2 relative to pattern frame
PM_POSE camera_offset;  // position of camera relative to laser/tool frame
PM_POSE standoff;  // offset from target point to laser standoff position
PM_POSE robot_pose_init[num_stations]; // location of arm to view AR tag
PM_POSE agv_pose_init[num_stations];  // recorded location of AGV corresponding to robot_pose_init

//////////////////////////////////////////////////////////////////////
// Make output streams global so they can log data from subroutines.
ofstream logger;
ofstream out;

struct globalHandle
{
	ulapi_mutex_struct *handle; 
	char buffer[REQUEST_MSG_SIZE]; 
	bool runThread;
	bool remoteArmConnected;
	ulapi_integer remoteID;
	bool t1;
	CrpiRobot<robType> *robArm;
	int status;

	robotPose curPose, curForces, curSpeeds;

	globalHandle (CrpiRobot<robType>* armptr)
	{
		robArm = armptr;
		handle = ulapi_mutex_new(89);
	}

	~globalHandle ()
	{
		robArm = NULL;
	}
};

/////////////////////////////////////////////////////////////////////////////
// Parse a single line of comma seperated values.  Return as a vector
// of strings.
vector<string> getNextLineAndSplitIntoTokens(istream& str)
{
    std::vector<std::string>   result;
    std::string                line;
    std::getline(str,line);

    std::stringstream          lineStream(line);
    std::string                cell;

    while(std::getline(lineStream,cell, ','))
    {
        result.push_back(cell);
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////
// Load in AR calibration table
bool load_ar_cal( string filename )
{
	fstream cal_file( filename.c_str(), ios_base::in );

	vector<string> items;

	// read in header line
	items = getNextLineAndSplitIntoTokens( cal_file );

	// read in data till end of file
	// FIXME

	return true;
}

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

///////////////////////////////////////////////////////////////////
// Pose conversion routines
PM_POSE rp2pm( robotPose rp )
{
	PM_POSE pm;
	pm.tran = PM_CARTESIAN( rp.x, rp.y, rp.z );
	pm.rot = PM_RPY( rp.xrot * TO_RAD, rp.yrot * TO_RAD, rp.zrot * TO_RAD );
	return pm;
}

robotPose pm2rp( PM_POSE pm )
{
	robotPose rp;
	PM_RPY rpy(pm.rot);
	rp.x = pm.tran.x;
	rp.y = pm.tran.y;
	rp.z = pm.tran.z;
	rp.xrot = rpy.r * TO_DEG;
	rp.yrot = rpy.p * TO_DEG;
	rp.zrot = rpy.y * TO_DEG;
	return rp;
}

//////////////////////////////////////////////////////////////////
// initialize offset lookup tables used to compute locations of targets
void init_ar()
{
	const double TO_MM = 25.4;

	// origin of target frame chosen to be at first target point
	point1_offset[0] = PM_POSE( PM_CARTESIAN(), PM_QUATERNION() );
	point1_offset[1] = PM_POSE( PM_CARTESIAN(), PM_QUATERNION() );

	point2_offset[0] = PM_POSE( PM_CARTESIAN(0, -18.0*TO_MM, 0), PM_QUATERNION() );
	point2_offset[1] = PM_POSE( PM_CARTESIAN(0, -12*TO_MM, 0), PM_QUATERNION() );

	// location of AR tag relative to target origin
	tag_offset[0] = PM_POSE( PM_CARTESIAN(-9.0*TO_MM, 0, 0), PM_QUATERNION() );
	tag_offset[1] = PM_POSE( PM_CARTESIAN(-6.0*TO_MM, 0, 0), PM_QUATERNION() );

	// overwatch position of camera, recorded as a pair (robot_position, agv_position)
	robot_pose_init[0] = PM_POSE( PM_CARTESIAN(), PM_RPY() );
	robot_pose_init[1] = PM_POSE( PM_CARTESIAN(), PM_RPY() );
	agv_pose_init[0] = PM_POSE( PM_CARTESIAN(), PM_RPY() );
	agv_pose_init[1] = PM_POSE( PM_CARTESIAN(), PM_RPY() );
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

	// initialize ACI link with AGV
	AgvComm agv( "192.168.0.62", 30009 );
	agv.setVerbose( true );

	// initialize AGV position status link
	ulapi_integer priority = ulapi_prio_lowest();
	AGV_Status agv_status( priority );
	std::cout << "Waiting for status connection...  ";
	while( !agv_status.is_connected() ) 
	{
		ulapi_sleep(.1);
	}
	std::cout << "connected" << endl;

	// initialize interface to AR toolkit
	priority = ulapi_prio_next_higher( priority );
	AR_Interface *ar_interface = AR_Interface::Get_Instance();
	if( useARToolkit )
	{
		ar_interface->Init( priority, ++mutex_key );
	}

	// initialize AR offsets
	init_ar();

	crpi_timer timer;
	crpi_timer pause;

	CrpiRobot<robType> arm("universal_ur10_agv.xml");
	Assembly square_spiral;

	// set up search parameters
	square_spiral.AddSearchSqSpiral( searchStep, searchRadius );
	square_spiral.AddTerminatorSignal( CANON_SUCCESS, 8 );
	square_spiral.AddTerminatorTimer( CANON_FAILURE, searchTimeout );

	cout << "Running development version of MobileManipulator" << endl;

	robotPose poseMe, curPose, initPose;
	robotAxes curAxes;
	robotPose position[10];
	robotPose startPose;

	globalHandle gh (&arm); //! State variable used to communicate with the two threads

	gh.robArm->InitCanon();
	gh.robArm->SetAngleUnits("degree");
	gh.robArm->SetLengthUnits("mm");
	gh.robArm->SetAbsoluteAcceleration(0.2f);
	gh.robArm->SetAbsoluteSpeed(0.40f);
	gh.status = CANON_SUCCESS;

	robotIO io;
	bool poseDefined = false;
	bool tool = false;
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
			} while( start == 0 );

			/*	agv.getStatus( agvX, agvY, agvAngle, agvTarget );*/

			/*	agv.setTarget( 0 );
			agv.getStatus( agvX, agvY, agvAngle, agvTarget );*/

			agvTarget = start;
		}
		else
		{
			cout << "Waiting for signal from AGV to begin..." << endl;

			agvTarget = 0;
			while( agvTarget == 0 )
			{
				agv.getStatus( agvX, agvY, agvAngle, agvTarget );
				ulapi_sleep(0.5);  // pause so we don't lock up the cpu
			}
		}

		// generate timestamped log files
		char stamp[100];
		time_t mytime;
		struct tm *mytm;
		mytime = time(NULL);
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
		out << "iter, marker, init_x, init_y, init_z, init_xrot, init_yrot, "
			<< "init_zrot, ok, time, steps, x, y, z, xrot, yrot, zrot" << endl;

		// read position from status link
		ulapi_sleep( 2.0 );  // wait for position to settle
		AGV_State agv_state;
		double sumX = 0;
		double sumY = 0;
		double sumAngle = 0;
		double offsetAngle;
		double minX, maxX, minY, maxY, minAngle, maxAngle;
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

			// signal AGV that robot is done and wait for new target
			agv.setTarget( 0 );
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
			gh.robArm->MoveStraightTo( stagePose1 );
			gh.robArm->MoveStraightTo( stagePose2 );
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
		PM_POSE agvPose = PM_POSE( PM_CARTESIAN(agvX, agvY, 0), PM_RPY(0, 0, agvAngle * TO_RAD));
		//if( useLargeTargets )
		//{
		//	// compute initial locations of large markers
		//	switch( agvTarget )
		//	{
		//	case 1: // square
		//		point1 = inv(agvToRobot) * inv(agvPose) * largeSquarePoint1World;
		//		point2 = inv(agvToRobot) * inv(agvPose) * largeSquarePoint2World;
		//		break;

		//	case 2: // circle
		//		point1 = inv(agvToRobot) * inv(agvPose) * largeCirclePoint1World;
		//		point2 = inv(agvToRobot) * inv(agvPose) * largeCirclePoint2World;
		//		break;

		//	default:
		//		cout << "Unknown target id value (" << agvTarget << ")"	<< endl;	
		//		break;
		//	}
		//	robotPose startPoint1, startPoint2;
		//	double searchTime1, searchTime2;
		//	int stepCount1, stepCount2;
		//	int success1 = 1;
		//	int success2 = 1;
		//	startPoint1 = refPoint1 = cart2rob( point1, basePose );
		//	startPoint2 = refPoint2 = cart2rob( point2, basePose );

		//	// move to markers and refine position
		//	gh.robArm->MoveStraightTo( refPoint1 );
		//	if( !bisect( logger, gh.robArm, refPoint1, largeTargetStepSize, &stepCount1, &searchTime1 ) )
		//	{
		//		cout << "Error finding center of reflector 1" << endl;
		//		logger << "Error finding center of reflector 1" << endl;
		//		aligned = false;
		//		success1 = 0;
		//	}

		//	gh.robArm->MoveStraightTo( refPoint2 );
		//	if( !bisect( logger, gh.robArm, refPoint2, largeTargetStepSize, &stepCount2, &searchTime2 ) )
		//	{
		//		cout << "Error finding center of reflector 2" << endl;
		//		logger << "Error finding center of reflector 2" << endl;
		//		aligned = false;
		//		success2 = 0;
		//	}

		//	// compute index locations
		//	point2Offset = rob2cart(refPoint2 - refPoint1);
		//	point2Offset.z = 0;
		//	PM_CARTESIAN offset90 = unit( PM_QUATERNION( PM_Z, PM_PI/2 ) * point2Offset );
		//	switch( agvTarget )
		//	{
		//	case 1: // square
		//		{
		//			PM_CARTESIAN cornerOffset = (-8 * 25.4) * offset90; 
		//			position[0] = cart2rob(rob2cart(refPoint1) + cornerOffset, basePose);
		//			break;
		//		}

		//	case 2: // circle
		//		{
		//			PM_CARTESIAN firstPoint = (rob2cart(refPoint1) + rob2cart(refPoint2)) / 2; // center
		//			firstPoint = firstPoint + offset90 * (6 * 25.4);
		//			position[0] = cart2rob( firstPoint, basePose );

		//			// recompute offset
		//			point2Offset = offset90 * (-12 * 25.4);
		//			break;
		//		}

		//	default:
		//		cout << "Unknown target id value (" << agvTarget << ")"	<< endl;	
		//		break;
		//	}
		//}
		//else
		//{
		//	// compute start point using AGV position
		//	switch( agvTarget )
		//	{
		//	case 1: // square
		//		point1 = inv(agvToRobot) * inv(agvPose) * squarePoint1World;
		//		point2 = inv(agvToRobot) * inv(agvPose) * squarePoint2World;
		//		break;

		//	case 2: // circle
		//		point1 = inv(agvToRobot) * inv(agvPose) * circlePoint1World;
		//		point2 = inv(agvToRobot) * inv(agvPose) * circlePoint2World;
		//		break;

		//	default:
		//		cout << "Unknown target id value (" << agvTarget << ")"	<< endl;	
		//		break;
		//	}
		//	point2Offset = point2 - point1;	
		//	position[0] = cart2rob( point1, basePose );
		//}
		
		// compute position of camera view position in world coordinates
		PM_POSE cam_pose_world = agv_pose_init[agvTarget-1] * agvToRobot * robot_pose_init[agvTarget-1];

		// Compute initial position for camera using agv position
		PM_POSE agv_pose = PM_POSE( PM_CARTESIAN(agvX, agvY, 0), PM_RPY(0, 0, agvAngle * TO_RAD) );
		PM_POSE look_pose = inv(agvToRobot) * inv(agv_pose) * cam_pose_world;
		robotPose look_pose_rp = pm2rp( look_pose );

		// move camera
		gh.robArm->MoveStraightTo( look_pose_rp );

		// get tag position from AR toolkit
		AR_Status ar_status =  ar_interface->Get_Status();
		

		// compute locations of reference points
		PM_POSE target_pose = inv(tag_offset[agvTarget-1]) * look_pose * ar_status.pose;
		PM_POSE point1_pose = target_pose * point1_offset[agvTarget-1];
		PM_POSE point2_pose = target_pose * point2_offset[agvTarget-1];

		// add sensor standoff
		point1_pose = point1_pose * standoff;
		point2_pose = point2_pose * standoff;

		// override height and orientation
		if( force_align )
		{
			// fix orientation
			point1_pose.rot = PM_RPY( sensorRot[0] * TO_RAD, sensorRot[1] * TO_RAD, sensorRot[2] * TO_RAD );
			point2_pose.rot = PM_RPY( sensorRot[0] * TO_RAD, sensorRot[1] * TO_RAD, sensorRot[2] * TO_RAD );

			// fix standoff height
			point1_pose.tran.z = sensorHeight;
			point2_pose.tran.z = sensorHeight;
		}

		// set move positions for first two points
		position[0] = pm2rp( point1_pose );
		position[1] = pm2rp( point2_pose );

		// compute offset between points  
		PM_POSE p1p2_offset = inv(point2_pose) * point1_pose;

		// iterate through the search
		gh.status = CANON_RUNNING;
		cout << "running " << numIters << " iterations" << endl;
		logger << "running " << numIters << " iterations" << endl;

		if( aligned )
		{
			for (int i = 0; i < numIters; ++i)
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

				for (int j = 0; j < numPoints; ++j)
				{
					//! Move to position j
					cout << "Moving to initial search position [" << j << "]... ";
#ifdef LOGEVERYTHING
					logger << ulapi_time() << " Moving to position[" << j << "] : [" << position[j].x << ", " << position[j].y 
						<< ", " << position[j].z << ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot
						<< "]" << endl;
#endif
					out << i << ", " << j << ", " << position[j].x << ", " << position[j].y << ", " << position[j].z 
						<< ", " << position[j].xrot << ", " << position[j].yrot << ", " << position[j].zrot << ", ";
					if (gh.robArm->MoveStraightTo (position[j]) == CANON_SUCCESS)
					{
						pause.waitUntil( robotSettleTime );

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
					}
					else
					{
						cout << "*** Move command failed! ***" << endl;
					}

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
						gh.robArm->MoveStraightTo (poseMe);
						pause.waitUntil( robotSettleTime );

						++counter;
					} while (retVal == CANON_RUNNING);
					double tim = timer.elapsedTime() / 1000.0;  // convert from milliseconds to seconds
					timer.stop();
#ifdef NOISY
					cout << logger << "Spiral search stopped " << (retVal == CANON_SUCCESS ? "successfully" : "unsuccessfully") << " at offset ["
						<< (poseMe.x - position[j].x) << ", " << (poseMe.y - position[j].y) << "] after " << tim << " seconds and " 
						<< counter << " steps." << endl;
#endif
#ifdef LOGEVERYTHING
					logger << ulapi_time() << " Search stopped " << (retVal == CANON_SUCCESS ? "successfully" : "unsuccessfully")
						<< " at offset [" << (poseMe.x - position[j].x) << ", " << (poseMe.y - position[j].y) << "] after " 
						<< tim << " seconds and " << counter << " steps." << endl;
#endif
					//! Record position[j] in log file
					out << (retVal == CANON_SUCCESS ? "1" : "0") << ", " << tim << ", ";
					out << counter << ", ";
					out << poseMe.x << ", " << poseMe.y << ", " << poseMe.z << ", " << poseMe.xrot << ", " 
						<< poseMe.yrot << ", " << poseMe.zrot << endl;

					if( retVal == CANON_SUCCESS )
						position[j] = poseMe;
					if (i == 0)
					{
						if (j == 0)
						{
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

						/*	position[1] = position[0];
							position[1].x = position[0].x + point2Offset.x;
							position[1].y = position[0].y + point2Offset.y;
							position[1].z = position[0].z;*/

							if( use_offset )
							{
								// compute location of point2 using offset
								point2_pose = p1p2_offset * rp2pm( position[0] );
								position[1] = pm2rp( point2_pose );
							}
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

							//switch( agvTarget )
							//{
							//case 1: // square
							//	{
							//		theta = atan2((position[1].y - position[0].y), (position[1].x - position[0].x));
							//		double dx = (18.0 * 25.4) * cos( theta + PM_PI/2 );
							//		double dy = (18.0 * 25.4) * sin( theta + PM_PI/2 );
							//		position[2] = position[1];
							//		position[3] = position[0];
							//		position[2].x += dx;
							//		position[2].y += dy;
							//		position[3].x += dx;
							//		position[3].y += dy;
							//	}
							//	break;
							//case 2: // circle
							//	{
							//		// find the center point of the circle and the angle for position[0]
							//		robotPose center = position[0];
							//		center.x = (position[0].x + position[1].x) / 2;
							//		center.y = (position[0].y + position[1].y) / 2;
							//		double t0 = atan2( position[0].y - position[1].y, position[0].x - position[1].x );

							//		// recompute positions based on center, radius and initial angle
							//		double radius = 6 * 25.4; // 6 inch radius
							//		for( int a=0; a<6; ++a )
							//		{
							//			double angle = a * 60 * PM_PI / 180;
							//			position[a] = center;  // to copy height and orientation from initial point
							//			position[a].x = center.x + radius * cos(angle + t0);
							//			position[a].y = center.y + radius * sin(angle + t0);
							//		}

							//		// skip directly to second loop where the robot will progress counter clockwise around the circle
							//		j = numPoints;
							//	}
							//	break;
							//}

							// compute the pose of all the rest of the points based on registration points
							switch( agvTarget )
							{
							case 0:
								// square
								{
									double size = 18.0 * IN_TO_MM;
									PM_POSE p0_pose = PM_POSE( PM_CARTESIAN(), PM_QUATERNION() );
									PM_POSE p1_pose = PM_POSE( PM_CARTESIAN(0, -size, 0), PM_QUATERNION() );
									PM_POSE p2_pose = PM_POSE( PM_CARTESIAN(size, -size, 0), PM_QUATERNION() );
									PM_POSE p3_pose = PM_POSE( PM_CARTESIAN(size, 0, 0), PM_QUATERNION() );
									position[0] = pm2rp( target_pose * p0_pose );
									position[1] = pm2rp( target_pose * p1_pose );
									position[2] = pm2rp( target_pose * p2_pose );
									position[3] = pm2rp( target_pose * p3_pose );
								}
								break;
							case 1:
								// circle
								{
									double radius = 6.0 * IN_TO_MM;
									PM_CARTESIAN center = PM_CARTESIAN( -radius, 0, 0 );
									for( int i=0; i<6; ++i )
									{
										double angle = 360.0 / 6 * i * TO_RAD;
										PM_CARTESIAN pn = radius * PM_CARTESIAN( sin(angle), cos(angle), 0 ) + center;
										PM_POSE pn_pose = PM_POSE( pn, PM_QUATERNION() );
										position[i] = pm2rp( target_pose * pn_pose );
									}

									// skip directly to second loop where the robot will progress counter clockwise around the circle
									j = numPoints;
								}
								break;
							default:
								cout << "Unrecognized target type, " << agvTarget << cout;
								ulapi_sleep(60);
								exit(-1);
								break;
							}


						} // if (i == 0)
					} // for (j = 0; j < 4; ++j)
					cout << "Completed iteration " << i << endl;
					logger << "Completed iteration " << i << endl;
				} // for (int j = 0; j < numPoints; ++j)

			} // for (i = 0; i < numIter; ++i)
		}

		// move robot back to stow location
		if( stowArm )
		{
			// move stow point by way of stage point 2 and 1
			gh.robArm->MoveStraightTo( stagePose2 );
			gh.robArm->MoveStraightTo( stagePose1 );
			gh.robArm->MoveStraightTo( stowPose );
		} 
		else
		{
			gh.status = 4;
			gh.robArm->MoveStraightTo (initPose);
			cout << "Moving back to initial position: " << initPose.x << " " << initPose.y << " " << initPose.z << endl;
			logger << "Moving back to initial position: " << initPose.x << " " << initPose.y << " " << initPose.z << endl;
		}

		// signal AGV that robot is done
		agv.setTarget( 0 );

		logger.close();
		out.close();
	} // while (i != -1)

	cout << "All done" << endl;
}
