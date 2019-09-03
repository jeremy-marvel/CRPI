
#ifndef __INC_SCAN_UTILS_H__
#define __INC_SCAN_UTILS_H__

#include "config.hh"

#include "crpi_robot.h"
#include "crpi_universal.h"

typedef crpi_robot::CrpiRobot<crpi_robot::CrpiUniversal> RobotType;

/////////////////////////////////////////////////////////////////
template <class T>
T sq( T val )
{
	return val * val;
}

/////////////////////////////////////////////////////////////////
bool accSearch( 
	RobotType *arm,
	robotPose &pose,
	bool up,
	bool xAxis,
	double stepSize,
	int &count );

bool bisect( 
	ofstream &logger,
	RobotType *arm,
	robotPose &pose,
	double stepSize,
	int *stepCount,
	double *searchTime );

void runTargetScan( 
	RobotType *arm,
	string filename,
	double stepSize,
	robotPose center );

#endif // __INC_SCAN_UTILS_H__
