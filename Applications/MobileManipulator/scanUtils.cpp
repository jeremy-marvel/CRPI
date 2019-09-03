#include "scanUtils.h"

#include <iostream>
#include <fstream>

using namespace crpi_robot;
using namespace std;

///////////////////////////////////////////////////////////////
// Fast search bisection search.  Starts moving at step size
// doubling the distance each time till the transition is detected.
// Then it starts searching with decreasing step size.
// Subroutine assumes that robot has been paused long enough 
// prior to call to allow for a different pause at beginning and
// end of search.
// Routine increments count by 1 per step.  Count should be reset 
// externally.
//
// @param arm    Pointer to robot control data structure
// @param pose   Returns center pose of reflector.
// @param up     True if serching in positive direction.
// @param xAxis  True if motion is along x-axis, false for y-axis.
// @param count  Incremented for each step taken by the robot.
bool accSearch( 
	RobotType *arm,
	robotPose &pose,
	bool up,
	bool xAxis,
	double stepSize,
	int &count )
{
	crpi_timer pause;
	robotIO io;

	// get current pose and io
	arm->GetRobotIO( &io );
	//arm->GetRobotPose( &pose );

	// fail if initial detect is false
	if( io.dio[9] )
	{
		return false;
	}

	// search state variables
	bool out = true;
	bool edgeCrossed = false;
	int stepExponent = 0;
	bool detected = true;
	bool prevDetected = true;

	// search till step size is back to its minimum
	//logger << "Start search" << endl;
	while( stepExponent >= 0 )
	{
		// compute move pose based on current step size and direction
		double step = stepSize * (1 << stepExponent) * (out ? 1 : -1) * (up ? 1 : -1);
		if( xAxis )
			pose.x += step;
		else
			pose.y += step;

		// make move, pause, and read sensor
		arm->MoveStraightTo( pose );
		pause.waitUntil( robotSettleTime );
		arm->GetRobotIO( &io );
		++count;

		// adjust search flags
		detected = !io.dio[9];
		if( !edgeCrossed && !detected )
			edgeCrossed = true;
		if( edgeCrossed )
			--stepExponent;
		else
			++stepExponent;
		if( detected != prevDetected )
			out = !out;

		// record previous state
		prevDetected = detected;

		/*logger << step 
			<< " " << stepExponent 
			<< " " << edgeCrossed
			<< " " << detected
			<< " " << out
			<< " " << up << endl; */
	}
	//logger << "end search" << endl;

	// Add a step if last move left us just inside the edge of the reflector
	//    for consistency with previous bisection algorithm which stopped
	//    after stepping out of the radius of the reflector.
	if( detected )
	{
		if( xAxis )
			pose.x += stepSize * (up ? 1 : -1);
		else
			pose.y += stepSize * (up ? 1 : -1);
		//logger << "Bumped out a step" << endl;
	}

	return true;
}

//////////////////////////////////////////////////////////////////
// Bisect the current target to find the center more accurately.
// Detects target when dio[8] is false;
bool bisect( 
	ofstream &logger,
	RobotType *arm,
	robotPose &pose,
	double stepSize,
	int *stepCount,
	double *searchTime )
{
	crpi_timer pause;
	crpi_timer timer;
	robotIO io;
	int count = 0;

	*stepCount = 0;
	*searchTime = 0;

	// give robot a chance to settle down
	cout << "Pausing " << bisectPauseMs/1000.0 << " seconds." << endl;
	logger << "Pausing " << bisectPauseMs/1000.0 << " seconds." << endl;
	pause.waitUntil( bisectPauseMs );

	// get current pose and io
	arm->GetRobotIO( &io );
  //! TODO:  uncomment this in the future...
	//arm->GetRobotPose( &pose );

	robotPose initialPose = pose;

	cout << "Bisecting point..." << endl;
	logger << "Bisecting point..." << endl;

	// make sure we have detection at initial location
	if( io.dio[9] )
	{
		cout << "no signal at p1" << std::endl;  // debug
		logger << "no signal at p1" << std::endl;  // debug
		return false;
	}

	timer.start();

	// search for x center
	robotPose x1, x2;
	robotPose startPose = pose;
	robotPose commandPose = pose;

	if( useNewBisectionAlgorithm )
	{
		x1 = startPose;
		accSearch( arm, x1, false, true, stepSize, count );
	}
	else
	{
		while( !io.dio[9] )
		{
			commandPose.x -= stepSize;
			while (arm->MoveStraightTo( commandPose ) != CANON_SUCCESS)
      {
        commandPose.print();
      }
			pause.waitUntil( robotSettleTime );

			arm->GetRobotIO( &io );

			++count;
		}
		arm->GetRobotPose( &x1 );
	}

	commandPose = startPose;
	while (arm->MoveStraightTo( commandPose ) != CANON_SUCCESS)
  {
    commandPose.print();
  }
	pause.waitUntil( robotSettleTime );
	arm->GetRobotIO( &io );
	if( io.dio[9] ) 
	{
		cout << "no signal at p2" << endl;   // debug
		logger << "no signal at p2" << endl;   // debug
		return false;
	}

	if( useNewBisectionAlgorithm )
	{
		x2 = startPose;
		accSearch( arm, x2, true, true, stepSize, count );
	}
	else
	{
		while( !io.dio[9] )
		{
			commandPose.x += stepSize;
			while (arm->MoveStraightTo( commandPose ) != CANON_SUCCESS)
      {
        commandPose.print();
      }
			pause.waitUntil( robotSettleTime );

			arm->GetRobotIO( &io );

			++count;
		}
		arm->GetRobotPose( &x2 );
	}

	double xCenter = (x1.x + x2.x) / 2;

	cout << "x dim = " << x2.x - x1.x << endl;  // debug
	logger << "x dim = " << x2.x - x1.x << endl;  // debug

	// search for y center
	robotPose y1, y2;
	commandPose = startPose;
	commandPose.x = xCenter;
	startPose = commandPose;
	arm->MoveStraightTo( commandPose );
	pause.waitUntil( robotSettleTime );
	arm->GetRobotIO( &io );
	if( io.dio[9] ) 
	{
		cout << "no signal at p3" << endl;   // debug
		logger << "no signal at p3" << endl;   // debug
		return false;
	}

	if( useNewBisectionAlgorithm )
	{
		y1 = startPose;
		accSearch( arm, y1, false, false, stepSize, count );
	}
	else
	{
		while( !io.dio[9] )
		{
			commandPose.y -= stepSize;
      while (arm->MoveStraightTo( commandPose ) != CANON_SUCCESS)
      {
        commandPose.print();
      }
			pause.waitUntil( robotSettleTime );

			arm->GetRobotIO( &io );

			++count;
		}
		arm->GetRobotPose( &y1 );
	}

	commandPose = startPose;
	arm->MoveStraightTo( commandPose );
	pause.waitUntil( robotSettleTime );
	arm->GetRobotIO( &io );
	if( io.dio[9] ) 
	{
		cout << "no signal at p4" << endl;   // debug
		logger << "no signal at p4" << endl;   // debug
		return false;
	}

	if( useNewBisectionAlgorithm )
	{
		y2 = startPose;
		accSearch( arm, y2, true, false, stepSize, count );
	}
	else
	{
		while( !io.dio[9] )
		{
			commandPose.y += stepSize;
			while (arm->MoveStraightTo( commandPose ) != CANON_SUCCESS)
      {
        commandPose.print();
      }
			pause.waitUntil( robotSettleTime );

			arm->GetRobotIO( &io );

			++count;
		}
	}

	arm->GetRobotPose( &y2 );
	double yCenter = (y1.y + y2.y) / 2;

	cout << "y dim = " << y2.y - y1.y << endl;  // debug
	logger << "y dim = " << y2.y - y1.y << endl;  // debug

	*searchTime = timer.elapsedTime() / 1000.0;
	timer.stop();
	*stepCount = count;

	// set pose to center
	pose.x = xCenter;
	pose.y = yCenter;

	// move robot to center point and pause there
	arm->MoveStraightTo( pose );
	cout << "Pausing " << bisectPauseMs/1000.0 << " seconds." << endl;
	logger << "Pausing " << bisectPauseMs/1000.0 << " seconds." << endl;
	pause.waitUntil( bisectPauseMs );

	cout << "point moved from " << initialPose.x << " " << initialPose.y << 
		" to " << pose.x << " " << pose.y << endl;
	cout << "   distance = " << sqrt( sq( initialPose.x - pose.x ) + sq( initialPose.y - pose.y ) ) << endl;
	
	logger << "point moved from " << initialPose.x << " " << initialPose.y << 
		" to " << pose.x << " " << pose.y << endl;
	logger << "   distance = " << sqrt( sq( initialPose.x - pose.x ) + sq( initialPose.y - pose.y ) ) << endl;

	return true;
}

void runTargetScan( 
	RobotType *arm,
	string filename,
	double stepSize,
	robotPose center )
{
	ofstream log( filename.c_str() );
	int count = 0; // not used
	bool ok;
	robotPose border;
	robotPose startPose;

	for( int xAxis=0; xAxis<2; ++xAxis )
	{ // scan x or y direction
		for( int up=0; up<2; ++up )
		{ // move up or down from center
			startPose = center;
			ok = true;
			while( ok )
			{
				// search to lower border
				border = startPose;
				arm->MoveStraightTo( startPose );
				ulapi_sleep( robotSettleTime/1000. );
				ok = ok && accSearch( arm, border, false, (xAxis==1), stepSize, count );
				if( ok )
					log << border.x << " " << border.y << endl;

				// search to higher border
				border = startPose;
				arm->MoveStraightTo( startPose );
				ulapi_sleep( robotSettleTime/1000. );
				ok = ok && accSearch( arm, border, true, (xAxis==1), stepSize, count );
				if( ok )
					log << border.x << " " << border.y << endl;

				// move up or down from center
				if( xAxis == 1 )
				{
					// advance along y axis
					startPose.y += stepSize * (up==1 ? 1 : -1);
				}
				else
				{
					// advance along x axis
					startPose.x += stepSize * (up==1 ? 1 : -1);
				}
			}
		}
	}
}

