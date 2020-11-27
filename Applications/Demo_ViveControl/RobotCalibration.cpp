///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       VR Robot Calibration
//  Workfile:        RobotCalibration.cpp
//  Revision:        1.0 - 22 June, 2017
//  Author:          M. Zimmerman
//
//  Description
//  ===========
//  Calibration methods for robot and VR system.
///////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <time.h>
#include <ctime>
#include "crpi_robot.h"
#include "crpi_abb.h"
#include "ulapi.h"
#include "NumericalMath.h" 
#include "MatrixMath.h"
#include "../../Libraries/MotionPrims/AssemblyPrims.h"
#include "CoordFrameReg.h"
//#include "ATI_Wired.h"
//#include "FT_COP.h"
#include<sstream>
using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;
//using namespace Sensor;

//int callibrate();

//toTokens
//Preconditions: Hashed splays object must have been created, given valid line and stream
//Postconditions: breaks the string into a vector by whitespace
/**vector<string> toTokens(vector<string>* line, string buffer, stringstream& stream)
{
	if (stream >> buffer)
	{
		line->push_back(buffer);
		toTokens(line, buffer, stream);
	}

	return *line;
}

int callibrate() 
{
	//! Calibration Variables
	vector<robotPose> pur;
	vector<point> world, rob;
	point wldpt[4], robpt[4];
	matrix W_T_R;
	string val;

	vector<string> tokens;

	ifstream rfile("yumi_pose.txt");
	if (rfile.is_open()) 
	{
		int i = 0;
		while(getline( rfile,val))
		{
			stringstream stream(val);
			string buffer;
			vector<string> tokens;
			//tokens = toTokens(&tokens, buffer, stream);
			
			rob[i].x = atoi(tokens[0].c_str());
			cout << rob[i].x << endl;
			rob[i].y = atoi(tokens[1].c_str());
			cout << rob[i].y << endl;
			rob[i].z = atoi(tokens[2].c_str());
			cout << rob[i].z << endl;

			i++;
			if (i > 3) 
			{
				break;
			}
		}
		
	}

	rfile.close();
	return 0;

}
**/
//To be run in conjunction with VR point capture
int getRobpoints()
{
	int i = 0;
	crpi_timer timer;


	char buf[1024];
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	ofstream outf;

	//! Pose Variables
	robotPose poseMe, curPose, curPose_r, poseR, curPose_l;
	robotAxes curAxes, curAxesR, tarAxes, tarAxesR;

	CrpiRobot<CrpiAbb> armL("abb_irb14000_left.xml");
	armL.SetAngleUnits("degree");
	armL.SetLengthUnits("mm");
	std::cout << "Left Connected and init" << endl;

	CrpiRobot<CrpiAbb> armR("abb_irb14000_right.xml");
	armR.SetAngleUnits("degree");
	armR.SetLengthUnits("mm");
	std::cout << "Right Connected and init" << endl;

	armL.Couple("Yumi_Parallel");
	armR.Couple("Yumi_Parallel");
	std::cout << "Tool Coupling Done" << endl;

	outf.open("rob_coords.csv", std::ios::app);


	//center robot L
	tarAxes.axis[0] = -128.393;	tarAxes.axis[1] = -126.96;	tarAxes.axis[2] = 21.8281;	tarAxes.axis[3] = 24.3266;\
	tarAxes.axis[4] = 182.648;	tarAxes.axis[5] = -71.2129;	tarAxes.axis[6] = -26.6042;

	//center robot R
	tarAxesR.axis[0] = 120.111;	tarAxesR.axis[1] = -124.582;	tarAxesR.axis[2] = -16.7288;	tarAxesR.axis[3] = 23.7964;
	tarAxesR.axis[4] = -175.136;	tarAxesR.axis[5] = -58.5175;	tarAxesR.axis[6] = 105.083;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	timer.waitUntil(2000);

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str(buffer);

	if (armL.GetRobotPose(&curPose_l) == CANON_SUCCESS)
	{
		outf << "Center Mid" << "," << "left" << "," << str << "," << curPose_l.x << ", " << curPose_l.y << ", " << curPose_l.z << ", " << curPose_l.xrot << ", " << curPose_l.yrot << ", " << curPose_l.zrot << "" << endl;
		std::cout << "Center Mid" << "," << "left" << "," << str << "," << curPose_l.x << ", " << curPose_l.y << ", " << curPose_l.z << ", " << curPose_l.xrot << ", " << curPose_l.yrot << ", " << curPose_l.zrot << "" << endl;

		armL.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Mid" << "," << "right" << "," << str << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Mid" << "," << "right" << "," << str << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}



	std::cout << str << " -- Center Mid Moved" << endl;

	timer.waitUntil(10000);

	//Center up
	tarAxes.axis[0] = -132.848;
	tarAxes.axis[1] = -114.333;
	tarAxes.axis[2] = 20.4009;
	tarAxes.axis[3] = 45.2134;
	tarAxes.axis[4] = 182.707;
	tarAxes.axis[5] = -37.1397;
	tarAxes.axis[6] = -26.9737;

	tarAxesR.axis[0] = 129.627;
	tarAxesR.axis[1] = -108.358;
	tarAxesR.axis[2] = -11.8478;
	tarAxesR.axis[3] = 35.5256;
	tarAxesR.axis[4] = -179.724;
	tarAxesR.axis[5] = -28.8165;
	tarAxesR.axis[6] = 106.815;


	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str1(buffer);

	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_l) == CANON_SUCCESS)
	{
		outf << "Center Up" << "," << "left" << "," << str1 << "," << curPose_l.x << ", " << curPose_l.y << ", " << curPose_l.z << ", " << curPose_l.xrot << ", " << curPose_l.yrot << ", " << curPose_l.zrot << "" << endl;
		std::cout << "Center Up" << "," << "left" << "," << str1 << "," << curPose_l.x << ", " << curPose_l.y << ", " << curPose_l.z << ", " << curPose_l.xrot << ", " << curPose_l.yrot << ", " << curPose_l.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Up" << "," << "right" << "," << str1 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Up" << "," << "right" << "," << str1 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}



	std::cout << str1 << " -- Center Up Moved" << endl;

	timer.waitUntil(10000);

	//Center down
	tarAxes.axis[0] = -128.808;
	tarAxes.axis[1] = -126.494;
	tarAxes.axis[2] = 20.9211;
	tarAxes.axis[3] = 10.4499;
	tarAxes.axis[4] = 182.598;
	tarAxes.axis[5] = -84.5459;
	tarAxes.axis[6] = -26.6959;

	tarAxesR.axis[0] = 119.606;
	tarAxesR.axis[1] = -133.352;
	tarAxesR.axis[2] = -21.4434;
	tarAxesR.axis[3] = 11.163;
	tarAxesR.axis[4] = -178.179;
	tarAxesR.axis[5] = -79.5924;
	tarAxesR.axis[6] = 102.813;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str2(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Down" << "," << "left" << "," << str2 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Down" << "," << "left" << "," << str2 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Down" << "," << "right" << "," << str2 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Down" << "," << "right" << "," << str2 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}

	std::cout << str2 << " -- Center Down Moved" << endl;

	timer.waitUntil(10000);

	//Center up
	tarAxes.axis[0] = -132.848;
	tarAxes.axis[1] = -114.333;
	tarAxes.axis[2] = 20.4009;
	tarAxes.axis[3] = 45.2134;
	tarAxes.axis[4] = 182.707;
	tarAxes.axis[5] = -37.1397;
	tarAxes.axis[6] = -26.9737;

	tarAxesR.axis[0] = 129.627;
	tarAxesR.axis[1] = -108.358;
	tarAxesR.axis[2] = -11.8478;
	tarAxesR.axis[3] = 35.5256;
	tarAxesR.axis[4] = -179.724;
	tarAxesR.axis[5] = -28.8165;
	tarAxesR.axis[6] = 106.815;


	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	timer.waitUntil(2000);

	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string st11(buffer);
	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Up" << "," << "left" << "," << st11 << ", " << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Up" << "," << "left" << "," << st11 << ", " << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armL.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Up" << "," << "right" << "," << st11 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Up" << "," << "right" << "," << st11 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}

	std::cout << st11 << " -- Center Up Moved" << endl;

	timer.waitUntil(10000);
	//mid left
	tarAxes.axis[0] = -114.971;
	tarAxes.axis[1] = -101.162;
	tarAxes.axis[2] = -0.646127;
	tarAxes.axis[3] = 10.1676;
	tarAxes.axis[4] = 167.757;
	tarAxes.axis[5] = -59.7578;
	tarAxes.axis[6] = -48.8376;

	tarAxesR.axis[0] = 139.346;
	tarAxesR.axis[1] = -127.821;
	tarAxesR.axis[2] = -31.8261;
	tarAxesR.axis[3] = 32.7211;
	tarAxesR.axis[4] = -189.849;
	tarAxesR.axis[5] = -45.3627;
	tarAxesR.axis[6] = 86.0906;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str3(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Mid Left" << "," << "left" << "," << str3 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Mid Left" << "," << "left" << "," << str3 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Mid Left" << "," << "right" << "," << str3 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Mid Left" << "," << "right" << "," << str3 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}

	std::cout << str3 << " -- Mid Left Moved" << endl;

	timer.waitUntil(10000);

	//up left
	tarAxes.axis[0] = -124.277;
	tarAxes.axis[1] = -78.2817;
	tarAxes.axis[2] = -13.0802;
	tarAxes.axis[3] = 15.8073;
	tarAxes.axis[4] = 191.596;
	tarAxes.axis[5] = -32.1906;
	tarAxes.axis[6] = -68.779;

	tarAxesR.axis[0] = 146.998;
	tarAxesR.axis[1] = -97.5997;
	tarAxesR.axis[2] = -34.3069;
	tarAxesR.axis[3] = 33.5556;
	tarAxesR.axis[4] = -162.486;
	tarAxesR.axis[5] = -15.612;
	tarAxesR.axis[6] = 62.3025;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str4(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Up Left" << "," << "left" << "," << str4 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Up Left" << "," << "left" << "," << str4 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Up Left" << "," << "right" << "," << str4 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Up Left" << "," << "right" << "," << str4 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str4 << " -- Up Left Moved" << endl;

	timer.waitUntil(10000);

	//center robot L
	tarAxes.axis[0] = -128.393;
	tarAxes.axis[1] = -126.96;
	tarAxes.axis[2] = 21.8281;
	tarAxes.axis[3] = 24.3266;
	tarAxes.axis[4] = 182.648;
	tarAxes.axis[5] = -71.2129;
	tarAxes.axis[6] = -26.6042;

	//center robot R
	tarAxesR.axis[0] = 120.111;
	tarAxesR.axis[1] = -124.582;
	tarAxesR.axis[2] = -16.7288;
	tarAxesR.axis[3] = 23.7964;
	tarAxesR.axis[4] = -175.136;
	tarAxesR.axis[5] = -58.5175;
	tarAxesR.axis[6] = 105.083;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str111(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Mid" << "," << "left" << "," << str111 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Mid" << "," << "left" << "," << str111 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Mid" << "," << "right" << "," << str111 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Mid" << "," << "right" << "," << str111 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}

	std::cout << str111 << " -- Center Mid Moved" << endl;

	timer.waitUntil(10000);


	//up right
	tarAxes.axis[0] = -150.86;
	tarAxes.axis[1] = -91.8481;
	tarAxes.axis[2] = 32.3698;
	tarAxes.axis[3] = 28.6446;
	tarAxes.axis[4] = 183.219;
	tarAxes.axis[5] = -25.306;
	tarAxes.axis[6] = -10.8134;

	tarAxesR.axis[0] = 124.202;
	tarAxesR.axis[1] = -75.0134;
	tarAxesR.axis[2] = 9.45675;
	tarAxesR.axis[3] = 3.26175;
	tarAxesR.axis[4] = -197.007;
	tarAxesR.axis[5] = -30.0039;
	tarAxesR.axis[6] = 141.524;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str5(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Up Right" << "," << "left" << "," << str5 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Up Right" << "," << "left" << "," << str5 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Up Right" << "," << "right" << "," << str5 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Up Right" << "," << "right" << "," << str5 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str5 << " -- Up Right Moved" << endl;

	timer.waitUntil(10000);

	//center right
	tarAxes.axis[0] = -144.974;
	tarAxes.axis[1] = -113.895;
	tarAxes.axis[2] = 27.7218;
	tarAxes.axis[3] = 23.1841;
	tarAxes.axis[4] = 191.375;
	tarAxes.axis[5] = -53.5743;
	tarAxes.axis[6] = -17.6697;

	tarAxesR.axis[0] = 115.083;
	tarAxesR.axis[1] = -103.007;
	tarAxesR.axis[2] = -0.644407;
	tarAxesR.axis[3] = 7.0277;
	tarAxesR.axis[4] = -172.3;
	tarAxesR.axis[5] = -52.8001;
	tarAxesR.axis[6] = 122.479;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str6(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Right" << "," << "left" << "," << str6 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Right" << "," << "left" << "," << str6 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Right" << "," << "right" << "," << str6 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Right" << "," << "right" << "," << str6 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str6 << " -- Center Right Moved" << endl;

	timer.waitUntil(10000);

	//down right
	tarAxes.axis[0] = -144.325;
	tarAxes.axis[1] = -118.838;
	tarAxes.axis[2] = 22.5016;
	tarAxes.axis[3] = 2.77659;
	tarAxes.axis[4] = 191.907;
	tarAxes.axis[5] = -79.3412;
	tarAxes.axis[6] = -18.3455;

	tarAxesR.axis[0] = 112.153;
	tarAxesR.axis[1] = -108.502;
	tarAxesR.axis[2] = -7.1042;
	tarAxesR.axis[3] = -5.44793;
	tarAxesR.axis[4] = -168.644;
	tarAxesR.axis[5] = -71.6514;
	tarAxesR.axis[6] = 119.572;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str7(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Down Right" << "," << "left" << "," << str7 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Down Right" << "," << "left" << "," << str7 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Down Right" << "," << "right" << "," << str7 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Down Right" << "," << "right" << "," << str7 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str7 << " -- Down Right Moved" << endl;

	timer.waitUntil(10000);


	//center up in
	tarAxes.axis[0] = -127.568;
	tarAxes.axis[1] = -132.361;
	tarAxes.axis[2] = 25.7819;
	tarAxes.axis[3] = 68.4407;
	tarAxes.axis[4] = 188.401;
	tarAxes.axis[5] = -32.3248;
	tarAxes.axis[6] = -29.5689;

	tarAxesR.axis[0] = 127.288;
	tarAxesR.axis[1] = -129.976;
	tarAxesR.axis[2] = -17.1123;
	tarAxesR.axis[3] = 59.908;
	tarAxesR.axis[4] = -185.19;
	tarAxesR.axis[5] = -25.8284;
	tarAxesR.axis[6] = 106.922;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str8(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Up In" << "," << "left" << "," << str8 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Up In" << "," << "left" << "," << str8 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Up In" << "," << "right" << "," << str8 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Up In" << "," << "right" << "," << str8 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str8 << " -- Center Up In Moved" << endl;

	timer.waitUntil(10000);

	//center in
	tarAxes.axis[0] = -74.4696;
	tarAxes.axis[1] = -142.312;
	tarAxes.axis[2] = 67.4461;
	tarAxes.axis[3] = 54.5009;
	tarAxes.axis[4] = 168.961;
	tarAxes.axis[5] = -78.5828;
	tarAxes.axis[6] = -11.5747;

	tarAxesR.axis[0] = 55.1688;
	tarAxesR.axis[1] = -142.812;
	tarAxesR.axis[2] = -79.5198;
	tarAxesR.axis[3] = 55.5085;
	tarAxesR.axis[4] = -162.107;
	tarAxesR.axis[5] = -76.5923;
	tarAxesR.axis[6] = 84.6715;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	timer.waitUntil(2000);

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str9(buffer);
	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center In" << "," << "left" << "," << str9 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center In" << "," << "left" << "," << str9 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center In" << "," << "right" << "," << str9 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center In" << "," << "right" << "," << str9 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str9 << " -- Center In Moved" << endl;

	timer.waitUntil(10000);

	//center down in
	tarAxes.axis[0] = -78.1765;
	tarAxes.axis[1] = -142.205;
	tarAxes.axis[2] = 69.7603;
	tarAxes.axis[3] = 35.4178;
	tarAxes.axis[4] = 172.793;
	tarAxes.axis[5] = -82.0188;
	tarAxes.axis[6] = -70.0245;

	tarAxesR.axis[0] = 56.9947;
	tarAxesR.axis[1] = -142.994;
	tarAxesR.axis[2] = -82.5392;
	tarAxesR.axis[3] = 30.7012;
	tarAxesR.axis[4] = -169.885;
	tarAxesR.axis[5] = -83.5411;
	tarAxesR.axis[6] = 85.8418;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str10(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Down In" << "," << "left" << "," << str10 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Down In" << "," << "left" << "," << str10 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Down In" << "," << "right" << "," << str10 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Down In" << "," << "right" << "," << str10 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str10 << " -- Center Down In Moved" << endl;

	timer.waitUntil(10000);


	//center up out
	tarAxes.axis[0] = -145.108;
	tarAxes.axis[1] = -89.5805;
	tarAxes.axis[2] = 20.4954;
	tarAxes.axis[3] = 1.58506;
	tarAxes.axis[4] = 184.229;
	tarAxes.axis[5] = -53.4151;
	tarAxes.axis[6] = -23.7945;

	tarAxesR.axis[0] = 142.002;
	tarAxesR.axis[1] = -90.4227;
	tarAxesR.axis[2] = -15.5709;
	tarAxesR.axis[3] = 0.340765;
	tarAxesR.axis[4] = -183.958;
	tarAxesR.axis[5] = -43.519;
	tarAxesR.axis[6] = 101.056;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	timer.waitUntil(2000);

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str11(buffer);
	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Up Out" << "," << "left" << "," << str11 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Up Out" << "," << "left" << "," << str11 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Up Out" << "," << "right" << "," << str11 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Up Out" << "," << "right" << "," << str11 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str11 << " -- Center Up Out Moved" << endl;

	timer.waitUntil(10000);

	//center out
	tarAxes.axis[0] = -122.099;
	tarAxes.axis[1] = -111.194;
	tarAxes.axis[2] = 38.281;
	tarAxes.axis[3] = 14.9299;
	tarAxes.axis[4] = 169.395;
	tarAxes.axis[5] = -70.7392;
	tarAxes.axis[6] = -8.91913;

	tarAxesR.axis[0] = 112.993;
	tarAxesR.axis[1] = -111.627;
	tarAxesR.axis[2] = -34.5471;
	tarAxesR.axis[3] = 13.4973;
	tarAxesR.axis[4] = -159.898;
	tarAxesR.axis[5] = -63.5455;
	tarAxesR.axis[6] = 86.3264;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str12(buffer);
	timer.waitUntil(2000);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Out" << "," << "left" << "," << str12 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Out" << "," << "left" << "," << str12 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Out" << "," << "right" << "," << str12 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Out" << "," << "right" << "," << str12 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str12 << " -- Center Out Moved" << endl;

	timer.waitUntil(10000);

	//center down out
	tarAxes.axis[0] = -124.378;
	tarAxes.axis[1] = -112.144;
	tarAxes.axis[2] = 37.2575;
	tarAxes.axis[3] = -1.11087;
	tarAxes.axis[4] = 172.215;
	tarAxes.axis[5] = -85.9748;
	tarAxes.axis[6] = -11.9132;

	tarAxesR.axis[0] = 114.792;
	tarAxesR.axis[1] = -114.435;
	tarAxesR.axis[2] = -36.2072;
	tarAxesR.axis[3] = -4.65455;
	tarAxesR.axis[4] = -164.31;
	tarAxesR.axis[5] = -82.7036;
	tarAxesR.axis[6] = 90.4722;

	if (armL.MoveToAxisTarget(tarAxes) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	if (armR.MoveToAxisTarget(tarAxesR) != CANON_SUCCESS)
	{
		std::cout << "motion error" << endl;
	}
	timer.waitUntil(2000);

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str13(buffer);

	if (armL.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Down Out" << "," << "left" << "," << str13 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Down Out" << "," << "left" << "," << str13 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}
	if (armR.GetRobotPose(&curPose_r) == CANON_SUCCESS)
	{
		outf << "Center Down Out" << "," << "right" << "," << str13 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;
		std::cout << "Center Down Out" << "," << "right" << "," << str13 << "," << curPose_r.x << ", " << curPose_r.y << ", " << curPose_r.z << ", " << curPose_r.xrot << ", " << curPose_r.yrot << ", " << curPose_r.zrot << "" << endl;

		armR.GetRobotPose(&poseR);
	}


	std::cout << str13 << " -- Center Down Out Moved" << endl;

	timer.waitUntil(1000);

	// Close the outstream
	outf.close();

	return 0;
}