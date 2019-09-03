

///////////////////////////////////////////////////////////////////////////////
//
//  Original System: ISD CRPI
//  Subsystem:       Peg In Hole
//  Workfile:        main.cpp
//  Revision:        16 July, 2015
//  Author:          J. Marvel/J. Falco/M. Culleton
//
//  Description
//  ===========
//  Applicatons to support LWR and UR10 Assembly work
//  Peg insertion test modified for trangular setup. Note: ONLY TESTED WITH LWR
//  Peg insertion test setup:
//		1. Teach Program: Hole locations are recorded in the robot's coordinate system by moving the robot to each location. 
//			Currently these locations are saved by manually editing the kuka_hole_coordinates.dat file
//		2. Registration: Hole locations are recorded in world coordinates. These locations are saved by manually editing 
//			the kuka_hole_coordinates.dat file. These locations are later converted to the robot's coordinate system using the 
//			transform matrix generated during robot registration.
//		3. Cognex Vision: Hole locations are detected by the Cognex system and sent to the program. As the Cognex system is registered
//			to the world, these locations are later converted to the robot's coordinate system using the transform matrix generated during robot registration.
//		4. Simulated Uncertainty: Same setup as teach program. During testing, a simulated gaussian or uniform error is added to the recorded hole locations.
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <time.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include "crpi_robot.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#include "../../Libraries/MotionPrims/AssemblyPrims.h"
#include <process.h> //Multi-threading using windows processes

#pragma warning (disable: 4996)

#define PEGTEST
//#define MAPVISERR //This code used for Tsai Cognex assessment
#define SIMULATED_UNCERTAINTY
//#define TEACH_PROGRAM
//#define COGNEX_VISION
//#define REGISTRATION
//#define ROBOTIQ

using namespace crpi_robot;
using namespace std;
using namespace MotionPrims;
using namespace Math;

//typedef CrpiUniversal robType;
//CrpiRobot<CrpiUniversal> arm("universal_ur10_right.xml");
typedef CrpiKukaLWR robType;
CrpiRobot<robType> arm("kuka_lwr.xml");

//Global Variables
string SEARCH = "random";	//spiral random sobol
string ARM = "kuka";		//kuka ur10
bool CHAMFER = false;		//chamfer on peg 
double CLEARANCE = 0.3f;	//insertion clearance between peg and hole
double SURFACE_Z = 11.0f;	//to top surface of blocks. Varies depending on setup and eef
#ifdef ROBOTIQ
string EEF = "robotiq";
#else
string EEF = "schunk";		
#endif

vector<double> COG_DATA(6);		
//Thread definition
bool cogThreadRun = false;
uintptr_t cogThread = 0;

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
	passMe()
	{
		grabmutex = ulapi_mutex_new(89);

		//demo = NULL;
		keeprunning = true;
	}

	//! @brief Arm constructor
	//!
	passMe(CrpiRobot<robType>* ptr)
	{
		grabmutex = ulapi_mutex_new(89);
		robArm = ptr;
		//demo = NULL;
		keeprunning = true;
	}
};

string ToString(double num) {	//converts a double to a string. 
	ostringstream ss;
	ss << num;
	return ss.str();
}

static unsigned int __stdcall cogThreadProc(void* inst)
{
	cout << "Connecting to camera..." << endl;
	//Initialization for Cognex and Camera
	ulapi_integer server_cam;

	server_cam = ulapi_socket_get_client_id(5005, "169.254.152.64");

	int get_cam;
	const int SIZE = 100;

	char inbuffer_cam[SIZE];

	while (cogThreadRun == true) {
		for (int x = 0; x < SIZE; ++x) { inbuffer_cam[x] = '\0'; }

		while (1)
		{
			get_cam = ulapi_socket_read(server_cam, inbuffer_cam, SIZE);
			if (get_cam > 0) { break; }
		}

		//Copy over into string and replace commas with spaces
		string temp(inbuffer_cam);

		int data_size = temp.size();
		replace(temp.begin(), temp.end(), ',', ' ');

		//Copy back over into existing char array
		for (int ii = 0; ii < data_size; ++ii)
		{
			inbuffer_cam[ii] = temp[ii];
		}

		//Pull out XY coordinates assuming they all exist and are in order, and are in a char array with spaces separating values
		char* pEnd;
		COG_DATA[0] = (strtod(inbuffer_cam, &pEnd)); //get initial (hole 1.x) coordinate
		for (unsigned short int jj = 1; jj <= 4; ++jj) //gets remaining hole coordinates 
		{
			COG_DATA[jj] = (strtod(pEnd, &pEnd));
		}
		COG_DATA[5] = strtod(pEnd, NULL); //get last data 
	}
	return 0;
}

void kill_thread() {
	if (cogThreadRun)
	{
		printf("Stopping Cognex thread...");
		cogThreadRun = false;
		WaitForSingleObject((HANDLE)cogThread, INFINITE);
		CloseHandle((HANDLE)cogThread);
		cogThread = 0;
	}
	return;
}

//Set robotiq fingers to a given position
void robotiq_pose(CrpiRobot<CrpiRobotiq> &riq, int position) {
	int execute = 1;
	riq.SetParameter("POSITION_FINGER_A", &position);
	riq.SetParameter("POSITION_FINGER_B", &position);
	riq.SetParameter("POSITION_FINGER_C", &position);
	riq.SetParameter("GRIP", &execute);
}

//Open / Close pneumatic gripper - 
void gripper_pose(passMe &pm, int position) {
	ulapi_mutex_take(pm.grabmutex);
	if (ARM == "kuka") {
		arm.SetTool((double)position);
	}
	else {
		arm.SetRobotDO(0, position);
	}
	ulapi_mutex_give(pm.grabmutex);
	Sleep(150);
}

void main()
{
	cout << "NIST ISD Collaborative Robot Programming Interface Socket Handler" << endl << endl;
	cout << "PEG INSERTION TEST" << endl;
	cout << "Initial Setup: Three blocks with holes are located in the robots workspace in an equilateral triangular configuration. A peg is placed within hole 1 and 3, leaving hole 2 vacant. " << endl;
	cout << "Test Overview: Robot repeatedly grasps a peg and transfers it to the vacant hole. If direct insertion is not successful, a search strategy is executed in order to find the hole" << endl << endl; 
	cout << "Press enter to continue...";
	cin.get();
	
#ifdef PEGTEST
	int i = 0;
	int counter;
	int eef_open = 0;
	int eef_close = 1;
	int const insertions = 60;
	char input = 'x';
	char delim;
	char curtool[32];
	bool toolopen = true;
	bool poseDefined = false;
	bool tool = false;
	double sd = -1;			// Standard deviation of the uncertainty distribution - Used to define the search region
	double peg_z = 0.0f;	// Height of the peg's top surface from block surface. Updated later
	string chamf = "";		// Used when saving files to differentiate between test iterations 
	if (CHAMFER) {
		chamf = "_c";
	}
	CanonReturn peg_in_hole = CANON_RUNNING;

	robotPose poseMe, curPose, offsetPose[insertions+1];
	robotPose hole_1, hole_2, hole_3;
	robotIO io;
	robotAxes *robot_axes;
	robot_axes = new robotAxes();
	
	Assembly asbly;
	AssemblyTimer timer_start, timer_inter;
	double tStart, tInter;

	passMe pm(&arm); //! State variable used to communicate with the two threads
	pm.robArm->SetAngleUnits("degree");
	pm.robArm->SetLengthUnits("mm");


#ifdef ROBOTIQ
	cout << endl << "Robotiq Hand needs to be calibrated. Ensure it is free to perform its full range of motions before continuing...";
	cin.get();
	cout << endl << endl;
	CrpiRobot<CrpiRobotiq> hand("robotiq.xml");

	int param;
	//eef_close = 110;  //fully closed in pinch configuration 
	//eef_open = 0; //fully open - 153mm gap
	//Modified positions for pegs
	eef_close = 105;  //8mm gap (close until desired force is reached)
	eef_open = 85; //30 mm gap (approx 15 mm clearance when grasping peg)

	param = 0;
	hand.SetParameter("ADVANCED_CONTROL", &param);
	param = 2; //Pinch
	hand.SetParameter("GRIP_TYPE", &param);
	param = 255;
	hand.SetParameter("SPEED_FINGER_A", &param);
	hand.SetParameter("SPEED_FINGER_B", &param);
	hand.SetParameter("SPEED_FINGER_C", &param);
	param = 255;
	hand.SetParameter("FORCE_FINGER_A", &param);
	hand.SetParameter("FORCE_FINGER_B", &param);
	hand.SetParameter("FORCE_FINGER_C", &param);
	robotiq_pose(hand, eef_open);

	strcpy(curtool, "robotiq");
	pm.robArm->Couple(curtool);

	hole_1.zrot = hole_2.zrot = hole_3.zrot = -90.0f;

#else
	strcpy(curtool, "gripper_parallel");
	pm.robArm->Couple(curtool);

	hole_1.zrot = hole_2.zrot = hole_3.zrot = 0.0f;
#endif
	// Hole Positions - Fixed Parameters
	hole_1.z = SURFACE_Z;	
	hole_2.z = SURFACE_Z;
	hole_3.z = SURFACE_Z;
	hole_1.xrot = hole_2.xrot = hole_3.xrot = 180.0f;
	hole_1.yrot = hole_2.yrot = hole_3.yrot = 0.0f;

#ifdef SIMULATED_UNCERTAINTY
	cout << endl << "Using Simulated Placement Uncertainty. Choose SD value, where zero selects uniform random (0,1,2): ";
	cin >> sd;
	cin.get();
	while (sd != 0 && sd != 1 && sd != 2) {
		cout << endl << "Enter SD value (0,1,2): ";
		cin >> sd;
		cin.get();
	}

	if (sd == 0) {
		ifstream infile("Data/Simulated/Perception_Spoofs/perception_spoof_random.csv");
		for (int i = 1; i <= insertions; i++) {
			offsetPose[i].x = offsetPose[i].y = offsetPose[i].z = offsetPose[i].xrot = offsetPose[i].yrot = offsetPose[i].zrot = 0.0f;
			infile >> offsetPose[i].x >> delim >> offsetPose[i].y >> delim >> offsetPose[i].z >> delim >> offsetPose[i].xrot >> delim >> offsetPose[i].yrot >> delim >> offsetPose[i].zrot;
		}
		infile.close();
		sd = 2.0f;
		chamf += "_rnd.csv";
	}
	else {
		ifstream infile("Data/Simulated/Perception_Spoofs/perception_spoof_" + ToString(sd) + "mm.csv");
		for (int i = 1; i <= insertions; i++) {
			offsetPose[i].x = offsetPose[i].y = offsetPose[i].z = offsetPose[i].xrot = offsetPose[i].yrot = offsetPose[i].zrot = 0.0f;
			infile >> offsetPose[i].x >> delim >> offsetPose[i].y >> delim >> offsetPose[i].z >> delim >> offsetPose[i].xrot >> delim >> offsetPose[i].yrot >> delim >> offsetPose[i].zrot;
		}
		infile.close();
		chamf += "_" + ToString(sd) + "mm.csv";
	}

	ifstream infile2("Data/Simulated/" + ARM + "_hole_coordinates.dat");
	infile2 >> hole_1.x >> hole_1.y >> hole_2.x >> hole_2.y >> hole_3.x >> hole_3.y;
	infile2.close();

	ofstream results("Data/Simulated/" + ARM + "_" + EEF + "_" + SEARCH + chamf, ios::app);


#elif defined TEACH_PROGRAM
	ofstream results("Data/Teach_Program/" + ARM + "_" + EEF + "_" + SEARCH + chamf + ".csv", ios::app);
	sd = 0.5f; //define a small uncertainty standard deviation for search region because positions have been trained by hand

	cout << endl << "TEACH PROGRAMMING" << endl;
	cout << "If required, reposition base blocks now. Do hole positions need to retrained? (y/n): ";
	cin.get(input);
	cin.get();
	while (input != 'y' && input != 'n') {
		cout << endl << "Do hole positions need to retrained? (y/n) : ";
		cin.get(input);
		cin.get();
}
	if (input == 'y') {
		ofstream outfile("Data/Teach_Program/" + ARM + "_hole_coordinates.dat");

		cout << "********" << endl << "Start of training sequence. Setup time will be recorded from this point on. Press enter to continue...";
		cin.get();
		tStart = clock();
		cout << endl << "To assist with alignment, a peg should first be grasped. Move tool over peg and press enter to grasp the peg...";
		cin.get();

#ifdef ROBOTIQ
		robotiq_pose(hand, eef_close);
#else
		gripper_pose(pm, eef_close);
#endif
		cout << endl << endl << "Programming Hole Positions" << endl << endl
			<< "Align the peg with the first hole and press enter to record its (x,y) coordinates...";
		cin.get();

		arm.GetRobotPose(&curPose);
		outfile << curPose.x << " " << curPose.y << endl;

		cout << endl << "First hole position recorded." << endl << endl;
		cout << "Align the peg with the second hole and press enter to record its (x,y) coordinates...";
		cin.get();

		arm.GetRobotPose(&curPose);
		outfile << curPose.x << " " << curPose.y << endl;

		cout << endl << "Second hole position recorded." << endl << endl;
		cout << "Align the peg with the final hole and press enter to record its (x,y) coordinates...";
		cin.get();

		arm.GetRobotPose(&curPose);
		outfile << curPose.x << " " << curPose.y << endl;
		
		outfile.close();
		cout << "All hole positions recorded successfully! Press enter to release peg and rise arm...";
		cin.get();

#ifdef ROBOTIQ
		robotiq_pose(hand, eef_open);
#else
		gripper_pose(pm, eef_open);
#endif

		curPose.z += 20;
		pm.robArm->MoveStraightTo(curPose);

		results << "Setup time, " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
		cout << endl << "Setup time: " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
	}
	ifstream infile2("Data/Teach_Program/" + ARM + "_hole_coordinates.dat");
	infile2 >> hole_1.x >> hole_1.y >> hole_2.x >> hole_2.y >> hole_3.x >> hole_3.y;
	infile2.close();

#elif defined REGISTRATION
	ofstream results("Data/Registration/" + ARM + "_" + EEF + "_" + SEARCH + chamf + ".csv", ios::app);
	robotPose reg_hole_1, reg_hole_2, reg_hole_3;
	sd = 1.5;

	cout << endl << "REGISTRATION";
	cout << endl << "Using predefined world coordinates for blocks. Have blocks been moved? (y/n): ";
	cin.get(input);
	cin.get();
	while (input != 'y' && input != 'n') {
		cout << endl << "Do hole positions need to retrained? (y/n) : ";
		cin.get(input);
		cin.get();
	}
	if (input == 'y') {
		cout << "Setup time will recorded from this point on. Press enter once kuka_hole_coordinates.dat file is open...";
		cin.get();
		tStart = clock();
		cout << endl << endl << "Determine block locations within the world and update their coordinates. Save and close file. Press enter when complete...";
		cin.get();

		results << "Setup time, " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
		cout << endl << "Setup time: " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
		cout << endl << endl << "Setup Complete. Ensure test is correctly setup and press enter to continue...";
		cin.get();
	}
	
	reg_hole_1 = hole_1;
	reg_hole_2 = hole_2;
	reg_hole_3 = hole_3;

	ifstream infile2("Data/Registration/" + ARM + "_hole_coordinates.dat");
	infile2 >> reg_hole_1.x >> reg_hole_1.y >> reg_hole_2.x >> reg_hole_2.y >> reg_hole_3.x >> reg_hole_3.y;
	infile2.close();

	pm.robArm->FromWorld(&reg_hole_1, &hole_1);
	pm.robArm->FromWorld(&reg_hole_2, &hole_2);
	pm.robArm->FromWorld(&reg_hole_3, &hole_3);


#elif defined COGNEX_VISION
	ofstream results("Data/Cognex/" + ARM + "_" + EEF + "_" + SEARCH + chamf + ".csv", ios::app);
	robotPose cog_hole_1, cog_hole_2, cog_hole_3;
	sd = 1.5f; //define an uncertainty standard deviation for search region

	cout << endl << "COGNEX VISION" << endl;
	cout << "If required, reposition base blocks now. Do hole positions need to retrained? (y/n): ";
	cin.get(input);
	cin.get();
	while (input != 'y' && input != 'n') {
		cout << endl << "Do hole positions need to retrained? (y/n) : ";
		cin.get(input);
		cin.get();
	}
	if (input == 'y') {
		//Setup Cognex Thread and start reading in data
		cogThreadRun = true;
		cogThread = _beginthreadex(NULL, 0, cogThreadProc, NULL, 0, NULL);
		Sleep(500);

		cout << endl << "Using Cognex Vision System to Detect Hole Positions" << endl;
		cout << "Setup time will be recorded from this point on. Press enter and then remove all pegs from holes...";
		cin.get();
		tStart = clock();
		cout << endl << endl << "Press enter when ready to detect hole positions...";
		cin.get();

		ofstream outfile("Data/Cognex/" + ARM + "_hole_coordinates.dat");
		cout << "Detected Hole Positions: ";
		for (i = 0; i < 6; i++) {
			if (COG_DATA[i] == 0) {
				cout << endl << endl << "ERROR: CORNER DISCS NOT DETECTED." << endl;
				cout << "PRESS ENTER TO EXIT...";
				cin.get();
				kill_thread();
				return;
			}
			cout << COG_DATA[i] << ", ";
			outfile << COG_DATA[i] << " ";
		}
		outfile.close();
		kill_thread(); //communication with Cognex no longer required

		results << "Setup time, " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
		cout << endl << "Setup time: " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
		cout << endl << endl << "Setup Complete. Replace two pegs and press enter to continue...";
		cin.get();
	}
	cog_hole_1 = hole_1;
	cog_hole_2 = hole_2;
	cog_hole_3 = hole_3;
	
	ifstream infile2("Data/Cognex/" + ARM + "_hole_coordinates.dat");
	infile2 >> cog_hole_1.x >> cog_hole_1.y >> cog_hole_2.x >> cog_hole_2.y >> cog_hole_3.x >> cog_hole_3.y;
	infile2.close();
	
	pm.robArm->FromWorld(&cog_hole_1, &hole_1);
	pm.robArm->FromWorld(&cog_hole_2, &hole_2);
	pm.robArm->FromWorld(&cog_hole_3, &hole_3);
#endif

	if (SEARCH == "spiral") {
		//asbly.AddSearchSpiral(10, 20.0f, 280.0f); 
		if (CHAMFER) {
			asbly.AddSearchSpiralOptimal(CLEARANCE*10, sd * 5, 360.0f);	//define search region as 5 times the standard deviation
		}
		else {
			asbly.AddSearchSpiralOptimal(CLEARANCE*2, sd * 5, 360.0f);	//define search region as 5 times the standard deviation
		}
	}
	else if (SEARCH == "random") {
		asbly.AddSearchRandom(sd * 5, false);	//define search region as 4 times the standard deviation
	}
	else { //sobol search
		asbly.AddSearchSobol(sd * 5, false);	//define search region as 5 times the standard deviation
	}
	//asbly.AddSearchConstOffset(0.0, 0.0, -12.0f);
	asbly.AddTerminatorTimer(CANON_FAILURE, 140.0f); //max search time before giving up 

	asbly.AddTerminatorDistance(CANON_SUCCESS, -1, -1, 5, -1); //10 previous

	cout << endl << endl << "Ready to run test. Select an option:" << endl;
	cout << "1) Run " << SEARCH << " insertion test   2) Open tool   3) Close tool  -1) Quit : ";
	cin >> i;

	while (i != -1)
	{
		if (!poseDefined)
		{
			curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
			ulapi_mutex_take(pm.grabmutex);
			pm.robArm->GetRobotPose(&curPose);
			ulapi_mutex_give(pm.grabmutex);
			poseDefined = true;
		}
		poseMe = curPose;
		switch (i)
		{
		case 1:
			results << endl << "Iteration," << "Approach," << "Pickup," << "Transport," << "Insert," << "Total" << endl;

			for (int j = 1; j <= insertions; j++)
			{
				results << j << ",";

				// Alternate between three holes
				if (peg_in_hole == CANON_SUCCESS || j == 1)
				{
					if (j % 3 == 1) { poseMe = hole_1; }
					else if (j % 3 == 2) { poseMe = hole_3; }
					else { poseMe = hole_2; }

					peg_z = poseMe.z + 25.0; //only half of peg is exposed above surface

					tStart = timer_start.startTimer();
					tInter = timer_inter.startTimer();

					// Hole approach++
					//poseMe.z = 50.0f;
					poseMe.z = peg_z + 10.0f; //allow 10mm clearance during approach
					ulapi_mutex_take(pm.grabmutex);
					if (pm.robArm->MoveStraightTo(poseMe) == CANON_SUCCESS) { curPose = poseMe; }
					ulapi_mutex_give(pm.grabmutex);

					// Open Gripper
#ifdef ROBOTIQ
					robotiq_pose(hand, eef_open);
#else
					gripper_pose(pm, eef_open);
#endif
					tInter = timer_inter.timeElapsed();
					results << tInter << ",";
					timer_inter.stopTimer();
					timer_inter.startTimer();

					// peg location
					poseMe.z = peg_z - 20.0f; //grab 20mm of peg (leaving 30 mm exposed)
					ulapi_mutex_take(pm.grabmutex);
					if (pm.robArm->MoveStraightTo(poseMe) == CANON_SUCCESS) { curPose = poseMe; }
					ulapi_mutex_give(pm.grabmutex);

					// Close Gripper
#ifdef ROBOTIQ
					robotiq_pose(hand, eef_close);
#else
					gripper_pose(pm, eef_close);
#endif
					// Hole 1 retract
					poseMe.z = peg_z + 15.0f; //10 mm clearance between bottom of peg and block surface
					ulapi_mutex_take(pm.grabmutex);
					if (pm.robArm->MoveStraightTo(poseMe) == CANON_SUCCESS) { curPose = poseMe; }
					ulapi_mutex_give(pm.grabmutex);
				}
				tInter = timer_inter.timeElapsed();
				results << tInter << ",";
				timer_inter.stopTimer();
				timer_inter.startTimer();

				// Alternate hole 1 and 2 insertions
				if (j % 3 == 1) { poseMe = hole_2; }
				else if (j % 3 == 2) { poseMe = hole_1; }
				else { poseMe = hole_3; }

				peg_z = poseMe.z + 25.0f;

#ifdef SIMULATED_UNCERTAINTY
				//apply offset error 
				poseMe.x = poseMe.x + offsetPose[j].x;
				poseMe.y = poseMe.y + offsetPose[j].y;
#endif
				poseMe.z = peg_z + 15.0f; //maintain 10 mm clearance between bottom of peg and block surface 

			    //move to approach pose
				ulapi_mutex_take(pm.grabmutex);
				if (pm.robArm->MoveStraightTo(poseMe) == CANON_SUCCESS) { curPose = poseMe; }
				ulapi_mutex_give(pm.grabmutex);
				tInter = timer_inter.timeElapsed();
				results << tInter << ",";
				timer_inter.stopTimer();
				timer_inter.startTimer();

				//move to insert
				poseMe.z = peg_z + 5.0f; //approach so that peg is touching block surface

				ulapi_mutex_take(pm.grabmutex);
				if (pm.robArm->MoveStraightTo(poseMe) == CANON_SUCCESS) { curPose = poseMe; }
				ulapi_mutex_give(pm.grabmutex);

				counter = 0;
				ulapi_mutex_take(pm.grabmutex);
				pm.robArm->GetRobotPose(&curPose);
				pm.robArm->GetRobotIO(&io);
				ulapi_mutex_give(pm.grabmutex);
				peg_in_hole = asbly.RunAssemblyStep(counter++, curPose, poseMe, io); //initialization of new search
				while (peg_in_hole == CANON_RUNNING)
				{
					poseMe.z = peg_z - 7.0f; //attempt to move peg down by 12 mm 
					ulapi_mutex_take(pm.grabmutex);
					pm.robArm->MoveAttractor(poseMe);
					pm.robArm->GetRobotPose(&curPose);
					pm.robArm->GetRobotIO(&io);
					pm.robArm->GetRobotAxes(robot_axes);	
					ulapi_mutex_give(pm.grabmutex);
					peg_in_hole = asbly.RunAssemblyStep(counter++, curPose, poseMe, io);
					//if (peg_in_hole != CANON_SUCCESS) {
					//	if (fabs(robot_axes->axis[2]) > 2.0f) { //search taking a long time. Reset e1 and j4, otherwise elbow will drift towards table and fail 
					//		ulapi_mutex_take(pm.grabmutex);
					//		robot_axes->axis[2] = 0.0f;
					//		robot_axes->axis[4] = 0.0f;
					//		pm.robArm->MoveToAxisTarget(*robot_axes);
					//		ulapi_mutex_give(pm.grabmutex);
					//	}
					//}
				}
				if (peg_in_hole == CANON_SUCCESS)
				{
					//poseMe.z = peg_z - 15.0f; // insert peg 20 mm into hole
					//ulapi_mutex_take(pm.grabmutex);
					//if (pm.robArm->MoveAttractor(poseMe) == CANON_SUCCESS) { curPose = poseMe; }
					//ulapi_mutex_give(pm.grabmutex);

					// Open Gripper
#ifdef ROBOTIQ
					robotiq_pose(hand, eef_open);
#else
					gripper_pose(pm, eef_open);
#endif
					tInter = timer_inter.timeElapsed();
					results << tInter << ",";
				}
				else {
					cout << "Insertion unsuccessful. Press enter to exit...";
					cin.get();
					return;
				}

				tStart = timer_start.timeElapsed();
				cout << endl << endl << "Iteration " << j << ":  " << tStart << " s." << endl;
				results << tStart << endl;
				timer_start.stopTimer();
				timer_inter.stopTimer();

				//retract straight up 
				pm.robArm->GetRobotPose(&poseMe);
				poseMe.z = peg_z + 20.0f;

				ulapi_mutex_take(pm.grabmutex);
				if (pm.robArm->MoveStraightTo(poseMe) == CANON_SUCCESS) { curPose = poseMe; }
				pm.robArm->GetRobotAxes(robot_axes);
				robot_axes->axis[2] = 0.0f;		//reset e1 and j4, otherwise elbow will drift towards table and fail 
				robot_axes->axis[4] = 0.0f;		
				pm.robArm->MoveToAxisTarget(*robot_axes);
				ulapi_mutex_give(pm.grabmutex);
			}
			results.close();
			break;

		case 2:
			// Open Gripper
#ifdef ROBOTIQ
			robotiq_pose(hand, eef_open);
#else
			gripper_pose(pm, eef_open);
#endif
			break;

		case 3:
			// Close Gripper
#ifdef ROBOTIQ
			robotiq_pose(hand, eef_close);
#else
			gripper_pose(pm, eef_close);
#endif
			break;

		default:
			break;
		}

		cout << endl << endl << "Task Complete. Select an option:" << endl;
		cout << "1) Run " << SEARCH << " insertion test   2) Open tool   3) Close tool  -1) Quit : ";
		cin >> i;
	} // while (i != -1)


#elif defined MAPVISERR  //This code used for Tsai Cognex assessment

	CrpiRobot<robType> arm("kuka_lwr.xml");
	Assembly asbly;
	char curtool[32];

	passMe pm(&arm); //! State variable used to communicate with the two threads
	pm.robArm->SetAngleUnits("degree");
	pm.robArm->SetLengthUnits("mm");

	strcpy(curtool, "gripper_parallel");

	pm.robArm->Couple(curtool);

	robotPose poseMe, curPose, calPose;

	robotIO io;
	bool poseDefined = false;
	int pauseTime, testPositions, testRepetitions;
	ifstream infile("Perception_DOE.csv");
	char delim;
	double testXY[2][31];

	testPositions = 31;
	testRepetitions = 1;
	pauseTime = 5000;

	// Read in datafile

	for (int k = 0; k < testPositions; k++)
	{
		infile >> testXY[0][k] >> delim >> testXY[1][k];
		//cout << "X: " << testXY[0][k] << "   Y: " << testXY[1][k] << endl;
		//Sleep(500);
	}

	cout << "1) Run, -1) quit : ";
	cin >> i;

	while (i != -1)
	{
		if (!poseDefined)
		{
			curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
			ulapi_mutex_take(pm.grabmutex);
			pm.robArm->GetRobotPose(&curPose);
			ulapi_mutex_give(pm.grabmutex);
			poseDefined = true;
		}

		calPose = curPose;
		poseMe = curPose;

		switch (i)
		{
		case 1:
			for (int n = 0; n < testRepetitions; n++)
			{
				for (int k = 0; k < testPositions; k++)
				{
					poseMe.x = testXY[0][k];
					poseMe.y = testXY[1][k];
					ulapi_mutex_take(pm.grabmutex);
					if (pm.robArm->MoveStraightTo(poseMe) == CANON_SUCCESS) { curPose = poseMe; }
					ulapi_mutex_give(pm.grabmutex);
					Sleep(pauseTime);
				}
			}

			infile.close();
			/*poseMe = calPose;
			ulapi_mutex_take(pm.grabmutex);
			if (pm.robArm->MoveStraightTo (poseMe) == CANON_SUCCESS){ curPose = poseMe; }
			ulapi_mutex_give(pm.grabmutex);*/
			break;

		default:
			break;
		}
		cout << "1) Run, -1) quit : ";
		cin >> i;
	} // while (i != -1) 

#endif
}