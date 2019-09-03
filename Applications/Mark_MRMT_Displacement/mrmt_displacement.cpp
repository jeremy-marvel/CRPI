
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "crpi_kuka_lwr.h"
#include "crpi_robotiq.h"
#include <vector>
#include <string>
#include "ulapi.h"
#include <algorithm>
#include <time.h>
#include <math.h>
//Multi-threading using windows processes
#include <process.h>

#pragma warning (disable: 4996)

#define PI 3.14159265
//#define TEACH_PROGRAM
#define COGNEX_VISION
#define ROBOTIQ

using namespace crpi_robot;
using namespace std;

typedef CrpiUniversal robType;
//CrpiRobot<CrpiUniversal> arm("universal_ur5_table.xml");
CrpiRobot<CrpiUniversal> arm("universal_ur10_right.xml");

//typedef CrpiKukaLWR robType;
//CrpiRobot<robType> arm("kuka_lwr.xml");


//Global Variables
vector<vector<double>> COG_DATA;
//vector<double> COG_DATA(8);
string TEST = "paper"; //board paper
string ARM = "ur10"; //kuka  ur5  ur10
string EEF = "robotiq"; //schunk  robotiq
double Z = 86.0; //Z value (mm) for picking disc. Must be hardcoded when using Cognex	//-706.0 gripper	-658.0 Robotiq 
double place_clearance = 0.0; //place clearance (mm): +ive (gap), or -ive (push slightly into table)
double ARM_VEL = 1.0; // 0 - 1: Robot arm's speed during testing (UR Robots only)

//Thread definition
bool cogThreadRun = false;
uintptr_t cogThread = 0;


bool sort2ndTerm(const vector<double> & p1, const vector<double>& p2) {
	return p1[1] < p2[1];
}

void sortCog(vector<vector<double>> & vec) {
	sort(vec.begin(), vec.end(), sort2ndTerm); //first sort by y coordinate (columns)
	for (int i = 0; i < vec.size(); i = i + 4) { //sort each column by x coordinate
		sort(vec.begin() + i, vec.begin() + (4 + i));
	}
}

static unsigned int __stdcall cogThreadProc(void* inst)
{
	cout << "Connecting to camera..." << endl;
	//Initialization for Cognex and Camera
	ulapi_integer server_cam;
	
	server_cam = ulapi_socket_get_client_id(5005, "169.254.152.64");

	int get_cam;
	const int MSG_SIZE = 320;

	char inbuffer_cam[MSG_SIZE];

	for (int i = 0; i < 20; i++) {
		vector<double> tmpVec;
		for (int j = 0; j < 2; j++) {
			tmpVec.push_back(-1);
		}
		COG_DATA.push_back(tmpVec);
	}

	while (cogThreadRun == true) {
		for (int x = 0; x < MSG_SIZE; ++x) { inbuffer_cam[x] = '\0'; }
		
		while (1)
		{
			get_cam = ulapi_socket_read(server_cam, inbuffer_cam, MSG_SIZE);
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

		//Pull out XYZ numbers assuming they all exist and are in order, and are in a char array with spaces separating values
		char* pEnd;
		COG_DATA[0][0] = (strtod(inbuffer_cam, &pEnd)); //get disc 1 x coordinate
		COG_DATA[0][1] = (strtod(pEnd, &pEnd)); //get disc 2 y coordinate

		for (unsigned short int jj = 1; jj <= 18; ++jj) //gets corner discs coordinates 
		{
			COG_DATA[jj][0] = (strtod(pEnd, &pEnd));
			COG_DATA[jj][1] = (strtod(pEnd, &pEnd));
		}
		COG_DATA[19][0] = (strtod(pEnd, &pEnd)); //get disc 2 y coordinate
		COG_DATA[19][1] = strtod(pEnd, NULL); //get last data (should equal 0)
		
		sortCog(COG_DATA);

		//cout << "Data: " << COG_DATA[0] << " " << COG_DATA[1] << " " << COG_DATA[2] << endl;
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

struct Coordinate
{
	double x, y, z;

	Coordinate() {
		x = y = z = 0;
	}
	
	Coordinate(double paramx, double paramy, double paramz) : x(paramx), y(paramy), z(paramz) {}

};

void robotiq_pose(CrpiRobot<CrpiRobotiq> &riq, int position) {
	int execute = 1;
	riq.SetParameter("POSITION_FINGER_A", &position);
	riq.SetParameter("POSITION_FINGER_B", &position);
	riq.SetParameter("POSITION_FINGER_C", &position);
	riq.SetParameter("GRIP", &execute);
}

void gripper_pose(int position) {
	if (ARM == "kuka") {
		arm.SetTool((double)position);
		Sleep(150);
	}
	else {
		arm.SetRobotDO(0, position);
		Sleep(150);
	}
}


void main()
{
	cout << "NIST ISD Collaborative Robot Programming Interface Socket Handler" << endl << endl;
	cout << "MRMT DISPLACEMENT TEST" << endl << endl;
	cout << "Overview: Using one hand, discs are transferred sequentially from one hole to the next."
		<< "For this test, 59 disc are placed in the board, with one hole free to allow discs to be moved." << endl << endl;
	cin.get();

	arm.SetAngleUnits("degree");
	arm.SetLengthUnits("mm");

	robotPose curPose, home, pickPose[60], placePose[60], approachPose[60];
	int i, j;
	double num_cols = 15;
	double num_rows = 4;
	Coordinate corner_1, corner_2, corner_3, corner_4; //corner hole coordinates
	double dx_rows, dy_rows, dx_cols, dy_cols;
	char input = 'w';
	clock_t tStart = clock();
	clock_t tInter = clock();
	int eef_open = 0;
	int eef_close = 1;
	double approach_clearance = 17.0;

#ifdef COGNEX_VISION
	ofstream outfile("Data/Completion_Times/" + ARM + "_" + EEF + "_cognex_" + TEST + ".csv", ios::app);
#endif // COGNEX_VISION
#ifdef TEACH_PROGRAM
	ofstream outfile("Data/Completion_Times/" + ARM + "_" + EEF + "_teach_" + TEST + ".csv", ios::app);
#endif // TEACH_PROGRAM


	//set a slow robot arm speed during pretest motions
	if (ARM == "ur5" || ARM == "ur10") {
		arm.SetRelativeSpeed(0.1);
	}
	if (ARM == "kuka") {
		//NOT YET IMPLEMENTED
	}

	//define and open pneumatic gripper
	char curtool[32];
	if (EEF == "schunk") {
		strcpy(curtool, "gripper_parallel");
		arm.Couple(curtool);
		gripper_pose(eef_open);
	}
#ifdef ROBOTIQ
	cout << "Robotiq Hand needs to be calibrated. Ensure it is free to perform its full range of motions before continuing...";
	cin.get();
	cout << endl << endl;
	CrpiRobot<CrpiRobotiq> hand("robotiq.xml");

	int param;
	//eef_close = 110;  //fully closed in pinch configuration 
	//eef_open = 0; //fully open - 153mm gap
	//Specific for MRMT Disc
	eef_close = 85;  //35 mm gap 
	eef_open = 71; //47 mm gap (approx 10 mm clearance when grasping disc)

	param = 0;
	hand.SetParameter("ADVANCED_CONTROL", &param);
	param = 2; //Pinch
	hand.SetParameter("GRIP_TYPE", &param);
	param = 255;
	hand.SetParameter("SPEED_FINGER_A", &param);
	hand.SetParameter("SPEED_FINGER_B", &param);
	hand.SetParameter("SPEED_FINGER_C", &param);
	param = 20;
	hand.SetParameter("FORCE_FINGER_A", &param);
	hand.SetParameter("FORCE_FINGER_B", &param);
	hand.SetParameter("FORCE_FINGER_C", &param);

	robotiq_pose(hand, eef_open);
#endif

	Sleep(1000);
	arm.GetRobotPose(&curPose);
	cout << "Current Arm Position: " << curPose.x << " " << curPose.y << " " << curPose.z << " " << curPose.xrot << " "
		<< curPose.yrot << " " << curPose.zrot << endl;

	ifstream infile("Data/" + ARM + "_home_position.dat");
	infile >> home.x >> home.y >> home.z >> home.xrot >> home.yrot >> home.zrot;
	infile.close();

	if (abs(abs(curPose.xrot) - abs(home.xrot)) > 40) {
		cout << endl << "Robot Pose mismatch. Check home position and try again...";
		cin.get();
		return;
	}

	cout << endl << "If required, reposition MRMT board now. Do corner positions need to retrained? (y/n): ";
	cin.get(input);
	cin.get();
	while (input != 'y' && input != 'n') {
		cout << endl << "(y / n) : ";
		cin.get(input);
		cin.get();
	}

#ifdef COGNEX_VISION
	if (input == 'y') {
		//Setup Cognex Thread and start reading in data
		cogThreadRun = true;
		cogThread = _beginthreadex(NULL, 0, cogThreadProc, NULL, 0, NULL);

		cout << endl << "Using Cognex Vision System to Detect MRMT Disc Locations" << endl << endl;
		cout << endl << endl << "Before running the program, place a disc in the vacant hole. The vision system will detect the four corner discs, "
			<< "which will then be used to calculate all pick / place locations" << endl << endl;
		cout << "********" << endl << "Robot will be moved out of cameras FOV. Setup time will be recorded from this point on. Press enter to continue...";
		cin.get();
		tStart = clock();
		//arm.MoveStraightTo(home); //move linearly to home position
		cout << endl << endl << "Press enter when ready to detect corner discs: ";
		cin.get();

		cout << "Detected Disc Positions: ";

		for (int i = 0; i < COG_DATA.size(); i++) {
			if (COG_DATA[i][0] == -1 || COG_DATA[i][1] == -1) {
				cout << endl << endl << "ERROR: DISCS NOT DETECTED." << endl;
				cout << "PRESS ENTER TO EXIT...";
				cin.get();
				kill_thread();
				return;
			}
			cout << "(" << COG_DATA[i][0] << "," << COG_DATA[i][1] << ")" << endl;
		}

		ofstream outfile2("Data/" + ARM + "_cognex_corner_positions.dat");
		outfile2 << COG_DATA[19][0] << " " << COG_DATA[19][1] << " ";
		outfile2 << COG_DATA[16][0] << " " << COG_DATA[16][1] << " ";
		outfile2 << COG_DATA[3][0] << " " << COG_DATA[3][1] << " ";
		outfile2 << COG_DATA[0][0] << " " << COG_DATA[0][1] << " ";
		outfile2.close();

		kill_thread(); //communication with Cognex no longer required

		outfile << "Setup time, " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
		cout << endl << "Setup time: " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
	}
	
	cout << endl << endl << "Setup Complete. Remove corner disc and press enter to calculate target positions...";
	cin.get();

	ifstream infile2("Data/" + ARM + "_cognex_corner_positions.dat");
	infile2 >> corner_1.x >> corner_1.y >> corner_2.x >> corner_2.y >> corner_3.x >> corner_3.y >> corner_4.x >> corner_4.y;
	infile2.close();

	double angle = atan2(corner_3.y - corner_1.y, corner_3.x - corner_1.x) * (180/PI);

	//Sanity Checks
	//cout << endl << "Detected Length between corners: " << sqrt(pow(corner_3.x - corner_1.x, 2) + pow(corner_3.y - corner_1.y, 2)) << endl;
	//if (abs(sqrt(pow(corner_3.x - corner_1.x, 2) + pow(corner_3.y - corner_1.y, 2)) - 0.8/3) > 0.02) {
	//	cout << endl << endl << "ERROR: CORNER POSITIONS INCORRECT..." << endl;
	//	cout << "PRESS ENTER TO EXIT...";
	//	cin.get();
	//	if (input == 'y') {
	//		kill_thread();
	//	}
	//	return;
	//}
	//if (abs(sqrt(pow(corner_2.x - corner_1.x, 2) + pow(corner_2.y - corner_1.y, 2)) - 0.175/3) > 0.02) {
	//	cout << endl << endl << "ERROR: CORNER POSITIONS INCORRECT..." << endl;
	//	cout << "PRESS ENTER TO EXIT...";
	//	cin.get();
	//	if (input == 'y') {
	//		kill_thread();
	//	}
	//	return;
	//}

	dx_cols = (corner_3.x - corner_1.x) / 4; //Only using a third of the board
	dy_cols = (corner_3.y - corner_1.y) / 4; 

	dx_rows = (corner_2.x - corner_1.x) / 3;
	dy_rows = (corner_2.y - corner_1.y) / 3;

	cout << endl << "dx_cols: " << dx_cols << " dy_cols: " << dy_cols << "dx_rows: " << dx_rows << " dy_rows: " << dy_rows << endl;

	corner_1.x -= 5 * dx_cols;
	corner_1.y -= 5 * dy_cols;

	std::vector<robotPose> poses;

	for (i = 0; i < num_cols; i++) {
		robotPose tmpPose;
		tmpPose.xrot = home.xrot;
		tmpPose.yrot = home.yrot;
		tmpPose.zrot = home.zrot + angle;
		tmpPose.z = Z;
		if (i % 2 == 0) {
			for (j = 0; j < num_rows; j++) {
				tmpPose.x = corner_1.x + (dx_cols*i) + (dx_rows*j);
				tmpPose.y = corner_1.y + (dy_cols*i) + (dy_rows*j);
				poses.push_back(tmpPose);
			}
		}
		else {
			for (j = num_rows - 1; j >= 0; j--) {
				tmpPose.x = corner_1.x + (dx_cols*i) + (dx_rows*j);
				tmpPose.y = corner_1.y + (dy_cols*i) + (dy_rows*j);
				poses.push_back(tmpPose);
			}
		}
	}

	for (i = 0; i < poses.size(); ++i) {
		arm.FromWorld(&poses[i], &pickPose[i]);
		placePose[i] = pickPose[i];
		approachPose[i] = pickPose[i];
		placePose[i].z += place_clearance;
		approachPose[i].z += approach_clearance;

		cout << "Pick Pose " << i << ": ";
		cout << pickPose[i].x << " " << pickPose[i].y << " " << pickPose[i].z << " "
			<< pickPose[i].xrot << " " << pickPose[i].yrot << " " << pickPose[i].zrot << endl;
	}
#endif //Cognex_Vision

#ifdef TEACH_PROGRAM
	double dz_rows, dz_cols;
	double xrot, yrot, zrot;

	if (input == 'y') {
		cout << "********" << endl << "Start of training sequence. Setup time will be recorded from this point on. Press enter to continue...";
		cin.get();
		tStart = clock();
		cout << endl << "To assist with alignment, a disc should first be grasped. Move gripper over disc and press enter to grab the disc...";
		cin.get();			
#ifdef ROBOTIQ
		robotiq_pose(hand, eef_close);
#endif
		if (EEF == "schunk") {
			gripper_pose(eef_close);
		}

		cout << endl << endl << "Programming Corner Positions" << endl << endl
			<< "First, align the robot with the top corner hole that is currently vancent."
			<< "Position the robot so that the gripper is in the place position." << endl << endl
			<< "Once the robot is in the place position, press enter to record the position...";

		ofstream outfile2("Data/" + ARM + "_teach_corner_poses.dat");

		cin.get();
		arm.GetRobotPose(&curPose);
		outfile2 << curPose.x << " " << curPose.y << " " << curPose.z << " " << curPose.xrot << " "
			<< curPose.yrot << " " << curPose.zrot << endl;

		cout << endl << "First corner position recorded." << endl << endl;
		cout << "Align the robot with the bottom corner hole, which is on the same side as the previous corner hole. "
			<< "This is the second corner hole that the robot will reach during the test." << endl << endl
			<< "Once the robot is in the place position, press enter to record the position...";

		cin.get();
		arm.GetRobotPose(&curPose);
		outfile2 << curPose.x << " " << curPose.y << " " << curPose.z << " " << curPose.xrot << " "
			<< curPose.yrot << " " << curPose.zrot << endl;

		cout << endl << "Second corner position recorded." << endl << endl;
		cout << "Align the robot with the top corner hole, located on the other side of the board. "
			<< "This is the third corner hole that the robot will reach during the test." << endl << endl
			<< "Once the robot is in the place position, press enter to record the position...";

		cin.get();
		arm.GetRobotPose(&curPose);
		outfile2 << curPose.x << " " << curPose.y << " " << curPose.z << " " << curPose.xrot << " "
			<< curPose.yrot << " " << curPose.zrot << endl;

		cout << endl << "Third corner position recorded." << endl << endl;
		cout << "Finally, align the robot with the fourth corner hole (bottom, same side). "
			<< "This is the last corner hole that the robot will reach during the test." << endl << endl
			<< "Once the robot is in the place position, press enter to record the position...";

		cin.get();
		arm.GetRobotPose(&curPose);
		outfile2 << curPose.x << " " << curPose.y << " " << curPose.z << " " << curPose.xrot << " "
			<< curPose.yrot << " " << curPose.zrot << endl;

		cout << endl << "Fourth corner position recorded." << endl << endl;
		outfile2.close();

		cout << "All corner positions recorded successfully! Press enter to open gripper and rise arm...";
		cin.get();
#ifdef ROBOTIQ
		robotiq_pose(hand, eef_open);
#endif
		if (EEF == "schunk") {
			gripper_pose(eef_open);
		}
		curPose.z += approach_clearance;
		arm.MoveStraightTo(curPose);

		outfile << "Setup time, " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
		cout << endl << "Setup time: " << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
	}
	
	cout << endl << "Ready to calculate target positions. Press enter to calculate...";
	cin.get();

	std::vector<Coordinate> coords;

	ifstream infile2("Data/" + ARM + "_teach_corner_poses.dat");
	infile2 >> corner_1.x >> corner_1.y >> corner_1.z >> xrot >> yrot >> zrot;
	infile2 >> corner_2.x >> corner_2.y >> corner_2.z >> xrot >> yrot >> zrot;
	infile2 >> corner_3.x >> corner_3.y >> corner_3.z >> xrot >> yrot >> zrot;
	infile2 >> corner_4.x >> corner_4.y >> corner_4.z >> xrot >> yrot >> zrot;
	infile2.close();

	dx_cols = (corner_3.x - corner_1.x) / (num_cols - 1);
	dy_cols = (corner_3.y - corner_1.y) / (num_cols - 1);
	dz_cols = (corner_3.z - corner_1.z) / (num_cols - 1);

	dx_rows = (corner_2.x - corner_1.x) / (num_rows - 1);
	dy_rows = (corner_2.y - corner_1.y) / (num_rows - 1);
	dz_rows = (corner_2.z - corner_1.z) / (num_rows - 1);

	for (i = 0; i < num_cols; i++) {
		if (i % 2 == 0) {
			for (j = 0; j < num_rows; j++) {
				coords.push_back(Coordinate(corner_1.x + (dx_cols*i) + (dx_rows*j), corner_1.y + (dy_cols*i) + (dy_rows*j), corner_1.z + (dz_cols*i) + (dz_rows*j)));
			}
		}
		else {
			for (j = num_rows - 1; j >= 0; j--) {
				coords.push_back(Coordinate(corner_1.x + (dx_cols*i) + (dx_rows*j), corner_1.y + (dy_cols*i) + (dy_rows*j), corner_1.z + (dz_cols*i) + (dz_rows*j)));
			}
		}
	}

	for (i = 0; i < coords.size(); ++i) {
		pickPose[i].x = placePose[i].x = approachPose[i].x = coords[i].x;
		pickPose[i].y = placePose[i].y = approachPose[i].y = coords[i].y;
		pickPose[i].z = coords[i].z;
		approachPose[i].z = coords[i].z + approach_clearance;
		placePose[i].z = coords[i].z + place_clearance;
		pickPose[i].xrot = placePose[i].xrot = approachPose[i].xrot = xrot;
		pickPose[i].yrot = placePose[i].yrot = approachPose[i].yrot = yrot;
		pickPose[i].zrot = placePose[i].zrot = approachPose[i].zrot = zrot;

		cout << "Pick Pose " << i << ": ";
		cout << pickPose[i].x << " " << pickPose[i].y << " " << pickPose[i].z << " "
			<< pickPose[i].xrot << " " << pickPose[i].yrot << " " << pickPose[i].zrot << endl;
	}
#endif TEACH_PROGRAM

	cout << endl << "MRMT Displacement Test - Execution " << endl << endl;
	cout << "WARNING: ENSURE THAT ROBOT WORKSPACE IS CLEAR PRIOR TO BEGINNING TEST" << endl << endl
		<< "WARNING: ENSURE THAT THE BOARD IS FIXED TO THE TABLE, AND THAT IT HAS NOT MOVED SINCE CORNER HOLE POSITIONS WERE LAST RECORDED" << endl << endl
		<< "WARNING: ENSURE THAT DISCS HAVE BEEN PLACED / RESET CORRECTLY IN THE BOARD" << endl << endl;
	//cout << "***********" << endl << "Read the above warnings. Press enter to move robot to home position...";
	cin.get();

	//arm.MoveStraightTo(home); //move linearly to home position

	outfile << endl << "Iteration, Completion Time" << endl;

	cout << endl << "******************" << endl << "Ready to begin MRMT Displacement Test. Press enter to move robot to starting position:";
	cin.get();

	arm.MoveStraightTo(approachPose[1]);

	arm.GetRobotPose(&curPose);
	cout << "Current Arm Position: " << curPose.x << " " << curPose.y << " " << curPose.z << " " << curPose.xrot << " "
		<< curPose.yrot << " " << curPose.zrot << endl;
	cin.get();


	arm.SetRelativeSpeed(ARM_VEL);
	cout << endl << endl << "Relative Arm Speed Set to: " << ARM_VEL;
	cout << endl << "Press enter to begin MRMT Displacement Test:";
	cin.get();

	tStart = clock();
	tInter = clock();

	for (i = 1; i < 60; i++) {
		//! Move above target pick location
		arm.MoveStraightTo(approachPose[i]);
		//! Approach and grip disc
		arm.MoveStraightTo(pickPose[i]);
#ifdef ROBOTIQ
		robotiq_pose(hand, eef_close);
#else
		gripper_pose(eef_close);
#endif
		//!Retract
		arm.MoveStraightTo(approachPose[i]);
		//! Move above target place location
		arm.MoveStraightTo(approachPose[i - 1]);
		//! Approach and release disc	
		if (TEST == "paper") {
			arm.MoveStraightTo(placePose[i - 1]);
		}

#ifdef ROBOTIQ
		robotiq_pose(hand, eef_open);
#else
		gripper_pose(eef_open);
#endif
		//!Retract
		if (TEST == "paper") {
			arm.MoveStraightTo(approachPose[i - 1]);
		}
		outfile <<  i << "," << (double)(clock() - tInter) / CLOCKS_PER_SEC << endl;
		tInter = clock();
	}
	outfile << "Total," << (double)(clock() - tStart) / CLOCKS_PER_SEC << endl;
	outfile.close();
	cout << endl << "Overall Completion time: " << (double)(clock() - tStart) / CLOCKS_PER_SEC;

	cout << endl << endl << "Minnesota Test Complete. Press enter to return robot to home position and exit program:";
	cin.get();

	//arm.SetRelativeSpeed(0.1);
	//arm.MoveStraightTo(home);
}
