///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       CRPI HRI/AR/Unity Interface
//  Workfile:        HRIInterface_Simple.cpp, derived from HRIInterface.cpp, derived from XMLInterface.cpp
//  Last Revision:   08/17/2021
//  Authors:         S. Bagchi
//
//  Description
//  ===========
//  Application to interface with Unity in order to send robot
//  information to AR devices or any interface communicating via websockets, 
//	e.g. Unity projects.
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <regex>
#include <fstream>

#ifdef WIN32
#include <stdlib.h>

#include "crpi_robot.h"
#include "crpi_abb.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"
#else
#include "../../Libraries/CRPI/crpi_robot.h"
#include "../../Libraries/CRPI/crpi_universal.h"
#include "../../Libraries/CRPI/crpi_abb.h"
#include "../../Libraries/CRPI/crpi_robotiq.h"
#include "../../Libraries/ulapi/src/ulapi.h"
#endif



using namespace std;
using namespace crpi_robot;

//typedef CrpiUniversal robArmType;



void main()
{
	// Initialize based on file contents
	ifstream infile("xmlsettings.dat");
	string robot, path;
	int port;
	//CrpiRobot<CrpiUniversal> arm("universal_ur5_table.xml");  // Redundant but needed a dummy
	bool consoleInput = false, clientConnected = false;
	ulapi_integer server, client;


	// Initialize UR5 arm
	path = "universal_ur5_table.xml";
	cout << ">Creating robot using " << path.c_str() << endl;
	CrpiRobot<CrpiUniversal> arm(path.c_str());
	cout << "Robot Created" << endl;

	//! Configure the default units
	arm.SetAngleUnits("degree");
	arm.SetLengthUnits("mm");
	// Set speed & acceleration to 20%
	arm.SetRelativeSpeed(0.2);
	arm.SetRelativeAcceleration(0.2);


	while (infile >> robot)
	{
		infile >> path >> port;

		cout << "Starting server for " << robot << " on port " << port << endl;

		/*if (robot == "UNIVERSAL") {
			cout << ">Creating robot using " << path.c_str() << endl;
			CrpiRobot<CrpiUniversal> arm(path.c_str());
			cout << "Robot Created" << endl;

			//! Configure the default units
			arm.SetAngleUnits("degree");
			arm.SetLengthUnits("mm");
			// Set speed & acceleration to 20%
			arm.SetRelativeSpeed(0.2);
			arm.SetRelativeAcceleration(0.2);

		}
		else */
		if (robot == "UNITY") {
			//! Create socket connection for Unity
			server = ulapi_socket_get_server_id(port);
			ulapi_socket_set_blocking(server);

			while (!clientConnected)
			{
				cout << ">Running on port " << port << " for the Unity server" << endl;
				client = ulapi_socket_get_connection_id(server);
				ulapi_socket_set_blocking(client);
				clientConnected = true;
				cout << "Remote server connected..." << endl;
			}

		}
		else if (robot == "COMMANDLINE") {
			consoleInput = true;

			cout << "Commandline input enabled." << endl;
			//cout << "1 Get Pose, 2 Get Forces, 3 Get Axes, 4 Set Axes, 5 Zero-G Mode.  0 to (??? TBD)" << endl;

			// TODO more commandline setup?
		}
		else
		{
			cout << "Error in 'xmlsettings.dat'.  Unknown robot type: " << robot << endl;
			//CrpiRobot<CrpiUniversal> arm("universal_ur5_table.xml");
		}
		Sleep(1000);
	} // while (infile >> robot)


	// Vars to store robot info
	robotPose curPose, curForces;
	robotAxes curAxes = robotAxes(6), targetAxes = robotAxes(6);
	bool freedriveStatus = false;

	// TEST ROBOT
	//CrpiRobot<CrpiUniversal> arm("universal_ur5_table.xml");
	//cout << "test" << endl;
	//arm.GetRobotPose(&curPose);
	//curPose.print();


	// Buffer for input
	char buffer[2048] = "";
	// Dummy for comparison
	char cmds[][2048] = { "Get Pose", "Get Forces", "Get Axes", "Send Axes", "Zero-G Mode" };
	// Buffer for sending data back
	char sendData[2048] = "";

	crpi_timer timer;
	ulapi_integer rec, sent;
	int key = 0;

	while (true)  // Main logic
	{
		//! --- Get input cmd ---
		if (clientConnected) 
		{
			rec = ulapi_socket_read(client, buffer, 2048);
			if (rec > 0)
			{
				cout << ">Buffer: " << buffer << endl;
			}

		}
		else if (consoleInput) // How do I make this not block for input?
		{
			cout << "Commands:  1 Get Pose, 2 Get Forces, 3 Get Axes, 4 Set Axes, 5 Zero-G Mode.  0 to (??? TBD)" << endl;
			cout << endl << "Enter cmd:" << endl;
			cin >> key;
			if (key > 0 && key <= 5) {
				strcpy(buffer, cmds[key - 1]);
			}
			else cout << "Unknown command entered." << endl;

			cout << ">Buffer: " << buffer << endl;
			key = 0;
		}
		else {
			cout << "No input connection, quitting program." << endl;
			return;
		}


		//! --- Check cmds ---
		//! 
		//! Get Pose
		if (strcmp(buffer, cmds[0]) == 0) {
			cout << ">Getting pose from robot..." << endl;

			arm.GetRobotPose(&curPose);

			//cout << "Current Pose:  (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", "
			//	<< curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;


			// Format for printing & sending back
			sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", curPose.x, curPose.y, curPose.z,
				curPose.xrot, curPose.yrot, curPose.zrot);
			cout << "Current pose:  " << sendData << endl;
			cout << endl;

			//sent = ulapi_socket_write(client, sendData, strlen(sendData));
		}

		//! Get Forces
		else if (strcmp(buffer, cmds[1]) == 0) {
			cout << ">Getting forces from robot..." << endl;

			arm.GetRobotForces(&curForces);

			// Format for printing & sending back
			sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", curForces.x, curForces.y, curForces.z,
				curForces.xrot, curForces.yrot, curForces.zrot);
			cout << "Current forces:  " << sendData << endl;
			cout << endl;

			//sent = ulapi_socket_write(client, sendData, strlen(sendData));
		}

		//! Get Axes (joint angles)
		else if (strcmp(buffer, cmds[2]) == 0) {
			cout << ">Getting joint angles from robot..." << endl;

			arm.GetRobotAxes(&curAxes);

			//! Format for printing & sending back - assumes 6 joints
			// sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", curAxes.axis.at(0), curAxes.axis.at(1),
			//	curAxes.axis.at(2), curAxes.axis.at(3), curAxes.axis.at(4), curAxes.axis.at(5));

			sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", curAxes.axis[0], curAxes.axis[1],
				curAxes.axis[2], curAxes.axis[3], curAxes.axis[4], curAxes.axis[5]);
			cout << "Current axes:  " << sendData << endl;
			//curAxes.print();
			cout << endl;

			//sent = ulapi_socket_write(client, sendData, strlen(sendData));
		}

		//! --- Set pose ---
		else if (strcmp(buffer, cmds[3]) == 0) {
			cout << ">Setting pose..." << endl;

			if (clientConnected) {
				// Send acknowledgement first
				sent = ulapi_socket_write(client, buffer, strlen(buffer));
				// Get buffer of actual pose
				rec = ulapi_socket_read(client, buffer, 2048);
			}
			else if (consoleInput) {
				cout << "Enter new pose axes.  Format:  .1f,.1f,.1f,.1f,.1f,.1f" << endl;
				cin >> buffer;
				rec = 1;
			}


			if (rec <= 0) {
				cout << ">No pose received." << endl;
				sent = ulapi_socket_write(client, "No pose received.", strlen(sendData));
			}
			else {
				string newPose = string(buffer);
				//cout << ">New Axes: " << newPose << endl;

				regex sep(",");
				sregex_token_iterator tokens(newPose.cbegin(), newPose.cend(), sep, -1);
				sregex_token_iterator end;
				int i = 0;
				string temp = "";
				for (; tokens != end; ++tokens) {
					temp = *tokens;
					//cout << "token found: " << temp << endl;

					targetAxes.axis[i] = atof(temp.c_str());
					//targetAxes.axis.at(i) = atof(temp.c_str());
					//cout << "added to targetAxes.axis: " << targetAxes.axis[i] << endl;
					i++;
				}

				// Move arm
				arm.MoveToAxisTarget(targetAxes);  // ***
				cout << "Set axes." << endl;


				sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", targetAxes.axis[0], targetAxes.axis[1],
					targetAxes.axis[2], targetAxes.axis[3], targetAxes.axis[4], targetAxes.axis[5]);
				cout << "Target axes:  " << sendData << endl;
				cout << endl;

				//sent = ulapi_socket_write(client, sendData, strlen(sendData));

			}
		}

		//! --- NEEDS CHECK:  Toggle 0-G mode ---
		else if (strcmp(buffer, cmds[4]) == 0) {
			cout << ">Toggling zero-G mode..." << endl;

			if (freedriveStatus) { // Is On, turn off
				arm.SetParameter("endfreedrive", NULL);
				freedriveStatus = false;
			}
			else { // Is off, turn on
				if (arm.SetParameter("freedrive", NULL) != CANON_SUCCESS)
				{
					cout << "Error entering freedrive mode!" << endl;
				}
				else {
					freedriveStatus = true;
				}
			}

			sprintf(sendData, "Freedrive Status: %d", freedriveStatus);
		}
		else {  // Echo if unrecognized

			//sent = ulapi_socket_write(client, buffer, strlen(buffer));
			strcpy(sendData, buffer);
		}

		// Clear buffer for next cmd
		memset(&buffer[0], 0, sizeof(buffer));


		//! Send back reply
		if (clientConnected) {
			sent = ulapi_socket_write(client, sendData, strlen(sendData));
		}
		else if (consoleInput) {
			//TODO commandline output
			cout << sendData << endl;
		}


		//! Throttle back evaluation of the main thread 
		timer.waitUntil(5);
	} // while (true)

}




/*
//! @brief Thread method for communicating with a UR robot
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void armUniversalHandlerThread(void* param)
{
	globalHandle* gH = (globalHandle*)param;
	ulapi_mutex_take(gH->handle);
	string path = gH->path;
	ulapi_mutex_give(gH->handle);
	cout << ">Creating robot using " << path.c_str() << endl;
	CrpiRobot<CrpiUniversal> arm(path.c_str());
	cout << "Robot Created" << endl;
	ulapi_integer server, client;
	bool clientConnected = true;  // Temporary workaround

	 // Set units to mm
	arm.SetLengthUnits("mm");
	// Set speed & acceleration to 20%
	arm.SetRelativeSpeed(0.2);
	arm.SetRelativeAcceleration(0.2);
	// Vars to store robot info
	robotPose curPose;
	robotPose curForces;
	robotAxes curAxes = robotAxes(6);
	robotAxes targetAxes = robotAxes(6);

	crpi_timer timer;
	char buffer[2048];
	string str;
	ulapi_integer rec, sent;

	//! Create socket connection
	ulapi_mutex_take(gH->handle);
	server = ulapi_socket_get_server_id(gH->port);
	client = ulapi_socket_get_connection_id(server);
	ulapi_mutex_give(gH->handle);
	ulapi_socket_set_blocking(server);

	while (gH->runThread)
	{
		// Fix this??
		if (!clientConnected)
		{
			cout << ">Running XML Interface on port " << gH->port << " for the Universal arm" << endl;
			client = ulapi_socket_get_connection_id(server);
			ulapi_socket_set_blocking(client);
			clientConnected = true;
			cout << "Remote Universal client connected..." << endl;
		}

		while (clientConnected && gH->runThread)
		{
			ulapi_mutex_take(gH->handle);

			// Get current pose from robot
			arm.GetRobotPose(&curPose);
			gH->curPose = curPose;
			//cout << "Current Pose:  (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", "
			//	<< curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;

			// Get forces
			arm.GetRobotForces(&curForces);
			gH->curForces = curForces;

			// Get joint angles
			arm.GetRobotAxes(&curAxes);
			gH->curAxes = curAxes;

			// Check if there is a new pose to set
			if (gH->setPose) {

				// Grab pose
				targetAxes = gH->newAxes;
				gH->setPose = false;

				// Move
				arm.MoveToAxisTarget(targetAxes);  // ***
				cout << "Set axes." << endl;
				cout << endl;
			}

			// Check if 0G mode should be toggled
			if (gH->toggle0G) {
				// Is off, turn on
				if (gH->freedriveStatus == false) {
					if (arm.SetParameter("freedrive", NULL) != CANON_SUCCESS)
					{
						cout << "Error entering freedrive mode!" << endl;
					};
				}

				// Is on, turn off
				else if (gH->freedriveStatus == true) {
					arm.SetParameter("endfreedrive", NULL);
				}

				gH->toggle0G = false;
			}

			ulapi_mutex_give(gH->handle);


			rec = ulapi_socket_read(client, buffer, 2048);
			if (rec > 0)
			{
				str = buffer;
				arm.CrpiXmlHandler(str);

				arm.CrpiXmlResponse(buffer);
				sent = ulapi_socket_write(client, buffer, strlen(buffer));

				cout << "Received: " << buffer << endl;
			}
			else
			{
				timer.waitUntil(5);
			}
		} // while (clientConnected && gH->runThread)
	} // while (gH->runThread)

	gH = NULL;
	return;
}
*/




/*

//! @brief Thread method for communicating with a Unity server
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void serverUnityHandlerThread(void* param)
{
	globalHandle* gH = (globalHandle*)param;
	//CrpiRobot<CrpiRobotiq> arm(gH->path.c_str());
	ulapi_integer server, client;
	bool clientConnected = false;

	crpi_timer timer;
	char buffer[2048] = "";
	// Clear buffer of previous data
	//memset(&buffer[0], 0, sizeof(buffer));
	//string str;
	ulapi_integer rec, sent;


	// Vars to store robot info
	robotPose curPose;
	robotPose curForces;
	robotAxes curAxes = robotAxes(6);
	robotAxes targetAxes = robotAxes(6);

	// Dummy for comparison
	char cmds[][2048] = { "Get Pose", "Get Forces", "Get Axes", "Send Axes", "Zero-G Mode" };
	// Buffer for sending data back
	char sendData[2048] = "";


	//! Create socket connection
	ulapi_mutex_take(gH->handle);
	server = ulapi_socket_get_server_id(gH->port);
	ulapi_mutex_give(gH->handle);
	ulapi_socket_set_blocking(server);

	while (gH->runThread)
	{
		if (!clientConnected)
		{
			cout << ">Running XML Interface on port " << gH->port << " for the Unity server" << endl;
			client = ulapi_socket_get_connection_id(server);
			ulapi_socket_set_blocking(client);
			clientConnected = true;
			cout << "Remote server connected..." << endl;
		}

		while (clientConnected && gH->runThread)
		{

			rec = ulapi_socket_read(client, buffer, 2048);
			if (rec > 0)
			{
				//str = buffer;
				//arm.CrpiXmlHandler(str);
				//arm.CrpiXmlResponse(buffer);
				//sent = ulapi_socket_write(client, buffer, strlen(buffer));
				cout << ">Buffer: " << buffer << endl;


				// Pose
				if (strcmp(buffer, cmds[0]) == 0) {
					cout << ">Getting pose from robot..." << endl;

					// Get info from Universal thread
					ulapi_mutex_take(gH->handle);
					curPose = gH->curPose;
					ulapi_mutex_give(gH->handle);
					//cout << "Current Pose:  (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", "
					//	<< curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;


					// Format for printing & sending back
					sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", curPose.x, curPose.y, curPose.z,
						curPose.xrot, curPose.yrot, curPose.zrot);
					cout << "Current pose:  " << sendData << endl;
					cout << endl;

					sent = ulapi_socket_write(client, sendData, strlen(sendData));
				}

				// Forces
				else if (strcmp(buffer, cmds[1]) == 0) {
					cout << ">Getting forces from robot..." << endl;

					// Get info from Universal thread
					ulapi_mutex_take(gH->handle);
					curForces = gH->curForces;
					ulapi_mutex_give(gH->handle);

					// Format for printing & sending back
					sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", curForces.x, curForces.y, curForces.z,
						curForces.xrot, curForces.yrot, curForces.zrot);
					cout << "Current forces:  " << sendData << endl;
					cout << endl;

					sent = ulapi_socket_write(client, sendData, strlen(sendData));
				}

				// Axes (joint angles)
				else if (strcmp(buffer, cmds[2]) == 0) {
					cout << ">Getting joint angles from robot..." << endl;


					// Get info from Universal thread
					ulapi_mutex_take(gH->handle);

					//cout << "curAxes" << endl;
					//curAxes.print();
					//cout << "gH->curAxes" << endl;
					//gH->curAxes.print();
					//cout << endl;

					//curAxes = gH->curAxes;
					//curAxes.axis = gH->curAxes.axis;

					curAxes = gH->curAxes;
					//curAxes.axis = gH->curAxes.getAxisVec();
					//curAxes.axes = gH->curAxes.getNum();

					ulapi_mutex_give(gH->handle);


					// Format for printing & sending back - assumes 6 joints!
					//sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", curAxes.axis.at(0), curAxes.axis.at(1),
					//	curAxes.axis.at(2), curAxes.axis.at(3), curAxes.axis.at(4), curAxes.axis.at(5));

					sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", curAxes.axis[0], curAxes.axis[1],
						curAxes.axis[2], curAxes.axis[3], curAxes.axis[4], curAxes.axis[5]);
					cout << "Current axes:  " << sendData << endl;
					//curAxes.print();
					cout << endl;

					sent = ulapi_socket_write(client, sendData, strlen(sendData));
				}

				// --- NEW:  Set pose ---
				else if (strcmp(buffer, cmds[3]) == 0) {
					cout << ">Setting pose..." << endl;

					// Send acknowledgement first
					sent = ulapi_socket_write(client, buffer, strlen(buffer));
					// Get buffer of actual pose
					rec = ulapi_socket_read(client, buffer, 2048);
					//rec = 0;
					if (rec <= 0) {
						cout << ">No pose received." << endl;
						sent = ulapi_socket_write(client, "No pose received.", strlen(sendData));
					}
					else {
						string newPose = string(buffer);
						//cout << ">New Axes: " << newPose << endl;

						regex sep(",");
						sregex_token_iterator tokens(newPose.cbegin(), newPose.cend(), sep, -1);
						sregex_token_iterator end;
						int i = 0;
						string temp = "";
						for (; tokens != end; ++tokens) {
							temp = *tokens;
							//cout << "token found: " << temp << endl;

							targetAxes.axis[i] = atof(temp.c_str());
							//targetAxes.axis.at(i) = atof(temp.c_str());
							//cout << "added to targetAxes.axis: " << targetAxes.axis[i] << endl;
							i++;
						}


						// Send info to Universal thread
						ulapi_mutex_take(gH->handle);

						gH->setPose = true;
						gH->newAxes = targetAxes;

						ulapi_mutex_give(gH->handle);


						//sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", targetAxes.axis.at(0), targetAxes.axis.at(1),
						//	targetAxes.axis.at(2), targetAxes.axis.at(3), targetAxes.axis.at(4), targetAxes.axis.at(5));

						sprintf(sendData, "(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)", targetAxes.axis[0], targetAxes.axis[1],
							targetAxes.axis[2], targetAxes.axis[3], targetAxes.axis[4], targetAxes.axis[5]);
						cout << "Target axes:  " << sendData << endl;
						cout << endl;

						sent = ulapi_socket_write(client, sendData, strlen(sendData));

					}
				}


				// --- NEW:  Turn on 0-G mode ---
				else if (strcmp(buffer, cmds[4]) == 0) {
					cout << ">Toggling zero-G mode..." << endl;

					// Send info to Universal thread
					ulapi_mutex_take(gH->handle);

					//gH->toggle0G = !gH->toggle0G;
					gH->toggle0G = true;

					ulapi_mutex_give(gH->handle);
				}


				else {  // Just echo if unrecognized

					sent = ulapi_socket_write(client, buffer, strlen(buffer));
				}


				// Clear buffer
				memset(&buffer[0], 0, sizeof(buffer));

			}
			else
			{
				timer.waitUntil(5);
			}
		} // while (clientConnected && gH->runThread)
	} // while (gH->runThread)

	gH = NULL;
	return;
}
*/