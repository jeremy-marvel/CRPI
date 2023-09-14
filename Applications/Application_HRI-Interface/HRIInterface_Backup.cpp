///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       CRPI HRI/AR Interface
//  Workfile:        HRIInterface.cpp, derived from XMLInterface.cpp
//  Last Revision:   07/29/2021
//  Authors:         S. Bagchi, J. Marvel
//
//  Description
//  ===========
//  Application to interface with Unity in order to send robot
//  information to AR devices or any interface communicating via websockets, 
//	e.g. Unity projects.
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <regex>

#include <fstream>
#include "crpi_robot.h"
#include "crpi_abb.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "ulapi.h"

//#define XMLINTERFACE_NOISY
//#define XMLINTERFACE_DEBUGTEST

using namespace std;
using namespace crpi_robot;

typedef CrpiUniversal robArmType;


struct globalHandle
{
	//! @brief Whether or not to continue running this thread
	//!
	bool runThread;

	//! @brief Path to the robot's CRPI configuration XML file
	string path;

	//! @brief Port to open for client connection
	//!
	int port;

	//! @brief Handle to use for ulapi_mutex
	//!
	ulapi_mutex_struct* handle;

	//! @brief Current robot status vars
	//!
	robotPose curPose;
	robotPose curForces;
	robotAxes curAxes = robotAxes(6);


	//! @brief Whether or not there is a new pose for the robot
	//! and what is it
	//!
	bool setPose;
	robotAxes newAxes = robotAxes(6);

	//! @brief Whether to toggle 0G mode
	//!
	bool toggle0G;
	bool freedriveStatus;

	//! @brief Constructor
	//!
	//! @param armptr Pointer to the robot object (for getting feedback from the robot)
	//!
	globalHandle()
	{
		runThread = false;
		path = string();

		handle = ulapi_mutex_new(19);

		curAxes = robotAxes(6);

		setPose = false;
		newAxes = robotAxes(6);

		toggle0G = false;
		freedriveStatus = false;
	}

	//! @brief Destructor
	//!
	~globalHandle()
	{
		/*ulapi_mutex_take(handle);
		delete(&curAxes);
		delete(&newPose);
		ulapi_mutex_give(handle);*/
	}
};


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




//! @brief Thread method for communicating with the HoloLens Demo
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void serverHLDemoHandlerThread(void* param)
{
	globalHandle* gH = (globalHandle*)param;
	//CrpiRobot<CrpiRobotiq> arm(gH->path.c_str());
	ulapi_integer server, client;
	bool clientConnected = false;

	crpi_timer timer;
	char buffer[2048] = "";
	// Clear buffer of previous data
	//memset(&buffer[0], 0, sizeof(buffer));
	string str;
	ulapi_integer rec, sent;

	/*
	// Vars to store robot info
	robotPose curPose;
	robotPose curForces;
	robotAxes curAxes = robotAxes(6);
	robotAxes targetAxes = robotAxes(6);*/

	// Dummy for comparison
	char cmds[][2048] = { "pick", "place", "gravcomp_on" , "gravcomp_off" };
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
			cout << "Remote Unity server connected..." << endl;
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


				// Pick Up
				if (strcmp(buffer, cmds[0]) == 0) {
					cout << ">Picking up object..." << endl;

					// Get info from Universal thread
					ulapi_mutex_take(gH->handle);
					// DO STUFF -----
					ulapi_mutex_give(gH->handle);


					// Format for printing & sending back
					sprintf(sendData, "Picked up object.");

					sent = ulapi_socket_write(client, sendData, strlen(sendData));
				}

				// Place
				else if (strcmp(buffer, cmds[1]) == 0) {
					cout << ">Placing object..." << endl;

					// Get info from Universal thread
					ulapi_mutex_take(gH->handle);
					// DO STUFF -----
					ulapi_mutex_give(gH->handle);

					// Format for printing & sending back
					sprintf(sendData, "Placed object.");

					sent = ulapi_socket_write(client, sendData, strlen(sendData));
				}

				// GravComp ON
				else if (strcmp(buffer, cmds[2]) == 0) {
					cout << ">Turning on gravity compensation..." << endl;


					// Get info from Universal thread
					ulapi_mutex_take(gH->handle);
					// DO STUFF -----
					ulapi_mutex_give(gH->handle);


					// Format for printing & sending back
					sprintf(sendData, "Turned on gravity compensation.");

					sent = ulapi_socket_write(client, sendData, strlen(sendData));
				}

				// GravComp OFF
				else if (strcmp(buffer, cmds[2]) == 0) {
					cout << ">Turning off gravity compensation..." << endl;


					// Get info from Universal thread
					ulapi_mutex_take(gH->handle);
					// DO STUFF -----
					ulapi_mutex_give(gH->handle);


					// Format for printing & sending back
					sprintf(sendData, "Turned off gravity compensation.");

					sent = ulapi_socket_write(client, sendData, strlen(sendData));
				}

				else {  // Echo

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



//! @brief Thread method for communicating with a Robotiq robot hand
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void armRobotiqHandlerThread(void* param)
{
	globalHandle* gH = (globalHandle*)param;
	CrpiRobot<CrpiRobotiq> arm(gH->path.c_str());
	ulapi_integer server, client;
	bool clientConnected = false;

	crpi_timer timer;
	char buffer[2048];
	string str;
	ulapi_integer rec, sent;

	//! Create socket connection
	server = ulapi_socket_get_server_id(gH->port);
	ulapi_socket_set_blocking(server);

	while (gH->runThread)
	{
		if (!clientConnected)
		{
			cout << "Running XML Interface on port " << gH->port << " for the Robotiq arm" << endl;
			client = ulapi_socket_get_connection_id(server);
			ulapi_socket_set_blocking(client);
			clientConnected = true;
			cout << "Remote RObotiq client connected..." << endl;
		}

		while (clientConnected && gH->runThread)
		{
			rec = ulapi_socket_read(client, buffer, 2048);
			if (rec > 0)
			{
				str = buffer;
				arm.CrpiXmlHandler(str);
				arm.CrpiXmlResponse(buffer);
				sent = ulapi_socket_write(client, buffer, strlen(buffer));
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


//! @brief Example client thread that sends simple up/down commands and gets feedback
//!
//! @param param Pointer to a globalHandle object containing runtime instructions
//!
void nodDemo(void* param)
{
	globalHandle* gH = (globalHandle*)param;
	ulapi_integer server, got;
	char inbuffer[REQUEST_MSG_SIZE];

	server = ulapi_socket_get_client_id(30012, "127.0.0.1");  //?
	if (server < 0)
	{
		cout << "could not connect sample client" << endl;
		return;
	}
	else
	{
		cout << "connected sample client" << endl;
	}

	sprintf(inbuffer, "<CRPICommand type=\"Couple\"><String Value = \"flange_ring\"/></CRPICommand>");
	ulapi_socket_write(server, inbuffer, strlen(inbuffer));
	Sleep(1000);

	bool on = true;
	while (true)
	{
		sprintf(inbuffer, "<CRPICommand type=\"SetRobotDO\"><Int Value = \"0\">/<Boolean Value = \"%s\"/></CRPICommand>", (on ? "true" : "false"));
		//cout << "sending " << inbuffer << endl;
		ulapi_socket_write(server, inbuffer, strlen(inbuffer));

		got = ulapi_socket_read(server, inbuffer, REQUEST_MSG_SIZE);
		inbuffer[got] = '\0';
		cout << got << " " << inbuffer << endl;
		Sleep(1000);
		on = !on;
	} // while (gH->runThread)

	gH = NULL;
	return;
}


//! @brief Main program method
//!
void main_old()
{
	ifstream infile("xmlsettings.dat");
	string robot, path;
	int port;
	vector<globalHandle> handles;
	vector<void*> armTasks;
	void* armtask;
	globalHandle handle;

#ifdef XMLINTERFACE_DEBUGTEST
	globalHandle demoHandle;
	void* demoTask;
#endif


	while (infile >> robot)
	{
		infile >> path >> port;

		//! Start the threads based on the information parsed from settings.dat
		cout << "Starting server for " << robot << " on port " << port << endl;


		if (robot == "ROBOTIQ")
		{
			ulapi_mutex_take(handle.handle);
			handle.runThread = true;
			handle.path = path;
			handle.port = port;

			armtask = ulapi_task_new();

			//! Start new Robotiq thread
			ulapi_task_start((ulapi_task_struct*)armtask, armRobotiqHandlerThread, &handle, ulapi_prio_lowest(), 0);
			handles.push_back(handle);
			armTasks.push_back(armtask);
			ulapi_mutex_give(handle.handle);
		}
		else if (robot == "UNIVERSAL")
		{
			ulapi_mutex_take(handle.handle);
			handle.runThread = true;
			handle.path = path;
			handle.port = port;

			armtask = ulapi_task_new();

			//! Start new Universal Robot thread
			ulapi_task_start((ulapi_task_struct*)armtask, armUniversalHandlerThread, &handle, ulapi_prio_lowest(), 0);
			handles.push_back(handle);
			armTasks.push_back(armtask);
			ulapi_mutex_give(handle.handle);
		}
		else if (robot == "UNITY")
		{
			ulapi_mutex_take(handle.handle);
			handle.runThread = true;
			handle.path = path;
			handle.port = port;

			armtask = ulapi_task_new();  //rename??

			//! Start new Unity thread
			ulapi_task_start((ulapi_task_struct*)armtask, serverUnityHandlerThread, &handle, ulapi_prio_lowest(), 0);
			handles.push_back(handle);
			armTasks.push_back(armtask);
			ulapi_mutex_give(handle.handle);
		}
		else if (robot == "HLDEMO")
		{
			ulapi_mutex_take(handle.handle);
			handle.runThread = true;
			handle.path = path;
			handle.port = port;

			armtask = ulapi_task_new();  //rename??

			//! Start new thread for demo
			ulapi_task_start((ulapi_task_struct*)armtask, serverHLDemoHandlerThread, &handle, ulapi_prio_lowest(), 0);
			handles.push_back(handle);
			armTasks.push_back(armtask);
			ulapi_mutex_give(handle.handle);
		}
		/*else if (robot == "UR10_LEFT")
		{
			ulapi_mutex_take(handle.handle);
			handle.runThread = true;
			handle.path = path;
			handle.port = port;

			armtask = ulapi_task_new();  //rename??

			//! Start new thread for
			ulapi_task_start((ulapi_task_struct*)armtask, serverHLDemoHandlerThread, &handle, ulapi_prio_lowest(), 0);
			handles.push_back(handle);
			armTasks.push_back(armtask);
			ulapi_mutex_give(handle.handle);
		}
		else if (robot == "UR10_RIGHT")
		{
			ulapi_mutex_take(handle.handle);
			handle.runThread = true;
			handle.path = path;
			handle.port = port;

			armtask = ulapi_task_new();  //rename??

			//! Start new thread for
			ulapi_task_start((ulapi_task_struct*)armtask, serverHLDemoHandlerThread, &handle, ulapi_prio_lowest(), 0);
			handles.push_back(handle);
			armTasks.push_back(armtask);
			ulapi_mutex_give(handle.handle);
		}*/
		else
		{
			cout << "Error in 'xmlsettings.dat'.  Unknown robot type: " << robot << endl;
		}
		Sleep(1000);
	} // while (infile >> robot)


#ifdef XMLINTERFACE_DEBUGTEST
	Sleep(5000);
	demoTask = ulapi_task_new();
	ulapi_task_start((ulapi_task_struct*)demoTask, nodDemo, &demoHandle, ulapi_prio_lowest(), 0);

#endif


	crpi_timer timer;
	while (true)
	{
		//! Put any additional logic here

		//! Throttle back evaluation of the main thread to keep from starving the other threads
		timer.waitUntil(500);
	} // while (true)

	vector<globalHandle>::iterator ghIter;
	vector<void*>::iterator taskIter;

	//! Garbage collection.  Stop the threads.
	for (ghIter = handles.begin(), taskIter = armTasks.begin(); ghIter != handles.end(); ++ghIter, ++taskIter)
	{
		ulapi_mutex_take(ghIter->handle);
		ghIter->runThread = false;
		timer.waitUntil(100);
		ulapi_task_stop((ulapi_task_struct*)*taskIter);
		ulapi_mutex_give(ghIter->handle);
	}
}
