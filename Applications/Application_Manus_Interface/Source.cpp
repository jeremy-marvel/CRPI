///////////////////////////////////////////////////////////////////////////////
//
//  Original System: source.cpp (Would be more: UR5_Unity_control_manus_vr.cpp) 
//  Subsystem:       Human-Robot Interaction
//  Workfile:        Manus_interpreter.cs
//  Revision:        1.0 7/17/2018
//  Author:          Esteban Segarra
//
//  Description
//  ===========
//  Integrated TPC client for communicating between Unity and UR5 standalone using positional coordinates based on
//	conversions between unity and realworld calculations. Scaling and rotation applied. 
//
//	Custom string unity provided by unity is phrased by this program as well. 
//
//	Quick Start instructions
//	============
//
//	Run the Manus_interface.exe program. The program will start and ask for the amount of cycles you would like to run the 
//	program for. Each cycle has a 2 second delay and is nesseary to avoid overloading the UR5 controller with too many pose 
//	commands. The program also depends on having information being delivered at a constant rate greater than 1 second
//	in order to avoid overflowing the recv buffer (too much garbage data can collect). 
///////////////////////////////////////////////////////////////////////////////


//#ifndef WIN32_LEAN_AND_MEAN
//#define WIN32_LEAN_AND_MEAN
//#endif

#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <stdio.h>
#include <conio.h>

#include<stdlib.h>
#include<iostream>
#include<time.h>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include <winsock.h>
#include <string>


// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

#define DEFAULT_BUFLEN 256

#define DEFAULT_PORT "27000"


using namespace std;
using namespace crpi_robot;



WSADATA wsaData;
SOCKET ConnectSocket = INVALID_SOCKET;
struct addrinfo *result = NULL,
	*ptr = NULL,
	hints;
const char *sendbuf = "this is a test";
int iResult;
int recvbuflen = DEFAULT_BUFLEN;

//TCP Message from unity acting as buffer
string action_string_TCP;
//CRPI Robot UR5
CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");
robotAxes pose_msg;
float open_grip;

//Start the CRPI connection 
void start() {
	cout << "Starting Robot..." << endl;
	arm.SetAngleUnits("degree");
	arm.SetLengthUnits("mm");
	arm.SetRelativeSpeed(0.1);
	arm.Couple("griper_parallel");
	cout << "Done." << endl;
}

//Double Comparison check constant
const double TOLERANCE = 0.05;

//Function to determine and move UR5 to a specified location
void send_crpi_msg(robotAxes unity_pose) {
	robotAxes address;
	arm.GetRobotAxes(&address);

	//Avoid sleep statements here - Will Unbalance TPC client and fill with garbage. 
	//Read data_in. Conditional to test pose accuracy/send msg. 
	if (arm.MoveToAxisTarget(unity_pose) == CANON_SUCCESS)
	{
		//address.print();
		cout << endl << endl;
		cout << "Success" << endl;
	}
	else
	{
		cout << "Failure" << endl;
	}

	
	//Avoid using gripper at the moment - Requires most likely some type of threading in order to use appropiately.
	//Otherwise it will freeze the whole robot from doing anything. 

	//if (open_grip >= 0.5F ) {
	//	if (arm.SetRobotDO(0, 1) == CANON_SUCCESS) {
	//		cout << "Gripped" << endl;
	//	}
	//}
	//else {
	//	if (arm.SetRobotDO(0, 0) == CANON_SUCCESS) {
	//		cout << "Un-Gripped" << endl;
	//	}//Open the gripper on UR
	//}


	cout << "Pose movement completed." << endl;
}



//Custom string phraser from incoming message from Unity
//I'm not well aware of standards on messages, so I made my own. 
//Apologies if this is gibberish :(

robotAxes string_converter(string msg) {
	//  UR5_pos:-42.58, -43.69, -99.57, 233.2, -89.66, -47.09;Gripper:0; 
	// array_of_pos[6] = {x,y,z,xrot,yrot,zrot};

	// {rot0,rot1,rot2,rot3,rot4,rot5}
	double array_of_pos[6], gripper;
	robotAxes unity_pose = robotAxes(6);
	string temp_msg;
	int ary_count = 0;
	bool chk_grip = false; 

	if (msg.length() > 1) {

		//Categorized phraser for the string input by unity.
		for (int i = 0; i < msg.length(); i++) {
			if (msg[i] == '>') {
				i++;
				cout << endl;
				while (true) {
					if (msg[i] == ',') {
						array_of_pos[ary_count] = strtof((temp_msg).c_str(), 0);
						ary_count++;
						temp_msg = "";
					}
					else if (msg[i] == ';') {
						array_of_pos[ary_count] = strtof((temp_msg).c_str(), 0);
						temp_msg = "";
						ary_count++;
						break;
					}
					else {
						temp_msg += msg[i];
					}
					i++;
				}
			}
			if (msg[i] == ':') {
				i++;
				while (true) {
					if (msg[i] == ';') {
						open_grip = strtof((temp_msg).c_str(), 0);
						chk_grip = true;
						temp_msg = "";
						break;
					}
					else {
						temp_msg += msg[i];
					}
					i++;
				}
			}


			//If the array is already full, break; 
			if (ary_count >= 6 && chk_grip) {
				break;
			}
		}

		//Phrase inbound angles into the robotAxes 
		for (int i = 0; i < 6; i++) {
			cout << "Revieved angles: " << endl;
			cout << array_of_pos[i] << ", " << endl;
			unity_pose.axis[i] = array_of_pos[i];
		}
	}
	else {
		cout << "Message is null." << endl;
	}


	//Assign phrase final pose position. 
	////unity_pose.x	= array_of_pos[0];
	////unity_pose.xrot = array_of_pos[1];
	////unity_pose.y	= array_of_pos[2];
	////unity_pose.yrot = array_of_pos[3];
	////unity_pose.z	= array_of_pos[4];
	////unity_pose.zrot = array_of_pos[5];

	return unity_pose;
}

//TPC Reception message function
void recieve_message() {
	int close_sess = 0,cycle_counter = 0 , CYCLE_RUNS = 10;
	cout << "Enter amount of cycles the program should run: " << endl;
	cin >> CYCLE_RUNS;
	do {

		//Highly sensitive stuff
		char recvbuf[DEFAULT_BUFLEN];
		action_string_TCP = "";
		iResult = recv(ConnectSocket, recvbuf, 256, 0);

		if (iResult > 0) {
			printf("Bytes received: %d\n", iResult);
		}
		else if (iResult == 0)
			printf("Connection closed\n");
		else
			printf("recv failed with error: %d\n", WSAGetLastError());


		//Bytes recieved - Phrase TPC input
		for (int i = 0; i < iResult; i++) {
			action_string_TCP += recvbuf[i];
			//cout << recvbuf[i];
		}

		cout << "Recieved message: " << action_string_TCP <<  endl;


		//Send a CRPI Message given the correct string
		pose_msg = string_converter(action_string_TCP);
		send_crpi_msg(pose_msg);

		//Put the brakes on TPC client
		for (int x = 0; x < 256; x++)
			recvbuf[x] = '\0';
 
		Sleep(500);
		cycle_counter++;
		cout << "Cycle " << cycle_counter << endl;
		cout << endl;

	} while (iResult > 0 && CYCLE_RUNS >= cycle_counter);
}


void send_message(string message) {
	// Send an initial buffer
	iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf), 0);
	if (iResult == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		//--return 1;
	}
	printf("Bytes Sent: %ld\n", iResult);
}

void close_client() {
	iResult = shutdown(ConnectSocket, SD_SEND);
	if (iResult == SOCKET_ERROR) {
		printf("Shutdown Failure");
	}



	// cleanup
	closesocket(ConnectSocket);
	WSACleanup();
	cout << "Closing Client" << endl;
	//return 0;
}


/*
DO NOT REBUILD SOLUTION - WILL CAUSE PAIN AND SUFFERING

Right click project and build exclusively. Otherwise you will have to rebuild CRPI.h and it

*/
int __cdecl main(int argc, char **argv)
{
	cout << "Starting Connection." << endl;
	//Start the CRPI Comms
	start();


	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("127.0.0.1", DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 2;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 3;
		}

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 4;
	}

	cout << "Initialization Done. Starting Client reception..." << endl;
	recieve_message();


	// Receive until the peer closes the connection
	close_client();




}
