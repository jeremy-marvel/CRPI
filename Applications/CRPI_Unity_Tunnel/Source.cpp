///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Android CRPI Interaction
//  Subsystem:       Human-Robot Interaction
//  Workfile:        Android UI Tests
//  Revision:        1.0 6/7/19
//  Author:          Esteban Segarra Martinez
//
//  Description
//  ===========
//  Unity-TCP reader and CRPI Commander. This is a middleware integration program that takes in data from Unity being provided in a 
//	selected format. This format allows the user to take in angle data, robot IDs, gripper status, and control digital output data.
//  
//  In the future this client will attempt to send back to unity response data from CRPI modules being broadcast from the robot. 
//
//	Custom string unity provided by unity is phrased by this program as well. 
//
//	Quick Start instructions
//	============
//
//	Run the CRPI_Unity_tunnel.exe program. The program will start and ask for the amount of cycles you would like to run the 
//	program for. Each cycle has a 2 second delay and is nesseary to avoid overloading the UR5 controller with too many pose 
//	commands. The program also depends on having information being delivered at a constant rate greater than 1 second
//	in order to avoid overflowing the recv buffer (too much garbage data can collect). 
//
//	Updates to Vicom integration
//  =======
//	>  Due to popular request, Vicom integration is being modified to be optional, not a requirement for functionality. 
//  >  An adoption of a protocol for common requests for data to be sent over from different servers will be incorporated, removing the single-client
//     specification. 
//
//
///////////////////////////////////////////////////////////////////////////////
 
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ulapi.h"
#include "crpi_abb.h"
#include <string>
#include "CRPI_Unity_Tunnel.h"
#include "DataStreamClient.h"
#include <crtdbg.h>
#include <thread>
#include <mutex>
#include <process.h>

/////////////////////////////////////////////////////////////////////////////////

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
#define DEFAULT_BUFLEN 256
#define DEFAULT_PORT "27000"
 
using namespace std;
using namespace crpi_robot;

struct addrinfo *result = NULL,	*ptr = NULL, hints;
struct addrinfo *result_2 = NULL, *ptr_2 = NULL, hints_2;

//Set this bit for debugging with or without CRPI 
const int SHUTOFF_CRPI = 0; 

//Set this bit for enabling or disabling the functionality of Vicom
const int DISABLE_VICOM = 1; 

/////////////////////////////////////////////////////////////////////////////////
//Dummy constructor 
Server_CRPI::Server_CRPI() {
}

//("169.254.152.27", "27000")
//int Server_CRPI::start_CRPI_SRV(string input_IP_ADDR, string input_PORT) {
//General setup and running of the TCP server connection 
int Server_CRPI::start_CRPI_SRV() {
    adr_IP_android = "169.254.152.27";
	port_android   = "27000";
	adr_IP_vicon   = "127.0.0.1";
	port_vicon     = "27001";
	timeout_interval = 0;

	//Welcome to the Unity 
	cout << "=======================================================================" << endl;
	cout << "Welcome to the Unity -> CRPI Controller with Vicom support" << endl;
	cout << "The program will run automatically when the Unity app is in the local " << endl;
	cout << "network. Optionally the Vicon TCP server can be active and be " << endl;
	cout << "transmitting and sync to send signals to CRPI." << endl;
	cout << "To enable the Vicon support, set DISABLE_VICOM flag to 0" << endl;
	cout << "=======================================================================" << endl;

	//Setup proceedures	
	vicon_socket = android_socket = INVALID_SOCKET;
	recvbuflen_V = recvbuflen_A = DEFAULT_BUFLEN;
	sendbuf_vicon = sendbuf_android = "this is a test";
	cout << "Starting Connection." << endl;

	// Initialize Winsock
	iResult_A = WSAStartup(MAKEWORD(2, 2), &android_cli);


	if (DISABLE_VICOM != 1)
		iResult_V = WSAStartup(MAKEWORD(2, 2), &vicon_cli);

	

	//Check the startup proceedures for the sockets 
	if (iResult_A != 0) {
		printf("WSAStartup failed with error: %d\n", iResult_A);
		return 1;
	}
	
	if (iResult_V != 0 && DISABLE_VICOM != 1) {
		printf("WSAStartup failed with error: %d\n", iResult_V);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	if (DISABLE_VICOM != 1) {
		ZeroMemory(&hints_2, sizeof(hints_2));
		hints_2.ai_family = AF_INET;
		hints_2.ai_socktype = SOCK_STREAM;
		hints_2.ai_protocol = IPPROTO_TCP;
	}
	// Resolve the server address and port
	iResult_A = getaddrinfo(adr_IP_android.c_str(), port_android.c_str(), &hints, &result);

	if (DISABLE_VICOM != 1)
		iResult_V = getaddrinfo(adr_IP_vicon.c_str(), port_vicon.c_str(), &hints_2, &result_2);

	//Check if the addresses are valid ports/IPs 
	if (iResult_A != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult_A);
		WSACleanup();
		return 2;
	}

	if (iResult_V != 0 && DISABLE_VICOM != 1) {
		printf("getaddrinfo failed with error: %d\n", iResult_V);
		WSACleanup();
		return 2;
	}


	cout << "Starting client Android" << endl;
	///////////////////////////////////////////////////////////////////////////////
	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		android_socket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
		if (android_socket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 3;
		}

		// Connect to server.
		iResult_A = connect(android_socket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult_A == SOCKET_ERROR) {
			closesocket(android_socket);
			android_socket = INVALID_SOCKET;
			continue;
		}
		break;
	}	

	if (android_socket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 4;
	}
	freeaddrinfo(result);

	if (DISABLE_VICOM != 1) {
		cout << "Starting client Vicon" << endl;
		/////////////////////////////////////////////////////////////////////////////////
		for (ptr_2 = result_2; ptr_2 != NULL; ptr_2 = ptr_2->ai_next) {
			// Create a SOCKET for connecting to server
			vicon_socket = socket(ptr_2->ai_family, ptr_2->ai_socktype, ptr_2->ai_protocol);
			if (vicon_socket == INVALID_SOCKET) {
				printf("socket failed with error: %ld\n", WSAGetLastError());
				WSACleanup();
				return 3;
			}

			// Connect to server.
			iResult_V = connect(vicon_socket, ptr_2->ai_addr, (int)ptr_2->ai_addrlen);
			if (iResult_V == SOCKET_ERROR) {
				closesocket(vicon_socket);
				vicon_socket = INVALID_SOCKET;
				continue;
			}
			break;
		}

		if (vicon_socket == INVALID_SOCKET) {
			printf("Unable to connect to server!\n");
			WSACleanup();
			return 4;
		}
		freeaddrinfo(result_2);
	}
	///////////////////////////////////////////////////////////////////////////////////

	cout << "Initialization Done. Starting Client reception..." << endl;
	cout << endl; 

	// Receive until the peer closes the connection
	//Loop for the reception of messages
	do {
		cout << endl;
		cout << "Android Bracket" << endl;
		recieve_message_android();
		cout << endl; 
		
		if (DISABLE_VICOM != 1) {
			cout << "Vicon Bracket" << endl;
			recieve_message_vicon();
		}

		//Send a CRPI Message given the correct string
		if (SHUTOFF_CRPI == 0 && action_TCP_Android.length() > 10) {
			pose_msg = string_converter(action_TCP_Android); //Interpret a robot_pose 

			//Selection of the robot based on user option (Big change robot button on UI) 
			if (override_robot_id == 1) {
				if (change_robots == 1) {
					robot_id = manual_robot_id;
				}
			}
			else if (change_robots == 1 && DISABLE_VICOM != 1) {
				action_TCP_Vicon.erase(0, 1);
				action_TCP_Vicon.erase(1, 1);
				robot_id = stoi(action_TCP_Vicon);
			}

			//robot_id states the robot ID that will be used to swap between Vicom and Override. 
			cout << robot_id << endl;
			if (old_robot_id != robot_id && robot_id != 0) { //Changer for robot IDs in Unity
				act_changer_unity(robot_id);
				start_CRPI_encoding();
				old_robot_id = robot_id;
			}
			if (robot_id != 0) //Precautionary action in case we fail to get any robot ID
				send_crpi_msg(pose_msg);
		}

		//Report to the Android tablet the status of which robot is being looked at by the user. 
		send_message_android(to_string(robot_id)); 

		//Timeout_interval refers to the amount of loss of messages dropped so far to justify closing the connection.  
		if (timeout_interval == 10) {
			activate_shutdown = true;
		}
		else {
			timeout_interval = 0; 
		}


	} while (activate_shutdown == false);
	
	cout << "=======================================================================" << endl;
	close_client_android();
	cout << "=======================================================================" << endl;
	if (DISABLE_VICOM != 1) {
		close_client_vicon();
		cout << "=======================================================================" << endl;
	}
	return 0; 
}

//TPC Reception message function
void Server_CRPI::recieve_message_android() {

	//Highly sensitive stuff
	char recvbuf[DEFAULT_BUFLEN];
	action_TCP_Android = "";
	iResult_A = recv(android_socket, recvbuf, 256, 0);

	if (iResult_A > 0) {
		printf("Bytes received: %d\n", iResult_A);
	}
	else if (iResult_A == 0)
		printf("Connection closed\n");
	else {
		printf("recv failed with error: %d\n", WSAGetLastError());
		activate_shutdown = true; 
	}

	//Bytes recieved - Phrase TPC input
	bool start_of_string = false; 

	for (int i = 0; i < iResult_A; i++) {
		if (recvbuf[i] == '#')
			break;
		else if (recvbuf[i] == '$')
			start_of_string = true;
		if (start_of_string)
			action_TCP_Android += recvbuf[i];
	}

	cout << "Recieved message: " << action_TCP_Android << endl;

	//Put the brakes on TPC client
	for (int x = 0; x < 256; x++)
		recvbuf[x] = '\0';
}

void Server_CRPI::recieve_message_vicon() {
	//Highly sensitive stuff
	char recvbuf[DEFAULT_BUFLEN];//Buffer
	action_TCP_Vicon = "";
	iResult_V = recv(vicon_socket, recvbuf, 256, 0);

	if (iResult_V > 0) {
		printf("Bytes received: %d\n", iResult_V);
	}
	else if (iResult_V == 0)
		printf("Connection closed\n");
	else {
		printf("recv failed with error: %d\n", WSAGetLastError());
		activate_shutdown = true;
	}

	bool start_of_string = false;
	//Bytes recieved - Phrase TPC input
	for (int i = 0; i < iResult_V; i++) {
		if (recvbuf[i] == '#')
			break;
		else if (recvbuf[i] == '$')
			start_of_string = true;
		if (start_of_string)
			action_TCP_Vicon += recvbuf[i];
	}

	cout << "Recieved message: " << action_TCP_Vicon << endl;

 
	//Put the brakes on TPC client
	for (int x = 0; x < 256; x++)
		recvbuf[x] = '\0';
}

//Start the CRPI connection 
void Server_CRPI::start_CRPI_encoding() {
	cout << "Starting Robot..." << endl;
	arm->SetAngleUnits("degree");
	arm->SetLengthUnits("mm");
	arm->SetRelativeSpeed(0.1);
	arm->Couple("griper_parallel");
	cout << "Done." << endl;

	//Double Comparison check constant
	const double TOLERANCE = 0.05;
}

//Function to determine and move UR5 to a specified location
void Server_CRPI::send_crpi_msg(robotAxes unity_pose) {
	robotAxes address;
	arm->GetRobotAxes(&address);
	address.print();
	cout << endl;
	unity_pose.print();

	//Avoid sleep statements here - Will Unbalance TPC client and fill with garbage. 
	//Read data_in. Conditional to test pose accuracy/send msg. 
	if (arm->MoveToAxisTarget(unity_pose) == CANON_SUCCESS)
	{
		//address.print();
		cout << endl << endl;
		cout << "Success" << endl;
	}
	else
	{
		cout << "Failure" << endl;
	}
	//send_gripper_cmd(gripper_ratio);
	send_DO_cmds(do_cmd_list);
	cout << "Pose movement completed." << endl;
}

//Custom string phraser from incoming message from Unity
//I'm not well aware of standards on messages, so I made my own. 
robotAxes Server_CRPI::string_converter(string msg) {
	//  UR5_pos:-42.58, -43.69, -99.57, 233.2, -89.66, -47.09;Gripper:0; 
	// array_of_pos[6] = {x,y,z,xrot,yrot,zrot};
	// Digital_data_in = {Robot_ID, gripper, DO_1, DO_2, etc };

	int DEBUG_LOG = 0; 


	//UR5_pos:100.9601,-80.56218,81.42348,89.13869,89.99997,-126.8599;Robot Utilities:0,0,0,0,0,0;

	// {rot0,rot1,rot2,rot3,rot4,rot5}
	float array_of_pos[6];
	float gripper;
	float robot_util_array[8];
	robotAxes unity_pose;
	bool action_cmd = false; 
	string temp_msg;
	int ary_count = 0;
	bool chk_DO = false;
	int temp_block = 0;
	if (msg.length() > 1) {
		//Categorized phraser for the string input by unity.
		for (int i = 0; i < msg.length(); i++) {
			if (msg[i] == ':') {
				i++;
				//cout << endl;
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
				temp_block = i;
				break;
			}
		}
		ary_count = 0;
		for (int i = temp_block; i < msg.length(); i++) {
			if (msg[i] == ':') {
				i++;
				//cout << endl;
				while (true) {
					if (msg[i] == ',') {
						robot_util_array[ary_count] = strtof((temp_msg).c_str(), 0);
						ary_count++;
						temp_msg = "";
					}
					if (msg[i] == ';') {
						robot_util_array[ary_count] = strtof((temp_msg).c_str(), 0);
						temp_msg = "";
						break;
					}
					else {
						if (msg[i] != ',') {
							temp_msg += msg[i];
						}
					}
					i++;
				}
				break;
			}
		}

		
		//Phrase inbound angles into the robotAxes 
		
		for (int i = 0; i < 6; i++) {
			if (DEBUG_LOG == 1) {
				cout << "Revieved angles: " << endl;
				cout << array_of_pos[i] << endl;
			}
			unity_pose.axis[i] = array_of_pos[i];
		}
		if (DEBUG_LOG == 1) {
			for (int i = 0; i < 6; i++) {
				cout << "Recieved robot utils " << endl;
				cout << robot_util_array[i] << endl;
			}
		}
		manual_robot_id = robot_util_array[0];
		gripper_ratio = robot_util_array[1];
		do_cmd_list[0] = robot_util_array[2];
		do_cmd_list[1] = robot_util_array[3];
		do_cmd_list[2] = robot_util_array[4];
		do_cmd_list[3] = robot_util_array[5];
		override_robot_id = robot_util_array[6];
		change_robots     = robot_util_array[7]; 
		
	}
	else {
		timeout_interval += 1; 
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

void Server_CRPI::send_gripper_cmd(float vals) {
	//Avoid using gripper at the moment - Requires most likely some type of threading in order to use appropiately.
	//Otherwise it will freeze the whole robot from doing anything. 

	if (vals != old_gripper_status) {
		
		if (vals == 0) {
			cout << "Not Gripped" << endl;
		}
		else {
			cout << "Gripped" << endl;
		}

		Sleep(250);
		arm->SetRobotDO(0, (int)vals);
	}
	
	
}

//Export a digital output value out to the robot
void Server_CRPI::send_DO_cmds(bool ary_in[4]) {
	Sleep(250);
	arm->SetRobotDO(0, gripper_ratio);
	Sleep(250);

	arm->SetRobotDO(9, ary_in[1]);
	Sleep(250);

}

//Sends a message to the connected TCP server
void Server_CRPI::send_message_android(string message) {
	// Send an initial buffer.
	sendbuf_android = message.c_str(); 
	iResult_A = send(android_socket, sendbuf_android, (int)strlen(sendbuf_android), 0);
	if (iResult_A == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(android_socket);
		WSACleanup();
 	}
	printf("Bytes Sent: %ld\n", iResult_A);
}

void Server_CRPI::send_message_vicon(string message) {
	// Send an initial buffer
	iResult_V = send(vicon_socket, sendbuf_vicon, (int)strlen(sendbuf_vicon), 0);
	if (iResult_V == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(vicon_socket);
		WSACleanup();
	}
	printf("Bytes Sent: %ld\n", iResult_V);
}

//Closes the TCP server client to cleanly exit and liberate the port
void Server_CRPI::close_client_android() {
	iResult_A = shutdown(android_socket, SD_SEND);
	if (iResult_A == SOCKET_ERROR) {
		printf("Shutdown Failure");
	}

	// cleanup
	closesocket(android_socket);
	WSACleanup();
	cout << "Closing Android Client" << endl;
	//return 0;
}
void Server_CRPI::close_client_vicon() {
	iResult_V = shutdown(vicon_socket, SD_SEND);
	if (iResult_V == SOCKET_ERROR) {
		printf("Shutdown Failure");
	}

	// cleanup
	closesocket(vicon_socket);
	WSACleanup();
	cout << "Closing Vicon Client" << endl;
	//return 0;
}


//Depending on the input provided, the server will switch from 
//different robot arms using this function
void Server_CRPI::act_changer_unity(int changer){
 	  
	if (changer == 1) {
		printf("starting UR5");
		arm = new CrpiRobot<CrpiUniversal>("universal_ur5.xml");
		printf("started UR5");
	}
	else if (changer == 2) {
		arm = new CrpiRobot<CrpiUniversal>("universal_ur10_left.xml");
	}
	else if (changer == 3) {
		arm = new CrpiRobot<CrpiUniversal>("universal_ur10_right.xml");
	}
	else if (changer == 4) {
		arm = new CrpiRobot<CrpiUniversal>("abb_irb14000_left.xml");
 	}
	else if (changer == 5) {
		arm = new CrpiRobot<CrpiUniversal>("abb_irb14000_right.xml");
	}
	else {
		printf("No suitable robot at the moment");
		return; 
	}
}


///////////////////////////////////////////////////////////////////////////////////////
int __cdecl main(int argc, char **argv) {
	Server_CRPI server; 
	server.start_CRPI_SRV();
	cout << "Closing main program." << endl;
	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////
 