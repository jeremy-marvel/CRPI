#ifndef CRPI_ANDROID_H
#define CRPI_ANDROID_H

#pragma once
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "ulapi.h"

using namespace crpi_robot;

class Server_CRPI {
public: 
	//Functions to be used as a means to perform the required actions. 
	Server_CRPI(); 
	//int start_CRPI_SRV(string input_IP_ADDR, string input_PORT);
	int start_CRPI_SRV();
	void start_CRPI_encoding();
	void send_crpi_msg(robotAxes unity_pose);
	robotAxes string_converter(string msg);
	void recieve_message_android();
	void recieve_message_vicon(); 
	void send_message_android(string message);
	void send_message_vicon(string message);
	void close_client_vicon();
	void close_client_android();
	void act_changer_unity(int changer);
	void port_scanner(string IP_name_in, int start_port, int end_port);
 
	//CRPI specific functions 
	void send_gripper_cmd(float vals);
	void send_DO_cmds(bool ary_in[4]); 
	
	//Server and Client TCP Settings
	WSADATA vicon_cli;
	WSADATA android_cli;

	//Connection socket
	SOCKET vicon_socket;
	SOCKET android_socket; 

	//A number that states the status of the connection
	int iResult_V;
	int iResult_A; 

	//A buffer that states how many bytes have been recieved 
	int recvbuflen_V;
	int recvbuflen_A; 

	//The following two strings are the incoming messages
	string action_TCP_Vicon;
	string action_TCP_Android; 

	//These state the IP and port for their respective uses
	string adr_IP_vicon;
	string port_vicon;
	string adr_IP_android;
	string port_android; 
	
	//This is a buffer that is used to send out strings to the servers
	const char *sendbuf_vicon; 
	const char *sendbuf_android;

	//This is a "manual override" to shut down CRPI 
	bool activate_shutdown = false; 

	//This is a timeout 
	int timeout_interval; 

	//Collects raw input values from the 
 	float digital_data_in[8]; //Adjust as needed
	bool do_cmd_list[4];
	float gripper_ratio; 
	int robot_id = 0, old_robot_id = 0,manual_robot_id = 0;

	//Additional blub for changing robots
	int override_robot_id; //Numbers correspond to a robot type
	int change_robots; 
	int old_gripper_status = 0; 

	//CRPI Settings
	robotAxes pose_msg;
	float open_grip;
	CrpiRobot <CrpiUniversal> *arm;
};

#endif