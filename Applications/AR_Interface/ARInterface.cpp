///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       AR Interface 
//  Revision:        August 23, 2025
//  Authors:         Medhavi Kamran, Shelly Bagchi
//
//  Description
//  ===========
//  AR interface for HoloLens 2 connection with CRPI.  Requires Unity 3D 
//  with <Sphere_record_v2> program running on HL2 or on PC.
///////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <thread>
#include <winsock2.h>
#include <windows.h>
#include "crpi_robot.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "crpi_abb.h"
#include "ulapi.h"
#include "NumericalMath.h"
#include <ws2tcpip.h>
#include <stdio.h>
#include <vector>
#include <conio.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#pragma comment(lib, "ws2_32.lib")
#pragma warning(disable : 4996)

using namespace std;
using namespace crpi_robot;

char buffer[512];
int bytesReceived;


int main()
{
    int i = 0;
    SOCKET serverSocket;


    cout << "crpi okay" << endl;
    cout << "Creating Robot..." << endl;
    CrpiRobot<CrpiUniversal> arm("universal_ur3e_testbed.xml");
    arm.SetAngleUnits("degree");
    arm.SetLengthUnits("mm");

    robotIO curIO, tarIO;
    robotPose curPose;
    robotAxes curAxes;
    arm.GetRobotPose(&curPose);
    arm.GetRobotAxes(&curAxes);

    cout << "Robot Pose (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", "
        << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;
    cout << "Robot Joints (" << curAxes.axis.at(0) << ", " << curAxes.axis.at(1) << ", "
        << curAxes.axis.at(2) << ", " << curAxes.axis.at(3) << ", " << curAxes.axis.at(4) << ", "
        << curAxes.axis.at(5) << ", " << curAxes.axis.at(6) << ")" << endl;

    //cout << "Enter 1 to run tests (robot will begin moving)" << endl;

    cout << "[DEBUG] Starting TCP Server..." << endl;

    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        cout << "[ERROR] WSAStartup failed: " << iResult << endl;
        return 1;
    }
    cout << "[DEBUG] Winsock initialized." << endl;

    // Step 2: Create a socket
    serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverSocket == INVALID_SOCKET) {
        cout << "[ERROR] Socket creation failed: " << WSAGetLastError() << endl;
        WSACleanup();
        return 1;
    }
    cout << "[DEBUG] Socket created successfully." << endl;

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr("169.254.152.43"); // Replace with your IP if needed
    serverAddr.sin_port = htons(80);

    cout << "[DEBUG] Attempting to bind socket to 169.254.152.43" << endl;
    if (bind(serverSocket, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        cout << "[ERROR] Bind failed with errors: " << WSAGetLastError() << endl;
        WSACleanup();
        return 1;
    }
    cout << "[DEBUG] Socket bound successfully." << endl;

    cout << "[DEBUG] Starting to listen on socket..." << endl;
    if (listen(serverSocket, SOMAXCONN) == SOCKET_ERROR) {
        cout << "[ERROR] Listen failed: " << WSAGetLastError() << endl;
        WSACleanup();
        return 1;
    }
    cout << "[DEBUG] Listening for connections." << endl;

    cout << "[DEBUG] Waiting for client to connect..." << endl;

    // Step 5: Accept a client socket
    SOCKADDR_IN clientAddr;
    int clientSize = sizeof(clientAddr);
    SOCKET clientSocket = accept(serverSocket, (SOCKADDR*)&clientAddr, &clientSize);
    if (clientSocket == INVALID_SOCKET) {
        cout << "[ERROR] Accept failed: " << WSAGetLastError() << endl;
        WSACleanup();
        return 1;
    }
    std::string clientIP = inet_ntoa(clientAddr.sin_addr);
    cout << "[DEBUG] Client connected from IP: " << clientIP << endl;

    // Add your HoloLens IP here
    if (clientIP == "169.254.152.45") {  // Example HoloLens IP; port for hololens: 20602
        cout << "[INFO] HoloLens connection detected." << endl;

    }
    else {
        cout << "[INFO] Unity PC or other client connected." << endl;
    }
    // else 
       //  cout << "[DEBUG] Unity connected from IP: " << inet_ntoa(clientAddr.sin_addr) << endl;

         // Marker storage variables
    std::vector<std::pair<std::string, int>> marker_points; // pair of pose string and grip_status
    std::vector<std::pair<std::string, int>> removed_markers;
    int marker_indices = 0;

    // Initialize  robot 
    CrpiRobot<CrpiUniversal> urRobot("universal_ur5.xml");
    urRobot.SetAngleUnits("degree");
    urRobot.SetLengthUnits("mm");

    // Just hold connection and process messages
    int grip_status;
    while (true)
    {
        // Receive data from Unity
        bytesReceived = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        if (bytesReceived > 0)
        {
            buffer[bytesReceived] = '\0'; // Null-terminate the received data
            string msg(buffer);

            // Remove newline and carriage return characters
            msg.erase(std::remove(msg.begin(), msg.end(), '\n'), msg.end());
            msg.erase(std::remove(msg.begin(), msg.end(), '\r'), msg.end());

            // Get current robot pose ONCE
            robotPose pose;
            urRobot.GetRobotPose(&pose);

            // Convert RPY rotation vector from degrees to radians
            double rxRad = pose.xrot * M_PI / 180.0;
            double ryRad = pose.yrot * M_PI / 180.0;
            double rzRad = pose.zrot * M_PI / 180.0;

            // Compute quaternion from RPY (ZYX convention)
            double qw, qx, qy, qz;
            double mag = sqrt(rxRad * rxRad + ryRad * ryRad + rzRad * rzRad);

            if (mag != 0) {
                double cy = cos(rzRad * 0.5);
                double sy = sin(rzRad * 0.5);
                double cp = cos(ryRad * 0.5);
                double sp = sin(ryRad * 0.5);
                double cr = cos(rxRad * 0.5);
                double sr = sin(rxRad * 0.5);

                qw = cr * cp * cy + sr * sp * sy;
                qx = sr * cp * cy - cr * sp * sy;
                qy = cr * sp * cy + sr * cp * sy;
                qz = cr * cp * sy - sr * sp * cy;
            }


            else {
                // Identity quaternion (no rotation)
                qw = 1.0;
                qx = qy = qz = 0.0;
            }

            if (msg == "record") {

                cout << "[DEBUG] Record command received from Unity." << endl;
                // Simulate getting current pose and grip status
                // Replace this with actual pose & grip status from your robot/system

               // Compose pose string: "x,y,z,qw,qx,qy,qz"
                std::ostringstream oss;
                oss << pose.x / 1000 << "," << pose.y / 1000 << "," << pose.z / 1000 << "," << qw << "," << qx << "," << qy << "," << qz;
                string CurrPoseStr = oss.str();

                int grip_status = 1;  // example grip status (1 = closed, 0 = open)

                marker_points.push_back(std::make_pair(CurrPoseStr, grip_status));
                marker_indices++;

                // Send back the pose string to Unity (like writeline in MATLAB)
                std::string response = CurrPoseStr + "\n";
                send(clientSocket, response.c_str(), (int)response.length(), 0);

                cout << "[DEBUG] Recorded marker #" << marker_indices << ": " << CurrPoseStr << ", grip: " << grip_status << endl;


            }

            else if (msg == "remove") {
                if (marker_indices > 0) {
                    // Remove last marker and store it in removed_markers
                    std::pair<std::string, int> last_marker = marker_points.back();
                    removed_markers.push_back(last_marker);
                    marker_points.pop_back();
                    marker_indices--;

                    std::string response = "removed\n";
                    send(clientSocket, response.c_str(), (int)response.length(), 0);

                    cout << "[DEBUG] Removed last marker. Remaining markers: " << marker_indices << endl;

                }
                else {
                    std::string response = "no_markers\n";
                    send(clientSocket, response.c_str(), (int)response.length(), 0);
                    cout << "[DEBUG] No markers to remove." << endl;
                }

            }
            else if (msg == "restore") {
                if (!removed_markers.empty()) {
                    std::pair<std::string, int> last_removed_marker = removed_markers.back();
                    removed_markers.pop_back();

                    marker_points.push_back(last_removed_marker);
                    marker_indices++;

                    std::string response = "restored\n";
                    send(clientSocket, response.c_str(), (int)response.length(), 0);

                    cout << "[DEBUG] Restored last removed marker. Total markers: " << marker_indices << endl;
                }
                else {
                    std::string response = "no_last_removed_marker\n";
                    send(clientSocket, response.c_str(), (int)response.length(), 0);
                    cout << "[DEBUG] No removed markers to restore." << endl;
                }
            }
            else if (msg == "grip_open") {
                cout << "[DEBUG] Grip open command received." << endl;

                const int A_Gripper = 0;  // Set the correct pin for your robot gripper digital output

                // Send command to open the gripper
                urRobot.SetRobotDO(A_Gripper, false);  // false = OFF = open gripper
                const int gripper_status = 0;  // Update grip status to open

                // Record current pose + grip status as marker_points like in "record"
                std::ostringstream oss;
                oss << pose.x << "," << pose.y << "," << pose.z << "," << qw << "," << qx << "," << qy << "," << qz;
                string CurrPoseStr = oss.str();

                marker_points.push_back(std::make_pair(CurrPoseStr, gripper_status));
                marker_indices++;

                // Send back confirmation / pose string to Unity
                std::string response = CurrPoseStr + "\n";
                send(clientSocket, response.c_str(), (int)response.length(), 0);

                cout << "[DEBUG] Gripper opened and marker recorded: " << CurrPoseStr << endl;
            }
            else if (msg == "grip_close") {
                cout << "[DEBUG] Grip close command received." << endl;

                const int A_Gripper = 0;  // Set the correct pin for your robot gripper digital output

                // Send command to close the gripper
                urRobot.SetRobotDO(A_Gripper, true);  // true = ON = close gripper
                const int gripper_status = 1;  // Update grip status to closed

                // Record current pose + grip status as marker_points like in "record"
                std::ostringstream oss;
                oss << pose.x << "," << pose.y << "," << pose.z << "," << qw << "," << qx << "," << qy << "," << qz;
                string CurrPoseStr = oss.str();

                marker_points.push_back(std::make_pair(CurrPoseStr, gripper_status));
                marker_indices++;

                // Send back confirmation / pose string to Unity
                std::string response = CurrPoseStr + "\n";
                send(clientSocket, response.c_str(), (int)response.length(), 0);

                cout << "[DEBUG] Gripper closed and marker recorded: " << CurrPoseStr << endl;
            }


            else {
                // Default echo back, add newline for Unity ReadLine()
                msg += "\n";
                int bytesSent = send(clientSocket, msg.c_str(), (int)msg.length(), 0);
                if (bytesSent == SOCKET_ERROR) {
                    cout << "[ERROR] Send failed: " << WSAGetLastError() << endl;
                    break;
                }
                cout << "[DEBUG] Echoed back to Unity: " << msg;
            }

        }
        else if (bytesReceived == 0)
        {
            cout << "[INFO] Connection closing by Unity..." << endl;
            break;
        }
        else
        {
            cout << "[ERROR] recv failed: " << WSAGetLastError() << endl;
            break;
        }

        Sleep(100); // small delay to avoid busy loop

    }

    // Keep process alive so connection doesn't close immediately
    cout << "[DEBUG] Server is now holding connection open. Press Enter to exit." << endl;
    cin.get();

    // Cleanup (optional, can enable after testing)
    //shutdown(clientSocket, SD_SEND);
    //closesocket(clientSocket);
    //closesocket(serverSocket);
    //WSACleanup();

    return 0;
}
