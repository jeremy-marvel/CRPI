// LIGHTHOUSETRACKING.h
#ifndef _LIGHTHOUSETRACKING_H_
#define _LIGHTHOUSETRACKING_H_

// OpenVR
#include <openvr.h>
#include "shared\Matrices.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include<time.h>
#pragma warning(disable:4996) 
class LighthouseTracking {
private:
	//if false, the program will continuously track and not wait for events
	bool isWaiting = false;

	//Variables for outputting translation live
	float rx, ry, rz; 
	float vx, vy, vz;

public:
	/* Variables that should be private but aren't for outward accessing------------------------------*/
	// Basic Variables
	vr::IVRSystem *m_pHMD = NULL;
	vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
	Matrix4 m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];

	//Possition and Rotation of pose
	vr::HmdQuaternion_t LighthouseTracking::GetRotation(vr::HmdMatrix34_t matrix);
	vr::HmdVector3_t LighthouseTracking::GetPosition(vr::HmdMatrix34_t matrix);
	/* -----------------------------------------------------------------------------------------------*/


	//constructor, destructor
	~LighthouseTracking();
	LighthouseTracking();

	// Main loop that listens for openvr events and calls process and parse routines, if false the service has quit
	bool RunProcedure(bool waiting);

	// Process a VR event and print some general info of what happens
	bool ProcessVREvent(const vr::VREvent_t & event);

	// Parse a tracking frame and print its position / rotation
	void ParseTrackingFrame();

	//Timekeeping for robot calibration
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

};

#endif _LIGHTHOUSETRACKING_H_