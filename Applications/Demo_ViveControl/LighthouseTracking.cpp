#include "LighthouseTracking.h"

LighthouseTracking::~LighthouseTracking() {
	if (m_pHMD != NULL)
	{
		vr::VR_Shutdown();
		m_pHMD = NULL;
	}
}

LighthouseTracking::LighthouseTracking() {
	vr::EVRInitError eError = vr::VRInitError_None;
	m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Background);

	if (eError != vr::VRInitError_None)
	{
		m_pHMD = NULL;
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
		printf_s(buf);
		exit(EXIT_FAILURE);
	}
}

/*
* Loop-listen for events then parses them (e.g. prints the to user)
* Returns true if success or false if openvr has quit
*/
bool LighthouseTracking::RunProcedure(bool waiting) {

	// waits for events if isWaiting
	if (waiting) 
	{
		// Process VREvent
		vr::VREvent_t event;
		while (m_pHMD->PollNextEvent(&event, sizeof(event)))
		{
			// Process event
			if (!ProcessVREvent(event)) {
				char buf[1024];
				sprintf_s(buf, sizeof(buf), "(OpenVR) service quit\n");
				printf_s(buf);
				return false;
			}
			// Parse while something is happening
			ParseTrackingFrame();
		}
	}
	//if not waiting automatically parses frames constantly
	else 
	{
		ParseTrackingFrame();
	}
	return true;
}

//-----------------------------------------------------------------------------
// Purpose: Processes a single VR event
//-----------------------------------------------------------------------------
bool LighthouseTracking::ProcessVREvent(const vr::VREvent_t & event)
{
	switch (event.eventType)
	{
	case vr::VREvent_TrackedDeviceActivated:
	{
		//SetupRenderModelForTrackedDevice(event.trackedDeviceIndex);
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Device : %d attached\n", event.trackedDeviceIndex);
		printf_s(buf);
	}
	break;

	case vr::VREvent_TrackedDeviceDeactivated:
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Device : %d detached\n", event.trackedDeviceIndex);
		printf_s(buf);
	}
	break;

	case vr::VREvent_TrackedDeviceUpdated:
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Device : %d updated\n", event.trackedDeviceIndex);
		printf_s(buf);
	}
	break;

	case (vr::VREvent_DashboardActivated) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Dashboard activated\n");
		printf_s(buf);
	}
										  break;

	case (vr::VREvent_DashboardDeactivated) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Dashboard deactivated\n");
		printf_s(buf);

	}
											break;

	case (vr::VREvent_ChaperoneDataHasChanged) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Chaperone data has changed\n");
		printf_s(buf);

	}
											   break;

	case (vr::VREvent_ChaperoneSettingsHaveChanged) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Chaperone settings have changed\n");
		printf_s(buf);
	}
													break;

	case (vr::VREvent_ChaperoneUniverseHasChanged) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Chaperone universe has changed\n");
		printf_s(buf);

	}
												   break;

	case (vr::VREvent_ApplicationTransitionStarted) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Application Transition: Transition has started\n");
		printf_s(buf);

	}
													break;

	case (vr::VREvent_ApplicationTransitionNewAppStarted) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Application transition: New app has started\n");
		printf_s(buf);

	}
														  break;

	case (vr::VREvent_Quit) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Received SteamVR Quit (%d", vr::VREvent_Quit, ")\n");
		printf_s(buf);

		return false;
	}
							break;

	case (vr::VREvent_ProcessQuit) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) SteamVR Quit Process (%d", vr::VREvent_ProcessQuit, ")\n");
		printf_s(buf);

		return false;
	}
								   break;

	case (vr::VREvent_QuitAborted_UserPrompt) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) SteamVR Quit Aborted UserPrompt (%d", vr::VREvent_QuitAborted_UserPrompt, ")\n");
		printf_s(buf);

		return false;
	}
											  break;

	case (vr::VREvent_QuitAcknowledged) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) SteamVR Quit Acknowledged (%d", vr::VREvent_QuitAcknowledged, ")\n");
		printf_s(buf);

		return false;
	}
										break;

	case (vr::VREvent_TrackedDeviceRoleChanged) :
	{

		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) TrackedDeviceRoleChanged: %d\n", event.trackedDeviceIndex);
		printf_s(buf);
		break;
	}

	case (vr::VREvent_TrackedDeviceUserInteractionStarted) :
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) TrackedDeviceUserInteractionStarted: %d\n", event.trackedDeviceIndex);
		printf_s(buf);
		break;
	}

	default:
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "(OpenVR) Event: %d\n", event.eventType);
		printf_s(buf);
		break;
	}

	return true;
}



vr::HmdQuaternion_t LighthouseTracking::GetRotation(vr::HmdMatrix34_t matrix) {
	vr::HmdQuaternion_t q;

	
	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
	return q;
}

vr::HmdVector3_t LighthouseTracking::GetPosition(vr::HmdMatrix34_t matrix) {
	vr::HmdVector3_t vector;

	vector.v[0] = matrix.m[0][3];
	vector.v[1] = matrix.m[1][3];
	vector.v[2] = matrix.m[2][3];

	return vector;
}

/*
* Parse a Frame with data from the tracking system
*
* Handy reference: // broken link, newtonVR adds button capabilities though
* https://github.com/TomorrowTodayLabs/NewtonVR/blob/master/Assets/SteamVR/Scripts/SteamVR_Utils.cs
*
* Also:
* Open VR Convention (same as OpenGL)
* right-handed system
* +y is up
* +x is to the right
* -z is going away from you
* http://www.3dgep.com/understanding-the-view-matrix/
*
*/
void LighthouseTracking::ParseTrackingFrame() {

	// Process SteamVR device states
	for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
	{
		if (!m_pHMD->IsTrackedDeviceConnected(unDevice))
			continue;

		vr::VRControllerState_t state;
		

		//For device
		vr::TrackedDevicePose_t trackedDevicePose;
		vr::TrackedDevicePose_t *devicePose = &trackedDevicePose;

		//For controller, of same type of device
		vr::TrackedDevicePose_t trackedControllerPose;
		vr::TrackedDevicePose_t *controllerPose = &trackedControllerPose;

		//Controllerstate type holds a struct for button pressed and touched, axis and last packet
		vr::VRControllerState_t controllerState;
		vr::VRControllerState_t *ontrollerState_ptr = &controllerState;

		vr::HmdVector3_t vector;
		vr::HmdQuaternion_t quaternion;

		if (vr::VRSystem()->IsInputFocusCapturedByAnotherProcess()) {
			char buf[1024];

			sprintf_s(buf, sizeof(buf), "\nInput Focus by Another Process\n");
			printf_s(buf);
		}

		bool bPoseValid = trackedDevicePose.bPoseIsValid;
		vr::HmdVector3_t vVel;
		vr::HmdVector3_t vAngVel;
		vr::ETrackingResult eTrackingResult;

		//for output
		char buf[1024];
		std::ofstream mystream;
		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
		std::string str(buffer);

		std::cout << str;
		double rx, ry, rz, iix, iiy, iiz;
		double lx, ly, lz, ix, iy, iz;


		vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);
		switch (trackedDeviceClass) {

		// print stuff for the HMD here, see controller example below
		case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:

			//get pose relative to safe bounds defined by user
			vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);

			//get position and rotation
			vector = GetPosition(devicePose->mDeviceToAbsoluteTracking);
			quaternion = GetRotation(devicePose->mDeviceToAbsoluteTracking);
				
			// get some data
			vVel = trackedDevicePose.vVelocity;
			vAngVel = trackedDevicePose.vAngularVelocity;
			eTrackingResult = trackedDevicePose.eTrackingResult;
			bPoseValid = trackedDevicePose.bPoseIsValid;

			sprintf_s(buf, sizeof(buf), "\nHMD\nx: %.4f y: %.4f z: %.4f\n", vector.v[0], vector.v[1], vector.v[2]);
			printf_s(buf);
			sprintf_s(buf, sizeof(buf), "qw: %.4f qx: %.4f qy: %.4f qz: %.4f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
			printf_s(buf);

			//io
			mystream.open("hmdq.txt", std::ofstream::out | std::ofstream::app);
			mystream << quaternion.w << " , " << quaternion.x << " , " << quaternion.y << " ," << quaternion.z << std::endl;
			mystream.close();
			
			mystream.open("hmdc.txt", std::ofstream::out | std::ofstream::app);
			mystream << std::endl;
			mystream << vector.v[0] << " , " << vector.v[1] << " , " << vector.v[2]<<std::flush;
			mystream.close();

			//Added with update ---------------------------------------------------------------------------------------------
			// and print some more info to the user about the state of the device/pose
			switch (eTrackingResult) 
			{
			case vr::ETrackingResult::TrackingResult_Uninitialized:
				sprintf_s(buf, sizeof(buf), "Invalid tracking result\n");
				printf_s(buf);
				break;
			case vr::ETrackingResult::TrackingResult_Calibrating_InProgress:
				sprintf_s(buf, sizeof(buf), "Calibrating in progress\n");
				printf_s(buf);
				break;
			case vr::ETrackingResult::TrackingResult_Calibrating_OutOfRange:
				sprintf_s(buf, sizeof(buf), "Calibrating Out of range\n");
				printf_s(buf);
				break;
			case vr::ETrackingResult::TrackingResult_Running_OK:
				sprintf_s(buf, sizeof(buf), "Running OK\n");
				printf_s(buf);
				break;
			case vr::ETrackingResult::TrackingResult_Running_OutOfRange:
				sprintf_s(buf, sizeof(buf), "WARNING: Running Out of Range\n");
				printf_s(buf);
				break;
			default:
				sprintf_s(buf, sizeof(buf), "Default\n");
				printf_s(buf);
				break;
			}

			// print if the pose is valid or not
			if (bPoseValid)
				sprintf_s(buf, sizeof(buf), "Valid pose\n");
			else
				sprintf_s(buf, sizeof(buf), "Invalid pose\n");
			printf_s(buf);
			//End added with update ----------------------------------------------------------------------------------------------
				
			break;

		case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
			vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState, sizeof(controllerState), &trackedDevicePose);

			vector = GetPosition(controllerPose->mDeviceToAbsoluteTracking);
			quaternion = GetRotation(controllerPose->mDeviceToAbsoluteTracking);

			//data
			vVel = trackedDevicePose.vVelocity;
			vAngVel = trackedDevicePose.vAngularVelocity;
			eTrackingResult = trackedDevicePose.eTrackingResult;
			bPoseValid = trackedDevicePose.bPoseIsValid;

			switch (vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(unDevice)) {
			case vr::TrackedControllerRole_Invalid:
				// invalid hand... 
				printf("hand invalid \n");
				break;

			case vr::TrackedControllerRole_LeftHand:

				//File io for lefthand coords at current possition
				mystream.open("l_coords.csv", std::ios::app);
				mystream << str << "," << vector.v[0] << "," << vector.v[1] << "," << vector.v[2] << "," << quaternion.w << "," << quaternion.x << "," << quaternion.y << "," << quaternion.z << std::endl;
				mystream.close();

				//File io for lefthand matrix at current possition
				mystream.open("l_rotmtx.csv", std::ios::app);
				mystream << str << "," << controllerPose->mDeviceToAbsoluteTracking.m[0][0] << "," << controllerPose->mDeviceToAbsoluteTracking.m[0][1] << "," << controllerPose->mDeviceToAbsoluteTracking.m[0][2] << "," << controllerPose->mDeviceToAbsoluteTracking.m[0][3];
				mystream << controllerPose->mDeviceToAbsoluteTracking.m[1][0] << "," << controllerPose->mDeviceToAbsoluteTracking.m[1][1] << "," << controllerPose->mDeviceToAbsoluteTracking.m[1][2] << "," << controllerPose->mDeviceToAbsoluteTracking.m[1][3];
				mystream << controllerPose->mDeviceToAbsoluteTracking.m[2][0] << "," << controllerPose->mDeviceToAbsoluteTracking.m[2][1] << "," << controllerPose->mDeviceToAbsoluteTracking.m[2][2] << "," << controllerPose->mDeviceToAbsoluteTracking.m[2][3];
				mystream << controllerPose->mDeviceToAbsoluteTracking.m[3][0] << "," << controllerPose->mDeviceToAbsoluteTracking.m[3][1] << "," << controllerPose->mDeviceToAbsoluteTracking.m[3][2] << std::endl;
				mystream.close();

					
				sprintf_s(buf, sizeof(buf), "\nLeft Controller\nx: %.4f y: %.4f z: %.4f\n", vector.v[0], vector.v[1], vector.v[2]);
				printf_s(buf);

				sprintf_s(buf, sizeof(buf), "qw: %.4f qx: %.4f qy: %.4f qz: %.4f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
				printf_s(buf);

				sprintf_s(buf, sizeof(buf), "------------\n");
				printf_s(buf);

				//Added with update ---------------------------------------------------------------------------------------------
				switch (eTrackingResult) {
				case vr::ETrackingResult::TrackingResult_Uninitialized:
					sprintf_s(buf, sizeof(buf), "Invalid tracking result\n");
					printf_s(buf);
					break;
				case vr::ETrackingResult::TrackingResult_Calibrating_InProgress:
					sprintf_s(buf, sizeof(buf), "Calibrating in progress\n");
					printf_s(buf);
					break;
				case vr::ETrackingResult::TrackingResult_Calibrating_OutOfRange:
					sprintf_s(buf, sizeof(buf), "Calibrating Out of range\n");
					printf_s(buf);
					break;
				case vr::ETrackingResult::TrackingResult_Running_OK:
					sprintf_s(buf, sizeof(buf), "Running OK\n");
					printf_s(buf);
					break;
				case vr::ETrackingResult::TrackingResult_Running_OutOfRange:
					sprintf_s(buf, sizeof(buf), "WARNING: Running Out of Range\n");
					printf_s(buf);

					break;
				default:
					sprintf_s(buf, sizeof(buf), "Default\n");
					printf_s(buf);
					break;
				}

				if (bPoseValid)
					sprintf_s(buf, sizeof(buf), "Valid pose\n");
				else
					sprintf_s(buf, sizeof(buf), "Invalid pose\n");
				printf_s(buf);
					//End added with update ----------------------------------------------------------------------------------------------


					/* --- // Output of old translation equation
					       // currently unused
					ix = vector.v[2];
					iy = vector.v[0];
					iz = vector.v[1];

					lx = (-10000000 * pow(ix, 6)) + (8000000 * pow(ix, 5)) + (-1000000 * pow(ix, 4)) + (-101344 * pow(ix, 3)) + (16055 * pow(ix, 2)) + (422.73 * ix) + 490.64;
					ly = (7558.4 * pow(iy, 4)) + (31807 * pow(iy, 3)) + (49016 * pow(iy, 2)) + (32057 * iy) + 7200.9;
					lz = (-0.0000008 * pow(iz, 2)) + (0.0011 * iz) + 0.9471;

					sprintf_s(buf, sizeof(buf), "robot tran: \n lx: %.4f ly: %.4f lz: %.6f \n", lx, ly, lz);
					printf_s(buf);
					*/

					/* ---  //File io for lefthand quaternion and cartesian coordinates in txt file for parsing by original unittest code
							//currently unused

					mystream.open("leftq.txt", std::ofstream::out | std::ofstream::app);
					mystream << str << quaternion.w << " , " << quaternion.x << " , " << quaternion.y << " ," << quaternion.z << std::endl;
					mystream.close();

					mystream.open("leftc.txt", std::ofstream::out | std::ofstream::app);
					mystream << std::endl;
					mystream << str << vector.v[0] << " , " << vector.v[1] << " , " << vector.v[2] << " , " << quaternion.w << " , " << quaternion.x << " , " << quaternion.y << " ," << quaternion.z;
					mystream.close();
					*/
				break;

			case vr::TrackedControllerRole_RightHand:

				//File io for righthand coords at current possition
				mystream.open("r_coords.csv", std::ios::app);
				mystream << str << "," << vector.v[0] << "," << vector.v[1] << "," << vector.v[2] << "," << quaternion.w << "," << quaternion.x << "," << quaternion.y << "," << quaternion.z << std::endl;
				mystream.close();

				//File io for righthand matrix at current possition
				mystream.open("r_rotmtx.csv", std::ios::app);
				mystream << str << "," << controllerPose->mDeviceToAbsoluteTracking.m[0][0] << "," << controllerPose->mDeviceToAbsoluteTracking.m[0][1] << "," << controllerPose->mDeviceToAbsoluteTracking.m[0][2] << "," << controllerPose->mDeviceToAbsoluteTracking.m[0][3];
				mystream << controllerPose->mDeviceToAbsoluteTracking.m[1][0] << "," << controllerPose->mDeviceToAbsoluteTracking.m[1][1] << "," << controllerPose->mDeviceToAbsoluteTracking.m[1][2] << "," << controllerPose->mDeviceToAbsoluteTracking.m[1][3];
				mystream << controllerPose->mDeviceToAbsoluteTracking.m[2][0] << "," << controllerPose->mDeviceToAbsoluteTracking.m[2][1] << "," << controllerPose->mDeviceToAbsoluteTracking.m[2][2] << "," << controllerPose->mDeviceToAbsoluteTracking.m[2][3];
				mystream << controllerPose->mDeviceToAbsoluteTracking.m[3][0] << "," << controllerPose->mDeviceToAbsoluteTracking.m[3][1] << "," << controllerPose->mDeviceToAbsoluteTracking.m[3][2] << std::endl;
				mystream.close();

					
				sprintf_s(buf, sizeof(buf), "\nRight Controller\nx: %.4f y: %.4f z: %.4f\n", vector.v[0], vector.v[1], vector.v[2]);
				printf_s(buf);

				sprintf_s(buf, sizeof(buf), "qw: %.4f qx: %.4f qy: %.4f qz: %.4f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
				printf_s(buf);

				sprintf_s(buf, sizeof(buf), "------------\n");
				printf_s(buf);

				//Added with update ---------------------------------------------------------------------------------------------
				switch (eTrackingResult) {
				case vr::ETrackingResult::TrackingResult_Uninitialized:
					sprintf_s(buf, sizeof(buf), "Invalid tracking result\n");
					printf_s(buf);
					break;
				case vr::ETrackingResult::TrackingResult_Calibrating_InProgress:
					sprintf_s(buf, sizeof(buf), "Calibrating in progress\n");
					printf_s(buf);
					break;
				case vr::ETrackingResult::TrackingResult_Calibrating_OutOfRange:
					sprintf_s(buf, sizeof(buf), "Calibrating Out of range\n");
					printf_s(buf);
					break;
				case vr::ETrackingResult::TrackingResult_Running_OK:
					sprintf_s(buf, sizeof(buf), "Running OK\n");
					printf_s(buf);
					break;
				case vr::ETrackingResult::TrackingResult_Running_OutOfRange:
					sprintf_s(buf, sizeof(buf), "WARNING: Running Out of Range\n");
					printf_s(buf);
					break;
				default:
					sprintf_s(buf, sizeof(buf), "Default\n");
					printf_s(buf);
					break;
				}

				if (bPoseValid)
					sprintf_s(buf, sizeof(buf), "Valid pose\n");
				else
					sprintf_s(buf, sizeof(buf), "Invalid pose\n");
				printf_s(buf);

				//End added with update ----------------------------------------------------------------------------------------------

					/* --- //Output of old translation equation
					       //currently unused
					iix = vector.v[2];
					iiy = vector.v[0];
					iiz = vector.v[1];

					rx = (-10000000 * pow(iix, 6)) + (8000000 * pow(iix, 5)) + (-1000000 * pow(iix, 4)) + (-101344 * pow(iix, 3)) + (16055 * pow(iix, 2)) + (422.73 * iix) + 490.64;
					ry = (-70465 * pow(iiy, 3)) + (-360108 * pow(iiy, 2)) + (-612667 * iiy)  -347204;
					rz = (-0.00000008 * pow(iz, 2)) + (0.0011 * iz) + 0.9471;

					sprintf_s(buf, sizeof(buf), "robot tran: \n rx: %.4f ry: %.4f rz: %.6f \n", rx, ry, rz);
					printf_s(buf);
					*/

					
					/*  ---	//File io for righthand quaternion and cartesian coordinates in txt file for parsing by original unittest code
					        //currently unused

					mystream.open("rightq.txt", std::ofstream::out | std::ofstream::app);
					mystream << str << quaternion.w << " , " << quaternion.x << " , " << quaternion.y << " ," << quaternion.z << std::endl;
					mystream.close();

					mystream.open("rightc.txt", std::ofstream::out | std::ofstream::app);
					mystream << std::endl;
					mystream << str << vector.v[0] << " , " << vector.v[1] << " , " << vector.v[2] << " , " << quaternion.w << " , " << quaternion.x << " , " << quaternion.y << " ," << quaternion.z;
					mystream.close();
					*/

				break;

			}

			break;
			}

		}
}