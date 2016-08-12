/* note on transitions:
pos.m[0][3]		X	away from screen
pos.m[1][3]		Y	to ceiling
pos.m[2][3]		Z	to left from screen
*/

#include <iostream>
#include <string>
#include <array>

#include <kinect.h>
#include <openvr.h>

#include "samples.hpp"


template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


void errexit(IKinectSensor* sensor, std::string message) {
	std::cerr << "ERROR: " << message << std::endl;
	sensor->Close();
	SafeRelease(sensor);
	exit(0);
}


int main(int argc, char** argv) {
	// initialize OpenVR
	vr::EVRInitError error = vr::VRInitError_None;
	vr::IVRSystem* hmd = vr::VR_Init(&error, vr::VRApplication_Background);
	if (error != vr::VRInitError_None) {
		hmd = NULL;
		std::cerr << "ERROR while initializing" << std::endl;
		return 0;
	}

	// initialize KinectSDK
	HRESULT hr;
	IKinectSensor* sensor = NULL;
	hr = GetDefaultKinectSensor(&sensor);
	if (FAILED(hr))
		errexit(sensor, "cannot find default kinect");
	hr = sensor->Open();
	if (FAILED(hr))
		errexit(sensor, "cannot open stream");
	IBodyFrameSource* source = NULL;
	hr = sensor->get_BodyFrameSource(&source);
	if (FAILED(hr))
		errexit(sensor, "cannot get frame source");
	IBodyFrameReader* reader = NULL;
	hr = source->OpenReader(&reader);
	if (FAILED(hr))
		errexit(sensor, "cannot open frame reader");
	source->Release();
	source = NULL;
	IBodyFrame* frame = NULL;

	// samples
	Samples samples;

	// positions
	vr::HmdMatrix34_t openvr_pos;
	CameraSpacePoint kinect_pos;

	std::cout << "Initialization finished..." << std::endl;

	while (true) {
		//acquire Kinect position
		hr = reader->AcquireLatestFrame(&frame);
		if (FAILED(hr)) {
			SafeRelease(frame);
			continue;
		}
		IBody* bodies[BODY_COUNT] = { 0 };
		hr = frame->GetAndRefreshBodyData(BODY_COUNT, bodies);
		if (FAILED(hr))
			errexit(sensor, "cannot get body data");
		for (IBody* body : bodies) {
			BOOLEAN tracked;
			body->get_IsTracked(&tracked);
			if (!tracked)
				continue;
			Joint joints[JointType_Count];
			body->GetJoints(JointType_Count, joints);
			kinect_pos = joints[11].Position;
		}
		SafeRelease(frame);

		//acquire openvr position and handle events
		vr::VREvent_t event;
		vr::TrackedDevicePose_t pose;
		while (hmd->PollNextEventWithPose(vr::TrackingUniverseStanding, &event, sizeof(event), &pose)) {
			if (hmd->GetTrackedDeviceClass(event.trackedDeviceIndex) == vr::TrackedDeviceClass_Controller) {
				if (event.eventType == vr::VREvent_TrackedDeviceDeactivated) {
					std::cout << "WARNING: device signal lost" << std::endl;
				}
				else if (event.eventType == vr::VREvent_TrackedDeviceActivated) {
					std::cout << "INFORMATION: device found" << std::endl;
				}
				else if (event.data.controller.button == vr::k_EButton_ApplicationMenu) {
					if (event.eventType == vr::VREvent_ButtonPress) {
						openvr_pos = pose.mDeviceToAbsoluteTracking;
						if (samples.add(openvr_pos, kinect_pos))
							std::cout << "added sample " << samples.length << std::endl;
						else {
							std::cout << "did not add sample:" << std::endl;
							std::cout << "    Openvr - " << Vec3(openvr_pos) << std::endl;
							std::cout << "    Kinect - " << Vec3(kinect_pos) << std::endl;
						}
					}
				}
				else if (event.data.controller.button == vr::k_EButton_Grip) {
					goto exit;
				}
			}
		}
	}

exit:
	std::cout << "\n---------- OpenVR ---------   SAMPLES   --------- Kinect ----------" << std::endl;
	for (std::pair<Vec3, Vec3> point : samples.data) {
		std::cout << point.first << "\t\t" << point.second << std::endl;
	}
	std::cout << "\nPress enter to exit..." << std::endl;
	std::cin.get();
	sensor->Close();
	vr::VR_Shutdown();
	return 0;
}